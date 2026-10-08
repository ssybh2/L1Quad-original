"""Mode29 opposite-motor matched-loss experiment for the MuJoCo simulator.

The original source branch remains untouched. This module deliberately adds
no hard yaw-rate clamp or yaw damping; pair mode is intended to expose spin.
It is NOT flight-certified or a substitute for closed-loop vehicle testing.
"""
import math
import numpy as np


def opposite_motor(motor_id: int) -> int:
    """M1<->M2 and M3<->M4; follow the frozen firmware's mixer numbering."""
    if motor_id not in (1, 2, 3, 4):
        raise ValueError("motor_id must be 1..4")
    return ((motor_id - 1) ^ 1) + 1


def allocate_opposite_pair(mixer, cmd, failed_motor_id, estimated_loss_percent):
    """Minimum-norm effectiveness-aware F, Mx, My allocator (yaw uncontrolled).

    This matches the experimental Pixhawk allocator A^T (A A^T)^-1 u,
    with A = B diag(eta) and the failed *and* opposite motors sharing eta.
    Return nominal PWM-space command [0..100]. The fault is applied separately.
    """
    other = opposite_motor(failed_motor_id)
    loss = float(estimated_loss_percent)
    if not np.isfinite(loss) or not 0.0 <= loss <= 100.0:
        raise ValueError("paired loss estimate must be in [0, 100]")
    eta = np.ones(4, dtype=float)
    eta[failed_motor_id - 1] = 1.0 - loss / 100.0
    eta[other - 1] = eta[failed_motor_id - 1]
    l_span, d_span = float(mixer.L), float(mixer.D)
    B = np.vstack((
        np.ones(4),
        0.5 * l_span * np.array([-1.0, 1.0, 1.0, -1.0]),
        0.5 * d_span * np.array([1.0, -1.0, 1.0, -1.0]),
    ))
    A = B @ np.diag(eta)
    gram = A @ A.T
    # The 100%+100% opposite pair leaves only two live actuator columns.
    # Such a configuration cannot independently track F, Mx and My.
    if not np.isfinite(gram).all() or np.linalg.matrix_rank(A, tol=1.0e-8) < 3:
        raise RuntimeError("opposite-pair allocator rank-deficient: cannot independently control F/Mx/My")
    if np.linalg.cond(gram) > 1.0e5:
        raise RuntimeError("opposite-pair allocator ill-conditioned: excessive control amplification")
    desired = np.asarray(cmd[:3], dtype=float)
    f_nominal = A.T @ np.linalg.solve(gram, desired)
    w = np.array(
        [mixer.motor.w_from_thrust(max(0.0, f)) for f in f_nominal],
        dtype=float,
    )
    if not np.isfinite(w).all():
        raise RuntimeError("opposite-pair produced non-finite motor commands")
    return np.clip(w, 0.0, 100.0)


def apply_opposite_model_loss(w_after_original_fault, motor, paired_motor_id, loss_pct):
    """Model synthetic opposite rotor fault in thrust space, not PWM percentage."""
    w = np.asarray(w_after_original_fault, dtype=float).copy()
    if paired_motor_id not in (1, 2, 3, 4):
        raise ValueError("invalid synthetic opposite motor")
    if not np.isfinite(loss_pct) or not 0 <= loss_pct <= 100:
        raise ValueError("invalid synthetic loss")
    idx = paired_motor_id - 1
    f_nom = motor.thrust(float(w[idx]))
    w[idx] = motor.w_from_thrust((1.0 - loss_pct / 100.0) * f_nom)
    return np.clip(w, 0.0, 100.0)


class OppositePairExperiment:
    """Use blind FDI output to enable one synthetic opposite rotor impairment.

    Injector truth is consulted only as a controlled-experiment *veto*.
    The observer never gets the injected motor or percentage as an input.
    FDI must remain frozen while two motors are degraded because its
    original one-motor signature model is no longer identifiable.
    """
    def __init__(self, cfg, vehicle, motor, max_tilt_deg):
        self.enabled = bool(cfg.get("enabled", False))
        self.source = str(cfg.get("source", "fdi")).lower()
        if self.source not in ("fdi", "oracle"):
            raise ValueError("opposite_pair.source must be fdi or oracle")
        self.motor = motor
        self.mass = float(vehicle["mass_kg"])
        self.gravity = float(vehicle["gravity_mps2"])
        self.max_tilt_deg = float(max_tilt_deg)
        self.min_condition = float(cfg.get("min_allocation_reciprocal_condition", 1.0e-5))
        self.max_bias = float(cfg.get("max_estimate_bias_percent", 8.0))
        self.severity_settle_s = float(cfg.get("severity_settle_time_s", 0.12))
        self.severity_step_tolerance_pp = float(cfg.get("severity_step_tolerance_pp", 1.0))
        self.L = float(vehicle["L_m"]) if "L_m" in vehicle else 0.28
        self.D = float(vehicle["D_m"]) if "D_m" in vehicle else 0.28
        self.min_margin = float(cfg.get("min_static_thrust_margin", 1.5))
        self.max_spin = float(cfg.get("max_body_yaw_rate_rps", 4.0))
        self.max_xy = float(cfg.get("max_xy_error_m", 0.5))
        self.max_z = float(cfg.get("max_z_error_m", 0.4))
        if self.enabled and (not (0 < self.min_condition <= 1) or
                             min(self.max_spin, self.max_xy, self.max_z, self.min_margin) <= 0
                             or self.severity_settle_s < 0 or self.severity_step_tolerance_pp <= 0):
            raise ValueError("invalid opposite_pair safety configuration")
        self.active = False
        self.inhibited = False
        self.failed_motor_id = 0
        self.opposite_motor_id = 0
        self.estimated_loss_percent = 0.0
        self.static_thrust_margin = 0.0
        self.reason = ""
        self.activated_at_s = None
        self.disengaged_at_s = None
        self.yaw_abort_limit_rps = self.max_spin
        self.severity_stable_since_s = None
        self.severity_prev_pct = None
        self.retry_pending = False
        self.transient_reason = ""

    def update(self, t, measured_pos, measured_omega, target_altitude,
               original_fault_active, injector, detector, yaw_envelope=None):
        if not self.enabled:
            return
        pos = np.asarray(measured_pos, dtype=float)
        omega = np.asarray(measured_omega, dtype=float)
        finite = np.isfinite(pos).all() and np.isfinite(omega).all()
        xy_error = float(np.linalg.norm(pos[:2])) if finite else float("inf")
        z_error = abs(float(pos[2]) + float(target_altitude)) if finite else float("inf")
        yaw_rate = abs(float(omega[2])) if finite else float("inf")
        # Physical yaw-rate ceiling is loss-dependent in paired experiments.
        # No simulator gyro clipping. A separate harder abort handles lack
        # of torque authority or excessive transients.
        chosen_loss = (self.estimated_loss_percent if self.active else
                       (injector.loss_percent if self.source == "oracle" else
                        float(detector.loss_estimate_percent)))
        yaw_abort = (yaw_envelope.abort_limit_rps(chosen_loss)
                     if yaw_envelope is not None and yaw_envelope.enabled
                     else self.max_spin)
        self.yaw_abort_limit_rps = yaw_abort
        guard = (not finite or xy_error > self.max_xy or z_error > self.max_z or
                 yaw_rate > yaw_abort)

        if self.active:
            if guard or not original_fault_active or (self.source == "fdi" and not detector.confirmed):
                self.active = False
                self.inhibited = True
                self.disengaged_at_s = float(t)
                if not original_fault_active:
                    self.reason = "injected fault ended"
                elif self.source == "fdi" and not detector.confirmed:
                    self.reason = "FDI confirmation lost"
                elif not finite:
                    self.reason = "non-finite position or angular rate"
                elif xy_error > self.max_xy:
                    self.reason = f"XY error {xy_error:.3f} m > {self.max_xy:.3f} m"
                elif z_error > self.max_z:
                    self.reason = f"height error {z_error:.3f} m > {self.max_z:.3f} m"
                elif yaw_rate > yaw_abort:
                    self.reason = f"yaw rate {yaw_rate:.3f} rad/s > hard abort {yaw_abort:.3f} rad/s"
                else:
                    self.reason = "unknown paired-fault disengagement"
            return
        if not original_fault_active:
            self.severity_stable_since_s = None
            self.severity_prev_pct = None
            self.retry_pending = False
            self.transient_reason = ""
            return
        if self.inhibited:
            return
        if guard:
            return
        if self.source == "oracle":
            # Simulation-only independent check of allocation physics:
            # do NOT mistake this for an observer-estimated failure.
            estimated_motor_id = injector.motor_id
            loss = float(injector.loss_percent)
        else:
            if not detector.confirmed:
                return
            estimated_motor_id = detector.detected_id
            if estimated_motor_id != injector.motor_id:
                self.inhibited = True
                self.reason = "FDI motor ID disagrees with controlled injected fault"
                return
            loss = float(detector.loss_estimate_percent)
            if not np.isfinite(loss) or not np.isfinite(detector.residual_ratio):
                return
            if loss <= 0.0 or detector.residual_ratio > 0.35:
                self.severity_stable_since_s = None
                self.severity_prev_pct = None
                self.retry_pending = True
                self.transient_reason = "FDI severity/residual not ready"
                return
        if not (0.0 <= loss <= 100.0) or not (0.0 <= injector.loss_percent <= 100.0):
            self.inhibited = True
            self.reason = "loss must be in [0,100]%"
            return
        if loss <= 0.0:
            # An exact 0% injection creates no faulty motor to identify.
            return
        if self.source == "fdi":
            # Motor ID confirmation is NOT a guarantee that severity has settled.
            # Temporary magnitude disagreement is retryable, not a permanent
            # actuator safety fault. We require a sustained stable estimate.
            if (self.severity_prev_pct is None or
                abs(loss - self.severity_prev_pct) > self.severity_step_tolerance_pp):
                self.severity_stable_since_s = float(t)
            self.severity_prev_pct = loss
            if abs(loss - injector.loss_percent) > self.max_bias:
                self.retry_pending = True
                self.transient_reason = "FDI severity still disagrees with injected benchmark"
                self.severity_stable_since_s = None
                return
            if self.severity_stable_since_s is None:
                self.severity_stable_since_s = float(t)
            if float(t) - self.severity_stable_since_s < self.severity_settle_s:
                self.retry_pending = True
                self.transient_reason = "FDI severity settling"
                return
        self.retry_pending = False
        self.transient_reason = ""
        eta = 1.0 - loss/100.0
        # Observability is independent from actuator feasibility. Check
        # rank/conditioning, not an arbitrary loss-percentage cutoff.
        L = self.L
        D = self.D
        geom = np.vstack((np.ones(4),
            0.5 * L * np.array([-1., 1., 1., -1.]),
            0.5 * D * np.array([1., -1., 1., -1.])))
        effectiveness = np.ones(4)
        effectiveness[estimated_motor_id - 1] = eta
        effectiveness[opposite_motor(estimated_motor_id) - 1] = eta
        allocation = geom @ np.diag(effectiveness)
        if (np.linalg.matrix_rank(allocation, tol=1.0e-8) < 3 or
                1.0 / np.linalg.cond(allocation @ allocation.T) < self.min_condition):
            self.inhibited = True
            self.reason = "insufficient independent control authority (rank/condition)"
            return
        required = (self.mass * self.gravity /
                    max(math.cos(math.radians(self.max_tilt_deg)), 0.1))
        # w=80 corresponds to the source calibration's 1800us measured range.
        self.static_thrust_margin = (2.0*(1.0+eta)*self.motor.thrust(80.0) /
                                     max(required, 1.0e-6))
        if self.static_thrust_margin < self.min_margin:
            self.inhibited = True
            self.reason = "insufficient estimated thrust margin"
            return
        self.active = True
        self.failed_motor_id = estimated_motor_id
        self.opposite_motor_id = opposite_motor(self.failed_motor_id)
        self.estimated_loss_percent = loss
        self.activated_at_s = float(t)
