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
    if not np.isfinite(loss) or not 0.0 <= loss <= 70.0:
        raise ValueError("paired loss estimate outside supported interval")
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
    if not np.isfinite(gram).all() or abs(np.linalg.det(gram)) <= 1.0e-9:
        raise RuntimeError("opposite-pair allocation singular")
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
        self.max_loss = float(cfg.get("max_loss_percent", 70.0))
        self.max_bias = float(cfg.get("max_estimate_bias_percent", 8.0))
        self.min_margin = float(cfg.get("min_static_thrust_margin", 1.5))
        self.max_spin = float(cfg.get("max_body_yaw_rate_rps", 4.0))
        self.max_xy = float(cfg.get("max_xy_error_m", 0.5))
        self.max_z = float(cfg.get("max_z_error_m", 0.4))
        if self.enabled and (not 0 < self.max_loss <= 70 or
                             min(self.max_spin, self.max_xy, self.max_z, self.min_margin) <= 0):
            raise ValueError("invalid opposite_pair safety configuration")
        self.active = False
        self.inhibited = False
        self.failed_motor_id = 0
        self.opposite_motor_id = 0
        self.estimated_loss_percent = 0.0
        self.static_thrust_margin = 0.0
        self.reason = ""
        self.activated_at_s = None

    def update(self, t, measured_pos, measured_omega, target_altitude,
               original_fault_active, injector, detector):
        if not self.enabled:
            return
        pos = np.asarray(measured_pos, dtype=float)
        omega = np.asarray(measured_omega, dtype=float)
        finite = np.isfinite(pos).all() and np.isfinite(omega).all()
        xy_error = float(np.linalg.norm(pos[:2])) if finite else float("inf")
        z_error = abs(float(pos[2]) + float(target_altitude)) if finite else float("inf")
        yaw_rate = abs(float(omega[2])) if finite else float("inf")
        guard = (not finite or xy_error > self.max_xy or z_error > self.max_z or
                 yaw_rate > self.max_spin)

        if self.active:
            if guard or not original_fault_active or (self.source == "fdi" and not detector.confirmed):
                self.active = False
                self.inhibited = True
                self.reason = "pair disengaged: fault ended, detector lost or state guard exceeded"
            return
        if self.inhibited or not original_fault_active:
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
            if loss < 60.0 or detector.residual_ratio > 0.35:
                return
        if loss > self.max_loss or injector.loss_percent > self.max_loss:
            self.inhibited = True
            self.reason = "partial-actuator experiment limited to <=70% paired loss"
            return
        if self.source == "fdi" and abs(loss - injector.loss_percent) > self.max_bias:
            self.inhibited = True
            self.reason = "FDI estimate disagrees with known injected loss"
            return
        eta = 1.0 - loss/100.0
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
