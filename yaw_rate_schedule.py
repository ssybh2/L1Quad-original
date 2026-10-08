"""Configurable 0..100% yaw-rate envelope for paired-motor MuJoCo tests.

This controller requests *reaction torque* through the motor mixer's real
nullspace, not an unphysical edit to the measured angular velocity.
A cap is a controller target; actuator saturation and delay can overshoot it.
"""
import math
import numpy as np


class YawRateEnvelope:
    def __init__(self, cfg, gain_schedule=None):
        self.enabled = bool(cfg.get("enabled", False))
        self.nodes = np.asarray(cfg.get("loss_nodes_percent", list(range(0, 101, 10))), dtype=float)
        # Each 10%-step gain anchor can own its yaw limit in the SAME
        # independently editable TOML block as kp,kv,kr,ko,max_tilt_deg.
        # The legacy global array remains a fallback for older configs.
        fallback = np.asarray(cfg.get("max_yaw_rate_deg_s", [180.0]*11), dtype=float)
        if fallback.shape != (11,):
            raise ValueError("yaw_rate_schedule.max_yaw_rate_deg_s must have 11 entries")
        anchors = gain_schedule or {}
        self.limits = np.asarray([
            float(anchors.get(f"loss_{int(loss)}", {}).get("max_yaw_rate_deg_s", fallback[i]))
            for i, loss in enumerate(self.nodes)
        ], dtype=float)
        self.target_spin_fraction = float(cfg.get("target_spin_fraction", 0.65))
        self.brake_start = float(cfg.get("brake_start_fraction", 0.75))
        self.gain = float(cfg.get("rate_gain_nm_per_rps", 0.08))
        self.max_moment = float(cfg.get("max_corrective_moment_nm", 0.20))
        self.hard_abort_multiplier = float(cfg.get("hard_abort_multiplier", 1.6))
        if (self.nodes.shape != (11,) or
            not np.allclose(self.nodes, np.arange(0, 101, 10)) or
            self.limits.shape != (11,) or
            not np.isfinite(self.limits).all() or
            (self.limits <= 0).any() or
            not 0.0 < self.target_spin_fraction < self.brake_start < 1.0 or
            not 0.0 < self.gain or
            not 0.0 < self.max_moment or
            not self.hard_abort_multiplier > 1.0):
            raise ValueError("yaw_rate_schedule requires 11 finite positive limits at 0,10,...,100% and valid braking coefficients")

    def limit_deg_s(self, loss_percent):
        x = float(loss_percent)
        if not math.isfinite(x):
            raise ValueError("yaw rate schedule needs a finite loss estimate")
        return float(np.interp(np.clip(x, 0.0, 100.0), self.nodes, self.limits))

    def target_rate_rps(self, loss_percent, spin_direction):
        """Choose a nonzero stable spin reference BELOW the configurable cap."""
        direction = float(spin_direction)
        if direction not in (-1.0, 1.0):
            raise ValueError("spin_direction must be +1 or -1")
        return direction * math.radians(self.limit_deg_s(loss_percent)) * self.target_spin_fraction

    def requested_moment(self, loss_percent, measured_body_yaw_rate_rps, spin_direction=1):
        """Rotor reaction-torque P feedback, preserving untracked yaw heading.

        Below the setpoint: drive the aircraft to spin; above: brake.
        Speed is never edited directly. Unavailable rotor yaw torque is
        reported as saturation by the allocator, not silently invented.
        """
        rate = float(measured_body_yaw_rate_rps)
        if not math.isfinite(rate):
            raise ValueError("non-finite gyro yaw rate")
        if not self.enabled:
            return 0.0
        target = self.target_rate_rps(loss_percent, spin_direction)
        cap = math.radians(self.limit_deg_s(loss_percent))
        requested = self.gain * (target - rate)
        if abs(rate) > self.brake_start * cap:
            # Additional anticipatory braking near the envelope.
            requested -= math.copysign(
                self.gain * (abs(rate) - self.brake_start * cap), rate
            )
        return float(np.clip(requested, -self.max_moment, self.max_moment))

    def abort_limit_rps(self, loss_percent):
        return math.radians(self.limit_deg_s(loss_percent)) * self.hard_abort_multiplier


def paired_primary_feasibility(mixer, cmd, failed_motor_id, loss_percent,
                               mirror_loss_percent=None, reserve_n=0.0):
    """Bounded F/Mx/My feasibility for ANY primary and opposite loss fractions.

    This test uses the actual asymmetric rotor caps. Unlike a static
    collective-thrust margin, it rejects transient roll/pitch moment requests
    outside the achievable wrench polytope. It DOES NOT use motor torque to
    repair an infeasible primary wrench.
    """
    from opposite_pair_sim import opposite_motor
    other = opposite_motor(int(failed_motor_id)) - 1
    primary = float(loss_percent)
    mirror = primary if mirror_loss_percent is None else float(mirror_loss_percent)
    if not np.isfinite([primary, mirror, reserve_n]).all() or not (
        0 <= primary <= 100 and 0 <= mirror <= 100 and reserve_n >= 0
    ):
        raise ValueError("motor losses must be finite in [0,100], reserve >=0")
    eta = np.ones(4)
    eta[failed_motor_id - 1] = 1 - primary / 100
    eta[other] = 1 - mirror / 100
    b = np.vstack((
        np.ones(4),
        np.array([-1., 1., 1., -1.]) * float(mixer.L) * .5,
        np.array([1., -1., 1., -1.]) * float(mixer.D) * .5,
    ))
    a = b @ np.diag(eta)
    empty = {"feasible": False, "reason": "rank deficient", "eta": eta,
             "interval_width_n": 0.0}
    if np.linalg.matrix_rank(a, tol=1e-8) < 3:
        return empty
    desired = np.asarray(cmd[:3], dtype=float)
    if desired.shape != (3,) or not np.isfinite(desired).all():
        raise ValueError("primary wrench must be finite F/Mx/My")
    f0 = b.T @ np.linalg.solve(b @ b.T, desired)
    n = np.array([1., 1., -1., -1.])
    cap = eta * float(mixer.motor.thrust(100.0))
    lo, hi = -float("inf"), float("inf")
    for fi, ni, ci in zip(f0, n, cap):
        left, right = (reserve_n - fi) / ni, (ci - reserve_n - fi) / ni
        lo, hi = max(lo, min(left, right)), min(hi, max(left, right))
    if not np.isfinite([lo, hi]).all() or lo > hi + 1e-9:
        return {**empty, "reason": "primary F/Mx/My wrench infeasible with rotor bounds"}
    return {"feasible": True, "reason": "", "eta": eta, "f0": f0,
            "null_direction": n, "cap": cap, "lo": lo, "hi": hi,
            "interval_width_n": max(0., float(hi-lo))}


def allocate_pair_yaw_control(mixer, cmd, failed_motor_id, loss_percent,
                              target_yaw_moment_nm, mirror_loss_percent=None,
                              reserve_n=0.0):
    """Highest priority F/Mx/My; Yaw moment uses ONLY the remaining nullspace.

    primary fault and synthetic opposite fault can have *different* effective
    losses during a smooth mirror ramp. Never clip the primary wrench silently.
    """
    result = paired_primary_feasibility(
        mixer, cmd, failed_motor_id, loss_percent,
        mirror_loss_percent=mirror_loss_percent, reserve_n=reserve_n
    )
    if not result["feasible"]:
        raise RuntimeError(result["reason"])
    if not math.isfinite(target_yaw_moment_nm):
        raise ValueError("non-finite yaw moment request")
    lo, hi = result["lo"], result["hi"]
    f0, n, cap, eta = (result[k] for k in
                       ("f0", "null_direction", "cap", "eta"))

    def get_yaw(s):
        force = f0 + s*n
        w_actual = [mixer.motor.w_from_thrust(max(0., f)) for f in force]
        moments = np.array([mixer.motor.moment(w) for w in w_actual])
        return float(np.array([1., 1., -1., -1.]) @ moments)

    yaw_a, yaw_b = get_yaw(lo), get_yaw(hi)
    target = float(np.clip(target_yaw_moment_nm,
                           min(yaw_a, yaw_b), max(yaw_a, yaw_b)))
    left, right = lo, hi
    increasing = yaw_b >= yaw_a
    for _ in range(28):
        mid = (left + right) * 0.5
        if (get_yaw(mid) < target) == increasing:
            left = mid
        else:
            right = mid
    scalar = 0.5*(left+right)
    applied = np.clip(f0 + scalar*n, 0., cap)
    if (eta <= 0).any():
        raise RuntimeError("pair allocation requires positive effectiveness")
    nominal = applied / eta
    command = np.array([mixer.motor.w_from_thrust(f) for f in nominal])
    if not np.isfinite(command).all():
        raise RuntimeError("non-finite paired motor command")
    return np.clip(command, 0., 100.), {
        "requested_yaw_nm": float(target_yaw_moment_nm),
        "bounded_yaw_nm": target,
        "predicted_yaw_nm": get_yaw(scalar),
        "available_yaw_min_nm": min(yaw_a, yaw_b),
        "available_yaw_max_nm": max(yaw_a, yaw_b),
        "saturated": abs(target - target_yaw_moment_nm) > 1e-7,
        "primary_wrench_feasible": True,
        "primary_nullspace_width_n": result["interval_width_n"],
        "mirrored_loss_pct": float(loss_percent if mirror_loss_percent is None
                                    else mirror_loss_percent),
    }
