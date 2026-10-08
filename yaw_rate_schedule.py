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
        self.brake_start = float(cfg.get("brake_start_fraction", 0.75))
        self.gain = float(cfg.get("rate_gain_nm_per_rps", 0.08))
        self.max_moment = float(cfg.get("max_corrective_moment_nm", 0.20))
        self.hard_abort_multiplier = float(cfg.get("hard_abort_multiplier", 1.6))
        if (self.nodes.shape != (11,) or
            not np.allclose(self.nodes, np.arange(0, 101, 10)) or
            self.limits.shape != (11,) or
            not np.isfinite(self.limits).all() or
            (self.limits <= 0).any() or
            not 0.0 < self.brake_start < 1.0 or
            not 0.0 < self.gain or
            not 0.0 < self.max_moment or
            not self.hard_abort_multiplier > 1.0):
            raise ValueError("yaw_rate_schedule requires 11 finite positive limits at 0,10,...,100% and valid braking coefficients")

    def limit_deg_s(self, loss_percent):
        x = float(loss_percent)
        if not math.isfinite(x):
            raise ValueError("yaw rate schedule needs a finite loss estimate")
        return float(np.interp(np.clip(x, 0.0, 100.0), self.nodes, self.limits))

    def requested_moment(self, loss_percent, measured_body_yaw_rate_rps):
        """Soft ceiling with anticipatory braking, keeping yaw *angle* free."""
        limit = math.radians(self.limit_deg_s(loss_percent))
        rate = float(measured_body_yaw_rate_rps)
        if not math.isfinite(rate):
            raise ValueError("non-finite gyro yaw rate")
        if not self.enabled:
            return 0.0
        excess = max(0.0, abs(rate) - self.brake_start * limit)
        return float(-math.copysign(min(self.max_moment, self.gain * excess), rate)) if excess > 0 else 0.0

    def abort_limit_rps(self, loss_percent):
        return math.radians(self.limit_deg_s(loss_percent)) * self.hard_abort_multiplier


def allocate_pair_yaw_control(mixer, cmd, failed_motor_id, loss_percent, target_yaw_moment_nm):
    """Allocate F/Mx/My plus bounded Mz through physical thrust nullspace.

    The equation B f_actual = [F,Mx,My] is enforced BEFORE using the remaining
    one-dimensional degree of freedom for yaw. No virtual yaw-rate clamp.
    If the primary wrench is infeasible, request a safe failure to the caller.
    """
    if failed_motor_id not in (1, 2, 3, 4):
        raise ValueError("failed motor id must be 1..4")
    loss = float(loss_percent)
    if not math.isfinite(loss) or not (0 <= loss <= 100):
        raise ValueError("fault percentage must be in [0,100]")
    if not math.isfinite(target_yaw_moment_nm):
        raise ValueError("yaw torque request must be finite")
    eta = np.ones(4, dtype=float)
    index = failed_motor_id - 1
    other = index ^ 1
    eta[index] = eta[other] = 1 - loss / 100.0
    B = np.vstack((
        np.ones(4),
        np.array([-1., 1., 1., -1.])*float(mixer.L)*0.5,
        np.array([1., -1., 1., -1.])*float(mixer.D)*0.5,
    ))
    A = B @ np.diag(eta)
    if np.linalg.matrix_rank(A, tol=1e-8) < 3:
        raise RuntimeError("paired motors leave an uncontrollable primary wrench")
    # Minimum-norm *actual* rotor thrust solution, then yaw-nullspace motion.
    y = np.asarray(cmd[:3], dtype=float)
    f0 = B.T @ np.linalg.solve(B @ B.T, y)
    n = np.array([1.0, 1.0, -1.0, -1.0])
    cap = eta * float(mixer.motor.thrust(100.0))
    lo = -float("inf")
    hi = float("inf")
    for fi, ni, ci in zip(f0, n, cap):
        a, b = (-fi / ni), ((ci-fi)/ni)
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
    if not (np.isfinite(lo) and np.isfinite(hi) and lo <= hi):
        raise RuntimeError("primary thrust/roll/pitch wrench infeasible with rotor thrust bounds")

    def get_yaw(s):
        thrust = f0 + s*n
        w_actual = [mixer.motor.w_from_thrust(max(0.0, f)) for f in thrust]
        moments = np.array([mixer.motor.moment(w) for w in w_actual])
        return float(np.array([1., 1., -1., -1.]) @ moments)

    yaw_a = get_yaw(lo)
    yaw_b = get_yaw(hi)
    target = float(np.clip(target_yaw_moment_nm, min(yaw_a, yaw_b), max(yaw_a, yaw_b)))
    left, right = lo, hi
    inc = yaw_b >= yaw_a
    for _ in range(28):
        mid = (left+right)*0.5
        ym = get_yaw(mid)
        if (ym < target) == inc:
            left = mid
        else:
            right = mid
    s = (left+right)*0.5
    f_applied = np.clip(f0+s*n, 0.0, cap)
    # f_nominal commands generate the actual f_applied only AFTER losses.
    f_nominal = f_applied / eta
    w_cmd = np.array([mixer.motor.w_from_thrust(f) for f in f_nominal])
    realized_target = get_yaw(s)
    if not np.isfinite(w_cmd).all():
        raise RuntimeError("non-finite yaw-limited actuator allocation")
    return np.clip(w_cmd, 0.0, 100.0), {
        "requested_yaw_nm": float(target_yaw_moment_nm),
        "bounded_yaw_nm": target,
        "predicted_yaw_nm": realized_target,
        "available_yaw_min_nm": min(yaw_a, yaw_b),
        "available_yaw_max_nm": max(yaw_a, yaw_b),
        "saturated": abs(target - target_yaw_moment_nm) > 1e-7,
        "primary_wrench_feasible": True,
    }
