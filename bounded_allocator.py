"""Bounded four-rotor thrust allocation: F first, Roll/Pitch next, Yaw last.

Pure NumPy, deterministic and valid for an arbitrary 0..100% loss on any
rotor. Unlike clipping an unconstrained pseudoinverse vector, output actual
thrust ALWAYS respects both rotor bounds and the physically reachable
collective. Roll/pitch is Euclidean-projected onto the *exact* 2-D convex
wrench polygon at that fixed collective; yaw then uses only its nullspace.
This is a simulated safety fallback, not a flight-qualified allocator.
"""
import itertools
import math
import numpy as np


def _hull(points):
    """CCW convex hull of (Mx,My,f-vector); degenerate 0/1/2D supported."""
    unique = {}
    for xy, forces in points:
        key = (round(float(xy[0]), 11), round(float(xy[1]), 11))
        unique[key] = (np.asarray(xy, dtype=float), np.asarray(forces, dtype=float))
    ordered = [unique[key] for key in sorted(unique)]
    if len(ordered) <= 2:
        return ordered

    def cross(o, a, b):
        x, y = a[0]-o[0], b[0]-o[0]
        return float(x[0]*y[1]-x[1]*y[0])

    lower = []
    for p in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-11:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-11:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def bounded_allocate(mixer, cmd, degraded_motor_id, loss_percent):
    """Return (nominal commands, metrics) for 4 physical rotor thrust bounds.

    A desired F outside the actuator collective envelope is clamped to that
    envelope. At the resulting exact F, unattainable (Mx,My) is projected
    onto the convex feasible polygon; yaw has strictly the lowest priority.
    The algorithm never silently trades collective thrust for attitude.
    """
    cmd = np.asarray(cmd, dtype=float)
    if cmd.shape != (4,) or not np.isfinite(cmd).all():
        raise ValueError("command must be four finite F/Mx/My/Mz values")
    if int(degraded_motor_id) not in (1, 2, 3, 4):
        raise ValueError("degraded_motor_id must be 1..4")
    if not np.isfinite(loss_percent) or not 0.0 <= loss_percent <= 100.0:
        raise ValueError("loss_percent must be within [0,100]")
    eta = np.ones(4, dtype=float)
    eta[int(degraded_motor_id)-1] = 1.0 - loss_percent/100.0
    max_nominal = float(mixer.motor.thrust(100.0))
    cap = eta * max_nominal
    collective = float(np.clip(cmd[0], 0.0, sum(cap)))
    B = np.array([
        [1., 1., 1., 1.],
        [-1., 1., 1., -1.],
        [1., -1., 1., -1.]
    ], dtype=float)
    B[1] *= .5 * float(mixer.L)
    B[2] *= .5 * float(mixer.D)
    if not np.isfinite(B).all() or min(mixer.L, mixer.D) <= 0:
        raise ValueError("invalid quad geometry")
    # All extreme points of the intersection
    #     {0 <= f_i <= cap_i} and {sum f_i = F}.
    # At a vertex, at least 3 of 4 independent rotor bounds are active.
    vertices = []
    for free in range(4):
        others = [i for i in range(4) if i != free]
        for bits in itertools.product((0, 1), repeat=3):
            f = np.zeros(4)
            f[others] = cap[others] * np.asarray(bits)
            f[free] = collective - float(f.sum())
            if -1e-8 <= f[free] <= cap[free] + 1e-8:
                f[free] = np.clip(f[free], 0.0, cap[free])
                vertices.append((B[1:] @ f, f))
    hull = _hull(vertices)
    if not hull:
        raise RuntimeError("bounded allocator failed to construct collective slice")

    want = cmd[1:3]
    closest = None
    best_distance2 = float("inf")
    # The 2D polygon is convex, so any interior request is exactly feasible.
    inside = len(hull) >= 3 and all(
        (float(np.cross(np.r_[hull[(i+1)%len(hull)][0]-hull[i][0], 0.0],
                        np.r_[want-hull[i][0], 0.0])[2]) >= -1e-9)
        for i in range(len(hull))
    )
    if inside:
        selected = want.copy()
    else:
        for i in range(len(hull)):
            a = hull[i][0]
            b = hull[(i+1) % len(hull)][0] if len(hull)>1 else a
            edge = b-a
            sq = float(edge@edge)
            frac = np.clip(float((want-a)@edge)/sq, 0.0, 1.0) if sq>1e-14 else 0.0
            p = a + frac*edge
            d2 = float((want-p)@(want-p))
            if d2 < best_distance2:
                best_distance2, closest = d2, p
        selected = closest
    demand = np.array([collective, selected[0], selected[1]])
    base = B.T @ np.linalg.solve(B @ B.T, demand)
    null = np.array([1., 1., -1., -1.])
    lo, hi = -float("inf"), float("inf")
    for fi, ni, fi_max in zip(base, null, cap):
        a, b = -fi/ni, (fi_max-fi)/ni
        lo, hi = max(lo, min(a,b)), min(hi, max(a,b))
    if lo > hi + 2e-7:
        raise RuntimeError("bounded allocator nullspace invariant violated")
    if lo > hi:
        lo = hi = .5*(lo+hi)

    def yaw_moment(s):
        f = np.clip(base+s*null, 0.0, cap)
        w_actual = [mixer.motor.w_from_thrust(float(v)) for v in f]
        return float(np.dot(
            np.array([1., 1., -1., -1.]),
            [mixer.motor.moment(w) for w in w_actual]
        ))
    yaw_lo, yaw_hi = yaw_moment(lo), yaw_moment(hi)
    clipped_yaw = float(np.clip(cmd[3], min(yaw_lo,yaw_hi), max(yaw_lo,yaw_hi)))
    increasing = yaw_hi >= yaw_lo
    start, stop = lo, hi
    for _ in range(22):
        mid = .5*(start+stop)
        if (yaw_moment(mid) < clipped_yaw) == increasing:
            start = mid
        else:
            stop = mid
    real_thrust = np.clip(base+.5*(start+stop)*null, 0.0, cap)
    nominal_thrust = np.zeros(4)
    viable = eta > 1e-10
    nominal_thrust[viable] = real_thrust[viable]/eta[viable]
    command = np.array(
        [mixer.motor.w_from_thrust(float(f)) for f in nominal_thrust],
        dtype=float,
    )
    if not np.isfinite(command).all() or (command < -1e-7).any() or (command > 100+1e-7).any():
        raise RuntimeError("invalid bounded motor command")
    # Diagnostic predicted outputs from *command* after effectiveness, not
    # from the unclipped mathematical reference.
    predicted = np.array([mixer.motor.thrust(float(w)) for w in command])*eta
    achieved = B @ predicted
    primary_error = achieved - cmd[:3]
    saturated = bool(
        abs(float(achieved[0]-cmd[0])) > 5e-4 or
        float(np.linalg.norm(primary_error[1:])) > 1e-3
    )
    return np.clip(command, 0.0, 100.0), {
        "primary_feasible": not saturated,
        "primary_saturated": saturated,
        "collective_clamped": abs(collective - cmd[0]) > 1e-7,
        "collective_requested_n": float(cmd[0]),
        "collective_achievable_n": collective,
        "collective_predicted_n": float(achieved[0]),
        "collective_error_n": float(achieved[0]-cmd[0]),
        "roll_error_nm": float(primary_error[1]),
        "pitch_error_nm": float(primary_error[2]),
        "yaw_requested_nm": float(cmd[3]),
        "yaw_min_nm": min(yaw_lo, yaw_hi),
        "yaw_max_nm": max(yaw_lo, yaw_hi),
        "yaw_predicted_nm": yaw_moment(.5*(start+stop)),
        "motor_lower_count": int(sum(predicted < 1e-5)),
        "motor_upper_count": int(sum((cap-predicted < 1e-5) & (cap > 1e-8))),
    }
