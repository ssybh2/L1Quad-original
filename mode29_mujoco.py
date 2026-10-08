from __future__ import annotations

import argparse
import csv
import math
import time
import tomllib
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from opposite_pair_sim import (OppositePairExperiment, allocate_opposite_pair,
                               apply_opposite_model_loss)
from yaw_rate_schedule import YawRateEnvelope, allocate_pair_yaw_control


# Controller NED/FRD <-> MuJoCo NWU/FLU.
S = np.diag([1.0, -1.0, -1.0])


def hat(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]], dtype=float)


def normalize(v):
    n = float(np.linalg.norm(v))
    if (not np.isfinite(n)) or n < 1.0e-12:
        raise RuntimeError("cannot normalize near-zero/non-finite vector")
    return v / n


def so3_exp(phi):
    """Small rotation vector -> rotation matrix."""
    phi = np.asarray(phi, dtype=float)
    theta = float(np.linalg.norm(phi))
    if theta < 1.0e-12:
        return np.eye(3) + hat(phi)
    axis = phi / theta
    K = hat(axis)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rpy_to_R_ned_frd(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], dtype=float)
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], dtype=float)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], dtype=float)
    return Rz @ Ry @ Rx


def R_to_rpy_ned_frd(R):
    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)


def mat_to_quat_wxyz(R):
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([
            0.25*s,
            (R[2,1]-R[1,2])/s,
            (R[0,2]-R[2,0])/s,
            (R[1,0]-R[0,1])/s,
        ])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
        q = np.array([
            (R[2,1]-R[1,2])/s,
            0.25*s,
            (R[0,1]+R[1,0])/s,
            (R[0,2]+R[2,0])/s,
        ])
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
        q = np.array([
            (R[0,2]-R[2,0])/s,
            (R[0,1]+R[1,0])/s,
            0.25*s,
            (R[1,2]+R[2,1])/s,
        ])
    else:
        s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
        q = np.array([
            (R[1,0]-R[0,1])/s,
            (R[0,2]+R[2,0])/s,
            (R[1,2]+R[2,1])/s,
            0.25*s,
        ])
    return q / np.linalg.norm(q)


def combined_tilt_deg(R):
    return math.degrees(math.acos(float(np.clip(R[2,2], -1.0, 1.0))))


def takeoff_trajectory(t, altitude, duration):
    s = float(np.clip(t/duration, 0.0, 1.0))
    s2 = s*s
    s3 = s2*s
    s4 = s3*s
    s5 = s4*s
    s6 = s5*s
    s7 = s6*s

    p = 35.0*s4 - 84.0*s5 + 70.0*s6 - 20.0*s7
    dp = 140.0*s3 - 420.0*s4 + 420.0*s5 - 140.0*s6
    d2p = 420.0*s2 - 1680.0*s3 + 2100.0*s4 - 840.0*s5
    d3p = 840.0*s - 5040.0*s2 + 8400.0*s3 - 4200.0*s4
    d4p = 840.0 - 10080.0*s + 25200.0*s2 - 16800.0*s3

    invT = 1.0/duration
    pos = np.array([0.0, 0.0, -altitude*p])
    vel = np.array([0.0, 0.0, -altitude*dp*invT])
    acc = np.array([0.0, 0.0, -altitude*d2p*invT**2])
    jerk = np.array([0.0, 0.0, -altitude*d3p*invT**3])
    snap = np.array([0.0, 0.0, -altitude*d4p*invT**4])

    yaw = np.array([1.0, 0.0])
    yaw_dot = np.zeros(2)
    yaw_ddot = np.zeros(2)
    return pos, vel, acc, jerk, snap, yaw, yaw_dot, yaw_ddot


def trajectory_at(t, altitude, duration):
    if t < duration:
        return takeoff_trajectory(t, altitude, duration)

    z3 = np.zeros(3)
    z2 = np.zeros(2)
    return (
        np.array([0.0, 0.0, -altitude]),
        z3.copy(), z3.copy(), z3.copy(), z3.copy(),
        np.array([1.0, 0.0]), z2.copy(), z2.copy()
    )


class MotorModel:
    def __init__(self, cfg):
        self.fd = float(cfg["f_w_dead"])
        self.fc3 = float(cfg["f_c3"])
        self.fc2 = float(cfg["f_c2"])
        self.fc1 = float(cfg["f_c1"])
        self.md = float(cfg["m_w_dead"])
        self.mc3 = float(cfg["m_c3"])
        self.mc2 = float(cfg["m_c2"])
        self.mc1 = float(cfg["m_c1"])

    @staticmethod
    def _eval(w, dead, c3, c2, c1):
        x = max(0.0, float(w)-dead)
        return max(0.0, ((c3*x + c2)*x + c1)*x)

    @staticmethod
    def _slope(w, dead, c3, c2, c1):
        x = max(0.0, float(w)-dead)
        return max(1.0e-6, (3.0*c3*x + 2.0*c2)*x + c1)

    def thrust(self, w):
        return self._eval(w, self.fd, self.fc3, self.fc2, self.fc1)

    def thrust_slope(self, w):
        return self._slope(w, self.fd, self.fc3, self.fc2, self.fc1)

    def moment(self, w):
        return self._eval(w, self.md, self.mc3, self.mc2, self.mc1)

    def moment_slope(self, w):
        return self._slope(w, self.md, self.mc3, self.mc2, self.mc1)

    def w_from_thrust(self, thrust_n):
        if (not np.isfinite(thrust_n)) or thrust_n <= 0.0:
            return 0.0
        lo, hi = self.fd, 100.0
        if thrust_n >= self.thrust(hi):
            return hi
        for _ in range(24):
            mid = 0.5*(lo+hi)
            if self.thrust(mid) < thrust_n:
                lo = mid
            else:
                hi = mid
        return 0.5*(lo+hi)


class Mixer:
    def __init__(self, L, D, motor):
        self.L = float(L)
        self.D = float(D)
        self.motor = motor

    def _refine(self, w, cmd):
        Fcmd, Mx, My, Mz = [float(x) for x in cmd]

        F = np.array([self.motor.thrust(x) for x in w])
        dF = np.array([self.motor.thrust_slope(x) for x in w])
        cF = F - dF*w

        M = np.array([self.motor.moment(x) for x in w])
        dM = np.array([self.motor.moment_slope(x) for x in w])
        cM = M - dM*w

        A = np.array([
            [ dF[0],  dF[1],  dF[2],  dF[3]],
            [-dF[0],  dF[1],  dF[2], -dF[3]],
            [ dF[0], -dF[1],  dF[2], -dF[3]],
            [ dM[0],  dM[1], -dM[2], -dM[3]],
        ])

        b = np.array([
            Fcmd - cF.sum(),
            2.0*Mx/self.L + cF[0] - cF[1] - cF[2] + cF[3],
            2.0*My/self.D - cF[0] + cF[1] - cF[2] + cF[3],
            Mz - cM[0] - cM[1] + cM[2] + cM[3],
        ])

        return np.linalg.solve(A, b)

    def allocate(self, cmd):
        Fcmd, Mx, My, Mz = [float(x) for x in cmd]

        w0 = self.motor.w_from_thrust(max(0.0, 0.25*Fcmd))
        dF = self.motor.thrust_slope(w0)
        dM = self.motor.moment_slope(w0)
        offsetF = self.motor.thrust(w0) - dF*w0
        thrust_biased = Fcmd - 4.0*offsetF

        w = np.array([
            thrust_biased/(4*dF) - Mx/(2*self.L*dF) + My/(2*self.D*dF) + Mz/(4*dM),
            thrust_biased/(4*dF) + Mx/(2*self.L*dF) - My/(2*self.D*dF) + Mz/(4*dM),
            thrust_biased/(4*dF) + Mx/(2*self.L*dF) + My/(2*self.D*dF) - Mz/(4*dM),
            thrust_biased/(4*dF) - Mx/(2*self.L*dF) - My/(2*self.D*dF) - Mz/(4*dM),
        ])

        w = self._refine(w, cmd)
        w = self._refine(w, cmd)

        if not np.isfinite(w).all():
            raise RuntimeError("non-finite motor allocation")

        return np.clip(w, 0.0, 100.0)

    def allocate_yaw_free(self, cmd):
        """Minimum-norm allocation for collective thrust, roll and pitch."""
        Fcmd, Mx, My = [float(x) for x in cmd[:3]]
        motor_thrust = np.array([
            0.25*Fcmd - Mx/(2.0*self.L) + My/(2.0*self.D),
            0.25*Fcmd + Mx/(2.0*self.L) - My/(2.0*self.D),
            0.25*Fcmd + Mx/(2.0*self.L) + My/(2.0*self.D),
            0.25*Fcmd - Mx/(2.0*self.L) - My/(2.0*self.D),
        ], dtype=float)
        w = np.array(
            [self.motor.w_from_thrust(max(0.0, fi)) for fi in motor_thrust],
            dtype=float,
        )
        if not np.isfinite(w).all():
            raise RuntimeError("non-finite yaw-free motor allocation")
        return np.clip(w, 0.0, 100.0)

    def allocate_effectiveness_aware(self, cmd, degraded_motor_id, loss_percent):
        """Allocate a degraded motor while retaining bounded yaw-rate authority.

        Collective thrust, roll and pitch define a one-dimensional family of
        actual rotor thrusts. Use that remaining null-space degree of freedom
        to track as much of the requested yaw moment as the degraded motors can
        produce, without sacrificing the three primary axes.
        """
        if degraded_motor_id not in (1, 2, 3, 4) or not np.isfinite(loss_percent):
            return self.allocate_yaw_free(cmd)

        eta = np.ones(4, dtype=float)
        eta[degraded_motor_id-1] = 1.0 - np.clip(float(loss_percent), 0.0, 100.0)/100.0
        roll = 0.5*self.L*np.array([-1.0, +1.0, +1.0, -1.0])
        pitch = 0.5*self.D*np.array([+1.0, -1.0, +1.0, -1.0])

        B = np.vstack((np.ones(4), roll, pitch))
        desired = np.asarray(cmd[:3], dtype=float)
        max_actual_thrust = eta*self.motor.thrust(100.0)

        try:
            gram = B @ B.T
            if abs(float(np.linalg.det(gram))) < 1.0e-12:
                return self.allocate_yaw_free(cmd)
            actual_thrust_0 = B.T @ np.linalg.solve(gram, desired)
        except np.linalg.LinAlgError:
            return self.allocate_yaw_free(cmd)

        # Null(B) is the yaw-allocation direction. Find its feasible interval
        # under both the healthy and degraded motor thrust limits.
        _, _, vh = np.linalg.svd(B)
        null_direction = vh[-1]
        s_lo = -float("inf")
        s_hi = float("inf")
        for fi, ni, fi_max in zip(actual_thrust_0, null_direction, max_actual_thrust):
            if abs(float(ni)) < 1.0e-12:
                if fi < 0.0 or fi > fi_max:
                    return self.allocate_yaw_free(cmd)
                continue
            bound_0 = -fi/ni
            bound_1 = (fi_max-fi)/ni
            s_lo = max(s_lo, min(bound_0, bound_1))
            s_hi = min(s_hi, max(bound_0, bound_1))

        if not np.isfinite(s_lo) or not np.isfinite(s_hi) or s_lo > s_hi:
            # The primary wrench is itself infeasible. Retain the previous
            # minimum-norm behavior instead of trading primary authority for yaw.
            A = B @ np.diag(eta)
            try:
                motor_thrust = A.T @ np.linalg.solve(A @ A.T, desired)
            except np.linalg.LinAlgError:
                return self.allocate_yaw_free(cmd)
            w = np.array(
                [self.motor.w_from_thrust(max(0.0, fi)) for fi in motor_thrust],
                dtype=float,
            )
            return np.clip(w, 0.0, 100.0)

        yaw_sign = np.array([1.0, 1.0, -1.0, -1.0])

        def yaw_moment_at(s):
            actual_thrust = np.clip(
                actual_thrust_0 + null_direction*s,
                0.0,
                max_actual_thrust,
            )
            applied_w = np.array(
                [self.motor.w_from_thrust(fi) for fi in actual_thrust],
                dtype=float,
            )
            reaction = np.array(
                [self.motor.moment(wi) for wi in applied_w],
                dtype=float,
            )
            return float(yaw_sign @ reaction)

        yaw_target = float(cmd[3]) if len(cmd) >= 4 and np.isfinite(cmd[3]) else 0.0
        yaw_lo = yaw_moment_at(s_lo)
        yaw_hi = yaw_moment_at(s_hi)
        increasing = yaw_hi >= yaw_lo
        yaw_target = float(np.clip(yaw_target, min(yaw_lo, yaw_hi), max(yaw_lo, yaw_hi)))

        # The reaction-moment fit is monotonic along this null-space direction.
        lo, hi = s_lo, s_hi
        for _ in range(24):
            mid = 0.5*(lo+hi)
            yaw_mid = yaw_moment_at(mid)
            if (yaw_mid < yaw_target) == increasing:
                lo = mid
            else:
                hi = mid

        actual_thrust = np.clip(
            actual_thrust_0 + null_direction*0.5*(lo+hi),
            0.0,
            max_actual_thrust,
        )
        motor_thrust = actual_thrust/np.maximum(eta, 1.0e-6)
        w = np.array(
            [self.motor.w_from_thrust(max(0.0, fi)) for fi in motor_thrust],
            dtype=float,
        )
        if not np.isfinite(w).all():
            raise RuntimeError("non-finite effectiveness-aware allocation")
        return np.clip(w, 0.0, 100.0)


def unit_vec(q, q_dot, q_ddot):
    nq = float(np.linalg.norm(q))
    if nq < 1.0e-12 or not np.isfinite(nq):
        raise RuntimeError("unit_vec invalid norm")

    qqd = float(q @ q_dot)
    u = q/nq
    u_dot = q_dot/nq - q*qqd/(nq**3)
    u_ddot = (
        q_ddot/nq
        - q_dot*(2.0*qqd)/(nq**3)
        - q*(float(q_dot@q_dot)+float(q@q_ddot))/(nq**3)
        + q*3.0*(qqd**2)/(nq**5)
    )
    return u, u_dot, u_ddot


class GeometricController:
    def __init__(self, vehicle, gains, safety):
        self.m = float(vehicle["mass_kg"])
        self.g = float(vehicle["gravity_mps2"])
        self.J = np.diag([
            float(vehicle["jxx_kgm2"]),
            float(vehicle["jyy_kgm2"]),
            float(vehicle["jzz_kgm2"]),
        ])
        self.kp = np.array([gains["kpx"], gains["kpy"], gains["kpz"]], dtype=float)
        self.kv = np.array([gains["kvx"], gains["kvy"], gains["kvz"]], dtype=float)
        self.kr = np.array([gains["krx"], gains["kry"], gains["krz"]], dtype=float)
        self.ko = np.array([gains["kox"], gains["koy"], gains["koz"]], dtype=float)
        self.max_tilt_deg = float(safety["max_tilt_deg"])

    def set_tuning(self, tuning):
        self.kp = np.asarray(tuning["kp"], dtype=float)
        self.kv = np.asarray(tuning["kv"], dtype=float)
        self.kr = np.asarray(tuning["kr"], dtype=float)
        self.ko = np.asarray(tuning["ko"], dtype=float)
        self.max_tilt_deg = float(tuning["max_tilt_deg"])

    def compute(self, state, ref, yaw_free=False):
        pos, vel, R, Omega = state
        tpos, tvel, tacc, tjerk, tsnap, tyaw, tyaw_dot, tyaw_ddot = ref

        r_error = pos - tpos
        v_error = vel - tvel

        target_force = self.m*tacc - self.kp*r_error - self.kv*v_error
        target_force[2] -= self.m*self.g

        # Same upright-thrust cone projection as current firmware.
        limited = False
        desired_body_z_force = -target_force.copy()
        min_upright_force = 0.05*self.m*self.g

        if desired_body_z_force[2] < min_upright_force:
            desired_body_z_force[2] = min_upright_force
            limited = True

        h = math.hypot(desired_body_z_force[0], desired_body_z_force[1])
        max_h = desired_body_z_force[2]*math.tan(math.radians(self.max_tilt_deg))

        if h > max_h and h > 1.0e-6:
            desired_body_z_force[:2] *= max_h/h
            limited = True

        target_force = -desired_body_z_force

        e3 = np.array([0.0, 0.0, 1.0])
        target_thrust = -float(target_force @ R[:,2])

        zdes = normalize(-target_force)
        xc = np.array([tyaw[0], tyaw[1], 0.0])
        xc_dot = np.array([tyaw_dot[0], tyaw_dot[1], 0.0])
        xc_ddot = np.array([tyaw_ddot[0], tyaw_ddot[1], 0.0])

        ydes = normalize(np.cross(zdes, xc))
        xdes = np.cross(ydes, zdes)
        Rdes = np.column_stack((xdes, ydes, zdes))

        eRM = 0.5*(Rdes.T@R - R.T@Rdes)
        eR = vee(eRM)

        a_error = e3*self.g - R[:,2]*(target_thrust/self.m) - tacc

        force_dot = -self.kp*v_error - self.kv*a_error + self.m*tjerk
        if limited:
            force_dot[:] = 0.0

        b3_dot = R @ hat(Omega) @ e3
        thrust_dot = -float(force_dot @ R[:,2]) - float(target_force @ b3_dot)

        j_error = -R[:,2]*(thrust_dot/self.m) - b3_dot*(target_thrust/self.m) - tjerk

        force_ddot = -self.kp*a_error - self.kv*j_error + self.m*tsnap
        if limited:
            force_ddot[:] = 0.0

        b3c, b3c_dot, b3c_ddot = unit_vec(-target_force, -force_dot, -force_ddot)

        if yaw_free:
            # Reduced-attitude control: track only the thrust direction.  The
            # rotation about body Z is deliberately absent from both attitude
            # and rate error, so a freely spinning heading cannot leak into
            # the roll/pitch commands used by the position loop.
            eR_reduced = R.T @ np.cross(b3c, R[:, 2])
            omega_ref_world = np.cross(b3c, b3c_dot)
            omega_ref_dot_world = np.cross(b3c, b3c_ddot)
            omega_ref_body = R.T @ omega_ref_world
            omega_ref_dot_body = (
                -hat(Omega) @ omega_ref_body
                + R.T @ omega_ref_dot_world
            )
            ew_reduced = Omega - omega_ref_body
            eR_reduced[2] = 0.0
            ew_reduced[2] = 0.0

            moment = -self.kr*eR_reduced - self.ko*ew_reduced
            moment += self.J @ omega_ref_dot_body
            moment += np.cross(Omega, self.J@Omega)
            moment[2] = 0.0

            return {
                "cmd": np.array([target_thrust, *moment]),
                "Rdes": Rdes,
                "limited": limited,
            }

        A2 = -hat(xc) @ b3c
        A2_dot = -hat(xc_dot)@b3c - hat(xc)@b3c_dot
        A2_ddot = -hat(xc_ddot)@b3c - 2.0*hat(xc_dot)@b3c_dot - hat(xc)@b3c_ddot

        b2c, b2c_dot, b2c_ddot = unit_vec(A2, A2_dot, A2_ddot)

        b1c_dot = hat(b2c_dot)@b3c + hat(b2c)@b3c_dot
        b1c_ddot = hat(b2c_ddot)@b3c + 2.0*hat(b2c_dot)@b3c_dot + hat(b2c)@b3c_ddot

        Rd_dot = np.column_stack((b1c_dot, b2c_dot, b3c_dot))
        Rd_ddot = np.column_stack((b1c_ddot, b2c_ddot, b3c_ddot))

        Omegad = vee(Rdes.T @ Rd_dot)
        Omegad_dot = vee(Rdes.T@Rd_ddot - hat(Omegad)@hat(Omegad))

        ew = Omega - R.T@Rdes@Omegad

        moment = -self.kr*eR - self.ko*ew
        moment -= self.J @ (
            hat(Omega)@R.T@Rdes@Omegad
            - R.T@Rdes@Omegad_dot
        )
        moment += np.cross(Omega, self.J@Omega)

        return {
            "cmd": np.array([target_thrust, *moment]),
            "Rdes": Rdes,
            "limited": limited,
        }


class L1AdaptiveAugmentation:
    """L1Quad state predictor, uncertainty estimator and filtered augmentation.

    This software uses or is derived from the L1Quad software developed by the
    Department of Mechanical Science and Engineering at the University of
    Illinois Urbana-Champaign.
    """

    def __init__(self, vehicle, controller_cfg, cfg, dt):
        self.enabled = bool(int(controller_cfg.get("l1enable", 0)))
        self.dt = float(dt)
        self.mass = float(vehicle["mass_kg"])
        self.gravity = float(vehicle["gravity_mps2"])
        self.J = np.diag([
            float(vehicle["jxx_kgm2"]),
            float(vehicle["jyy_kgm2"]),
            float(vehicle["jzz_kgm2"]),
        ])
        self.Jinv = np.linalg.inv(self.J)

        # Flight-tested softdrone values from the upstream Mode29 profile.
        self.as_v = float(cfg.get("as_v", -10.0))
        self.as_omega = float(cfg.get("as_omega", -15.0))
        self.cutoff_thrust = float(cfg.get("cutoff_thrust_rad_s", 15.0))
        self.cutoff_moment_1 = float(cfg.get("cutoff_moment_1_rad_s", 5.0))
        self.cutoff_moment_2 = float(cfg.get("cutoff_moment_2_rad_s", 15.0))
        self.topology_transition_hold = float(cfg.get("topology_transition_hold_s", 0.0))

        if self.dt <= 0.0 or not np.isfinite(self.dt):
            raise ValueError("L1 timestep must be finite and positive")
        if self.as_v >= 0.0 or self.as_omega >= 0.0:
            raise ValueError("L1 as_v and as_omega must be negative")
        if min(self.cutoff_thrust, self.cutoff_moment_1, self.cutoff_moment_2) <= 0.0:
            raise ValueError("L1 cutoff frequencies must be positive")
        if self.topology_transition_hold < 0.0:
            raise ValueError("L1 topology_transition_hold_s must be nonnegative")

        self.initialized = False
        self.v_prev = np.zeros(3)
        self.omega_prev = np.zeros(3)
        self.R_prev = np.eye(3)
        self.u_b_prev = np.zeros(4)
        self.v_hat = np.zeros(3)
        self.omega_hat = np.zeros(3)
        self.u_ad = np.zeros(4)
        self.sigma_m = np.zeros(4)
        self.sigma_um = np.zeros(2)
        self.lpf1 = np.zeros(4)
        self.lpf2 = np.zeros(4)

    def reset(self, state, baseline_cmd=None):
        _, vel, R, omega = state
        self.v_prev = np.asarray(vel, dtype=float).copy()
        self.omega_prev = np.asarray(omega, dtype=float).copy()
        self.R_prev = np.asarray(R, dtype=float).copy()
        self.v_hat = self.v_prev.copy()
        self.omega_hat = self.omega_prev.copy()
        if baseline_cmd is None:
            self.u_b_prev.fill(0.0)
        else:
            self.u_b_prev = np.asarray(baseline_cmd, dtype=float).copy()
        self.u_ad.fill(0.0)
        self.sigma_m.fill(0.0)
        self.sigma_um.fill(0.0)
        self.lpf1.fill(0.0)
        self.lpf2.fill(0.0)
        self.initialized = True

    def _synchronize_disabled(self, state, baseline_cmd):
        _, vel, R, omega = state
        self.v_prev = np.asarray(vel, dtype=float).copy()
        self.omega_prev = np.asarray(omega, dtype=float).copy()
        self.R_prev = np.asarray(R, dtype=float).copy()
        self.v_hat = self.v_prev.copy()
        self.omega_hat = self.omega_prev.copy()
        self.u_b_prev = np.asarray(baseline_cmd, dtype=float).copy()
        self.u_ad.fill(0.0)
        self.sigma_m.fill(0.0)
        self.sigma_um.fill(0.0)
        self.lpf1.fill(0.0)
        self.lpf2.fill(0.0)

    def update(self, state, baseline_cmd, suppress_yaw_control=False):
        """Return [adaptive thrust, adaptive body moments] for this sample."""
        _, vel, R, omega = state
        vel = np.asarray(vel, dtype=float)
        R = np.asarray(R, dtype=float)
        omega = np.asarray(omega, dtype=float)
        baseline_cmd = np.asarray(baseline_cmd, dtype=float)

        if not self.initialized:
            self.reset(state)

        if not self.enabled:
            self._synchronize_disabled(state, baseline_cmd)
            return self.u_ad.copy()

        e3 = np.array([0.0, 0.0, 1.0])
        vpred_error_prev = self.v_hat - self.v_prev
        omegapred_error_prev = self.omega_hat - self.omega_prev

        matched_thrust_prev = (
            self.u_b_prev[0] + self.u_ad[0] + self.sigma_m[0]
        )
        self.v_hat = self.v_hat + (
            e3*self.gravity
            - self.R_prev[:, 2]*(matched_thrust_prev/self.mass)
            + self.R_prev[:, 0]*(self.sigma_um[0]/self.mass)
            + self.R_prev[:, 1]*(self.sigma_um[1]/self.mass)
            + self.as_v*vpred_error_prev
        )*self.dt

        matched_moment_prev = (
            self.u_b_prev[1:4] + self.u_ad[1:4] + self.sigma_m[1:4]
        )
        self.omega_hat = self.omega_hat + (
            -self.Jinv@np.cross(self.omega_prev, self.J@self.omega_prev)
            + self.Jinv@matched_moment_prev
            + self.as_omega*omegapred_error_prev
        )*self.dt

        vpred_error = self.v_hat - vel
        omegapred_error = self.omega_hat - omega

        exp_v = math.exp(self.as_v*self.dt)
        exp_omega = math.exp(self.as_omega*self.dt)
        phi_inv_mu_v = (
            vpred_error * self.as_v * exp_v / math.expm1(self.as_v*self.dt)
        )
        phi_inv_mu_omega = (
            omegapred_error * self.as_omega * exp_omega
            / math.expm1(self.as_omega*self.dt)
        )

        self.sigma_m[0] = self.mass*float(R[:, 2]@phi_inv_mu_v)
        self.sigma_m[1:4] = -self.J@phi_inv_mu_omega
        self.sigma_um[0] = -self.mass*float(R[:, 0]@phi_inv_mu_v)
        self.sigma_um[1] = -self.mass*float(R[:, 1]@phi_inv_mu_v)

        a_thrust = math.exp(-self.cutoff_thrust*self.dt)
        a_moment_1 = math.exp(-self.cutoff_moment_1*self.dt)
        self.lpf1[0] = a_thrust*self.lpf1[0] + (1.0-a_thrust)*self.sigma_m[0]
        self.lpf1[1:4] = (
            a_moment_1*self.lpf1[1:4]
            + (1.0-a_moment_1)*self.sigma_m[1:4]
        )

        a_moment_2 = math.exp(-self.cutoff_moment_2*self.dt)
        self.lpf2[0] = self.lpf1[0]
        self.lpf2[1:4] = (
            a_moment_2*self.lpf2[1:4]
            + (1.0-a_moment_2)*self.lpf1[1:4]
        )
        self.u_ad = -self.lpf2.copy()
        if suppress_yaw_control:
            self.u_ad[3] = 0.0

        values = np.concatenate((
            self.v_hat,
            self.omega_hat,
            self.sigma_m,
            self.sigma_um,
            self.u_ad,
        ))
        if not np.isfinite(values).all():
            raise RuntimeError("non-finite L1 adaptive state")

        self.v_prev = vel.copy()
        self.omega_prev = omega.copy()
        self.R_prev = R.copy()
        self.u_b_prev = baseline_cmd.copy()
        return self.u_ad.copy()



class SensorModel:
    """Adds configurable measurement bias and white noise before Mode29."""

    def __init__(self, cfg):
        self.enabled = bool(cfg.get("enabled", True))
        self.rng = np.random.default_rng(int(cfg.get("seed", 2904)))

        self.pos_std = np.asarray(cfg.get("position_std_m", [0, 0, 0]), dtype=float)
        self.vel_std = np.asarray(cfg.get("velocity_std_mps", [0, 0, 0]), dtype=float)
        self.att_std_deg = np.asarray(cfg.get("attitude_std_deg", [0, 0, 0]), dtype=float)
        self.gyro_std = np.asarray(cfg.get("gyro_std_rps", [0, 0, 0]), dtype=float)

        self.pos_bias = np.asarray(cfg.get("position_bias_m", [0, 0, 0]), dtype=float)
        self.vel_bias = np.asarray(cfg.get("velocity_bias_mps", [0, 0, 0]), dtype=float)
        self.att_bias_deg = np.asarray(cfg.get("attitude_bias_deg", [0, 0, 0]), dtype=float)
        self.gyro_bias = np.asarray(cfg.get("gyro_bias_rps", [0, 0, 0]), dtype=float)

        for name, value in (
            ("position_std_m", self.pos_std),
            ("velocity_std_mps", self.vel_std),
            ("attitude_std_deg", self.att_std_deg),
            ("gyro_std_rps", self.gyro_std),
            ("position_bias_m", self.pos_bias),
            ("velocity_bias_mps", self.vel_bias),
            ("attitude_bias_deg", self.att_bias_deg),
            ("gyro_bias_rps", self.gyro_bias),
        ):
            if value.shape != (3,):
                raise ValueError(f"{name} must contain exactly 3 values")

    def measure(self, true_state):
        pos, vel, R, omega = true_state
        if not self.enabled:
            return pos.copy(), vel.copy(), R.copy(), omega.copy()

        pos_m = pos + self.pos_bias + self.rng.normal(0.0, self.pos_std, 3)
        vel_m = vel + self.vel_bias + self.rng.normal(0.0, self.vel_std, 3)

        att_error_deg = self.att_bias_deg + self.rng.normal(0.0, self.att_std_deg, 3)
        R_m = R @ so3_exp(np.radians(att_error_deg))

        omega_m = omega + self.gyro_bias + self.rng.normal(0.0, self.gyro_std, 3)

        return pos_m, vel_m, R_m, omega_m


class MotorDisturbance:
    """Motor lag, command jitter, static motor mismatch and force/torque noise."""

    def __init__(self, cfg, dt):
        self.enabled = bool(cfg.get("enabled", True))
        self.rng = np.random.default_rng(int(cfg.get("seed", 2905)))
        self.dt = float(dt)

        self.tau = float(cfg.get("time_constant_s", 0.0))
        self.command_noise_std_w = float(cfg.get("command_noise_std_w", 0.0))
        self.thrust_noise_std_fraction = float(cfg.get("thrust_noise_std_fraction", 0.0))
        self.moment_noise_std_fraction = float(cfg.get("moment_noise_std_fraction", 0.0))

        self.thrust_scale = np.asarray(cfg.get("thrust_scale", [1, 1, 1, 1]), dtype=float)
        self.moment_scale = np.asarray(cfg.get("moment_scale", [1, 1, 1, 1]), dtype=float)

        if self.thrust_scale.shape != (4,) or self.moment_scale.shape != (4,):
            raise ValueError("motor thrust_scale and moment_scale must contain exactly 4 values")

        self.w_state = np.zeros(4, dtype=float)

    def step_command(self, w_cmd):
        w_cmd = np.asarray(w_cmd, dtype=float)

        if self.enabled and self.command_noise_std_w > 0.0:
            w_input = w_cmd + self.rng.normal(0.0, self.command_noise_std_w, 4)
        else:
            w_input = w_cmd.copy()

        w_input = np.clip(w_input, 0.0, 100.0)

        tau = self.tau if self.enabled else 0.0
        if tau > 0.0:
            alpha = min(1.0, self.dt / tau)
            self.w_state += alpha * (w_input - self.w_state)
        else:
            self.w_state[:] = w_input

        return self.w_state.copy()

    def actual_forces(self, w, motor):
        nominal_thrust = np.array([motor.thrust(x) for x in w], dtype=float)
        nominal_moment = np.array([motor.moment(x) for x in w], dtype=float)

        if not self.enabled:
            return nominal_thrust, nominal_moment

        thrust_noise = self.rng.normal(0.0, self.thrust_noise_std_fraction, 4)
        moment_noise = self.rng.normal(0.0, self.moment_noise_std_fraction, 4)

        thrusts = nominal_thrust * self.thrust_scale * np.maximum(0.0, 1.0 + thrust_noise)
        moments = nominal_moment * self.moment_scale * np.maximum(0.0, 1.0 + moment_noise)
        return thrusts, moments


def make_xml(cfg):
    v = cfg["vehicle"]
    sim = cfg["simulation"]
    traj = cfg["trajectory"]
    viz = cfg.get("visualization", {})
    L = float(v["L_m"])
    D = float(v["D_m"])

    # Final hover target is NED [0,0,-altitude]. Convert it to MuJoCo NWU.
    target_ned = np.array([0.0, 0.0, -float(traj["takeoff_altitude_m"])])
    origin_offset = np.array([0.0, 0.0, float(sim["controller_origin_height_m"])])
    target_mj = S @ target_ned + origin_offset
    target_marker_radius = float(viz.get("target_marker_radius_m", 0.05))

    sites = [
        ("m1", +D/2, -L/2),  # front-right CCW
        ("m2", -D/2, +L/2),  # rear-left CCW
        ("m3", +D/2, +L/2),  # front-left CW
        ("m4", -D/2, -L/2),  # rear-right CW
    ]

    motor_xml = "\n".join(
        f'''<site name="{name}" pos="{x} {y} 0" size="0.012" rgba="1 0.2 0.2 1"/>
        <geom type="cylinder" pos="{x} {y} 0" size="0.025 0.008"
              contype="0" conaffinity="0" rgba="0.1 0.1 0.1 1"/>'''
        for name, x, y in sites
    )

    return f'''
<mujoco model="softdrone_mode29">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="{float(sim["timestep_s"])}"
          gravity="0 0 -{float(v["gravity_mps2"])}"
          integrator="RK4"/>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="8 8 0.1" rgba="0.72 0.72 0.72 1"/>

    <!-- Requested final hover target marker: NED [0,0,-H], shown in red. -->
    <geom name="hover_target"
          type="sphere"
          pos="{target_mj[0]} {target_mj[1]} {target_mj[2]}"
          size="{target_marker_radius}"
          rgba="1 0 0 1"
          contype="0"
          conaffinity="0"/>

    <body name="softdrone">
      <freejoint/>
      <inertial pos="0 0 0"
                mass="{float(v["mass_kg"])}"
                diaginertia="{float(v["jxx_kgm2"])} {float(v["jyy_kgm2"])} {float(v["jzz_kgm2"])}"/>

      <geom name="body_collision" type="box" size="0.07 0.055 0.025"
            rgba="0.15 0.35 0.9 1"/>

      <geom type="capsule" fromto="{-D/2} {-L/2} 0 {D/2} {L/2} 0"
            size="0.007" contype="0" conaffinity="0" rgba="0.12 0.12 0.12 1"/>
      <geom type="capsule" fromto="{-D/2} {L/2} 0 {D/2} {-L/2} 0"
            size="0.007" contype="0" conaffinity="0" rgba="0.12 0.12 0.12 1"/>

      {motor_xml}
    </body>
  </worldbody>
</mujoco>
'''


def set_initial_state(model, data, cfg):
    init = cfg["initial_state"]
    sim = cfg["simulation"]

    p_ned = np.array(init["position_ned_m"], dtype=float)
    v_ned = np.array(init["velocity_ned_mps"], dtype=float)
    rpy = np.radians(np.array(init["attitude_rpy_deg"], dtype=float))
    omega_frd = np.array(init["body_rate_frd_rps"], dtype=float)

    origin_offset = np.array([0.0, 0.0, float(sim["controller_origin_height_m"])])

    data.qpos[:3] = S@p_ned + origin_offset
    data.qvel[:3] = S@v_ned

    R_ned_frd = rpy_to_R_ned_frd(*rpy)
    R_mj = S @ R_ned_frd @ S
    data.qpos[3:7] = mat_to_quat_wxyz(R_mj)
    data.qvel[3:6] = S@omega_frd

    mujoco.mj_forward(model, data)


def get_state(data, body_id, origin_height):
    origin_offset = np.array([0.0, 0.0, origin_height])
    p_ned = S@(data.qpos[:3] - origin_offset)
    v_ned = S@data.qvel[:3]

    R_mj = data.xmat[body_id].reshape(3,3).copy()
    R_ned_frd = S @ R_mj @ S
    omega_frd = S@data.qvel[3:6]

    return p_ned, v_ned, R_ned_frd, omega_frd, R_mj


def clamp_frd_yaw_rate(data, max_abs_rate_rps):
    """Hard-limit the FRD yaw-rate state and report the pre-limit value.

    This is a simulation safety guard, not actuator-generated yaw authority.
    It follows the same FRD rate convention used by get_state().
    """
    omega_frd = S @ np.asarray(data.qvel[3:6], dtype=float)
    raw_yaw_rate = float(omega_frd[2])
    limited_yaw_rate = float(np.clip(
        raw_yaw_rate,
        -float(max_abs_rate_rps),
        float(max_abs_rate_rps),
    ))
    active = not math.isclose(raw_yaw_rate, limited_yaw_rate, rel_tol=0.0, abs_tol=1.0e-12)
    if active:
        omega_frd[2] = limited_yaw_rate
        data.qvel[3:6] = S @ omega_frd
    return raw_yaw_rate, limited_yaw_rate, active


def apply_rotors(model, data, body_id, site_ids, thrusts, moments, R_mj):
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0

    # FRD yaw moment signs in firmware: +M1 +M2 -M3 -M4.
    # FLU +Z is opposite FRD +Z, so local MuJoCo reaction signs are flipped.
    yaw_sign_flu = np.array([-1.0, -1.0, +1.0, +1.0])

    for i, site_id in enumerate(site_ids):
        force_world = R_mj @ np.array([0.0, 0.0, thrusts[i]])
        torque_world = R_mj @ np.array([0.0, 0.0, yaw_sign_flu[i]*moments[i]])
        point_world = data.site_xpos[site_id].copy()

        mujoco.mj_applyFT(
            model, data,
            force_world, torque_world, point_world,
            body_id, data.qfrc_applied
        )

    return thrusts, moments


def analytic_wrench_frd(thrusts, moments, L, D):
    f1,f2,f3,f4 = thrusts
    m1,m2,m3,m4 = moments
    return np.array([
        f1+f2+f3+f4,
        (L/2.0)*(-f1+f2+f3-f4),
        (D/2.0)*( f1-f2+f3-f4),
        m1+m2-m3-m4,
    ])


class MotorFaultInjector:
    """Runtime motor thrust-effectiveness loss; separate from the detector."""

    def __init__(self, cfg, takeoff_time, settle_time):
        self.enabled = bool(cfg.get("enabled", False))
        self.motor_id = int(cfg.get("motor_id", 1))
        self.loss_percent = float(cfg.get("loss_percent", 0.0))
        self.start_time = float(cfg.get("start_time_s", takeoff_time + settle_time))
        self.duration = float(cfg.get("duration_s", 0.0))
        self.request_yaw_free = bool(cfg.get("request_yaw_free", False))
        self.blind_fdi = bool(cfg.get("blind_fdi", True))
        self.gate_time = float(takeoff_time + settle_time)

        if self.motor_id not in (1, 2, 3, 4):
            raise ValueError("fault_injection.motor_id must be 1..4")
        if not 0.0 <= self.loss_percent <= 100.0:
            raise ValueError("fault_injection.loss_percent must be in [0, 100]")
        if self.start_time < self.gate_time:
            raise ValueError(
                "fault injection must start after takeoff_time_s + settle_time_s "
                f"({self.gate_time:.3f} s)"
            )

    def is_active(self, t):
        if not self.enabled or self.loss_percent <= 0.0 or t < self.start_time:
            return False
        return self.duration <= 0.0 or t < self.start_time + self.duration

    def apply(self, w_nominal, motor, t):
        """Apply loss in thrust space, then invert the same 6S motor model."""
        w_plant = np.asarray(w_nominal, dtype=float).copy()
        active = self.is_active(t)
        if active:
            idx = self.motor_id - 1
            effectiveness = 1.0 - self.loss_percent/100.0
            nominal_thrust = motor.thrust(w_plant[idx])
            w_plant[idx] = motor.w_from_thrust(effectiveness*nominal_thrust)
        else:
            effectiveness = 1.0
        return np.clip(w_plant, 0.0, 100.0), active, effectiveness


class BlindMotorFaultDetector:
    """Moment-residual FDI port of the advanced Mode29 branch.

    The detector intentionally has no reference to MotorFaultInjector. It uses
    measured body rates, known inertia, the previous desired moment, and the
    previous nominal actuator command.
    """

    def __init__(self, cfg, vehicle, motor, dt, gate_time):
        self.enabled = bool(cfg.get("enabled", True))
        self.J = np.diag([
            float(vehicle["jxx_kgm2"]),
            float(vehicle["jyy_kgm2"]),
            float(vehicle["jzz_kgm2"]),
        ])
        self.L = float(vehicle["L_m"])
        self.D = float(vehicle["D_m"])
        self.motor = motor
        self.dt = float(dt)
        self.gate_time = float(gate_time)

        self.confirm_samples = max(1, int(math.ceil(float(cfg.get("confirm_time_s", 0.060))/self.dt)))
        self.recover_samples = max(1, int(math.ceil(float(cfg.get("recovery_time_s", 0.100))/self.dt)))
        self.min_loss_fraction = float(cfg.get("min_loss_percent", 60.0))/100.0
        self.release_loss_fraction = float(cfg.get("release_loss_percent", 35.0))/100.0
        self.max_residual_ratio = float(cfg.get("max_residual_ratio", 0.35))
        self.min_rp_moment = float(cfg.get("min_roll_pitch_moment_nm", 0.20))
        self.sigma_alpha = float(cfg.get("sigma_filter_alpha", 0.05))
        self.baseline_alpha_fast = float(cfg.get("baseline_alpha_fast", 0.02))
        self.baseline_alpha_slow = float(cfg.get("baseline_alpha_slow", 0.0005))
        self.loss_update_gain = float(cfg.get("confirmed_loss_update_gain", 0.05))
        self.confirmed_fit_limit = float(cfg.get("confirmed_fit_limit", 0.55))
        self.post_recovery_yaw_rate_damping = float(
            cfg.get("post_recovery_yaw_rate_damping_nm_per_rps", 0.05)
        )
        self.post_recovery_max_yaw_moment = float(
            cfg.get("post_recovery_max_yaw_moment_nm", 0.15)
        )
        self.fault_yaw_rate_damping = float(
            cfg.get("fault_yaw_rate_damping_nm_per_rps", 0.05)
        )
        self.fault_max_yaw_moment = float(
            cfg.get("fault_max_yaw_moment_nm", 0.15)
        )
        self.yaw_rate_safety_limit_enabled = bool(
            cfg.get("yaw_rate_safety_limit_enabled", False)
        )
        self.yaw_rate_safety_limit = float(
            cfg.get("yaw_rate_safety_limit_rps", 20.0)
        )
        if self.yaw_rate_safety_limit_enabled and (
            not np.isfinite(self.yaw_rate_safety_limit)
            or self.yaw_rate_safety_limit <= 0.0
        ):
            raise ValueError("fault_detection.yaw_rate_safety_limit_rps must be positive")

        self.confirmed = False
        self.yaw_free_latched = False
        self.detected_id = 0
        self.candidate_id = 0
        self.confirm_count = 0
        self.recovery_count = 0
        self.loss_estimate_percent = 0.0
        self.residual_ratio = 1.0
        self.identity_fit_ratio = 1.0  # remembered *detection* fit, not innovation fit
        self.sigma_filtered = np.zeros(3)
        self.signature_filtered = np.zeros((4, 3))
        self.sigma_baseline = np.zeros(3)
        self.sigma_observed = np.zeros(3)
        self.sigma_valid = False
        self.prev_omega = None
        self.prev_cmd = None
        self.prev_w_nominal = None
        self.confirmed_at_s = None
        self.recovered_at_s = None
        self.cooldown_seconds = float(cfg.get("recovery_cooldown_s", 2.0))
        self.cooldown_until_s = 0.0
        self.loss_instant_percent = 0.0
        self.loss_rate_pp_s = 0.0
        self.prev_estimate_for_rate = 0.0
        if self.cooldown_seconds < 0.0:
            raise ValueError("recovery_cooldown_s must be >=0")

    @property
    def state(self):
        if self.confirmed:
            return 2
        return 1 if self.candidate_id else 0

    def _loss_signature(self, motor_index, w):
        f_nom = self.motor.thrust(float(np.clip(w, 0.0, 100.0)))
        m_nom = self.motor.moment(float(np.clip(w, 0.0, 100.0)))
        signatures = np.array([
            [+0.5*self.L*f_nom, -0.5*self.D*f_nom, -m_nom],
            [-0.5*self.L*f_nom, +0.5*self.D*f_nom, -m_nom],
            [-0.5*self.L*f_nom, -0.5*self.D*f_nom, +m_nom],
            [+0.5*self.L*f_nom, +0.5*self.D*f_nom, +m_nom],
        ], dtype=float)
        return signatures[motor_index]

    def record_control(self, cmd, w_nominal):
        self.prev_cmd = np.asarray(cmd, dtype=float).copy()
        self.prev_w_nominal = np.asarray(w_nominal, dtype=float).copy()

    def _reset_candidate(self):
        self.candidate_id = 0
        self.confirm_count = 0
        self.recovery_count = 0

    def begin_cooldown(self, t):
        """Re-anchor detector when the synthetic second loss disengages.

        During cooldown no single-fault inference is valid because the
        two-motor residual and topology transients contaminate the history.
        """
        self.cooldown_until_s = max(self.cooldown_until_s,
                                    float(t) + self.cooldown_seconds)
        self.confirmed = False
        self.detected_id = 0
        self._reset_candidate()
        self.loss_estimate_percent = 0.0
        self.residual_ratio = 1.0
        self.identity_fit_ratio = 1.0
        self.prev_cmd = None
        self.prev_w_nominal = None
        self.sigma_baseline = self.sigma_filtered.copy()
        self.loss_instant_percent = 0.0
        self.loss_rate_pp_s = 0.0
        self.prev_estimate_for_rate = 0.0

    def update(self, t, measured_omega, freeze_confirmed_estimate=False):
        omega = np.asarray(measured_omega, dtype=float)
        if self.prev_omega is None:
            self.prev_omega = omega.copy()
            return

        omega_dot = (omega - self.prev_omega)/self.dt
        self.prev_omega = omega.copy()

        if self.prev_cmd is None or self.prev_w_nominal is None:
            return

        observed_moment = self.J@omega_dot + np.cross(omega, self.J@omega)

        # Predict the moment from the previous *nominal actuator state*, not
        # directly from the controller request. This retains the motor lag in
        # the observer model and prevents a reallocation step from looking like
        # an additional actuator loss. The injector state is never consulted.
        nominal_thrust = np.array(
            [self.motor.thrust(w) for w in self.prev_w_nominal],
            dtype=float,
        )
        nominal_reaction = np.array(
            [self.motor.moment(w) for w in self.prev_w_nominal],
            dtype=float,
        )
        predicted_moment = analytic_wrench_frd(
            nominal_thrust,
            nominal_reaction,
            self.L,
            self.D,
        )[1:4]

        if self.confirmed and self.detected_id in (1, 2, 3, 4):
            idx = self.detected_id-1
            predicted_moment = predicted_moment + (
                self.loss_estimate_percent/100.0
            )*self._loss_signature(idx, self.prev_w_nominal[idx])

        sigma_now = observed_moment - predicted_moment
        if not np.isfinite(sigma_now).all():
            self._reset_candidate()
            return

        if not self.sigma_valid:
            self.sigma_filtered = sigma_now.copy()
            self.sigma_baseline = sigma_now.copy()
            self.sigma_valid = True
            return

        # Filter signatures with the same kernel as the measured disturbance:
        # fitting historical filtered residual against current motor thrust
        # systematically underestimates loss during fast fault transients.
        signatures_now = np.array([
            self._loss_signature(i, self.prev_w_nominal[i]) for i in range(4)
        ], dtype=float)
        self.signature_filtered += self.sigma_alpha * (
            signatures_now - self.signature_filtered
        )
        self.sigma_filtered += self.sigma_alpha*(sigma_now - self.sigma_filtered)
        detector_gate = self.enabled and t >= self.gate_time

        if t < self.cooldown_until_s:
            self.sigma_baseline = self.sigma_filtered.copy()
            self._reset_candidate()
            self.loss_estimate_percent = 0.0
            self.residual_ratio = 1.0
            return

        if not detector_gate:
            self.sigma_baseline += self.baseline_alpha_fast*(self.sigma_filtered - self.sigma_baseline)
            self._reset_candidate()
            self.loss_estimate_percent = 0.0
            self.residual_ratio = 1.0
            return

        observed = self.sigma_filtered - self.sigma_baseline
        self.sigma_observed = observed.copy()
        observed_rp = float(np.linalg.norm(observed[:2]))
        observed_norm = float(np.linalg.norm(observed))

        if self.confirmed:
            idx = self.detected_id - 1
            if idx not in range(4):
                self.confirmed = False
                self.detected_id = 0
                return
            # A hard state clamp removes angular momentum without a modeled
            # actuator moment. Freeze an already-confirmed estimate while the
            # clamp is active so that this artificial intervention cannot be
            # mistaken for motor recovery or a different loss magnitude.
            if freeze_confirmed_estimate:
                self.recovery_count = 0
                return
            signature = self.signature_filtered[idx]
            signature_norm_sq = float(signature@signature)
            if signature_norm_sq < 1.0e-9 or not np.isfinite(observed_norm):
                self.recovery_count = 0
                return

            signed_fraction = float(observed@signature/signature_norm_sq)
            self.loss_instant_percent = 100.0*signed_fraction
            fit_error = observed - signature*signed_fraction
            self.residual_ratio = float(np.linalg.norm(fit_error)/max(observed_norm, 1.0e-4))
            if np.isfinite(signed_fraction) and (
                observed_norm < 1.0e-4 or self.residual_ratio <= self.confirmed_fit_limit
            ):
                loss_fraction = np.clip(self.loss_estimate_percent/100.0, 0.0, 1.0)
                loss_fraction = np.clip(
                    loss_fraction + self.loss_update_gain*signed_fraction,
                    0.0,
                    1.0,
                )
                self.loss_estimate_percent = 100.0*loss_fraction

            recovered = (
                self.loss_estimate_percent <= 100.0*self.release_loss_fraction
                and (observed_norm < self.min_rp_moment or self.residual_ratio <= self.confirmed_fit_limit)
            )
            self.recovery_count = self.recovery_count + 1 if recovered else 0
            if self.recovery_count >= self.recover_samples:
                self.recovered_at_s = float(t)
                self.confirmed = False
                self.detected_id = 0
                self._reset_candidate()
                self.loss_estimate_percent = 0.0
                self.residual_ratio = 1.0
                self.sigma_baseline = self.sigma_filtered.copy()
                self.cooldown_until_s = float(t) + self.cooldown_seconds
                self.identity_fit_ratio = 1.0
                self.prev_omega = omega.copy()
                self.prev_cmd = None
                self.prev_w_nominal = None
            return

        if not np.isfinite(observed_norm) or observed_rp < self.min_rp_moment:
            self.sigma_baseline += self.baseline_alpha_slow*(self.sigma_filtered - self.sigma_baseline)
            self._reset_candidate()
            self.loss_estimate_percent = 0.0
            self.residual_ratio = 1.0
            return

        best_motor = 0
        best_loss = 0.0
        best_ratio = float("inf")
        for idx in range(4):
            signature = self.signature_filtered[idx]
            signature_norm_sq = float(signature@signature)
            if signature_norm_sq < 1.0e-9:
                continue
            loss_fraction = float(np.clip(observed@signature/signature_norm_sq, 0.0, 1.0))
            fit_error = observed - signature*loss_fraction
            ratio = float(np.linalg.norm(fit_error)/max(observed_norm, 1.0e-4))
            if np.isfinite(ratio) and ratio < best_ratio:
                best_motor, best_loss, best_ratio = idx+1, loss_fraction, ratio

        self.loss_instant_percent = 100.0*best_loss
        self.loss_rate_pp_s = abs(100.0*best_loss - self.prev_estimate_for_rate)/self.dt
        self.prev_estimate_for_rate = 100.0*best_loss
        self.loss_estimate_percent = 100.0*best_loss
        self.residual_ratio = best_ratio
        severe = (
            best_motor != 0
            and best_loss >= self.min_loss_fraction
            and best_ratio <= self.max_residual_ratio
        )
        if not severe:
            self.sigma_baseline += self.baseline_alpha_slow*(self.sigma_filtered - self.sigma_baseline)
            self._reset_candidate()
            return

        if self.candidate_id == best_motor:
            self.confirm_count += 1
        else:
            self.candidate_id = best_motor
            self.confirm_count = 1

        if self.confirm_count >= self.confirm_samples:
            self.confirmed = True
            self.yaw_free_latched = True
            self.detected_id = best_motor
            self.identity_fit_ratio = best_ratio
            self.recovery_count = 0
            # The residual filter still contains the pre-reconfiguration fault
            # step. Reset only that transient memory; otherwise it is counted
            # again as post-allocation mismatch and drives the loss estimate to
            # 100% before the compensated plant has responded.
            self.sigma_filtered = self.sigma_baseline.copy()
            self.sigma_observed = np.zeros(3)
            if self.confirmed_at_s is None:
                self.confirmed_at_s = float(t)


class GainScheduler:
    """Editable 0,10,...,100% gain/tilt anchors; linear interpolate any severity."""

    def __init__(self, cfg, base_gains, safety):
        mode = cfg.get("mode", "disabled")
        self.mode = {0: "disabled", 1: "oracle", 2: "automatic"}.get(mode, str(mode).lower())
        if self.mode not in ("disabled", "oracle", "automatic"):
            raise ValueError("gain_schedule.mode must be disabled, oracle, or automatic")

        self.base = {
            "kp": np.array([base_gains["kpx"], base_gains["kpy"], base_gains["kpz"]], dtype=float),
            "kv": np.array([base_gains["kvx"], base_gains["kvy"], base_gains["kvz"]], dtype=float),
            "kr": np.array([base_gains["krx"], base_gains["kry"], base_gains["krz"]], dtype=float),
            "ko": np.array([base_gains["kox"], base_gains["koy"], base_gains["koz"]], dtype=float),
            "max_tilt_deg": float(safety["max_tilt_deg"]),
        }
        self.anchors = {0: self._copy(self.base)}
        # The user can independently tune 0% just like 10%..100%.
        # If absent, use historical healthy baseline [gains]/[safety].
        zero = cfg.get("loss_0", {})
        self.anchors[0] = {
            "kp": np.asarray(zero.get("kp", self.base["kp"]), dtype=float),
            "kv": np.asarray(zero.get("kv", self.base["kv"]), dtype=float),
            "kr": np.asarray(zero.get("kr", self.base["kr"]), dtype=float),
            "ko": np.asarray(zero.get("ko", self.base["ko"]), dtype=float),
            "max_tilt_deg": float(zero.get("max_tilt_deg", self.base["max_tilt_deg"])),
        }
        for key in ("kp", "kv", "kr", "ko"):
            if self.anchors[0][key].shape != (3,) or not np.isfinite(self.anchors[0][key]).all():
                raise ValueError(f"loss_0.{key} must contain 3 finite gains")
        if not 5.0 <= self.anchors[0]["max_tilt_deg"] <= 90.0:
            raise ValueError("loss_0.max_tilt_deg must be [5,90]; control clips to 60")
        # Existing 50..100 anchors are preserved. Missing intermediate anchors
        # inherit the 0..50 straight line until each 10% point is tuned.
        fifty = cfg.get("loss_50", {})
        default_50 = {
            "kp": np.asarray(fifty.get("kp", self.base["kp"]), dtype=float),
            "kv": np.asarray(fifty.get("kv", self.base["kv"]), dtype=float),
            "kr": np.asarray(fifty.get("kr", self.base["kr"]), dtype=float),
            "ko": np.asarray(fifty.get("ko", self.base["ko"]), dtype=float),
            "max_tilt_deg": float(fifty.get("max_tilt_deg", self.base["max_tilt_deg"])),
        }
        for loss in range(10, 101, 10):
            if loss < 50:
                defaults = self._interpolate(self.anchors[0], default_50, loss / 50.0)
            else:
                defaults = self.base
            anchor = cfg.get(f"loss_{loss}", {})
            self.anchors[loss] = {
                "kp": np.asarray(anchor.get("kp", defaults["kp"]), dtype=float),
                "kv": np.asarray(anchor.get("kv", defaults["kv"]), dtype=float),
                "kr": np.asarray(anchor.get("kr", defaults["kr"]), dtype=float),
                "ko": np.asarray(anchor.get("ko", defaults["ko"]), dtype=float),
                "max_tilt_deg": float(anchor.get("max_tilt_deg", defaults["max_tilt_deg"])),
            }
            for key in ("kp", "kv", "kr", "ko"):
                if self.anchors[loss][key].shape != (3,) or not np.isfinite(self.anchors[loss][key]).all():
                    raise ValueError(f"loss_{loss}.{key} must contain 3 finite gains")
            if not (5.0 <= self.anchors[loss]["max_tilt_deg"] <= 90.0):
                raise ValueError(f"loss_{loss}.max_tilt_deg must be in [5,90] degrees; the controller clips to 60")
        self.alpha = float(cfg.get("loss_filter_alpha", 0.08))
        self.max_step = float(cfg.get("max_loss_step_percent", 2.5))
        self.confidence_gate = float(cfg.get("confidence_gate", 0.20))
        self.raw_loss = 0.0
        self.scheduled_loss = 0.0
        self.confidence = 0.0
        self.active = self._copy(self.base)

    @staticmethod
    def _copy(tuning):
        return {
            "kp": np.asarray(tuning["kp"], dtype=float).copy(),
            "kv": np.asarray(tuning["kv"], dtype=float).copy(),
            "kr": np.asarray(tuning["kr"], dtype=float).copy(),
            "ko": np.asarray(tuning["ko"], dtype=float).copy(),
            "max_tilt_deg": float(tuning["max_tilt_deg"]),
        }

    @staticmethod
    def _interpolate(a, b, t):
        t = float(np.clip(t, 0.0, 1.0))
        return {
            "kp": a["kp"] + (b["kp"]-a["kp"])*t,
            "kv": a["kv"] + (b["kv"]-a["kv"])*t,
            "kr": a["kr"] + (b["kr"]-a["kr"])*t,
            "ko": a["ko"] + (b["ko"]-a["ko"])*t,
            "max_tilt_deg": float(np.clip(
                a["max_tilt_deg"] + (b["max_tilt_deg"]-a["max_tilt_deg"])*t,
                5.0,
                60.0,
            )),
        }

    def at_loss(self, loss_percent):
        loss = float(np.clip(loss_percent, 0.0, 100.0))
        if loss >= 100.0:
            return self._copy(self.anchors[100])
        lower = min(90, int(math.floor(loss / 10.0)) * 10)
        return self._interpolate(self.anchors[lower], self.anchors[lower + 10],
                                 (loss - lower) / 10.0)

    def update(self, injector_active, injected_loss, detector):
        raw_loss = 0.0
        confidence = 0.0
        if self.mode == "oracle" and injector_active:
            raw_loss = float(np.clip(injected_loss, 0.0, 100.0))
            confidence = 1.0
        elif self.mode == "automatic":
            fit_limit = 0.55
            fit_confidence = float(np.clip(1.0-detector.residual_ratio/fit_limit, 0.0, 1.0))
            valid = (
                np.isfinite(detector.loss_estimate_percent)
                and np.isfinite(detector.residual_ratio)
                and detector.loss_estimate_percent > 0.0
                and (detector.confirmed or detector.residual_ratio <= fit_limit)
            )
            if valid:
                raw_loss = float(np.clip(detector.loss_estimate_percent, 0.0, 100.0))
                confidence = 1.0 if detector.confirmed else fit_confidence

        self.raw_loss = raw_loss
        self.confidence = confidence
        if self.mode == "disabled":
            self.scheduled_loss = 0.0
            self.active = self.at_loss(0.0)
            return self.active

        target = raw_loss if confidence >= self.confidence_gate else 0.0
        filtered = self.scheduled_loss + self.alpha*(target-self.scheduled_loss)
        delta = float(np.clip(filtered-self.scheduled_loss, -self.max_step, self.max_step))
        self.scheduled_loss = float(np.clip(self.scheduled_loss+delta, 0.0, 100.0))
        self.active = self.at_loss(self.scheduled_loss)
        return self.active


def run(cfg):
    v = cfg["vehicle"]
    sim = cfg["simulation"]
    traj = cfg["trajectory"]

    birth = np.array(cfg["initial_state"]["position_ned_m"], dtype=float)
    if bool(cfg["controller"]["enforce_firmware_entry_guard"]):
        xy = math.hypot(birth[0], birth[1])
        if xy > float(cfg["controller"]["entry_max_xy_m"]) or abs(birth[2]) > float(cfg["controller"]["entry_max_z_m"]):
            raise RuntimeError(f"entry guard rejected birth point: xy={xy:.3f}, z={birth[2]:.3f}")

    model = mujoco.MjModel.from_xml_string(make_xml(cfg))
    data = mujoco.MjData(model)
    set_initial_state(model, data, cfg)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "softdrone")
    site_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        for name in ("m1","m2","m3","m4")
    ]

    dt = float(sim["timestep_s"])
    duration = float(sim["duration_s"])
    origin_height = float(sim["controller_origin_height_m"])

    motor = MotorModel(cfg["motor_model"])
    mixer = Mixer(v["L_m"], v["D_m"], motor)
    controller = GeometricController(v, cfg["gains"], cfg["safety"])
    l1 = L1AdaptiveAugmentation(
        v,
        cfg["controller"],
        cfg.get("l1_adaptive", {}),
        dt,
    )
    sensor = SensorModel(cfg.get("sensor_noise", {}))
    motor_dist = MotorDisturbance(cfg.get("motor_disturbance", {}), dt)
    fault = MotorFaultInjector(
        cfg.get("fault_injection", {}),
        float(traj["takeoff_time_s"]),
        float(traj["settle_time_s"]),
    )
    detector = BlindMotorFaultDetector(
        cfg.get("fault_detection", {}),
        v,
        motor,
        dt,
        float(traj["takeoff_time_s"])+float(traj["settle_time_s"]),
    )
    scheduler = GainScheduler(cfg.get("gain_schedule", {}), cfg["gains"], cfg["safety"])
    pair = OppositePairExperiment(
        cfg.get("opposite_pair", {}), v, motor, float(cfg["safety"]["max_tilt_deg"])
    )
    yaw_envelope = YawRateEnvelope(cfg.get("yaw_rate_schedule", {}), cfg.get("gain_schedule", {}))

    log_path = Path(sim["log_csv"])
    if not log_path.is_absolute():
        log_path = Path.cwd()/log_path
    log_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "t","target_x","target_y","target_z",
        "x","y","z","vx","vy","vz",
        "meas_x","meas_y","meas_z","meas_vx","meas_vy","meas_vz",
        "target_roll_deg","target_pitch_deg","target_yaw_deg",
        "roll_deg","pitch_deg","yaw_deg",
        "meas_roll_deg","meas_pitch_deg","meas_yaw_deg",
        "yaw_rate_rps","yaw_rate_prelimit_rps","yaw_rate_limiter_active",
        "target_tilt_deg","actual_tilt_deg",
        "thrust_cmd_N","mx_cmd_Nm","my_cmd_Nm","mz_cmd_Nm",
        "l1_enabled","l1_transition_hold",
        "l1_sigma_thrust_N","l1_sigma_mx_Nm","l1_sigma_my_Nm","l1_sigma_mz_Nm",
        "l1_sigma_um_x_N","l1_sigma_um_y_N",
        "l1_uad_thrust_N","l1_uad_mx_Nm","l1_uad_my_Nm","l1_uad_mz_Nm",
        "total_thrust_cmd_N","total_mx_cmd_Nm","total_my_cmd_Nm","total_mz_cmd_Nm",
        "l1_vhat_x","l1_vhat_y","l1_vhat_z",
        "l1_omegahat_x","l1_omegahat_y","l1_omegahat_z",
        "wcmd1","wcmd2","wcmd3","wcmd4",
        "wnom1","wnom2","wnom3","wnom4",
        "w1","w2","w3","w4","pwm1_us","pwm2_us","pwm3_us","pwm4_us",
        "f1_N","f2_N","f3_N","f4_N",
        "F_actual_N","Mx_actual_Nm","My_actual_Nm","Mz_actual_Nm",
        "tilt_limiter","allocator_mode","yaw_free_active","candidate_protection_active",
        "fault_active","fault_motor","fault_loss_pct","fault_effectiveness",
        "pair_enabled","pair_source","pair_active","pair_inhibited","pair_failed_motor",
        "pair_opposite_motor","pair_loss_estimate_pct","pair_static_margin",
        "pair_estimate_bias_pp","pair_xy_error_m","pair_z_error_m",
        "pair_disengaged_time_s","pair_guard_reason",
        "pair_retry_pending","pair_transient_reason","pair_severity_ready",
        "pair_yaw_cap_deg_s","pair_yaw_target_deg_s","pair_yaw_hard_abort_rps",
        "pair_yaw_brake_cmd_nm","pair_yaw_allocated_nm",
        "pair_yaw_min_nm","pair_yaw_max_nm","pair_yaw_saturated",
        "fdi_state","fdi_motor","fdi_candidate","fdi_confirm_count","fdi_recovery_count",
        "fdi_loss_estimate_pct","fdi_residual_ratio","fdi_sigma_mx","fdi_sigma_my","fdi_sigma_mz",
        "fdi_loss_instant_pct","fdi_loss_rate_pp_s","fdi_cooldown_remaining_s",
        "schedule_mode","schedule_raw_loss_pct","schedule_loss_pct","schedule_confidence",
        "allocator_effectiveness_loss_pct","schedule_allocator_mismatch_pp",
        "active_kpx","active_kpy","active_kpz","active_kvx","active_kvy","active_kvz",
        "active_krx","active_kry","active_krz","active_kox","active_koy","active_koz",
        "active_max_tilt_deg"
    ]

    print("="*72)
    print("SOFTDRONE MODE29 / MUJOCO")
    print("="*72)
    print(f"mass             : {float(v['mass_kg']):.4f} kg")
    print(f"J                : {float(v['jxx_kgm2']):.5f}, {float(v['jyy_kgm2']):.5f}, {float(v['jzz_kgm2']):.5f}")
    print(f"L / D            : {float(v['L_m']):.3f} / {float(v['D_m']):.3f} m")
    birth_rpy = np.asarray(cfg["initial_state"]["attitude_rpy_deg"], dtype=float)
    print(f"birth NED        : {birth.tolist()}")
    print(f"birth RPY deg    : {birth_rpy.tolist()}")
    print(f"hover target NED : [0, 0, {-float(traj['takeoff_altitude_m']):.3f}]  (red sphere)")
    print(f"sensor noise     : {'ON' if sensor.enabled else 'OFF'}")
    print(f"motor disturbance: {'ON' if motor_dist.enabled else 'OFF'}")
    if motor_dist.enabled:
        print(f"motor scales     : {motor_dist.thrust_scale.tolist()}")
        print(f"motor tau        : {motor_dist.tau:.4f} s")
    print(f"fault injection  : {'ON' if fault.enabled else 'OFF'}")
    if fault.enabled:
        fault_end = "until end" if fault.duration <= 0.0 else f"{fault.start_time+fault.duration:.2f}s"
        print(
            f"fault scenario   : M{fault.motor_id}, loss={fault.loss_percent:.1f}%, "
            f"{fault.start_time:.2f}s..{fault_end}, blind={fault.blind_fdi}"
        )
    print(f"blind FDI        : {'ON' if detector.enabled else 'OFF'}")
    print(f"opposite pair    : {('ENABLED source='+pair.source) if pair.enabled else 'OFF / original mode'}")
    print(f"yaw envelope     : {('10% anchors / motor torque feedback' if yaw_envelope.enabled else 'OFF')}")
    if yaw_envelope.enabled:
        print("yaw rate nodes   :", list(zip(yaw_envelope.nodes.astype(int).tolist(), yaw_envelope.limits.tolist())))

    if detector.yaw_rate_safety_limit_enabled:
        print(f"yaw-rate safety  : +/-{detector.yaw_rate_safety_limit:.3f} rad/s (yaw-free only)")
    else:
        print("yaw-rate safety  : OFF")
    print(f"gain schedule    : {scheduler.mode}")
    print(
        f"L1 adaptive      : {'ON' if l1.enabled else 'OFF'} "
        f"(As_v={l1.as_v:g}, As_w={l1.as_omega:g}, "
        f"cutoffs={l1.cutoff_thrust:g}/{l1.cutoff_moment_1:g}/{l1.cutoff_moment_2:g} rad/s, "
        f"switch hold={l1.topology_transition_hold:g}s)"
    )
    print("="*72)

    ctx = mujoco.viewer.launch_passive(model, data) if bool(sim["viewer"]) else None
    viewer = ctx.__enter__() if ctx is not None else None

    wall0 = time.time()
    next_print = 0.0
    max_err = 0.0
    max_tilt = 0.0
    aborted = False
    reason = ""
    previous_protection_key = (False, 0, False)
    l1_hold_until = 0.0
    yaw_rate_prelimit_rps = float((S @ data.qvel[3:6])[2])
    yaw_rate_limiter_active = False
    yaw_feedback_request = 0.0
    yaw_allocation_diag = {}

    try:
        with log_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            while data.time < duration:
                if viewer is not None and not viewer.is_running():
                    break

                # True plant state from MuJoCo.
                pos, vel, R, Omega, R_mj = get_state(data, body_id, origin_height)

                # The controller does NOT receive the perfect plant state anymore.
                meas_pos, meas_vel, meas_R, meas_Omega = sensor.measure((pos, vel, R, Omega))

                # FDI is blind to the injector: it only consumes measured body
                # rates plus the previous desired moment/nominal motor command.
                if pair.active and pair.source == "oracle":
                    # Freeze blind detector in the *oracle* comparison to avoid
                    # interpreting the deliberate second fault as one-motor FDI.
                    # Preserve a fresh gyro baseline for a later disengagement.
                    detector.prev_omega = np.asarray(meas_Omega, dtype=float).copy()
                else:
                    detector.update(
                        data.time,
                        meas_Omega,
                        freeze_confirmed_estimate=yaw_rate_limiter_active or pair.active,
                    )
                fault_active_now = fault.is_active(data.time)
                was_pair_active = pair.active
                pair.update(
                    data.time, meas_pos, meas_Omega,
                    float(traj["takeoff_altitude_m"]), fault_active_now,
                    fault, detector, yaw_envelope=yaw_envelope,
                )
                if was_pair_active and not pair.active:
                    detector.begin_cooldown(data.time)
                    l1_hold_until = data.time + l1.topology_transition_hold
                candidate_protection_active = (
                    not detector.confirmed
                    and detector.candidate_id in (1, 2, 3, 4)
                    and detector.loss_estimate_percent >= 100.0*detector.min_loss_fraction
                )
                protection_active = detector.confirmed or candidate_protection_active
                protection_motor = (
                    detector.detected_id
                    if detector.confirmed
                    else detector.candidate_id
                )
                protection_loss = detector.loss_estimate_percent
                protection_key = (
                    bool(protection_active),
                    int(protection_motor) if protection_active else 0,
                    bool(pair.active),
                )
                fdi_topology_changed = protection_key != previous_protection_key
                previous_protection_key = protection_key
                if fdi_topology_changed:
                    l1_hold_until = data.time + l1.topology_transition_hold
                active_tuning = scheduler.update(
                    fault_active_now,
                    fault.loss_percent,
                    detector,
                )
                controller.set_tuning(active_tuning)

                yaw_feedback_request = 0.0
                yaw_allocation_diag = {}
                if pair.active and yaw_envelope.enabled:
                    # Under the current reaction-moment convention,
                    # reducing CCW pair M1/M2 yields negative body yaw,
                    # reducing CW pair M3/M4 yields positive body yaw.
                    spin_direction = (
                        -1 if pair.failed_motor_id in (1, 2) else +1
                    )
                    yaw_feedback_request = yaw_envelope.requested_moment(
                        pair.estimated_loss_percent, meas_Omega[2], spin_direction
                    )

                yaw_free_active = detector.yaw_free_latched or candidate_protection_active or (
                    fault_active_now and fault.request_yaw_free and not fault.blind_fdi
                ) or pair.active or (pair.enabled and pair.activated_at_s is not None)

                ref = trajectory_at(data.time, float(traj["takeoff_altitude_m"]), float(traj["takeoff_time_s"]))
                if yaw_free_active:
                    # Track the current measured heading instead of recapturing
                    # a fixed yaw after a severe fault or its recovery.
                    yaw_now = math.atan2(meas_R[1, 0], meas_R[0, 0])
                    ref = (
                        ref[0], ref[1], ref[2], ref[3], ref[4],
                        np.array([math.cos(yaw_now), math.sin(yaw_now)]),
                        np.zeros(2), np.zeros(2),
                    )

                try:
                    out = controller.compute(
                        (meas_pos,meas_vel,meas_R,meas_Omega),
                        ref,
                        yaw_free=yaw_free_active,
                    )
                    cmd = out["cmd"].copy()
                    if yaw_free_active:
                        if pair.enabled and pair.active:
                            # Yaw heading remains free; track bounded nonzero
                            # self-spin using physical reaction torque.
                            cmd[3] = yaw_feedback_request
                        elif detector.confirmed or candidate_protection_active:
                            # Keep yaw angle free during the fault and damp only
                            # its rate through the allocator's null-space authority.
                            cmd[3] = float(np.clip(
                                -detector.fault_yaw_rate_damping*meas_Omega[2],
                                -detector.fault_max_yaw_moment,
                                detector.fault_max_yaw_moment,
                            ))
                        else:
                            cmd[3] = 0.0
                    if (not np.isfinite(cmd).all()) or cmd[0] <= 0.0:
                        raise RuntimeError(f"invalid geometric-controller output: {cmd}")

                    measured_state = (meas_pos, meas_vel, meas_R, meas_Omega)
                    if l1.enabled and (
                        fdi_topology_changed or data.time < l1_hold_until
                    ):
                        # The effectiveness-aware/full allocator switch changes
                        # the input channel abruptly. Re-seed the predictor at
                        # that boundary so pre-switch uncertainty is not
                        # replayed through the post-switch plant.
                        l1.reset(measured_state, baseline_cmd=cmd)
                        l1_cmd = np.zeros(4)
                    else:
                        l1_cmd = l1.update(
                            measured_state,
                            cmd,
                            suppress_yaw_control=yaw_free_active,
                        )
                    total_cmd = cmd + l1_cmd
                    if yaw_free_active:
                        # L1 yaw adaptation remains suppressed; preserve the
                        # explicit rate-only damping request from the baseline.
                        total_cmd[3] = cmd[3]
                    if (not np.isfinite(total_cmd).all()) or total_cmd[0] <= 0.0:
                        raise RuntimeError(f"invalid L1-augmented output: {total_cmd}")

                    if pair.active:
                        if yaw_envelope.enabled:
                            allocator_mode = "opposite_pair_yaw_rate_feedback"
                            w_cmd, yaw_allocation_diag = allocate_pair_yaw_control(
                                mixer, total_cmd, pair.failed_motor_id,
                                pair.estimated_loss_percent, yaw_feedback_request,
                            )
                        else:
                            allocator_mode = "opposite_pair_fdi_yaw_free"
                            w_cmd = allocate_opposite_pair(
                                mixer, total_cmd,
                                pair.failed_motor_id, pair.estimated_loss_percent,
                            )
                    elif protection_active:
                        allocator_mode = (
                            "fdi_effectiveness_aware"
                            if detector.confirmed
                            else "fdi_candidate_protection"
                        )
                        w_cmd = mixer.allocate_effectiveness_aware(
                            total_cmd,
                            protection_motor,
                            protection_loss,
                        )
                    elif detector.yaw_free_latched and detector.recovered_at_s is not None:
                        # The source branch deliberately keeps yaw angle free
                        # after recovery. MuJoCo has no aerodynamic body drag,
                        # so add rate-only damping and use the full allocator to
                        # stop unbounded spin without recapturing a heading.
                        total_cmd[3] = float(np.clip(
                            -detector.post_recovery_yaw_rate_damping*meas_Omega[2],
                            -detector.post_recovery_max_yaw_moment,
                            detector.post_recovery_max_yaw_moment,
                        ))
                        allocator_mode = "post_recovery_yaw_rate_damped"
                        w_cmd = mixer.allocate(total_cmd)
                    elif pair.enabled and detector.yaw_free_latched:
                        # Pair is disabled or cooling down: yaw is still free
                        # but physical yaw-rate damping must remain available.
                        total_cmd[3] = float(np.clip(
                            -detector.post_recovery_yaw_rate_damping*meas_Omega[2],
                            -detector.post_recovery_max_yaw_moment,
                            detector.post_recovery_max_yaw_moment,
                        ))
                        allocator_mode = "pair_fallback_yaw_rate_damped"
                        w_cmd = mixer.allocate(total_cmd)
                    elif yaw_free_active:
                        allocator_mode = "yaw_free"
                        w_cmd = mixer.allocate_yaw_free(total_cmd)
                    else:
                        allocator_mode = "full_wrench"
                        w_cmd = mixer.allocate(total_cmd)
                except Exception as e:
                    aborted = True
                    reason = str(e)
                    break

                # The normal plant path is unchanged: command jitter and motor
                # lag produce a nominal actuator state. Fault effectiveness is
                # then injected in thrust space before static mismatch/noise.
                w_nominal = motor_dist.step_command(w_cmd)
                detector.record_control(total_cmd, w_nominal)
                w_applied, fault_active, fault_effectiveness = fault.apply(
                    w_nominal,
                    motor,
                    data.time,
                )
                if pair.active:
                    w_applied = apply_opposite_model_loss(
                        w_applied, motor, pair.opposite_motor_id,
                        pair.estimated_loss_percent,
                    )
                thrusts, moments = motor_dist.actual_forces(w_applied, motor)

                apply_rotors(model, data, body_id, site_ids, thrusts, moments, R_mj)
                wrench = analytic_wrench_frd(thrusts, moments, float(v["L_m"]), float(v["D_m"]))

                Rdes = out["Rdes"]
                rpy = np.degrees(R_to_rpy_ned_frd(R))
                meas_rpy = np.degrees(R_to_rpy_ned_frd(meas_R))
                rpy_des = np.degrees(R_to_rpy_ned_frd(Rdes))
                tilt = combined_tilt_deg(R)
                tilt_des = combined_tilt_deg(Rdes)

                target_pos = ref[0]
                err = float(np.linalg.norm(pos-target_pos))
                max_err = max(max_err, err)
                max_tilt = max(max_tilt, tilt)

                writer.writerow({
                    "t":data.time,
                    "target_x":target_pos[0],"target_y":target_pos[1],"target_z":target_pos[2],
                    "x":pos[0],"y":pos[1],"z":pos[2],
                    "vx":vel[0],"vy":vel[1],"vz":vel[2],
                    "meas_x":meas_pos[0],"meas_y":meas_pos[1],"meas_z":meas_pos[2],
                    "meas_vx":meas_vel[0],"meas_vy":meas_vel[1],"meas_vz":meas_vel[2],
                    "target_roll_deg":rpy_des[0],"target_pitch_deg":rpy_des[1],"target_yaw_deg":rpy_des[2],
                    "roll_deg":rpy[0],"pitch_deg":rpy[1],"yaw_deg":rpy[2],
                    "meas_roll_deg":meas_rpy[0],"meas_pitch_deg":meas_rpy[1],"meas_yaw_deg":meas_rpy[2],
                    "yaw_rate_rps":Omega[2],
                    "yaw_rate_prelimit_rps":yaw_rate_prelimit_rps,
                    "yaw_rate_limiter_active":int(yaw_rate_limiter_active),
                    "target_tilt_deg":tilt_des,"actual_tilt_deg":tilt,
                    "thrust_cmd_N":cmd[0],"mx_cmd_Nm":cmd[1],"my_cmd_Nm":cmd[2],"mz_cmd_Nm":cmd[3],
                    "l1_enabled":int(l1.enabled),
                    "l1_transition_hold":int(l1.enabled and data.time < l1_hold_until),
                    "l1_sigma_thrust_N":l1.sigma_m[0],
                    "l1_sigma_mx_Nm":l1.sigma_m[1],"l1_sigma_my_Nm":l1.sigma_m[2],"l1_sigma_mz_Nm":l1.sigma_m[3],
                    "l1_sigma_um_x_N":l1.sigma_um[0],"l1_sigma_um_y_N":l1.sigma_um[1],
                    "l1_uad_thrust_N":l1_cmd[0],
                    "l1_uad_mx_Nm":l1_cmd[1],"l1_uad_my_Nm":l1_cmd[2],"l1_uad_mz_Nm":l1_cmd[3],
                    "total_thrust_cmd_N":total_cmd[0],
                    "total_mx_cmd_Nm":total_cmd[1],"total_my_cmd_Nm":total_cmd[2],"total_mz_cmd_Nm":total_cmd[3],
                    "l1_vhat_x":l1.v_hat[0],"l1_vhat_y":l1.v_hat[1],"l1_vhat_z":l1.v_hat[2],
                    "l1_omegahat_x":l1.omega_hat[0],"l1_omegahat_y":l1.omega_hat[1],"l1_omegahat_z":l1.omega_hat[2],
                    "wcmd1":w_cmd[0],"wcmd2":w_cmd[1],"wcmd3":w_cmd[2],"wcmd4":w_cmd[3],
                    "wnom1":w_nominal[0],"wnom2":w_nominal[1],"wnom3":w_nominal[2],"wnom4":w_nominal[3],
                    "w1":w_applied[0],"w2":w_applied[1],"w3":w_applied[2],"w4":w_applied[3],
                    "pwm1_us":1000+10*w_applied[0],"pwm2_us":1000+10*w_applied[1],
                    "pwm3_us":1000+10*w_applied[2],"pwm4_us":1000+10*w_applied[3],
                    "f1_N":thrusts[0],"f2_N":thrusts[1],"f3_N":thrusts[2],"f4_N":thrusts[3],
                    "F_actual_N":wrench[0],"Mx_actual_Nm":wrench[1],
                    "My_actual_Nm":wrench[2],"Mz_actual_Nm":wrench[3],
                    "tilt_limiter":int(out["limited"]),
                    "allocator_mode":allocator_mode,
                    "yaw_free_active":int(yaw_free_active),
                    "candidate_protection_active":int(candidate_protection_active),
                    "fault_active":int(fault_active),
                    "fault_motor":fault.motor_id if fault_active else 0,
                    "fault_loss_pct":fault.loss_percent if fault_active else 0.0,
                    "fault_effectiveness":fault_effectiveness,
                    "pair_enabled":int(pair.enabled),
                    "pair_source":pair.source if pair.enabled else "off",
                    "pair_active":int(pair.active),
                    "pair_inhibited":int(pair.inhibited),
                    "pair_failed_motor":pair.failed_motor_id,
                    "pair_opposite_motor":pair.opposite_motor_id,
                    "pair_loss_estimate_pct":pair.estimated_loss_percent,
                    "pair_static_margin":pair.static_thrust_margin,
                    "pair_estimate_bias_pp":(
                        detector.loss_estimate_percent-fault.loss_percent
                        if fault_active else 0.0
                    ),
                    "pair_xy_error_m":float(np.linalg.norm(meas_pos[:2])),
                    "pair_z_error_m":abs(float(meas_pos[2]+traj["takeoff_altitude_m"])),
                    "pair_disengaged_time_s":(
                        pair.disengaged_at_s if pair.disengaged_at_s is not None else ""
                    ),
                    "pair_guard_reason":pair.reason,
                    "pair_retry_pending":int(pair.retry_pending),
                    "pair_transient_reason":pair.transient_reason,
                    "pair_severity_ready":int(pair.active),
                    "pair_yaw_cap_deg_s":(
                        yaw_envelope.limit_deg_s(pair.estimated_loss_percent)
                        if pair.active and yaw_envelope.enabled else 0.0
                    ),
                    "pair_yaw_target_deg_s":(
                        math.degrees(yaw_envelope.target_rate_rps(
                            pair.estimated_loss_percent,
                            -1 if pair.failed_motor_id in (1, 2) else +1
                        ))
                        if pair.active and yaw_envelope.enabled else 0.0
                    ),
                    "pair_yaw_hard_abort_rps":pair.yaw_abort_limit_rps if pair.active else 0.0,
                    "pair_yaw_brake_cmd_nm":yaw_feedback_request,
                    "pair_yaw_allocated_nm":yaw_allocation_diag.get("predicted_yaw_nm", 0.0),
                    "pair_yaw_min_nm":yaw_allocation_diag.get("available_yaw_min_nm", 0.0),
                    "pair_yaw_max_nm":yaw_allocation_diag.get("available_yaw_max_nm", 0.0),
                    "pair_yaw_saturated":int(yaw_allocation_diag.get("saturated", False)),
                    "fdi_state":detector.state,
                    "fdi_motor":detector.detected_id,
                    "fdi_candidate":detector.candidate_id,
                    "fdi_confirm_count":detector.confirm_count,
                    "fdi_recovery_count":detector.recovery_count,
                    "fdi_loss_estimate_pct":detector.loss_estimate_percent,
                    "fdi_residual_ratio":detector.residual_ratio,
                    "fdi_sigma_mx":detector.sigma_observed[0],
                    "fdi_sigma_my":detector.sigma_observed[1],
                    "fdi_sigma_mz":detector.sigma_observed[2],
                    "fdi_loss_instant_pct":detector.loss_instant_percent,
                    "fdi_loss_rate_pp_s":detector.loss_rate_pp_s,
                    "fdi_cooldown_remaining_s":max(0., detector.cooldown_until_s-data.time),
                    "schedule_mode":scheduler.mode,
                    "schedule_raw_loss_pct":scheduler.raw_loss,
                    "schedule_loss_pct":scheduler.scheduled_loss,
                    "schedule_confidence":scheduler.confidence,
                    "allocator_effectiveness_loss_pct":(
                        pair.estimated_loss_percent if pair.active
                        else protection_loss if protection_active else 0.0
                    ),
                    "schedule_allocator_mismatch_pp":(
                        scheduler.scheduled_loss - (
                            pair.estimated_loss_percent if pair.active
                            else protection_loss if protection_active else 0.0
                        )
                    ),
                    "active_kpx":active_tuning["kp"][0],"active_kpy":active_tuning["kp"][1],"active_kpz":active_tuning["kp"][2],
                    "active_kvx":active_tuning["kv"][0],"active_kvy":active_tuning["kv"][1],"active_kvz":active_tuning["kv"][2],
                    "active_krx":active_tuning["kr"][0],"active_kry":active_tuning["kr"][1],"active_krz":active_tuning["kr"][2],
                    "active_kox":active_tuning["ko"][0],"active_koy":active_tuning["ko"][1],"active_koz":active_tuning["ko"][2],
                    "active_max_tilt_deg":active_tuning["max_tilt_deg"],
                })

                if data.time + 1e-12 >= next_print:
                    print(
                        f"t={data.time:6.2f}s "
                        f"target=({target_pos[0]:+.3f},{target_pos[1]:+.3f},{target_pos[2]:+.3f}) "
                        f"pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                        f"measZ={meas_pos[2]:+.3f} "
                        f"tilt={tilt:5.2f}/{tilt_des:5.2f}deg "
                        f"fault={'M'+str(fault.motor_id) if fault_active else '-'} "
                        f"FDI={detector.state}/M{detector.detected_id} "
                        f"loss={detector.loss_estimate_percent:4.1f}% "
                        f"L1=({l1_cmd[0]:+.2f},{l1_cmd[1]:+.3f},"
                        f"{l1_cmd[2]:+.3f},{l1_cmd[3]:+.3f}) "
                        f"PWM=({1000+10*w_applied[0]:.0f},{1000+10*w_applied[1]:.0f},"
                        f"{1000+10*w_applied[2]:.0f},{1000+10*w_applied[3]:.0f})"
                    )
                    next_print += float(sim["print_period_s"])

                mujoco.mj_step(model, data)
                yaw_rate_prelimit_rps = float((S @ data.qvel[3:6])[2])
                yaw_rate_limiter_active = False
                if (
                    detector.yaw_rate_safety_limit_enabled
                    and yaw_free_active
                    and not pair.enabled
                ):
                    (
                        yaw_rate_prelimit_rps,
                        _,
                        yaw_rate_limiter_active,
                    ) = clamp_frd_yaw_rate(
                        data,
                        detector.yaw_rate_safety_limit,
                    )
                if viewer is not None:
                    viewer.sync()

                if bool(sim["realtime"]):
                    sleep_s = wall0 + data.time - time.time()
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)
    finally:
        if ctx is not None:
            ctx.__exit__(None, None, None)

    pos, vel, _, _, _ = get_state(data, body_id, origin_height)
    final_target = trajectory_at(data.time, float(traj["takeoff_altitude_m"]), float(traj["takeoff_time_s"]))[0]

    print()
    print("="*72)
    print("RESULT")
    print("="*72)
    print("STATUS           :", "ABORTED" if aborted else "FINISHED")
    if aborted:
        print("reason           :", reason)
    print(f"sim time         : {data.time:.3f} s")
    print(f"final target NED : {final_target}")
    print(f"final pos NED    : {pos}")
    print(f"final vel NED    : {vel}")
    print(f"final error norm : {np.linalg.norm(pos-final_target):.4f} m")
    print(f"max error norm   : {max_err:.4f} m")
    print(f"max actual tilt  : {max_tilt:.3f} deg")
    print(f"FDI confirmed at : {detector.confirmed_at_s if detector.confirmed_at_s is not None else 'not detected'}")
    print(f"FDI recovered at : {detector.recovered_at_s if detector.recovered_at_s is not None else 'not recovered'}")
    print(f"yaw-free latched : {detector.yaw_free_latched}")
    print(f"pair source      : {pair.source}")
    print(f"pair engaged at  : {pair.activated_at_s if pair.activated_at_s is not None else 'never'}")
    print(f"pair active/lock : {pair.active}/{pair.inhibited}")
    print(f"pair stopped at  : {pair.disengaged_at_s if pair.disengaged_at_s is not None else '-'}")
    print(f"pair reason      : {pair.reason or '-'}")
    print(f"CSV              : {log_path}")
    print("="*72)

    if aborted:
        raise RuntimeError(reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.toml")
    ap.add_argument("--headless", action="store_true", help="disable MuJoCo viewer")
    ap.add_argument("--pair-oracle", action="store_true",
                    help="SIMULATION ONLY: use injected fault truth for paired control benchmark")
    ap.add_argument("--no-realtime", action="store_true", help="run as fast as possible")
    ap.add_argument("--duration", type=float, help="override simulation duration in seconds")
    l1_group = ap.add_mutually_exclusive_group()
    l1_group.add_argument(
        "--l1",
        dest="l1enable",
        action="store_const",
        const=1,
        help="enable L1 adaptive augmentation for this run",
    )
    l1_group.add_argument(
        "--no-l1",
        dest="l1enable",
        action="store_const",
        const=0,
        help="disable L1 adaptive augmentation for this run",
    )
    ap.add_argument(
        "--loss-percent",
        type=float,
        help="override fault_injection.loss_percent for this run (0..100)",
    )
    args = ap.parse_args()
    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)
    if args.pair_oracle:
        cfg.setdefault("opposite_pair", {})["enabled"] = True
        cfg["opposite_pair"]["source"] = "oracle"
    if args.headless:
        cfg["simulation"]["viewer"] = False
    if args.no_realtime:
        cfg["simulation"]["realtime"] = False
    if args.duration is not None:
        cfg["simulation"]["duration_s"] = args.duration
    if args.l1enable is not None:
        cfg["controller"]["l1enable"] = args.l1enable
    if args.loss_percent is not None:
        if not 0.0 <= args.loss_percent <= 100.0:
            ap.error("--loss-percent must be in the range 0..100")
        cfg.setdefault("fault_injection", {})["loss_percent"] = args.loss_percent
    run(cfg)


if __name__ == "__main__":
    main()
