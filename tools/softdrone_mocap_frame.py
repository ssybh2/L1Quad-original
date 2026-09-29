#!/usr/bin/env python3
"""Softdrone mocap -> ArduPilot NED/FRD frame conversion helpers.

Mocap world:
    +X_M right
    +Y_M forward
    +Z_M up

ArduPilot navigation:
    +X_N forward/North
    +Y_N right/East
    +Z_N down

Body frame is FRD: +X forward, +Y right, +Z down.

Quaternion helpers below use scipy's [x, y, z, w] ordering and assume the
incoming rotation maps body-FRD vectors into the mocap-world frame.
"""

from __future__ import annotations

import numpy as np

R_NED_FROM_MOCAP = np.array(
    [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=float,
)


def position_mocap_to_ned(position_m, origin_m=(0.0, 0.0, 0.0)):
    """Convert mocap world position to NED relative to a mocap origin."""
    p = np.asarray(position_m, dtype=float)
    p0 = np.asarray(origin_m, dtype=float)
    return R_NED_FROM_MOCAP @ (p - p0)


def vector_mocap_to_ned(vector_m):
    """Convert a free vector (velocity/acceleration) from mocap world to NED."""
    return R_NED_FROM_MOCAP @ np.asarray(vector_m, dtype=float)


def rotation_body_to_ned(rotation_mocap_from_body):
    """Convert R_M_B (body FRD -> mocap world) into R_N_B (body FRD -> NED)."""
    r_m_b = np.asarray(rotation_mocap_from_body, dtype=float).reshape(3, 3)
    return R_NED_FROM_MOCAP @ r_m_b


def quaternion_xyzw_body_to_ned(q_mocap_from_body_xyzw):
    """Convert body->mocap quaternion to body->NED quaternion, scipy xyzw order."""
    try:
        from scipy.spatial.transform import Rotation
    except ImportError as exc:
        raise RuntimeError("Install scipy to use quaternion conversion") from exc

    q = np.asarray(q_mocap_from_body_xyzw, dtype=float)
    r_m_b = Rotation.from_quat(q).as_matrix()
    r_n_b = rotation_body_to_ned(r_m_b)
    return Rotation.from_matrix(r_n_b).as_quat()


if __name__ == "__main__":
    # At the requested zero-yaw starting pose, the body axes expressed in the
    # mocap world are: X_B=+Y_M, Y_B=+X_M, Z_B=-Z_M.
    r_m_b_zero = np.array(
        [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
    )
    print("R_NED_FROM_MOCAP =\n", R_NED_FROM_MOCAP)
    print("Zero-pose R_NED_FROM_BODY =\n", rotation_body_to_ned(r_m_b_zero))
    print("Forward 1 m ->", position_mocap_to_ned((0.0, 1.0, 0.0)))
    print("Right   1 m ->", position_mocap_to_ned((1.0, 0.0, 0.0)))
    print("Up      1 m ->", position_mocap_to_ned((0.0, 0.0, 1.0)))
