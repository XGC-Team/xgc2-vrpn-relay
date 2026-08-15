"""Pose quality gate used by every robot.

Checks come from the old mocap_direct_forward / Adapter path:
non-finite, degenerate quaternion, (0,0) origin, |xy|>900, jump, frozen.
"""
from __future__ import print_function

import math


def is_finite(*values):
    for value in values:
        if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
            return False
        if not isinstance(value, (int, float)):
            return False
    return True


def evaluate_pose(
    x,
    y,
    z,
    qx,
    qy,
    qz,
    qw,
    last_ok_xyz,
    last_move_xyz,
    last_move_s,
    now_s,
    reject_origin=True,
    max_abs_xy=900.0,
    max_jump_m=2.0,
    frozen_s=0.5,
):
    """Return (ok, reason, normalized_quat).

    last_ok_xyz / last_move_xyz are (x, y, z) or None.
    last_move_s / now_s are seconds.
    """
    if not is_finite(x, y, z, qx, qy, qz, qw):
        return False, "nonfinite", None
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not is_finite(norm) or norm < 1e-6:
        return False, "bad_quat", None
    if reject_origin and x == 0.0 and y == 0.0:
        return False, "origin", None
    if max_abs_xy > 0.0 and (abs(x) > max_abs_xy or abs(y) > max_abs_xy):
        return False, "abs_xy", None
    if last_ok_xyz is not None:
        dx = x - last_ok_xyz[0]
        dy = y - last_ok_xyz[1]
        dz = z - last_ok_xyz[2]
        jump = math.sqrt(dx * dx + dy * dy + dz * dz)
        if max_jump_m > 0.0 and jump > max_jump_m:
            return False, "jump", None
    if last_move_xyz is not None and frozen_s > 0.0:
        dx = x - last_move_xyz[0]
        dy = y - last_move_xyz[1]
        dz = z - last_move_xyz[2]
        moved = math.sqrt(dx * dx + dy * dy + dz * dz)
        if moved < 1e-4 and (now_s - last_move_s) >= frozen_s:
            return False, "frozen", None
    return True, "", (qx / norm, qy / norm, qz / norm, qw / norm)
