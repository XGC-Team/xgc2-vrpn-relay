"""ROS-independent helpers for the Experiment localization projection."""

from __future__ import print_function

import math


def finite_values(*values):
    return all(
        isinstance(value, (int, float))
        and not math.isnan(value)
        and not math.isinf(value)
        for value in values
    )


def translated_position(x, y, z, offset_x, offset_y, offset_z):
    values = (x, y, z, offset_x, offset_y, offset_z)
    if not finite_values(*values):
        raise ValueError("position and localization offset must be finite")
    return x + offset_x, y + offset_y, z + offset_z


def valid_pose(x, y, z, qx, qy, qz, qw):
    if not finite_values(x, y, z, qx, qy, qz, qw):
        return False
    return qx * qx + qy * qy + qz * qz + qw * qw >= 1.0e-12
