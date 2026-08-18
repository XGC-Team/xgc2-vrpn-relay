"""World-frame translation applied after the quality gate.

Twist / accel stay in the Motive frame: a origin shift does not change
velocity. Z is left alone unless offset_z is set.
"""
from __future__ import print_function


def compute_align_offset(x, y, align_to_x, align_to_y):
    """Return (dx, dy) so (x, y) becomes (align_to_x, align_to_y)."""
    return (align_to_x - x, align_to_y - y)


def apply_world_offset(x, y, z, dx, dy, dz=0.0):
    """Translate a point in the world / Motive frame."""
    return (x + dx, y + dy, z + dz)
