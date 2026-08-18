#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.offset import apply_world_offset, compute_align_offset


class OffsetTests(unittest.TestCase):
    def test_align_current_to_origin(self):
        dx, dy = compute_align_offset(-13.19, -4.36, 0.0, 0.0)
        x, y, z = apply_world_offset(-13.19, -4.36, 0.38, dx, dy, 0.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.38, places=6)

    def test_align_current_to_zero_minus_two(self):
        dx, dy = compute_align_offset(-13.19, -4.36, 0.0, -2.0)
        x, y, z = apply_world_offset(-13.19, -4.36, 0.38, dx, dy, 0.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, -2.0, places=6)
        self.assertAlmostEqual(z, 0.38, places=6)

    def test_fixed_offset_leaves_z(self):
        x, y, z = apply_world_offset(1.0, 2.0, 3.0, 0.5, -1.0, 0.0)
        self.assertEqual((x, y, z), (1.5, 1.0, 3.0))

    def test_zero_offset(self):
        self.assertEqual(apply_world_offset(1.0, 2.0, 3.0, 0.0, 0.0, 0.0), (1.0, 2.0, 3.0))


if __name__ == "__main__":
    unittest.main()
