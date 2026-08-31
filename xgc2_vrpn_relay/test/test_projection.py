#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.projection import finite_values, translated_position, valid_pose


class ProjectionTests(unittest.TestCase):
    def test_translation_changes_only_position_values(self):
        self.assertEqual(translated_position(1.0, -2.0, 3.0, 0.5, 1.0, -1.5), (1.5, -1.0, 1.5))

    def test_nonfinite_translation_is_rejected(self):
        with self.assertRaises(ValueError):
            translated_position(1.0, 2.0, 3.0, float("nan"), 0.0, 0.0)

    def test_pose_validation_does_not_normalize_orientation(self):
        self.assertTrue(valid_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0))
        self.assertFalse(valid_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(valid_pose(float("inf"), 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))

    def test_finite_values_accepts_ints_and_floats(self):
        self.assertTrue(finite_values(0, 1.5, -2.0))


if __name__ == "__main__":
    unittest.main()
