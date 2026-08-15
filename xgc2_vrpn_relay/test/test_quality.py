#!/usr/bin/env python3
from __future__ import print_function

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.quality import evaluate_pose, is_finite


class FiniteTests(unittest.TestCase):
    def test_numbers(self):
        self.assertTrue(is_finite(1, 0.0, -2.5))

    def test_nan(self):
        self.assertFalse(is_finite(float("nan")))

    def test_inf(self):
        self.assertFalse(is_finite(float("inf")))

    def test_not_number(self):
        self.assertFalse(is_finite("1"))


class PoseQualityTests(unittest.TestCase):
    def _ok(self, **kwargs):
        args = dict(
            x=1.0,
            y=2.0,
            z=0.5,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
            last_ok_xyz=None,
            last_move_xyz=None,
            last_move_s=0.0,
            now_s=1.0,
        )
        args.update(kwargs)
        return evaluate_pose(**args)

    def test_accepts_unit_quat(self):
        ok, reason, nq = self._ok()
        self.assertTrue(ok)
        self.assertEqual(reason, "")
        self.assertAlmostEqual(nq[3], 1.0)

    def test_normalizes_quat(self):
        ok, reason, nq = self._ok(qw=2.0)
        self.assertTrue(ok)
        self.assertAlmostEqual(math.sqrt(sum(c * c for c in nq)), 1.0)

    def test_nonfinite(self):
        ok, reason, nq = self._ok(x=float("nan"))
        self.assertFalse(ok)
        self.assertEqual(reason, "nonfinite")
        self.assertIsNone(nq)

    def test_bad_quat(self):
        ok, reason, _nq = self._ok(qx=0.0, qy=0.0, qz=0.0, qw=0.0)
        self.assertFalse(ok)
        self.assertEqual(reason, "bad_quat")

    def test_origin(self):
        ok, reason, _nq = self._ok(x=0.0, y=0.0)
        self.assertFalse(ok)
        self.assertEqual(reason, "origin")

    def test_origin_allowed(self):
        ok, reason, _nq = self._ok(x=0.0, y=0.0, reject_origin=False)
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_abs_xy(self):
        ok, reason, _nq = self._ok(x=901.0)
        self.assertFalse(ok)
        self.assertEqual(reason, "abs_xy")

    def test_jump(self):
        ok, reason, _nq = self._ok(last_ok_xyz=(1.0, 2.0, 0.5), x=4.1, y=2.0, z=0.5)
        self.assertFalse(ok)
        self.assertEqual(reason, "jump")

    def test_frozen(self):
        ok, reason, _nq = self._ok(
            last_move_xyz=(1.0, 2.0, 0.5),
            last_move_s=0.0,
            now_s=0.5,
        )
        self.assertFalse(ok)
        self.assertEqual(reason, "frozen")

    def test_not_frozen_if_moved(self):
        ok, reason, _nq = self._ok(
            x=1.2,
            last_move_xyz=(1.0, 2.0, 0.5),
            last_move_s=0.0,
            now_s=0.5,
        )
        self.assertTrue(ok)
        self.assertEqual(reason, "")


if __name__ == "__main__":
    unittest.main()
