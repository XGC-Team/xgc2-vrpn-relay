#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.rate import remember_vision_emit, should_emit_vision


class VisionRateTests(unittest.TestCase):
    def test_first_frame(self):
        self.assertTrue(should_emit_vision(0.0, [], 30.0))

    def test_disabled_target(self):
        self.assertTrue(should_emit_vision(0.01, [0.0], 0.0))

    def test_drop_when_faster(self):
        self.assertFalse(should_emit_vision(0.01, [0.0], 30.0))

    def test_emit_at_period(self):
        self.assertTrue(should_emit_vision(1.0 / 30.0, [0.0], 30.0))

    def test_emit_when_slower(self):
        self.assertTrue(should_emit_vision(0.1, [0.0], 30.0))

    def test_clock_rollback(self):
        self.assertTrue(should_emit_vision(0.0, [1.0], 30.0))

    def test_window_catch_up_when_recent_rate_is_low(self):
        published = [0.00, 0.05, 0.10, 0.15, 0.20]
        self.assertTrue(should_emit_vision(0.21, published, 30.0))

    def test_120hz_in_stays_near_30hz_out(self):
        published = []
        emits = 0
        for index in range(120):
            now = index / 120.0
            if should_emit_vision(now, published, 30.0):
                remember_vision_emit(now, published)
                emits += 1
        self.assertEqual(len(published), 5)
        self.assertEqual(emits, 30)

    def test_remember_keeps_five(self):
        published = []
        for index in range(8):
            remember_vision_emit(float(index), published)
        self.assertEqual(published, [3.0, 4.0, 5.0, 6.0, 7.0])


if __name__ == "__main__":
    unittest.main()
