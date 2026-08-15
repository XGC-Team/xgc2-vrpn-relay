#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.rate import should_emit_vision


class VisionRateTests(unittest.TestCase):
    def test_first_frame(self):
        self.assertTrue(should_emit_vision(0.0, None, 30.0))

    def test_disabled_target(self):
        self.assertTrue(should_emit_vision(0.01, 0.0, 0.0))

    def test_drop_when_faster(self):
        self.assertFalse(should_emit_vision(0.01, 0.0, 30.0))

    def test_emit_at_period(self):
        self.assertTrue(should_emit_vision(1.0 / 30.0, 0.0, 30.0))

    def test_emit_when_slower(self):
        self.assertTrue(should_emit_vision(0.1, 0.0, 30.0))

    def test_clock_rollback(self):
        self.assertTrue(should_emit_vision(0.0, 1.0, 30.0))


if __name__ == "__main__":
    unittest.main()
