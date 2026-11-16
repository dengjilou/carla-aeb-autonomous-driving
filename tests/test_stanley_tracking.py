"""Check the CARLA sign convention, heading wrap and steering units."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers'))
from stanley_tracking import steering


class StanleyTrackingTests(unittest.TestCase):
    def test_offset_sign(self):
        self.assertLess(steering(0, 0, 1, 8, 1.2, 0.3), 0)
        self.assertGreater(steering(0, 0, -1, 8, 1.2, 0.3), 0)

    def test_heading_wrap(self):
        result = steering(math.radians(179), math.radians(-179), 0, 8, 1, 0.3)
        self.assertAlmostEqual(result, math.radians(2))

    def test_angle_normalization(self):
        self.assertAlmostEqual(steering(0, 0.2, 0, 8, 1.25, 0.3), 0.16)

    def test_stopped_on_center(self):
        self.assertEqual(steering(0, 0, 0, 0, 1.2, 0.3), 0)
