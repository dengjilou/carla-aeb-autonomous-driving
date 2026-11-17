import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from aeb_protection import distance_level


class DistanceProtectionTests(unittest.TestCase):
    def test_legacy_thresholds_ignore_closing(self):
        for closing in (-10,0,10):
            for distance,expected in ((9.99,3),(10,2),(14.99,2),(15,0)):
                self.assertEqual(distance_level(distance,closing),expected)

    def test_guard_suppresses_equal_and_opening_targets(self):
        for closing in (-10,0,.1):
            self.assertEqual(distance_level(13.208,closing,True),0)
            self.assertEqual(distance_level(5,closing,True),0)

    def test_approaching_and_projected_overlap_remain_protected(self):
        self.assertEqual(distance_level(14,2.8,True),2)
        self.assertEqual(distance_level(14,2.799,True),0)
        self.assertEqual(distance_level(13,.100001,True),0)
        self.assertEqual(distance_level(9,5,True),3)
        for closing in (-1,0,1):
            self.assertEqual(distance_level(0,closing,True),3)
            self.assertEqual(distance_level(-.1,closing,True),3)
