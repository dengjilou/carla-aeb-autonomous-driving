import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers'))
from lqr_tracking import LQR


class LQRTests(unittest.TestCase):
    def test_riccati_residual_and_stability(self):
        for speed in (6, 8, 11, 12):
            controller = LQR(speed, 3)
            a, b, p, q, k = controller.a, controller.b, controller.p, controller.q, controller.k
            np.testing.assert_allclose(p, q+a.T@p@(a-b@k), atol=1e-8)
            self.assertLess(max(abs(np.linalg.eigvals(a-b@k))), 1)

    def test_error_sign(self):
        controller = LQR(8, 3)
        self.assertLess(controller.steering(1, 0, 0, 1.2), 0)
        self.assertLess(controller.steering(0, 0.1, 0, 1.2), 0)

    def test_feedforward_and_heading_wrap(self):
        controller = LQR(8, 3)
        self.assertAlmostEqual(controller.steering(0, 0, 0.02, 1.2), math.atan(0.06)/1.2)
        self.assertAlmostEqual(controller.steering(0, 0.1, 0, 1.2),
                               controller.steering(0, 0.1+2*math.pi, 0, 1.2))

    def test_invalid_weight(self):
        with self.assertRaises(ValueError):
            LQR(8, 3, position_weight=0)
