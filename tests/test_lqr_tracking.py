import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers'))
from lqr_tracking import LQR, ScheduledLQR


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

    def test_experiments_cannot_silently_change_native_backend(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scenarios'))
        from run_longitudinal import run_case
        for cfg in ({'backend':'cpp','lqr_schedule':True},
                    {'backend':'ros2','steering_filter_alpha':1}):
            with self.assertRaisesRegex(ValueError,'Python backend'):
                run_case(cfg,'acc',{})

    def test_scheduled_gain_grid_clamp_and_interpolation(self):
        controller = ScheduledLQR(3)
        for speed in (1, 8, 12, 25, 30):
            fixed = LQR(speed, 3)
            np.testing.assert_allclose(controller.gain(speed), fixed.k[0], atol=1e-12)
            self.assertAlmostEqual(controller.steering(.2,.03,.02,1.2,speed),
                                   fixed.steering(.2,.03,.02,1.2))
        np.testing.assert_allclose(controller.gain(0), controller.gain(1))
        np.testing.assert_allclose(controller.gain(40), controller.gain(30))
        np.testing.assert_allclose(controller.gain(8.5),(controller.gain(8)+controller.gain(9))/2)
        for speed in np.arange(1,30,.5):
            model=LQR(speed,3)
            self.assertLess(max(abs(np.linalg.eigvals(model.a-model.b@controller.gain(speed)[None,:]))),1)
        with self.assertRaises(ValueError):
            controller.gain(float('nan'))
