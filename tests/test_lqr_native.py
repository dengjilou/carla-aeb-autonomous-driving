"""Eigen LQR parity plus independent Riccati/stability and state-reset checks."""
import ctypes as ct
import math
import random
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from control_native import Config, CONFIG_FIELDS, NativeController, PythonController, default_config
from lqr_tracking import LQR
from test_control_native import sample


class NativeLqrTests(unittest.TestCase):
    def test_riccati_residual_stability_and_python_solution(self):
        for speed in (.5,4.,8.,12.,25.):
            for weight in (.05,.2,1.):
                config=dict(lateral_mode=1,lqr_speed=speed,lqr_wheelbase=2.9,lqr_position_weight=weight)
                native=NativeController(config)
                try:
                    gain,p=native.lqr_solution()
                    gain=np.array(gain).reshape(1,2)
                    p=np.array(p).reshape(2,2)
                    python=LQR(speed,2.9,position_weight=weight)
                    np.testing.assert_allclose(gain,python.k,rtol=0,atol=1e-9)
                    np.testing.assert_allclose(p,python.p,rtol=0,atol=1e-8)
                    a,b,q=python.a,python.b,python.q
                    np.testing.assert_allclose(p,q+a.T@p@(a-b@gain),rtol=0,atol=1e-8)
                    self.assertLess(max(abs(np.linalg.eigvals(a-b@gain))),1.)
                finally:
                    native.close()

    def test_sequential_commands_with_changing_measured_speed(self):
        python=PythonController(dict(lateral_mode=1))
        native=NativeController(dict(lateral_mode=1))
        rng=random.Random(2026)
        try:
            for tick in range(2000):
                if tick%200==0:
                    python.reset()
                    native.reset()
                row=sample(speed=rng.uniform(0,30),gap=rng.uniform(-2,80),closing=rng.uniform(-8,12),
                           heading=rng.uniform(-math.pi,math.pi),path_heading=rng.uniform(-math.pi,math.pi),
                           cross_track=rng.uniform(-2,2),curvature=rng.uniform(-.06,.06),
                           has_target=int(tick%17!=0),acc_enabled=int(tick%23!=0),
                           aeb_request=.4 if tick%200==150 else 1. if tick%200==180 else 0.)
                a,b=python.step(row),native.step(row)
                for key in a:
                    self.assertAlmostEqual(a[key],b[key],delta=1e-9,msg=key)
        finally:
            native.close()

    def test_feedforward_and_heading_wrap(self):
        native=NativeController(dict(lateral_mode=1))
        try:
            expected=.4*math.atan(3*.02)/1.2
            self.assertAlmostEqual(native.step(sample(curvature=.02))['steer'],expected,delta=1e-12)
            native.reset()
            a=native.step(sample(heading=.1))['steer']
            native.reset()
            b=native.step(sample(heading=.1+2*math.pi))['steer']
            self.assertAlmostEqual(a,b,delta=1e-12)
        finally:
            native.close()

    def test_reset_retains_gain_and_clears_control_state(self):
        python=PythonController(dict(lateral_mode=1))
        native=NativeController(dict(lateral_mode=1))
        try:
            solution=native.lqr_solution()
            native.step(sample(aeb_request=1.,cross_track=1.))
            native.reset()
            self.assertEqual(solution,native.lqr_solution())
            self.assertEqual(native.step(sample()),python.step(sample()))
        finally:
            native.close()

    def test_invalid_parameters_and_frame_preserve_state(self):
        native=NativeController(dict(lateral_mode=1))
        python=PythonController(dict(lateral_mode=1))
        try:
            config=default_config()
            config=Config(**{k:config[k] for k in CONFIG_FIELDS})
            for speed,weight in ((0.,.2),(12.,0.),(float('nan'),.2),(1e-8,.2)):
                self.assertFalse(native.library.control_create_lqr(ct.byref(config),speed,3.,.05,weight))
            for controller in (native,python):
                with self.assertRaises(ValueError):
                    controller.step(sample(curvature=float('nan')))
            self.assertEqual(native.step(sample()),python.step(sample()))
        finally:
            native.close()


if __name__=='__main__':
    unittest.main()
