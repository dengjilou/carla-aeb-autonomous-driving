"""Sequential parity against the deployed Python controller composition."""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from control_native import NativeController, PythonController, OUTPUT_FIELDS


def sample(**overrides):
    row=dict(dt=.05,speed=10.,cruise=12.,gap=26.,closing=0.,heading=0.,
             path_heading=0.,cross_track=0.,max_angle=1.2,aeb_request=0.,
             has_target=1,acc_enabled=1)
    row.update(overrides)
    return row


class NativeParityTests(unittest.TestCase):
    def setUp(self):
        self.python=PythonController()
        self.native=NativeController()

    def tearDown(self):
        self.native.close()

    def compare(self,row):
        a,b=self.python.step(row),self.native.step(row)
        for name in OUTPUT_FIELDS:
            self.assertAlmostEqual(a[name],b[name],delta=1e-9,msg=name)
        return b

    def test_deterministic_sequence(self):
        rng=random.Random(42)
        for tick in range(1000):
            if tick%100==0:
                self.python.reset()
                self.native.reset()
            self.compare(sample(speed=rng.uniform(0,25),gap=rng.uniform(-2,60),
                                closing=rng.uniform(-8,12),heading=rng.uniform(-math.pi,math.pi),
                                path_heading=rng.uniform(-math.pi,math.pi),cross_track=rng.uniform(-2,2),
                                has_target=int(tick%17!=0),acc_enabled=int(tick%23!=0),
                                aeb_request=.4 if tick%100==75 else 1. if tick%100==90 else 0.))

    def test_target_loss_limits_and_wrap(self):
        for row in [sample(speed=0,gap=0),sample(heading=math.radians(179),path_heading=math.radians(-179)),
                    sample(has_target=0),sample(cross_track=100),sample(cross_track=-100),
                    sample(aeb_request=.4),sample(has_target=0),sample(aeb_request=1)]:
            self.compare(row)

    def test_reset_and_aeb_latch(self):
        self.compare(sample(aeb_request=.4))
        self.assertGreaterEqual(self.compare(sample())['brake'],.4)
        self.python.reset()
        self.native.reset()
        self.assertEqual(self.compare(sample())['latched_brake'],0)

    def test_invalid_step_preserves_state(self):
        for backend in (self.python,self.native):
            with self.assertRaises(ValueError):
                backend.step(sample(dt=0))
        self.compare(sample())

    def test_close_is_idempotent(self):
        self.native.close()
        self.native.close()
        with self.assertRaises(RuntimeError):
            self.native.step(sample())


if __name__=='__main__':
    unittest.main()
