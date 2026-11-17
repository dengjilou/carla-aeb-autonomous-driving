"""Actual TCP -> DDS -> C++ checks; opt in after starting the ROS launch."""
import json
import math
import os
import random
import socket
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from control_native import PythonController, OUTPUT_FIELDS
from control_ros import RosController
from test_control_native import sample


@unittest.skipUnless(os.environ.get('ROS_LIVE_TEST')=='1','start ROS and set ROS_LIVE_TEST=1')
class RosLiveTests(unittest.TestCase):
    def setUp(self):
        self.python=PythonController()
        self.ros=RosController()

    def tearDown(self):
        self.ros.close()

    def compare(self,row):
        expected=self.python.step(row)
        actual=self.ros.step(row)
        for key in OUTPUT_FIELDS:
            self.assertAlmostEqual(expected[key],actual[key],delta=1e-9,msg=key)
        return actual

    def packet(self,sequence,row):
        request=dict(session=self.ros.session,sequence=sequence,input=row,config=self.ros.config)
        self.ros.stream.write((json.dumps(request)+'\n').encode())
        self.ros.stream.flush()
        return json.loads(self.ros.stream.readline())

    def test_sequential_parity(self):
        rng=random.Random(42)
        for tick in range(1000):
            if tick%100==0:
                self.python.reset()
                self.ros.reset()
            self.compare(sample(speed=rng.uniform(0,25),gap=rng.uniform(-2,60),
                                closing=rng.uniform(-8,12),heading=rng.uniform(-math.pi,math.pi),
                                path_heading=rng.uniform(-math.pi,math.pi),cross_track=rng.uniform(-2,2),
                                has_target=int(tick%17!=0),acc_enabled=int(tick%23!=0),
                                aeb_request=.4 if tick%100==75 else 1. if tick%100==90 else 0.))

    def test_duplicate_and_out_of_order_preserve_state(self):
        first=self.compare(sample())
        self.assertEqual(self.packet(0,sample())['output'],first)
        self.assertIn('error',self.packet(3,sample()))
        self.compare(sample(speed=11.))

    def test_invalid_first_frame_and_reset(self):
        self.assertIn('error',self.packet(0,sample(dt=0)))
        self.assertIn('error',self.packet(5,sample()))
        self.compare(sample(aeb_request=.4))
        self.python.reset()
        self.ros.reset()
        self.assertEqual(self.compare(sample())['latched_brake'],0.)

    def test_abrupt_peer_disconnect_and_reconnect(self):
        self.compare(sample())
        layout='hh' if os.name=='nt' else 'ii'
        self.ros.socket.setsockopt(socket.SOL_SOCKET,socket.SO_LINGER,struct.pack(layout,1,0))
        self.ros.close()
        self.ros=RosController()
        self.python.reset()
        self.compare(sample())


if __name__=='__main__':
    unittest.main()
