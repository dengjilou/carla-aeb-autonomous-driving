import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from route_tracking import RouteTracking


def actor(number,x,y,speed=8,yaw=0):
    location=NS(x=x,y=y,z=0)
    location.distance=lambda other:math.hypot(x-other.x,y-other.y)
    forward=NS(x=math.cos(yaw),y=math.sin(yaw))
    return NS(id=number,get_location=lambda:location,
              get_velocity=lambda:NS(x=speed*forward.x,y=speed*forward.y),
              get_transform=lambda:NS(get_forward_vector=lambda:forward),
              bounding_box=NS(extent=NS(x=2.,y=1.)))


class RouteTargetingTests(unittest.TestCase):
    def setUp(self):
        self.route=RouteTracking.__new__(RouteTracking)
        self.route.points=np.array([[0.,0.],[100.,0.]])
        self.route.vectors=np.array([[100.,0.]])
        self.route.lengths2=np.array([10000.])
        self.route.stations=np.array([0.,100.])
        self.ego=actor(1,0,0,10)

    def detect(self,actors):
        world=NS(get_actors=lambda:NS(filter=lambda _:actors))
        return self.route.nearest_target(self.ego,world)

    def test_nearest_forward_target_and_closing(self):
        distance,number,closing=self.detect([self.ego,actor(2,30,.1),actor(3,20,0)])
        self.assertAlmostEqual(distance,20)
        self.assertEqual(number,3)
        self.assertAlmostEqual(closing,2)

    def test_adjacent_lane_is_excluded(self):
        self.assertIsNone(self.detect([actor(2,10,3.5)])[1])

    def test_behind_opposite_and_outside_range(self):
        for candidate in [actor(2,-10,0),actor(2,10,0,yaw=math.pi),actor(2,55,0)]:
            with self.subTest(candidate=candidate.id):
                self.assertIsNone(self.detect([candidate])[1])
