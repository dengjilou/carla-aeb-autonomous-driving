import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scenarios'))
from longitudinal_metrics import evaluate


class CurvedMetricsTests(unittest.TestCase):
    def rows(self):
        return [dict(time_s=i*.05,aeb_brake=0,cross_track_m=0,path_error_m=0,
                     heading_error_rad=0,ego_speed_mps=10,lead_speed_mps=10,gap_m=26,
                     chord_gap_m=25,lateral_clearance_m=1,longitudinal_m=30,
                     sumo_background=1,event_observed=True,lead_id=2,target_id=2,
                     ego_yaw_rad=i*.005,lead_path_error_m=.1,path_gap_m=26,path_distance_m=31)
                for i in range(40)]

    def evaluate(self,rows):
        return evaluate(rows,[],dict(kind='follow',step_s=.05,duration_s=2,curved=True))

    def test_route_gap_and_target_coverage(self):
        result=self.evaluate(self.rows())
        self.assertEqual(result['verdict'],'pass')
        self.assertEqual(result['target_coverage'],1)
        self.assertAlmostEqual(result['steady_gap_rmse_m'],0)

    def test_lost_target_fails_even_without_collision(self):
        rows=self.rows()
        for row in rows[:10]:
            row['target_id']=None
        self.assertEqual(self.evaluate(rows)['verdict'],'fail')

    def test_lead_route_error_invalidates_input(self):
        rows=self.rows()
        rows[10]['lead_path_error_m']=.6
        self.assertIn('lead_left_reference_route',self.evaluate(rows)['invalid_reasons'])

    def test_heading_wrap_and_straight_rejection(self):
        rows=self.rows()
        for i,row in enumerate(rows):
            angle=math.radians(179+i*4/39)
            row['ego_yaw_rad']=math.atan2(math.sin(angle),math.cos(angle))
        self.assertAlmostEqual(self.evaluate(rows)['turn_deg'],4)
        for row in rows:
            row['ego_yaw_rad']=0
        self.assertIn('insufficient_observed_turn',self.evaluate(rows)['invalid_reasons'])
