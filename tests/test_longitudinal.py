import json
import tempfile
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "controllers"), str(ROOT / "scenarios")]
from acc import ACC, SpeedPID, arbitrate
from longitudinal_metrics import evaluate
from run_longitudinal import load_config


class LongitudinalTests(unittest.TestCase):
    def test_free_cruise(self):
        self.assertEqual(ACC().target_speed(8, 12), 12)

    def test_equilibrium(self):
        self.assertAlmostEqual(ACC().target_speed(10, 12, gap=26, closing=0), 10)

    def test_close_and_closing_decelerates(self):
        self.assertLess(ACC().target_speed(12, 15, gap=15, closing=4), 12)

    def test_reference_is_bounded(self):
        for gap in (-2, 0, 1000):
            for closing in (-50, 50):
                self.assertTrue(0 <= ACC().target_speed(10, 12, gap, closing) <= 12)

    def test_pid_exclusive_actuators(self):
        pid = SpeedPID()
        for target in (12, 0, 12, 0):
            throttle, brake = pid.update(target, 8, .05)
            self.assertEqual(throttle * brake, 0)
            self.assertTrue(0 <= throttle <= .75 and 0 <= brake <= 1)

    def test_pid_rejects_zero_step(self):
        with self.assertRaises(ValueError):
            SpeedPID().update(10, 8, 0)

    def test_emergency_survives_target_loss(self):
        first = arbitrate(.7, 0, 1, 0)
        self.assertEqual(first, (0, 1, 1))
        self.assertEqual(arbitrate(.7, 0, 0, first[2]), (0, 1, 1))

    def test_configuration_loads_offline(self):
        self.assertEqual(len(load_config(ROOT / "configs/longitudinal.json")["cases"]), 10)

    def test_config_rejects_duplicate_case(self):
        cfg = load_config(ROOT / "configs/longitudinal.json")
        cfg["cases"].append(cfg["cases"][0])
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.json"
            path.write_text(json.dumps(cfg))
            with self.assertRaises(ValueError):
                load_config(path)

    def rows(self):
        return [dict(time_s=i * .05, aeb_brake=0, sumo_background=1,
                     event_observed=False, cross_track_m=0, lead_id=2, target_id=2,
                     ego_speed_mps=10, lead_speed_mps=10, gap_m=26,
                     lateral_clearance_m=-2, longitudinal_m=30) for i in range(40)]

    def test_absent_event_is_invalid_without_collision(self):
        result = evaluate(self.rows(), [], dict(step_s=.05, duration_s=2))
        self.assertFalse(result["valid"])
        self.assertFalse(result["collided"])

    def test_late_brake_and_callback_count(self):
        rows = self.rows()
        for row in rows:
            row["event_observed"] = True
        rows[30]["aeb_brake"] = 1
        contacts = [dict(time_s=1, other_id=2), dict(time_s=1.05, other_id=2)]
        result = evaluate(rows, contacts, dict(step_s=.05, duration_s=2))
        self.assertTrue(result["collided"])
        self.assertFalse(result["braked_before_contact"])
        self.assertEqual(result["collision_callbacks"], 2)

    def test_other_contact_invalidates_case(self):
        result = evaluate(self.rows(), [dict(time_s=1, other_id=3)], dict(step_s=.05, duration_s=2))
        self.assertIn("non_lead_contact", result["invalid_reasons"])


if __name__ == "__main__":
    unittest.main()
