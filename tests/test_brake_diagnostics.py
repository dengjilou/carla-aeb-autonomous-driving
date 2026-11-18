import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scenarios"))
from report_brake_diagnostics import audit_samples
from run_longitudinal import run_case


class BrakeDiagnosticTests(unittest.TestCase):
    def samples(self):
        rows = []
        for i in range(2):
            rows.append(dict(frame=str(100+i), snapshot_frame=str(100+i), time_s=str(i*.05),
                             elapsed_s=str(5+i*.05), delta_s="0.05", substepping="True",
                             max_substeps="10", max_substep_delta_time="0.005",
                             applied_brake=str(i), brake="1", applied_throttle="0", throttle="0",
                             ego_speed_mps=str(10-i), longitudinal_acceleration_mps2="-20",
                             velocity_x_mps=str(10-i), velocity_y_mps="0", velocity_z_mps="0",
                             acceleration_x_mps2="-20", acceleration_y_mps2="0", acceleration_z_mps2="0"))
        return rows

    def test_previous_command_is_compared_with_current_applied_control(self):
        config = dict(step_s=.05, trigger_s=0, physics_substep_s=.005)
        rows = self.samples()
        result = audit_samples(rows, config)
        self.assertEqual(result["vector_acceleration_max_error_mps2"], 0)
        self.assertEqual(result["peak_time_s"], .05)
        rows[1]["applied_brake"] = "0"
        with self.assertRaisesRegex(AssertionError, "Control timing"):
            audit_samples(rows, config)

    def test_skipped_frame_is_rejected(self):
        rows = self.samples()
        rows[1]["frame"] = rows[1]["snapshot_frame"] = "102"
        with self.assertRaisesRegex(AssertionError, "Skipped frame"):
            audit_samples(rows, dict(step_s=.05, trigger_s=0, physics_substep_s=.005))

    def test_invalid_physics_substep_rejected_before_connecting(self):
        for value in (0, -.01, .0025, .1, float("nan"), float("inf")):
            with self.subTest(substep=value), self.assertRaisesRegex(ValueError, "1 to 16"):
                run_case(dict(step_s=.05, physics_substep_s=value), "baseline", {})
