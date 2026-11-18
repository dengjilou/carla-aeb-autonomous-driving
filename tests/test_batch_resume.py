import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scenarios"))
import run_longitudinal as runner


class BatchRecoveryTests(unittest.TestCase):
    def test_resume_preserves_failed_verdict_and_repairs_corruption(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            argv = ["runner", "--case", "short_cutin_failure", "--mode", "acc",
                    "--resume", "--output", tmp]
            def simulate(cfg, mode, gains, trace_path):
                trace_path.write_text("time_s,brake\n0,1\n")
                return [], [{"time_s": 0.0}]
            with patch.object(sys, "argv", argv), patch.object(runner, "run_case", side_effect=simulate) as sim, patch.object(runner, "evaluate", return_value={"verdict": "fail"}):
                runner.main()
                summary = output / "summary.json"
                saved = summary.read_bytes()
                runner.main()
                runner.main()
                self.assertEqual(sim.call_count, 1)
                self.assertEqual(summary.read_bytes(), saved)
                trace = next(output.glob("*.csv"))
                trace.write_text("damaged")
                runner.main()
                self.assertEqual(sim.call_count, 2)
                archive = next(output.glob("*-attempt-*"))
                self.assertEqual((archive / trace.name).read_text(), "damaged")
                with patch.object(sys, "argv", argv + ["--gap-gain", "1.2"]):
                    with self.assertRaisesRegex(ValueError, "identity"):
                        runner.main()
                self.assertEqual(sim.call_count, 2)
                self.assertEqual(json.loads(summary.read_text())["runs"]["short_cutin_failure_acc"]["metrics"]["verdict"], "fail")

