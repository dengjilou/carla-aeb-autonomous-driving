"""Regression check for transform direction and first-frame-only alignment."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
from evaluate import evaluate


class EvaluationTest(unittest.TestCase):
    def test_lidar_x_becomes_camera_z_without_scale_fitting(self):
        n = 122
        with tempfile.TemporaryDirectory() as location:
            root = Path(location)
            calib = root / "sequences" / "test"
            calib.mkdir(parents=True)
            (root / "poses").mkdir()
            (calib / "calib.txt").write_text("Tr: 0 -1 0 0 0 0 -1 0 1 0 0 0\n")
            gt = np.tile(np.eye(4), (n,1,1))
            gt[:,2,3] = 20 + np.arange(n)
            np.savetxt(root/"poses/test.txt",gt[:,:3].reshape(n,12))
            lidar = np.tile(np.eye(4),(n,1,1))
            lidar[:,0,3] = 1.01*np.arange(n)
            columns = ["frame","converged","registration_ms","frame_ms"] + [
                f"T{r}{c}" for r in range(3) for c in range(4)]
            table = np.column_stack([np.arange(n),np.ones((n,3)),lidar[:,:3].reshape(n,12)])
            np.savetxt(root/"run.csv",table,delimiter=",",header=",".join(columns),comments="")
            metrics,_,_,err,_,_ = evaluate(root,"test",root/"run.csv")
            self.assertAlmostEqual(err[-1],1.21)
            # Ground truth first strictly exceeds the nominal 100 m at 101 m.
            self.assertAlmostEqual(metrics["drift_100m_pct"],1.01)
            self.assertEqual(metrics["segments"],3)
