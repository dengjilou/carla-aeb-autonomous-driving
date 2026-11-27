"""Score both frozen voxel settings against the complete KITTI 04 trajectory."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import figure


def main(dataset, out):
    times = np.loadtxt(dataset / "sequences/04/times.txt")
    intervals = np.diff(times)
    if not np.all(intervals > 0):
        raise ValueError("Expected increasing KITTI timestamps")
    runs = [(f"{voxel} m", out / f"04_{voxel}.csv") for voxel in ("1.0", "0.5")]
    metrics = figure(dataset, "04", runs, out)
    for result in metrics.values():
        if result["frames"] != len(times):
            raise ValueError("Timestamp and pose counts differ")
    result = dict(median_scan_interval_ms=float(1000 * np.median(intervals)), metrics=metrics)
    (out / "metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    main(args.dataset, args.out)
