"""Pair the same 100 m windows across published 1 m voxel ICP runs."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from evaluate import evaluate, segments_100m


def main(dataset, runs):
    out = runs / "initial-guesses"
    out.mkdir(exist_ok=True)
    summary = {}
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), layout="constrained")
    with (out / "segments.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(("sequence", "start", "end", "identity_m", "motion_m", "motion_minus_identity_m"))
        for seq, ax in zip(("04", "05", "07"), axes):
            matches = {}
            metrics = {}
            for mode in ("identity", "motion"):
                path = runs / f"{seq}_{mode}.csv"
                if seq == "04":
                    name = "04_1.0.csv" if mode == "identity" else "04_motion_1.0.csv"
                    path = runs / "sequence-04" / name
                metrics[mode], gt, est, *_ = evaluate(dataset, seq, path)
                _, segments = segments_100m(gt, est)
                matches[mode] = {(i, j): drift for i, j, drift in segments}
            baseline, motion = matches["identity"], matches["motion"]
            if baseline.keys() != motion.keys():
                raise ValueError(f"Unpaired windows in sequence {seq}")
            frames = list(baseline)
            delta = np.array([motion[k] - baseline[k] for k in frames])
            for (i, j), diff in zip(frames, delta):
                writer.writerow((seq, i, j, baseline[i, j], motion[i, j], diff))
            summary[seq] = dict(metrics=metrics, improved=int(sum(delta < -1e-6)),
                                worse=int(sum(delta > 1e-6)), tied=int(sum(abs(delta) <= 1e-6)),
                                median_delta_m=float(np.median(delta)))
            ax.axhline(0, color="#555555", linewidth=.8)
            starts = [i for i, _ in frames]
            ax.fill_between(starts, delta, 0, where=delta <= 0, interpolate=True, color="#198675", alpha=.65)
            ax.fill_between(starts, delta, 0, where=delta >= 0, interpolate=True, color="#c46336", alpha=.7)
            ax.plot(starts, delta, color="#273a43", linewidth=1)
            ax.set(title=f"KITTI {seq}   |   {len(frames)} paired windows", ylabel="Difference (m)")
            ax.grid(alpha=.17)
    axes[-1].set_xlabel("Starting frame of the 100 m window")
    fig.suptitle("Drift with previous-motion initial guess minus identity\nBelow zero = lower drift with previous motion")
    fig.savefig(out / "paired_drift.png", dpi=145)
    plt.close(fig)
    (out / "metrics.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("runs", type=Path)
    args = parser.parse_args()
    main(args.dataset, args.runs)
