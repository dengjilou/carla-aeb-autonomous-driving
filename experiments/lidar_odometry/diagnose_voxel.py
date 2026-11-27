"""Compare voxel sizes on two previously inspected KITTI 100 m windows."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from evaluate import transforms


def traveled(steps):
    return np.r_[0, np.linalg.norm(steps, axis=1).cumsum()]


def analyze(dataset, seq, frames, csv_path):
    rows = np.genfromtxt(csv_path, delimiter=",", names=True)
    if not np.array_equal(rows["frame"], frames):
        raise ValueError(f"Missing or unexpected frames in {csv_path}")
    calib = (dataset / "sequences" / seq / "calib.txt").read_text().splitlines()
    extrinsic = next(line[3:] for line in calib if line.startswith("Tr:"))
    tr = transforms(np.fromstring(extrinsic, sep=" ")[None, :])[0]
    pose_columns = [rows[f"T{r}{c}"] for r in range(3) for c in range(4)]
    estimate = tr @ transforms(np.column_stack(pose_columns)) @ np.linalg.inv(tr)
    truth = transforms(np.loadtxt(dataset / "poses" / f"{seq}.txt")[frames])
    truth = np.linalg.inv(truth[0]) @ truth

    true_step = (np.linalg.inv(truth[:-1]) @ truth[1:])[:, :3, 3]
    est_step = (np.linalg.inv(estimate[:-1]) @ estimate[1:])[:, :3, 3]
    length = np.linalg.norm(true_step, axis=1)
    if np.any(length < 1e-6):
        raise ValueError("Stationary step has no travel direction")
    direction = true_step / length[:, None]
    error = est_step - true_step
    along = (error * direction).sum(axis=1)
    across = error - along[:, None] * direction
    true_distance, est_distance = traveled(true_step), traveled(est_step)
    result = dict(
        start=int(frames[0]), end=int(frames[-1]),
        true_distance_m=float(true_distance[-1]), estimated_distance_m=float(est_distance[-1]),
        endpoint_drift_m=float(np.linalg.norm((np.linalg.inv(estimate[-1]) @ truth[-1])[:3, 3])),
        along_mean_m=float(along.mean()), along_rms_m=float(np.sqrt(np.mean(along**2))),
        perpendicular_rms_m=float(np.sqrt(np.mean((across**2).sum(axis=1)))),
        underestimated_steps=int(np.sum(along < 0)), steps=len(along),
        unconverged=int(sum(rows["converged"][1:] == 0)),
        median_points=float(np.median(rows["points"])),
        registration_p95_ms=float(np.percentile(rows["registration_ms"][1:], 95)),
    )
    if seq == "07":
        i = int(np.flatnonzero(frames[1:] == 810)[0])
        result["step_809_to_810"] = dict(true_m=float(length[i]),
            estimate_along_m=float(length[i] + along[i]), along_error_m=float(along[i]),
            perpendicular_error_m=float(np.linalg.norm(across[i])),
            converged=bool(rows["converged"][i+1]))
    return result, true_distance, est_distance


def main(dataset, folder):
    result = {}
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for axis, (seq, first, last) in zip(axes, [("05", 2060, 2185), ("07", 780, 871)]):
        frames = np.arange(first, last + 1)
        result[seq] = {}
        for voxel, color in [("1.0", "#a34926"), ("0.5", "#137a77")]:
            metrics, truth, estimate = analyze(dataset, seq, frames, folder / f"{seq}_{voxel}.csv")
            result[seq][voxel] = metrics
            if voxel == "1.0":
                axis.plot(frames, truth, "--", color="#333333", label="KITTI truth")
            axis.plot(frames, estimate, color=color, label=f"{voxel} m voxel")
        axis.set(xlabel="KITTI frame", title=f"Sequence {seq}  |  frames {first} to {last}")
        axis.grid(alpha=.2)
        axis.legend()
    axes[0].set_ylabel("Cumulative distance traveled (m)")
    fig.tight_layout()
    fig.savefig(folder / "distance.png", dpi=160)
    plt.close(fig)
    (folder / "metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    main(args.dataset, args.folder)
