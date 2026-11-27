"""Replay local translations and rotations separately on KITTI 04 frames 150–216."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from evaluate import evaluate


def replay(translations, rotations):
    poses = [np.eye(4)]
    for xyz, rotation in zip(translations, rotations, strict=True):
        step = np.eye(4)
        step[:3, 3] = xyz
        step[:3, :3] = rotation
        poses.append(poses[-1] @ step)
    return np.stack(poses)


def diagnose(truth, estimate):
    truth = np.linalg.inv(truth[0]) @ truth
    estimate = np.linalg.inv(estimate[0]) @ estimate
    steps = np.linalg.inv(estimate[:-1]) @ estimate[1:]
    reference = np.linalg.inv(truth[:-1]) @ truth[1:]
    t, r = steps[:, :3, 3], steps[:, :3, :3]
    gt, gr = reference[:, :3, 3], reference[:, :3, :3]
    np.testing.assert_allclose(replay(t, r), estimate, atol=1e-7)
    np.testing.assert_allclose(replay(gt, gr), truth, atol=1e-7)
    paths = {"Original": estimate, "True rotations": replay(t, gr),
             "True translations": replay(gt, r)}
    errors = {name: np.linalg.norm(p[:, :3, 3] - truth[:, :3, 3], axis=1)
              for name, p in paths.items()}
    length = np.linalg.norm(gt, axis=1)
    along = np.sum(t * gt, axis=1) / length
    # Camera 0 forward is +Z; project the final forward vector onto the start XZ plane.
    headings = [np.arctan2(p[-1, 0, 2], p[-1, 2, 2]) for p in (estimate, truth)]
    delta = headings[0] - headings[1]
    metrics = dict(endpoint_error_m={name: float(e[-1]) for name, e in errors.items()},
                   gt_distance_m=float(length.sum()), estimated_distance_m=float(np.linalg.norm(t, axis=1).sum()),
                   sum_local_forward_shortfall_m=float((length - along).sum()),
                   undertravel_steps=int(np.sum(along < length)), steps=len(t),
                   end_heading_error_deg=float(np.degrees(np.arctan2(np.sin(delta), np.cos(delta)))))
    return metrics, errors


def main(dataset, out):
    configs = [("Identity / 1.0 m", "04_1.0.csv"), ("Identity / 0.5 m", "04_0.5.csv"),
               ("Previous motion / 1.0 m", "04_motion_1.0.csv")]
    frames = np.arange(150, 217)
    fig, axes = plt.subplots(1, 3, sharey=True, figsize=(11, 3.7))
    result = {}
    for axis, (label, name) in zip(axes, configs):
        full, truth, estimate, *_ = evaluate(dataset, "04", out / name)
        window, errors = diagnose(truth[frames], estimate[frames])
        result[label] = {"full": full, "frames_150_216": window}
        for (kind, curve), style in zip(errors.items(), ("-", "--", ":")):
            axis.plot(frames, curve, style, label=kind, linewidth=2)
        axis.set(title=label, xlabel="KITTI frame")
        axis.grid(alpha=.22)
    axes[0].set_ylabel("Position error from frame 150 (m)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3)
    fig.tight_layout(rect=(0, .12, 1, 1))
    fig.savefig(out / "translation_rotation.png", dpi=150)
    plt.close(fig)
    (out / "translation_rotation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    main(args.dataset, args.out)
