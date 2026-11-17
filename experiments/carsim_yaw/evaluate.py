"""Fit on DDK CarSim recordings 1–4 and evaluate recordings 5–6."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


def score(prediction, target):
    error = np.abs(prediction - target)
    return {"rmse_rad_s": float(np.sqrt(np.mean(error**2))),
            "p95_abs_rad_s": float(np.percentile(error, 95))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    runs, hashes = {}, {}
    for index in range(1, 7):
        path = args.data / f"data_unit_{index}.mat"
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        data = loadmat(path, variable_names=["data"])["data"]
        if data.shape[1] != 30 or not np.allclose(np.diff(data[:, 0]), 0.01):
            raise ValueError(f"Unexpected schema or sample times in {path}")
        # Time, longitudinal speed, yaw rate and front-wheel angle. SI units.
        runs[index] = data[::5, [0, 4, 6, 27]]
        if not np.isfinite(runs[index]).all():
            raise ValueError(f"Missing or non-finite channel in {path}")

    inputs, targets = [], []
    for index in (1, 2, 3, 4):
        _, speed, yaw_rate, steer = runs[index].T
        inputs.append(np.column_stack((yaw_rate[:-1], (speed * np.tan(steer))[:-1])))
        targets.append(yaw_rate[1:])
    coefficients = np.linalg.lstsq(np.vstack(inputs), np.concatenate(targets), rcond=None)[0]
    feedback, gain = coefficients
    report = {"source_sha256": hashes, "training_files": [1, 2, 3, 4],
              "step_s": 0.05, "horizon_s": 0.5, "coefficients": coefficients.tolist(),
              "holdout": {}}
    errors = [[], []]
    for index in (5, 6):
        time, speed, yaw_rate, steer = runs[index].T
        # Non-overlapping 0.5 s intervals. Only the initial inputs are known.
        starts = np.arange(0, len(time) - 10, 10)
        baseline = yaw_rate[starts]
        estimate = baseline.copy()
        drive = speed[starts] * np.tan(steer[starts])
        for _ in range(10):
            estimate = feedback * estimate + gain * drive
        truth = yaw_rate[starts + 10]
        report["holdout"][str(index)] = {"windows": len(starts),
                                         "persistence": score(baseline, truth),
                                         "model": score(estimate, truth)}
        for collection, prediction in zip(errors, (baseline, estimate)):
            collection.append(prediction - truth)
        worst = int(np.argmax(np.abs(estimate - truth)))
        report["holdout"][str(index)]["worst_window"] = {
            "start_s": float(time[starts[worst]]), "initial_steer_rad": float(steer[starts[worst]]),
            "end_steer_rad": float(steer[starts[worst]+10]), "prediction_rad_s": float(estimate[worst]),
            "actual_rad_s": float(truth[worst])}
    report["pooled"] = {label: score(np.concatenate(samples), 0.0)
                        for label, samples in zip(("persistence", "model"), errors)}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report["pooled"], indent=2))


if __name__ == "__main__":
    main()
