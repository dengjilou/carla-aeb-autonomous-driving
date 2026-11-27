"""Compare the speed timelines of 72 fixed-input CARLA and co-simulation runs."""
import argparse
import csv
import hashlib
import io
import itertools
import json
import math
import zipfile
from pathlib import Path


def compare(archive):
    summary = json.loads(archive.read("summary.json"))
    config = summary["config"]
    step, trigger = config["step_s"], config["trigger_s"]
    groups = {}
    for name, run in summary["runs"].items():
        c = run["case"]
        key = (c["speed_mps"], c["brake"], c["substep_s"], int(name.rsplit("_r", 1)[1]))
        data = archive.read(name + ".csv")
        if hashlib.sha256(data).hexdigest() != run["metadata"]["trace_sha256"]:
            raise ValueError(f"Trace hash mismatch: {name}")
        pair = groups.setdefault(key, {})
        if c["backend"] in pair:
            raise ValueError(f"Duplicate backend: {key}")
        pair[c["backend"]] = list(csv.DictReader(io.StringIO(data.decode())))
    expected = {(c["speed_mps"], c["brake"], c["substep_s"], r)
                for c, r in itertools.product(config["cases"], range(config["repeats"]))}
    if groups.keys() != expected:
        raise ValueError("Configuration coverage mismatch")
    results = []
    for (speed, brake, substep, repeat), pair in sorted(groups.items()):
        solo, joint = pair["carla"], pair["cosim"]
        if len(solo) != round(config["duration_s"] / step) + 1:
            raise ValueError("Incomplete observation")
        before, after = [], []
        first = None
        for i, (a, b) in enumerate(zip(solo, joint, strict=True)):
            time = float(a["time_s"])
            if abs(time-i*step) > 1e-6 or time != float(b["time_s"]):
                raise ValueError("Mismatched clocks")
            request = brake if time >= trigger else 0.0
            if any(abs(float(row["requested_brake"])-request) > 1e-6 for row in (a, b)):
                raise ValueError("Mismatched brake requests")
            gap = abs(float(a["speed_mps"]) - float(b["speed_mps"]))
            if not math.isfinite(gap):
                raise ValueError("Invalid speed")
            if first is None and gap > 1e-6:
                first = time
            (before if time < trigger else after).append((gap, time))
        peak, peak_time = max(before + after, key=lambda p: p[0])
        results.append(dict(speed_mps=speed, brake=brake, substep_ms=1000*substep, repeat=repeat,
                            first_diff_s=first, peak_mps=peak, peak_at_s=peak_time if peak else None,
                            pre_request_peak_mps=max(before)[0], from_request_peak_mps=max(after)[0]))
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("archive", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    with zipfile.ZipFile(args.archive) as z:
        rows = compare(z)
    with args.output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
