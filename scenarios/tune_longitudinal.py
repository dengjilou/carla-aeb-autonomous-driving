"""Evaluate a fixed development-only grid and select by safety, error, jerk."""
import json
import subprocess
import sys
from itertools import product
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    results = []
    output = ROOT / "output" / "acc-tuning"
    output.mkdir(parents=True, exist_ok=True)
    for gap, relative in product((0.1, 0.2, 0.3), (0.4, 0.8, 1.2)):
        directory = output / ("g{}_r{}".format(gap, relative))
        existing = directory / "summary.json"
        report = json.loads(existing.read_text(encoding="utf-8")) if existing.exists() else {}
        complete = len(report.get("runs", {})) == 2 and all("metrics" in r for r in report["runs"].values())
        if not complete:
            subprocess.run([sys.executable, str(ROOT / "scenarios/run_longitudinal.py"),
                        "--split", "development", "--mode", "acc", "--gap-gain", str(gap),
                        "--relative-gain", str(relative), "--output", str(directory)], check=True)
        report = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
        metrics = [run["metrics"] for run in report["runs"].values()]
        violations = sum(not m["valid"] or m["collided"] or m["first_aeb_s"] is not None
                         or m["min_projected_gap_m"] is None or m["min_projected_gap_m"] < 2
                         or m["steady_gap_rmse_m"] is None for m in metrics)
        rank = [violations, sum(m["steady_gap_rmse_m"] or 0 for m in metrics),
                sum(m["jerk_rms_mps3"] or 0 for m in metrics), gap, relative]
        results.append(dict(gap_gain=gap, relative_gain=relative, rank=rank))
        (output / "grid.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    selected = min(results, key=lambda r: r["rank"])
    (output / "selected.json").write_text(json.dumps(selected, indent=2) + "\n", encoding="utf-8")
    print("Selected:", selected)
    if selected["rank"][0]:
        raise SystemExit("No candidate satisfies development safety/validity constraints")


if __name__ == "__main__":
    main()
