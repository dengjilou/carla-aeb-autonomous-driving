"""Count review candidates and report individual pairs stable across thresholds and speed methods."""
import argparse
import csv
import json
from pathlib import Path

from prepare import METHODS

THRESHOLDS = (0.8, 0.9, 1.0, 1.1, 1.2)


def evaluate(rows):
    result = {}
    for row in rows:
        cohort = result.setdefault(row["cohort"], {"counts": {}, "events": {}})
        key = f'{row["recording"]}#{row["event"]}'
        if key in cohort["events"]:
            raise ValueError(f"Repeated event {key}")
        votes = 0
        for t in THRESHOLDS:
            counts = cohort["counts"].setdefault(str(t), dict.fromkeys(METHODS, 0))
            for method in METHODS:
                drop = row[method]
                hit = row["different_approaches"] == "1" and (
                    row["short_gap"] == "1" or (drop != "" and float(drop) >= t))
                if t == 1.0 and hit != bool(int(row["review_" + method])):
                    raise ValueError(f"Frozen 1.0 m/s decision differs for {key} and {method}")
                counts[method] += hit
                votes += hit
        cohort["events"][key] = votes
    for cohort in result.values():
        votes = cohort.pop("events")
        total = len(METHODS) * len(THRESHOLDS)
        cohort["pairs"] = len(votes)
        cohort["always"] = [key for key, v in votes.items() if v == total]
        cohort["sometimes"] = {key: v for key, v in votes.items() if 0 < v < total}
        cohort["never"] = [key for key, v in votes.items() if v == 0]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pairs", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.pairs.open(encoding="utf-8", newline="") as stream:
        summary = evaluate(csv.DictReader(stream))
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
