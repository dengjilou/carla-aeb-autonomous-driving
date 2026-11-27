"""Reduce the three speed audit outputs to one row per paired event."""
import argparse
import csv
import json
from pathlib import Path

METHODS = ("V", "native_xy", "xy_50ms")


def prepare(folder, output):
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cohort", "recording", "event", "different_approaches", "short_gap",
                         *METHODS, *["review_" + m for m in METHODS]])
        for cohort in ("development", "earlier", "heldout"):
            source = json.loads((folder / f"{cohort}.json").read_text(encoding="utf-8"))
            for row in source["rows"]:
                drops = [max((p["drop_mps"] for p in row["peaks"][m] if p is not None),
                             default=None) for m in METHODS]
                writer.writerow([cohort, row["recording"], row["id"],
                                 int(row["path"] == "entry_and_circulating"), int(row["time_hit"]),
                                 *drops, *[int(row["review"][m]) for m in METHODS]])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audits", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    prepare(args.audits, args.output)
