"""Verify equal detections and reproduce published TrackEval metrics."""
import argparse
from collections import Counter
import json
from pathlib import Path

from evaluate import evaluate_tracks, METHODS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("study", type=Path)
    args = parser.parse_args()
    for split in ("development", "holdout"):
        root = args.study / split
        for seq in sorted((root / METHODS[0] / "data").glob("*.txt")):
            expected = None
            for method in METHODS:
                lines = (root / method / "data" / seq.name).read_text().splitlines()
                fields = [line.split() for line in lines]
                detections = Counter(tuple(s[:1] + s[2:]) for s in fields)
                assert len({(s[0], s[1]) for s in fields}) == len(fields)
                if expected is None:
                    expected = detections
                assert detections == expected, (split, seq.name, method)
        actual = evaluate_tracks(root)
        published = json.loads((Path(__file__).parent / (split + ".json")).read_text())
        assert actual == published, split
        print(split, "identical detections and published metrics verified")


if __name__ == "__main__":
    main()
