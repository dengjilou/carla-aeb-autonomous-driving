"""Compare online vehicle ID assignment using identical KITTI Regionlets boxes."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from zipfile import ZipFile

import numpy as np
from scipy.optimize import linear_sum_assignment


# Frame counts from the training left-camera archive, including empty frames.
LENGTHS = [154,447,233,144,314,297,270,800,390,803,294,373,78,340,106,376,209,145,339,1059,837]
METHODS = ("greedy", "static_hungarian", "kalman_hungarian")
ABLATIONS = ("greedy_gap2", "static_hungarian_gap0")
TRANSITION = np.eye(4)
TRANSITION[:2, 2:] = np.eye(2)


def overlaps(left, right):
    left, right = left[:, None], right[None, :]
    intersection = np.maximum(0, np.minimum(left[..., 2:], right[..., 2:]) -
                              np.maximum(left[..., :2], right[..., :2])).prod(axis=2)
    union = (left[..., 2:] - left[..., :2]).prod(axis=2) + (right[..., 2:] - right[..., :2]).prod(axis=2) - intersection
    return intersection / union


class Track:
    def __init__(self, ident, box, frame):
        self.ident, self.last = ident, frame
        self.size = box[2:] - box[:2]
        self.state = np.r_[(box[2:] + box[:2]) / 2, 0., 0.]
        self.cov = np.diag([4., 4., 100., 100.])

    def predict(self, motion):
        if motion:
            self.state = TRANSITION @ self.state
            self.cov = TRANSITION @ self.cov @ TRANSITION.T + np.diag([1., 1., 4., 4.])
        return np.r_[self.state[:2] - self.size / 2, self.state[:2] + self.size / 2]

    def update(self, box, frame, motion):
        center = (box[2:] + box[:2]) / 2
        self.last, self.size = frame, box[2:] - box[:2]
        if motion:
            gain = np.linalg.solve(self.cov[:2, :2] + 4 * np.eye(2), self.cov[:, :2].T).T
            self.state += gain @ (center - self.state[:2])
            self.cov -= gain @ self.cov[:2, :]
        else:
            self.state[:2] = center


def associate(sim, greedy):
    eligible = sim >= .3
    if greedy:
        used_rows, used_cols = set(), set()
        for i, j in sorted(zip(*np.nonzero(eligible)), key=lambda ij: -sim[ij]):
            if i not in used_rows and j not in used_cols:
                used_rows.add(i)
                used_cols.add(j)
                yield i, j
    else:
        # Maximize the number of eligible matches, then their total IoU.
        rows, cols = linear_sum_assignment(np.where(eligible, min(sim.shape) + 1 + sim, 0), maximize=True)
        yield from ((i, j) for i, j in zip(rows, cols) if eligible[i, j])


def track(frames, length, method, output):
    active, next_id = [], 0
    greedy = method in ("greedy", "greedy_gap2")
    gap = 0 if method in ("greedy", "static_hungarian_gap0") else 2
    motion = method == "kalman_hungarian"
    with output.open("w", encoding="ascii") as out:
        for frame in range(length):
            detections = frames[frame]
            boxes = np.array([d[:4] for d in detections]).reshape(-1, 4)
            active = [t for t in active if frame - t.last <= gap + 1]
            predicted = np.array([t.predict(motion) for t in active]).reshape(-1, 4)
            assigned = {}
            if len(predicted) and len(boxes):
                for i, j in associate(overlaps(predicted, boxes), greedy):
                    active[i].update(boxes[j], frame, motion)
                    assigned[j] = active[i].ident
            for j, (x1, y1, x2, y2, score) in enumerate(detections):
                if j not in assigned:
                    active.append(Track(next_id, boxes[j], frame))
                    assigned[j] = next_id
                    next_id += 1
                print(frame, assigned[j], "Car -1 -1 -10", x1, y1, x2, y2,
                      "-1 -1 -1 -1000 -1000 -1000 -10", score, file=out)


def metrics(hota, clear):
    return {**{k: round(100 * float(np.mean(hota[k])), 4) for k in ("HOTA", "DetA", "AssA")},
            **{k: int(clear[k]) for k in ("IDSW", "CLR_FP", "CLR_FN", "CLR_TP")}}


def evaluate_tracks(root, methods=METHODS):
    import trackeval
    dataset = trackeval.datasets.Kitti2DBox(dict(GT_FOLDER=str(root / "gt"), TRACKERS_FOLDER=str(root),
        TRACKERS_TO_EVAL=list(methods), CLASSES_TO_EVAL=["car"], PRINT_CONFIG=False))
    hota, clear = trackeval.metrics.HOTA(), trackeval.metrics.CLEAR(dict(PRINT_CONFIG=False))
    result = {}
    for method in methods:
        hs, cs = {}, {}
        for seq in dataset.seq_list:
            data = dataset.get_preprocessed_seq_data(dataset.get_raw_seq_data(method, seq), "car")
            hs[seq], cs[seq] = hota.eval_sequence(data), clear.eval_sequence(data)
        result[method] = {seq: metrics(hs[seq], cs[seq]) for seq in dataset.seq_list}
        result[method]["combined"] = metrics(hota.combine_sequences(hs), clear.combine_sequences(cs))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path, help="folder containing labels.zip and regionlets.zip")
    parser.add_argument("output", type=Path)
    parser.add_argument("split", choices=("development", "holdout"))
    parser.add_argument("--factorial", action="store_true", help="Include two matching / max-gap controls")
    args = parser.parse_args()
    methods = METHODS + ABLATIONS if args.factorial else METHODS
    ids = range(15) if args.split == "development" else range(15, 21)
    root = args.output / args.split
    gt = root / "gt"
    (gt / "label_02").mkdir(parents=True, exist_ok=True)
    with ZipFile(args.data / "labels.zip") as labels, ZipFile(args.data / "regionlets.zip") as det:
        for seq in ids:
            name = f"{seq:04}.txt"
            (gt / "label_02" / name).write_bytes(labels.read("training/label_02/" + name))
            frames = defaultdict(list)
            for line in det.read("training/det_02/" + name).splitlines():
                fields = line.split()
                if fields[2].lower() == b"car" and float(fields[-1]) >= 0:
                    box = list(map(float, fields[6:10]))
                    assert box[2] > box[0] and box[3] > box[1] and 0 <= int(fields[0]) < LENGTHS[seq]
                    frames[int(fields[0])].append(box + [float(fields[-1])])
            for method in methods:
                folder = root / method / "data"
                folder.mkdir(parents=True, exist_ok=True)
                track(frames, LENGTHS[seq], method, folder / name)
    (gt / "evaluate_tracking.seqmap.training").write_text(
        "".join(f"{i:04} empty 0 {LENGTHS[i]}\n" for i in ids), encoding="ascii")
    result = evaluate_tracks(root, methods)
    (root / "metrics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print({m: result[m]["combined"] for m in methods})


if __name__ == "__main__":
    main()
