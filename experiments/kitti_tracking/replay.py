"""Render the two frame-level examples for annotated KITTI sequence 0019."""
import argparse
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from evaluate import overlaps


def load(path):
    boxes = defaultdict(dict)
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields[2] == "Car":
            boxes[int(fields[0])][int(fields[1])] = np.array(fields[6:10], float)
    return boxes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", type=Path, help="directory with left-camera PNGs for 0019")
    parser.add_argument("holdout", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    gt = load(args.holdout / "gt/label_02/0019.txt")
    methods = ["greedy", "kalman_hungarian"]
    tracks = [load(args.holdout / method / "data/0019.txt") for method in methods]
    args.output.mkdir(parents=True, exist_ok=True)
    for name, start, end in (("recovery", 560, 569), ("failure", 569, 578)):
        frames = []
        for t in range(start, end + 1):
            raw = cv2.imread(str(args.images / f"{t:06}.png"))
            if raw is None:
                raise FileNotFoundError(f"Missing camera frame {t}")
            box = gt[t][72]
            panels = []
            for label, stream in zip(("Greedy IoU", "Kalman + Hungarian"), tracks):
                overlay = raw.copy()
                a,b,c,d = box.round().astype(int)
                cv2.rectangle(overlay, (a,b), (c,d), (0,215,255), 2)
                status = "no matched detection"
                objects = stream[t]
                if objects:
                    ious = overlaps(box[None], np.array(list(objects.values())))[0]
                    j = int(ious.argmax())
                    if ious[j] >= .5:
                        ident = list(objects)[j]
                        a,b,c,d = objects[ident].round().astype(int)
                        cv2.rectangle(overlay, (a,b), (c,d), (255,220,40), 2)
                        status = f"ID {ident}   IoU {ious[j]:.2f}"
                # Same native-pixel crop for both methods throughout both clips.
                crop = overlay[95:315, 310:750]
                panel = cv2.copyMakeBorder(crop, 66, 0, 0, 0, cv2.BORDER_CONSTANT, value=(34,32,25))
                cv2.putText(panel, label, (12,26), 0, .7, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(panel, status, (12,52), 0, .6, (255,220,40), 1, cv2.LINE_AA)
                panels.append(panel)
            pair = cv2.copyMakeBorder(np.hstack(panels), 0, 62, 0, 0, cv2.BORDER_CONSTANT, value=(34,32,25))
            cv2.putText(pair, f"KITTI 0019 | frame {t} | yellow: GT 72 | cyan: matched box", (12,311),
                        0, .62, (225,240,255), 1, cv2.LINE_AA)
            cv2.putText(pair, "Camera: KITTI / Geiger et al. | cropped + annotated | CC BY-NC-SA 3.0",
                        (12,336), 0, .51, (215,215,215), 1, cv2.LINE_AA)
            frames.append(Image.fromarray(cv2.cvtColor(pair, cv2.COLOR_BGR2RGB)))
        frames[0].save(args.output / (name + ".gif"), save_all=True, append_images=frames[1:],
                       duration=550, loop=0, disposal=2)


if __name__ == "__main__":
    main()
