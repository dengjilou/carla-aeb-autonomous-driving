"""Render the 15 threshold-sensitive pairs from the original rdb5 observations."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from audit_speed_source import position_speed_segments
from mine import ROOT, find_visits, mine, read_frames, speed_drop
from prepare import METHODS

HERE = Path(__file__).resolve().parent
COLORS = ("#143e65", "#bd5326", "#18806e")


def split_gaps(points, gap):
    groups = []
    for pt in points:
        if not groups or pt[0] - groups[-1][-1][0] > gap:
            groups.append([])
        groups[-1].append(pt)
    return groups


def render(frames, event, zone, rules, record, frozen, votes, output):
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), layout="constrained")
    map_ax = axes[0]
    map_ax.add_patch(Circle((0, 0), zone["radius"], color="#8295a3", alpha=.3))
    maxima = {method: [] for method in METHODS}
    for index, role in enumerate(("first", "second"), 1):
        obj = event[role]
        enter = event[role + "_enter"]
        gate = event[role + "_gate_s"]
        start = (enter if gate is None else gate) - rules["speed"]["pre_gate_s"]
        track = [(t, f[obj]) for t, f in frames.items() if obj in f]
        xy = [(t, p[0], p[1]) for t, p in track]
        clipped = [(t, x-zone["x"], y-zone["y"]) for t, x, y in xy if start <= t <= event[role+"_leave"]]
        for i, seg in enumerate(split_gaps(clipped, rules["max_frame_gap_s"])):
            t, x, y = zip(*seg)
            color = ("#89324f", "#397a94")[index-1]
            map_ax.plot(x, y, color=color, label=f"{role} ({obj})" if i == 0 else None)
            if i == 0:
                map_ax.scatter(x[0], y[0], marker="o", color=color, s=14)
        native = [(t, p[5]) for t, p in track if start <= t <= enter and p[5] is not None and math.isfinite(p[5])]
        ax = axes[index]
        for method, color in zip(METHODS, COLORS):
            segments = (split_gaps(native, rules["max_frame_gap_s"]) if method == "V" else
                        position_speed_segments(xy, start, enter, rules, method == "xy_50ms"))
            best = None
            for i, seg in enumerate(segments):
                t, v = np.asarray(seg).T
                ax.plot(t-enter, v, color=color, linewidth=1.15, label=method if i == 0 else None)
                peak = speed_drop(seg, rules["speed"]["drop_window_s"], rules["max_frame_gap_s"])
                if peak and (best is None or peak["drop_mps"] > best[0]["drop_mps"]):
                    best = peak, t, v
            if best is not None:
                peak, t, v = best
                ends = np.array([peak["start_s"], peak["end_s"]])
                ax.scatter(ends-enter, np.interp(ends, t, v), color=color, s=15)
                maxima[method].append(peak["drop_mps"])
        ax.set(title=f"{role.capitalize()} vehicle {obj}", xlabel="Seconds relative to own zone entry", ylabel="Speed (m/s)")
        ax.legend(fontsize=8)
        ax.grid(alpha=.2)
    values = []
    for method in METHODS:
        value = max(maxima[method], default=None)
        expected = float(frozen[method]) if frozen[method] else None
        if value != expected:
            raise ValueError(f"Peak mismatch for {record} #{event['id']} {method}: {value} vs {expected}")
        values.append(value)
    map_ax.set(title="Paths near the review zone", xlabel="Local east (m)", ylabel="Local north (m)", aspect="equal")
    map_ax.legend(fontsize=8)
    map_ax.grid(alpha=.2)
    fig.suptitle(f"OpenDD {record}  |  pair #{event['id']}  |  gap {event['pet_s']:.2f} s  |  selected {votes}/15")
    fig.savefig(output, dpi=125)
    plt.close(fig)
    return values


def main(database, output):
    output.mkdir(parents=True, exist_ok=True)
    votes = json.loads((HERE/"results.json").read_text())["heldout"]["sometimes"]
    config = json.loads((ROOT/"zones.json").read_text(encoding="utf-8"))
    zone, rules = config["sites"]["rdb5"], config["rules"]
    with (HERE/"pairs.csv").open() as f:
        rows = {(r["recording"], r["event"]): r for r in csv.DictReader(f) if r["cohort"] == "heldout"}
    table = []
    for rec in sorted({k.split("#")[0] for k in votes}):
        _, frames, classes = read_frames(database, rec)
        events = mine(frames, find_visits(frames, classes, zone, rules["max_frame_gap_s"]), rules)
        for event in events:
            num = str(event["id"])
            key = rec + "#" + num
            if key not in votes:
                continue
            file = rec + "_" + num + ".png"
            vals = render(frames, event, zone, rules, rec, rows[rec, num], votes[key], output/file)
            numbers = " | ".join("无完整窗口" if v is None else f"{v:.4f}" for v in vals)
            table.append(f"| [{rec} #{num}]({file}) | {votes[key]}/15 | {numbers} |")
    if len(table) != len(votes):
        raise ValueError("Missing review image")
    header = """# OpenDD 环岛人工复核图册

收录 rdb5 主分析中全部 15 对随速度算法或降速阈值改变入选状态的车辆。点击编号可查看两车的局部轨迹与速度。地图圆圈是观测区，轨迹起点用圆点标记。两张速度图各以该车进区时间为零点，各色圆点标出对应算法的最大一秒降速窗口。表中降速取两车最大值，单位 m/s。筛选使用未舍入值。

**示例**：[rdb5_379 #6](rdb5_379_6.png) 的过区间隔为 3.51 s。阈值为 1.0 m/s 时，仅原始位置差分入选。50 ms 重采样后所得 0.9957 m/s 刚好低于阈值。此图用于复核筛选依据，缺少人工风险标签。

| 片段 | 入选设置数 | 数据库速度 | 原始位置差分 | 50 ms 重采样差分 |
|---|---:|---:|---:|---:|
"""
    (output/"README.md").write_text(header+"\n".join(table)+"\n", encoding="utf-8")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("database", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    main(args.database, args.output)
