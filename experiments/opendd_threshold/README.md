# OpenDD 环岛汇入：降速阈值如何影响待复核名单？

从原始 SQLite 轨迹库重算同一环岛 rdb5 的 60 段既有记录。固定车辆配对、不同来向条件、过区时间间隔与连续 1 s 降速窗口，仅改变速度来源及降速阈值。不同来向且过区间隔在 0–2 s 的配对直接入选，其他不同来向配对在任一车辆达到降速阈值时入选。位置差分在相邻时刻的中点计算速度，50 ms 插值网格以记录时间零点为基准。三种速度各自搜索原有观测时段内的最大降速，比较未舍入数值。

## 30 段主分析，88 对车辆

[打开 15 个敏感片段的轨迹与速度图册](gallery/README.md) · [直接查看 rdb5_379 第 6 对车辆](gallery/rdb5_379_6.png)

| 1 s 降速阈值（m/s） | 数据库速度 V | 原始位置差分 | 50 ms 重采样位置差分 |
|---:|---:|---:|---:|
| 0.8 | 14 | 22 | 18 |
| 0.9 | 11 | 20 | 17 |
| 1.0 | 7 | 16 | 13 |
| 1.1 | 7 | 15 | 10 |
| 1.2 | 7 | 13 | 8 |

7 对在全部 15 种设置下入选，其中 6 对满足短间隔条件。另 1 对 rdb5_377 #6 靠降速条件持续入选。15 对随设置改变，66 对始终未入选。以 rdb5_379 #6 为例，两车通过间隔为 3.51 s，按三种方法算得的最大降速分别为 0.9185、1.1077、0.9957 m/s。在 1.0 m/s 阈值下仅原始位置差分将它选入；在 0.9 m/s 下三种方法均选入。该片段适合人工复查速度曲线与轨迹。

另两批分别有 13 对和 14 对车辆，各有 1 对始终入选、1 对随设置改变。三批共 115 对。1.0 m/s 设置下，三批所有逐对决定都与已存档基线相同。本次比较衡量筛选规则对复核工作量的影响。交通风险准确率需要人工标注，跨场地表现需要其他环岛数据。

[逐对峰值与冻结判定](pairs.csv) · [各阈值汇总与三类名单](results.json) · [原环岛项目](https://github.com/dengjilou/roundabout-conflict-mining/tree/481cee2eae3db295ab408d5b41b4c7854888c5ef)

## 从逐对数据复算

从本目录运行，Python 3.10 及以上，仅用标准库。

```bash
python evaluate.py pairs.csv reproduced.json
```

## 从原始 SQLite 再做一次

从本目录执行 Bash 命令。将 DB 设置为本地 OpenDD rdb5 SQLite 的绝对路径。

```bash
set -e
git clone https://github.com/dengjilou/roundabout-conflict-mining.git roundabout
git -C roundabout checkout --detach 481cee2eae3db295ab408d5b41b4c7854888c5ef
mkdir -p audits
for group in development earlier heldout; do
  PYTHONPATH="$PWD/roundabout" python audit_speed_source.py "$DB" "audits/$group.json" --cohort "$group"
done
python prepare.py audits regenerated.csv
python evaluate.py regenerated.csv reproduced.json
```

安装 Matplotlib 与 NumPy 后，可在同一目录从原始数据库生成图册。程序重新计算图中的一秒降速，并逐项核对已发布的逐对峰值。

```bash
PYTHONPATH="$PWD/roundabout" MPLBACKEND=Agg python case_gallery.py "$DB" gallery
```
