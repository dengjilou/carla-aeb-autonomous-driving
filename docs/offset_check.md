# 初始车距对 AEB 介入时刻的影响

同一条 Town04 弯道，自车初速 17 m/s，前车初速 13 m/s，在 1.4 s 开始急刹。唯一改变的场景参数是前车生成时沿路的纵向偏移。每个偏移分别使用原分级策略和 TTC ≤ 2.5 s 提前全力策略，固定种子 42、步长 0.05 s、观测 12 s，各运行两轮。[配置](../configs/offset_check.json) · [两轮原始 CSV 与碰撞记录](offset_check_traces.zip)

| 初始偏移 | 策略 | 首次制动 / s | 首次全力 / s | 首次接触 / s | 最小投影间距 / m |
|---|---|---:|---:|---:|---:|
| 31 m | 原分级 | 2.80 | 3.30 | 4.35 | −0.11 |
| 31 m | 提前全力 | 2.40 | 2.40 | 无 | 6.48 |
| 39 m | 原分级 | 3.20 | 3.85 | 无 | 0.17 |
| 39 m | 提前全力 | 2.85 | 2.85 | 无 | 7.67 |

8 次运行均通过场景有效性检查。两轮的首次制动、全力和接触时刻一致。31 m 复现了冻结实验中的一个工况，39 m 是看到此前结果后追加的诊断。原有独立验证仍计 16 个工况。[冻结实验](independent_aeb.md)

### 从一帧解释触发原因

原策略在 31 m 工况的 2.80 s 帧选中前车，沿路线估计的车身间距为 14.57 m，闭合速度为 8.94 m/s，TTC 约 1.63 s。此时距离已小于 15 m，且处在 5 s 追近窗口内，距离保护先发出部分制动请求。3.30 s 间距降至 9.76 m，触发全力制动。碰撞传感器随后在 4.35 s 报告与前车接触。

39 m 工况中，原策略首次制动帧的车身间距仍有 19.12 m，闭合速度为 12.37 m/s，TTC 约 1.545 s，已越过 1.6 s 部分制动阈值。增大初始偏移同时改变了首次介入时刻和触发条件。该次无接触，但按项目指标计算的最小余量只有 0.17 m。

表中最小值取自 `min_projected_gap_m`，为车身横向投影重合时的纵向投影间距，发生接触时仅统计首次回调前的帧。弯道上的投影近似可以出现负值；碰撞以传感器事件判定。控制器用沿路线的车身间距计算 TTC。两种距离在 CSV 中分别记录。实验使用物理车辆与真值目标，结论限定于上述配置。

### 复现

从仓库根目录使用 Python 3.10 或更新版本即可离线复算。

```bash
python -m zipfile -e docs/offset_check_traces.zip output/offset-audit
python scenarios/audit_offset_check.py output/offset-audit
```

在 README 指定的 CARLA 0.9.13、SUMO 1.18.0 与 Python 3.7 环境中，可在空闲 Town04 重新运行。

```powershell
py -3.7 scenarios/run_longitudinal.py --config configs/offset_check.json --lateral lqr --mode scene --output output/offset-live/a
py -3.7 scenarios/run_longitudinal.py --config configs/offset_check.json --lateral lqr --mode scene --output output/offset-live/b
py -3.12 scenarios/audit_offset_check.py output/offset-live
```
