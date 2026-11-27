# AEB 仿真 CAN 报文与时序验证

从已有 CARLA 的两轮 brake_0 分级策略与提前全力策略日志生成四份离线 candump 文本。每份 240 帧，周期 50 ms，标准帧 ID 0x500。数据库为本项目自定义 DBC。解码器仅读取 DBC 和报文。这个实验没有连接车辆、CAN 适配器或 CANoe。

[实测结果 JSON](results.json) · [教学 DBC](aeb.dbc) · [编码与离线时序检查](replay.py) · [pytest 用例](test_replay.py) · [CARLA 源日志](../../docs/independent_aeb_traces.zip)

原始 CSV 的 brake 是仲裁后的制动，aeb_brake 是 AEB 请求。普通制动使用项目原 SpeedPID、日志参考车速和自车速度重算；编码前逐帧验证它与锁定制动取最大值后等于原始 brake。DBC 把车速和归一化制动取整到 0.01，验证按半个量化单位容差比较。

| 两轮各一份日志 | 首次 AEB 请求 | 正常时序告警 |
|---|---:|---:|
| 分级策略 | 2.80 s | 0 |
| 提前全力 | 2.40 s | 0 |

**人工复算。** 分级策略第 56 帧（从零计）车速 16.177896 m/s，四舍五入成 1618，即 0x0652，小端存为 52 06。普通制动为 0，AEB 与最终制动均为 0.40，后两者各存 0x28。第六字节高半字节是等级 2，低半字节是计数器 8。完整载荷为 `52 06 00 28 28 28 00 00`。

| 在副本中构造的异常 | 首次可从下一条或重复报文观测到的时间 | 原因 |
|---|---:|---|
| 移除 450 ms 报文 | 500 ms | 计数器跳号 |
| 同时刻重复第 9 帧 | 450 ms | 重复报文 |
| 移除 450、500、550 ms 报文 | 600 ms | 超时及计数器跳号 |
| 末尾移除 3 帧，最后一帧为 11800 ms | 独立观测至 11950 ms | 末尾静默达到 150 ms |

相邻报文间隔达到 150 ms 时记录恢复时刻。连续 15 → 0 属正常回绕。对末尾静默，另用完整源日志的最后时刻作为故障注入实验的观测终点。观测至 11949 ms 时只过去 149 ms；至 11950 ms 时产生一次末尾超时。若观测延至 12000 ms，仍只记录一次，同时保留阈值到达时刻 11950 ms 和实际检查时刻 12000 ms。省略观测终点时，只检查日志中可见的报文间隔。

在仓库根目录使用 Windows PowerShell 和 Python 3.12 复跑。

```powershell
py -3.12 -m venv tmp/can-env
tmp/can-env/Scripts/python -m pip install cantools==45.2.0 pytest==9.2.0
tmp/can-env/Scripts/python experiments/can_aeb/replay.py output/can-aeb
tmp/can-env/Scripts/python experiments/can_aeb/replay.py --decode output/can-aeb/drop.log
tmp/can-env/Scripts/python experiments/can_aeb/replay.py --decode output/can-aeb/terminal.log --end-ms 11950
tmp/can-env/Scripts/python -m pytest -q experiments/can_aeb/test_replay.py
```

本机在 Ubuntu 24.04 和 Windows 的 Python 3.12 环境分别运行，13 项 pytest 均通过。
