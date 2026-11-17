# 基于公开 CarSim 轨迹的横摆角速度预测

给定当前横摆角速度、纵向速度和前轮转角，预测 0.5 s 后的横摆角速度。数据取自 Xiao 等人 ICRA 2023 DDK 项目的公开 CarSim 记录。本项目实现数据读取、模型辨识、留出评测和误差分析；原始工况及仿真由数据作者生成。[原始数据与字段定义](https://github.com/yongqianxiao/DDK/tree/a2b08c0d84bef1df4b8d1d5bf9b5bf2a56cc57a7/CarsimDatasets)

**划分与预测方法。** 使用编号 1–4 的完整记录拟合，5–6 两条完整记录仅用于评估。按作者字段定义读取时间、纵向速度、横摆角速度和前轮转角，从 10 ms 抽样到 50 ms。最小二乘拟合一阶离散模型 `r[k+1] = a*r[k] + b*vx[k]*tan(delta[k])`，得到 a=0.8535、b=0.05149。预测时只读窗口起点的三个输入，固定速度和转角递推 10 步。基线直接沿用起点横摆角速度。两者对相同的未来端点评分。

| 留出记录 | 0.5 s 窗口数 | 基线 RMSE | 模型 RMSE | 基线绝对误差 P95 | 模型绝对误差 P95 |
|---|---:|---:|---:|---:|---:|
| 5 | 900 | 0.01351 | 0.01237 | 0.02251 | 0.02411 |
| 6 | 1,000 | 0.01255 | 0.01111 | 0.02050 | 0.02101 |
| 合并 | 1,900 | 0.01301 | 0.01172 | 0.02167 | 0.02285 |

单位为 rad/s。窗口首尾相接，统计量由所有预测端点直接计算。合并 RMSE 下降约 9.9%，绝对误差 P95 上升约 5.4%。例如记录 6 从 239.51 s 开始的窗口，前轮转角从 0.00324 增至 0.05438 rad；模型按初始转角预测 0.01081 rad/s，实际达到 0.13851 rad/s。窗口内转向变化是起点固定输入假设的局限。这里仅检验同一公开数据源的两条留出记录，未校验跨车型、跨道路或实车表现。[完整指标、失败窗口与文件哈希](results.json)

### 复现

从仓库根目录运行，Python 环境需有 NumPy、SciPy。原始 MAT 文件从作者固定版本下载到本机，仓库仅发布分析程序与派生指标。

```powershell
$source = 'https://raw.githubusercontent.com/yongqianxiao/DDK/a2b08c0d84bef1df4b8d1d5bf9b5bf2a56cc57a7/CarsimDatasets'
New-Item -ItemType Directory -Force output/carsim-ddk | Out-Null
foreach ($i in 1..6) {
    Invoke-WebRequest "$source/data_unit_$i.mat" -OutFile "output/carsim-ddk/data_unit_$i.mat"
}
python experiments/carsim_yaw/evaluate.py output/carsim-ddk output/carsim-ddk/results.json
```
