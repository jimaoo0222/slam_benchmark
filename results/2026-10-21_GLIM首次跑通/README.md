# 2026-10-21 GLIM 首次跑通

> 目标：在 MID-360s 的数据上跑通 GLIM，验证其轨迹质量。

## 结果

| 项 | 值 |
|---|---|
| 包 | `rosbag/my_static_map1`（PointCloud2，142.2 s）|
| 算法 | GLIM 1.2.2（CPU 模式，无 CUDA）|
| 位姿数 | 1391 |
| 时长 | 139.0 s |
| 路径长 | 16.55 m |
| 跨度 | x 1.83 / y 6.46 / z 0.09 m |
| ① 稳定性 | **8.7 mm** |
| ④ 闭合误差 | 76.1 mm |
| 运行耗时 | 约 40 s（**不用回放，直接读 bag**）|

## 和 FAST-LIO2 的交叉验证

| | GLIM | FAST-LIO2 | 差异 |
|---|---|---|---|
| 位姿数 | 1391 | 1418 | -1.9% |
| 路径长 | 16.55 m | 18.92 m | -12.5% |
| **x 跨度** | **1.83 m** | **1.83 m** | **0%** |
| **y 跨度** | **6.46 m** | **6.50 m** | **-0.6%** |
| ① 稳定性 | 8.7 mm | 7.4 mm | +17.6% |

跨度几乎完全一致 → **两个独立算法互相验证了尺度是对的**。
路径长差 12.5% 属正常：GLIM 做了更重的平滑，FAST-LIO 的逐帧抖动会累加进路径长度。

## IMU 是否起作用

```
No-IMU errors rot=0.409 ± 0.397 deg
IMU    errors rot=0.023 ± 0.018 deg     ← 小 18 倍
IMU better ratios rot=0.99, trans=0.38, vel=0.84
```

`rot` 比值 0.99（接近 0 越好，接近 1 表示 IMU 没用）→ IMU 确实在起作用。

## 复现

```bash
bash ~/slam_benchmark/scripts/12_run_glim_on_bag.sh rosbag/my_static_map1
```

## 目录内容

| 文件 | 内容 |
|---|---|
| `map1/traj_glim.txt` | TUM 轨迹（可直接喂 `protocol_eval.py`）|
| `map1/traj_imu.txt` | 以 IMU 为基准的轨迹 |
| `map1/submaps/` | 6 个子图（GLIM 私有二进制格式）|
| `map1/glim.log` | 完整日志 |
| `map1/eval/protocol_report.png` | 四项指标报告图 |

## 环境

GLIM 依赖链很深，本机装的配套组合：

| 组件 | 版本 | 关键选项 |
|---|---|---|
| GTSAM | 4.3a1 | `-DGTSAM_WITH_TBB=OFF` |
| gtsam_points | 1.2.2 (tag) | 系统 boost 1.74 + 系统 Eigen |
| GLIM | 1.2.2 | `-DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_CUDA=OFF` |

安装过程见 [docs/10_GLIM安装进度.md](../../docs/03_四算法手动启动与RViz.md)，
运行方法见 [docs/11_跑GLIM.md](../../docs/03_四算法手动启动与RViz.md)。
