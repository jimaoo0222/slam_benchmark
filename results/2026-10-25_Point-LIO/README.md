# Point-LIO 运行记录（2026-10-25）

**本项目自移植的 ROS 2 版**（官方只有 ROS 1）。移植补丁：
[`patches/0003-pointlio-ros2-port.patch`](../../patches/0003-pointlio-ros2-port.patch)

## 结果

| bag | 位姿 | 时长 | 点位 RMS | 闭合误差 | 尺度偏差 | 路径长 |
|---|---|---|---|---|---|---|
| 01_1 | 2116 | 212 s | 0.31 cm | 0.43 cm | −0.115% | 24.85 m |
| 01_2 | 2510 | 251 s | 0.42 cm | 0.78 cm | −0.178% | 26.70 m |
| 02_1 | 2399 | 240 s | 0.23 cm | 0.53 cm | −0.029% | 26.08 m |
| 02_2 | 2241 | 224 s | 0.43 cm | 0.24 cm | −0.128% | 25.20 m |

平均 **RMS 0.35 cm / 闭合 0.49 cm** —— 与 GLIM（0.32 / 0.56）并列第一，
而 Point-LIO **没有任何回环检测**。

> 路径长比真值（15.15 / 14.65 m）多 63~78%，是 Point-LIO 输出抖动大的体现
> （静止时单步位移中位 0.46 cm，GLIM 只有 0.04 cm）。抖动已确认是**纯随机噪声**
> （符号翻转率 60%），静止段位置均值几乎为零 → **无系统性漂移，只是方差大**。

## 评测必须加 --stop-from

```bash
/usr/bin/python3 scripts/14_eval_on_site.py \
  results/2026-10-25_Point-LIO/01_1/traj.txt \
  --site config/site_mid360_demo.json \
  --stop-from results/2026-10-21_对比/01_1_glim/traj_glim.txt
```

不加的话自动停车检测只能找到 3/6 个点（噪声把停车段切碎了）。

## 每个子目录

- `traj.txt` — 轨迹（TUM 格式）
- `plio.log` — Point-LIO 日志
- `conv.log` — PointCloud2 → CustomMsg 转换节点日志
- `recorder.log` / `play.log`

## 复现

```bash
bash scripts/24_get_pointlio.sh                # 首次：clone + 打补丁 + 编译
bash scripts/23_run_pointlio_on_bag.sh rosbag/01_1 results/demo
```
