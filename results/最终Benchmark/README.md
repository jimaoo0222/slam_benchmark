# 最终 Benchmark 图表与数据（v2.0，含 Point-LIO）

> 完整报告：[`docs/13_最终Benchmark报告.md`](../../docs/02_四算法最终总结.md)

## 文件

| 文件 | 内容 |
|---|---|
| `论文格式表.png` | **论文 Table V 格式**：ATE / 闭合误差 / 尺度偏差，前三名标色 |
| `论文格式表_序列信息.png` | 论文 Table IV 格式：序列时长 / 真值路径长 |
| `论文格式表.md` | 上述三张表的 Markdown 版（可直接粘进文档）|
| `综合指标总览.png` | 六指标分面柱状图（点位 RMS / 最大 / 尺度 / 闭合 / 漂移率 / 稳定性）|
| `误差分布.png` | 逐停车点误差箱线图 + 散点（每算法 24 个样本）|
| `误差放大.png` | **把各算法相对真值的偏差画在 cm 坐标里** —— 区分度最高 |
| `轨迹对比.png` | 对齐后轨迹 vs 真值（四条彩线都贴合 → 形状都对）|
| `回环效果.png` | 闭合误差对比 —— **Point-LIO 无回环却最好** |
| `metrics.json` | 全部指标机器可读版（`per_run` 16 条 + `summary`）|

## 核心数据（4 算法 × 4 bag = 16 次运行）

| 算法 | 回环 | 点位 RMS | 尺度偏差 | 闭合误差 | 漂移率 | 稳定性 |
|---|---|---|---|---|---|---|
| **GLIM** | 有 | **0.32 cm** | **0.061%** | **0.56 cm** | **0.022 cm/m** | 0.23 cm |
| **Point-LIO** | 无 | **0.35 cm** | 0.113% | **0.49 cm** | **0.023 cm/m** | **0.20 cm** |
| **LIO-SAM** | 有 | 0.70 cm | 0.176% | 1.30 cm | 0.047 cm/m | 1.11 cm |
| **FAST-LIO2** | 无 | 1.93 cm | 0.189% | 5.99 cm | 0.129 cm/m | 0.25 cm |

## 最重要的一条

**「回环闭合是分水岭」不成立。**

- Point-LIO **零回环**，闭合误差 **0.49 cm** —— 四个里最好
- FAST-LIO2 **同样零回环**，闭合误差 **5.99 cm** —— 是 Point-LIO 的 12 倍
- LIO-SAM **带闭环**，闭合 **1.30 cm** —— 反而不如 Point-LIO

→ **决定精度的是估计器质量，回环只是兜底机制。**

## 统一口径说明

所有数字都用 **`--stop-from`**（参考轨迹定停车窗口）计算：

```bash
--stop-from "results/2026-10-21_对比/{bag}_glim/traj_glim.txt"
```

原因：各算法输出噪声差异大（静止单步位移 GLIM 0.04 cm vs Point-LIO 0.46 cm），
用同一个绝对速度阈值会系统性惩罚噪声大的算法 —— Point-LIO 曾因此只被识别出 3/6 个停车点。

## 重新生成

```bash
cd /home/jimaoo/slam_benchmark
/usr/bin/python3 scripts/22_final_report_plots.py \
  --site config/site_mid360_demo.json config/site_mid360_demo_path2.json \
  --alg "GLIM:results/2026-10-21_对比/{bag}_glim/traj_glim.txt" \
  --alg "FAST-LIO2:results/2026-10-21_对比/{bag}_flio/traj.txt" \
  --alg "LIO-SAM:results/2026-10-22_LIO-SAM/{bag}/traj.txt" \
  --alg "Point-LIO:results/2026-10-25_Point-LIO/{bag}/traj.txt" \
  --bags 01_1 01_2 02_1 02_2 \
  --stop-from "results/2026-10-21_对比/{bag}_glim/traj_glim.txt" \
  --out results/最终Benchmark
```

## 数据来源

| 算法 | 轨迹目录 | 生成脚本 |
|---|---|---|
| GLIM | `results/2026-10-21_对比/{bag}_glim/traj_glim.txt` | `scripts/12_run_glim_on_bag.sh` |
| FAST-LIO2 | `results/2026-10-21_对比/{bag}_flio/traj.txt` | `scripts/05_run_fastlio_on_bag.sh` |
| LIO-SAM | `results/2026-10-22_LIO-SAM/{bag}/traj.txt` | `scripts/17_run_liosam_on_bag.sh` |
| Point-LIO | `results/2026-10-25_Point-LIO/{bag}/traj.txt` | `scripts/23_run_pointlio_on_bag.sh` |
