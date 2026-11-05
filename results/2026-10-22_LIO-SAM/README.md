# LIO-SAM 运行记录（2026-10-22）

4 个 bag 各跑一次，产物是每个子目录下的 `traj.txt`（TUM 格式）+ 日志。

| bag | 位姿 | 时长 | 点位 RMS | 闭合误差 | 尺度偏差 |
|---|---|---|---|---|---|
| 01_1 | 997 | 211s | 0.35 cm | 0.57 cm | −0.168% |
| 01_2 | 1244 | 250s | 0.46 cm | 0.69 cm | −0.172% |
| 02_1 | 1187 | 240s | 0.50 cm | 0.34 cm | −0.153% |
| 02_2 | 1111 | 223s | 1.47 cm | 3.63 cm | −0.221% |

> 路径 01 = `0→1→2→3→4→0`（用 `config/site_mid360_demo.json`）
> 路径 02 = `0→2→3→4→1→0`（用 `config/site_mid360_demo_path2.json`）

每个子目录：
- `traj.txt` — 轨迹（`timestamp tx ty tz qx qy qz qw`）
- `lio.log` — 四个节点的日志
- `conv.log` — PointCloud2 → CustomMsg 转换节点日志
- `recorder.log` / `play.log`

三算法完整对比（含 GLIM / FAST-LIO2）见
[`../2026-10-22_三算法对比/README.md`](../2026-10-22_三算法对比/README.md)。

复现：

```bash
bash scripts/17_run_liosam_on_bag.sh rosbag/01_1 results/2026-10-22_LIO-SAM/01_1
/usr/bin/python3 scripts/14_eval_on_site.py \
    results/2026-10-22_LIO-SAM/01_1/traj.txt --site config/site_mid360_demo.json
```
