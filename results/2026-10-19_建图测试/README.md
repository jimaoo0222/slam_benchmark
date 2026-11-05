# 2026-10-19 建图测试（两个真雷达包）

> 目标：拿手头的 `rosbag/my_static_map1` / `my_static_map2` 跑通 FAST-LIO2，
> 看轨迹和建图效果。**顺带挖出并修掉了一个上游 bug。**

## 数据

| 包 | 时长 | 雷达帧 | IMU 帧 | 体积 | 消息类型 |
|---|---|---|---|---|---|
| `my_static_map1` | 142.2 s | 1422 @10 Hz | 28437 @200 Hz | 719 MB | `sensor_msgs/msg/PointCloud2` |
| `my_static_map2` | 326.0 s | 3260 @10 Hz | 65201 @200 Hz | 1.7 GB | `sensor_msgs/msg/PointCloud2` |

注意：**是 `PointCloud2` 不是 `CustomMsg`**（用 `rviz_MID360_launch.py` 录的）。
所以要用 `lidar_type: 4` + `mid360_pc2.yaml`，不能直接用官方的 `mid360.yaml`。

## 跑了什么

```bash
bash scripts/05_run_fastlio_on_bag.sh rosbag/my_static_map1 results/.../map1_fixed
bash scripts/05_run_fastlio_on_bag.sh rosbag/my_static_map2 results/.../map2_fixed
python3 scripts/07_plot_map.py <目录>/map.pcd --traj <目录>/traj.txt --max-range 30
```

每个目录里：

| 文件 | 内容 |
|---|---|
| `traj.txt` | TUM 轨迹（8 列） |
| `map.pcd` | 建图点云（binary PCD，订阅 `/Laser_map` 存的） |
| `map_preview.png` | 俯视图 + 侧视图，轨迹叠加 |
| `eval/` | `protocol_eval.py` 的四项指标 + 报告图 |
| `lio.log` / `play.log` / `recorder.log` | 各环节日志 |

## 核心发现：`PointCloud2` 包必须打 patch

`map1` / `map2` 这两个目录是**打 patch 之前**的结果，`map1_fixed` / `map2_fixed` 是**之后**的。
保留两份就是为了看差距。

| | map1 修复前 | map1 修复后 | map2 修复前 | map2 修复后 |
|---|---|---|---|---|
| 地图点数 | 327,476 | **196,721** | 691,503 | **204,043** |
| 变化 | — | **−40%** | — | **−70%** |
| 地板 | 糊成一片 | 清晰扫描线 | 糊成一片 | 清晰扫描线 |
| 楼层结构 | 混在一起 | 1~4 层可辨 | 混在一起 | 1~5 层可辨 |

**原因**：上游 `mid360_handler` 用「yaw 变化 / omega_l」反推每个点的时间偏移 ——
这套公式只对**重复扫描**雷达（Avia）成立。MID-360 是**非重复扫描**，yaw 不单调，
算出的 `offset_time` 完全错乱 → 运动畸变校正失效 → 地图重影。

**修复**：MID-360 的 `PointCloud2` 每个点自带 `timestamp` 字段（单位**纳秒**），
上游没用。patch 让它优先用真实时间戳，没有才回退到 yaw 估算。

```
patches/0001-fastlio-mid360-point-timestamp.patch
```

验证生效的标志（日志里出现）：

```
[mid360_handler] 每点时间戳来源: PointCloud2.timestamp  帧内跨度 0.0998 s
```

## 轨迹与稳定性

| 目录 | 位姿数 | 轨迹跨度 | ① 稳定性 | ④ 闭合误差 |
|---|---|---|---|---|
| `map1_fixed` | 1418 | x 1.83 m / y 6.50 m | 7.4 mm | 87.8 mm |
| `map2_fixed` | 3256 | x 2.45 m / y 6.42 m | 6.5 mm | 20.7 mm |

雷达走过的范围只有约 2×6.5 米，但能看到 30 米外的多层建筑 ——
**侧视图的 z 到 22 米是真实的楼层高度，不是漂移**（过滤掉 30 米外后用
`--max-range 30` 能看清）。

> 注意：这两个包是「走过去看看」的采集，**不是**按评测协议采的：
> 没有卷尺量的停车点距离、没回到原点、也不确定是不是同一路径。
> 所以只有 ①（稳定性）和 ④（闭合，参考值）有意义，
> **②可重复性 / ③尺度偏差算不出来**。要出排名表请按
> [02_操作手册.md](../../docs/03_四算法手动启动与RViz.md) 第 3 步的协议重采。

## 复现

```bash
# 1) 打 patch（只对 PointCloud2 包需要）
cd ~/lio_ws/src/fast_lio && patch -p1 < ~/slam_benchmark/patches/0001-*.patch
cd ~/lio_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2) 跑
cd ~/slam_benchmark
bash scripts/05_run_fastlio_on_bag.sh rosbag/my_static_map1 /tmp/m1
python3 scripts/07_plot_map.py /tmp/m1/map.pcd --traj /tmp/m1/traj.txt --max-range 30
```

## 以后录包的建议

用 **`msg_MID360_launch.py`**（发 `CustomMsg`），而不是 `rviz_` 那个：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # 有 CustomMsg，走 avia_handler，天生正确
ros2 launch livox_ros_driver2 rviz_MID360_launch.py  # 注意：PointCloud2，需要 lidar_type 4 + patch
```

## 看图的三种方式

| 文件 | 怎么用 |
|---|---|
| `map_preview.png` | 俯视图 + 侧视图，一眼看整体形状和轨迹 |
| `map_views.png` | **四角度 3D**（透视 / 正面 / 侧面 / 俯视），看楼层最直观 |
| `map_3d.html` | **浏览器打开，鼠标拖着转、滚轮缩放**（8.8 MB，plotly 已内联，离线可用）|

生成命令：

```bash
python3 scripts/07_plot_map.py <目录>/map.pcd --traj <目录>/traj.txt --max-range 30
python3 scripts/08_view_map_3d.py <目录>/map.pcd --traj <目录>/traj.txt --max-range 30
```

> 一定要带 `--max-range 30`：MID-360 量程 40 米，远处有玻璃/镜面反射的离群点，
> 不过滤的话主体会被压小看不清。

## 两个包是同一个地方

`map1_fixed` 和 `map2_fixed` 建出的建筑结构完全一致 —— 是同一个场景走了两遍
（142 s 与 326 s，不是同一路径）。这也侧面验证了建图的可重复性：
同样的墙、同样的楼梯、同样的楼层高度，两次都复现出来了。
