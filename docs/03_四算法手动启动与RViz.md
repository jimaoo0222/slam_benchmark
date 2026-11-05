# 四算法手动启动 + RViz 查看（不用一键脚本）

> 本文全部是**手敲命令**，一步一步来。想省事可以用 `scripts/` 下的一键脚本，
> 但那样你永远不知道它到底做了什么 —— 出问题也改不动。
>
> 算法结果见 [02_四算法最终总结.md](02_四算法最终总结.md)。

---

## 0. 总览：每个算法要开几个终端

| 算法 | 终端数 | 要 RViz 吗 | bag 回放加 `--clock` 吗 | 点云格式 |
|---|---|---|---|---|
| **FAST-LIO2** | 2 | 要（launch 里 `rviz:=true`）| 有 要 | `PointCloud2` 直接吃 |
| **Point-LIO** | 3 | 要（launch 里 `rviz:=true`）| 无 不要 | **只吃 `CustomMsg`**，要转换 |
| **LIO-SAM** | 3 | 要（launch 里 `rviz:=true`）| 无 不要 | **只吃 `CustomMsg`**，要转换 |
| **GLIM** | 2（离线）/ 3（在线）| 要，用预配好的 `scripts/rviz/glim_ros2.rviz` 另起 `rviz2` | 无 不要 | 直接吃 `PointCloud2` |

「终端」指的是**独立的 shell 窗口**，不是一个窗口里 `&` 挂后台 ——
看日志、Ctrl+C 收尾都需要独立操作。

---

## 1. 公共准备（每开一个新终端都要跑一遍）

这 6 行是**所有算法**都要的，后面每节就不再重复写了。

```bash
# 1) 剔掉 conda 的 python（否则 ROS 的 rclpy 会去用 conda 3.13，报各种怪错）
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)

# 2) ROS 基础环境
source /opt/ros/humble/setup.bash

# 3) Livox SDK（装在 ~/.local，不在 ldconfig 路径里，漏了驱动起不来）
source ~/livox_ws/livox_env.sh

# 4) 驱动工作区（CustomMsg 的消息定义在这里）
source ~/livox_ws/install/setup.bash

# 5) 算法自己的工作区（下面按算法二选一/多选）
#    FAST-LIO2：  source ~/lio_ws/install/setup.bash
#    Point-LIO：  source ~/pointlio_ws/install/setup.bash
#    LIO-SAM：    source ~/liosam_ws/install/setup.bash
#    GLIM：       source ~/glim_ws/install/setup.bash

# 6) 回到项目根目录
cd ~/slam_benchmark
```

> 嫌烦就写进 `~/.bashrc` 做成函数，但**别把 `source` 写死在 `.bashrc` 里** ——
> 四个算法的工作区会互相覆盖同名包，容易莫名其妙地跑到别的版本。

---

## 2. 先搞清楚你的 bag 是什么格式

这一步决定后面用哪个配置、要不要起转换节点。

```bash
# 任何装了 ROS 的终端都行
source /opt/ros/humble/setup.bash
ros2 bag info rosbag/你的包 | grep -A1 "livox/lidar"
```

| 看到 | 说明 | 后面的配置 |
|---|---|---|
| `Type: sensor_msgs/msg/PointCloud2` | `rviz_MID360_launch.py` 录的 | 用 `mid360_pc2.yaml` / `c01_pc2.yaml` |
| `Type: livox_ros_driver2/msg/CustomMsg` | `msg_MID360_launch.py` 录的 | 用 `mid360.yaml` / `c01.yaml`，**不需要转换节点** |

本项目现有的 8 个包（`01_1` ~ `04_02`）**全是 `PointCloud2`**。

---

## 3. FAST-LIO2

### 3.1 终端 A —— 算法 + RViz

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
source ~/lio_ws/install/setup.bash
cd ~/slam_benchmark

ros2 launch fast_lio mapping.launch.py \
    rviz:=true \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/fastlio_ros2.rviz \
    use_sim_time:=true \
    config_path:=$HOME/lio_ws/src/fast_lio/config \
    config_file:=c01_pc2.yaml
```

参数逐个说：

| 参数 | 作用 | 不改会怎样 |
|---|---|---|
| `rviz:=true` | 顺便把 RViz 拉起来 | 得自己另开 rviz2 |
| `rviz_cfg:=...fastlio_ros2.rviz` | **Fixed Frame = `odom`** 的修好版 | 用官方自带配置会红字 `Fixed Frame [camera_init] does not exist`，画面全空 |
| `use_sim_time:=true` | 用 bag 里的 `/clock`，而不是墙上时间 | 时间戳错乱，点云不显示 |
| `config_path` / `config_file` | 指定 yaml | 默认 `mid360.yaml`（吃 CustomMsg），PointCloud2 的包会**一条点都收不到** |

等终端 A 出现这行再往下：

```
Node init finished
```

### 3.2 终端 B —— 回放

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

ros2 bag play rosbag/04_02 --clock -r 0.7
```

- `--clock` **必须有**（配合 `use_sim_time:=true`）
- `-r 0.7` 是倍率。C01 的包建议 0.7，MID-360s 的可以 1.0
- 长包（>200 s）在 1.0 倍速下偶发发散，降速就好

### 3.3 RViz 里该看到什么

`fastlio_ros2.rviz` 已经配好了，打开就该有东西：

| Display | 话题 | 内容 |
|---|---|---|
| `Cloud` | `/cloud_registered` | 当前帧配准后的点云 |
| `Map` | `/Laser_map` | 累积地图，越走越密 |
| `Path` | `/path` | 轨迹线 |
| Odometry | `/Odometry` | 位姿箭头 |

**Fixed Frame = `odom`**（配置里已写好）。如果还是空，手动改：
左侧 `Global Options → Fixed Frame` → 选 `odom`。

### 3.4 验证

另开终端：

```bash
source /opt/ros/humble/setup.bash && source ~/lio_ws/install/setup.bash
ros2 topic hz /cloud_registered     # 应约 10 Hz
ros2 topic echo /Odometry --once    # 能动就说明在跑
```

---

## 4. Point-LIO

Point-LIO 走 `CustomMsg`，所以**多一个转换终端**。

### 4.1 终端 A —— PointCloud2 → CustomMsg 转换

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

/usr/bin/python3 scripts/16_pc2_to_custom.py
```

看到这几行就对了：

```
[pc2_to_custom]: 转换节点已启动: /livox/lidar (PointCloud2) -> /livox/lidar_custom (CustomMsg)
[pc2_to_custom]: 帧 1: 21600 点, 跨度 99.5 ms, reflectivity 0~163, line 0~3
```

> 注意是 `/usr/bin/python3`，不是 `python3`（后者是 conda 3.13，import 不到 `livox_ros_driver2`）。

### 4.2 终端 B —— 算法 + RViz

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
source ~/pointlio_ws/install/setup.bash
cd ~/slam_benchmark

ros2 launch point_lio mapping_mid360.launch.py \
    rviz:=true \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/pointlio_ros2.rviz \
    config_file:=$HOME/pointlio_ws/install/point_lio/share/point_lio/config/mid360.yaml
```

> 注意：**必须用 `scripts/rviz/pointlio_ros2.rviz`，不能用 Point-LIO 自带的 `loam_livox.rviz`。**
> 那个文件是 **ROS 1 格式**（`Class: rviz/Displays`、`Class: rviz/Grid`…），
> 灌进 ROS 2 的 rviz2 会出现 —— 左侧 Displays 面板整块变成报错文字、
> 底部 Time 面板同样报错、中间画面**全黑**：
>
> ```
> The class required for this panel, 'rviz/Displays', could not be loaded.
> Error: ... the class rviz/Displays with base class type rviz_common::Panel does not exist.
> ```
>
> 项目里的 `pointlio_ros2.rviz` 是按 `fastlio_ros2.rviz` 改的：
> 面板类名全换成 `rviz_common/*`、显示插件换成 `rviz_default_plugins/*`、
> Fixed Frame 改成 `camera_init`、位姿话题改成 `/aft_mapped_to_init`。

等到节点出现：

```bash
ros2 node list | grep laserMapping
```

### 4.3 终端 C —— 回放（**不加** `--clock`）

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

ros2 bag play rosbag/04_02 -r 0.35
```

`-r 0.35` 不是随便写的：Point-LIO 处理 C01 的 21.7k 点/帧只能跑到约 0.4 倍实时，
1.0 倍速下它只吃得下 21% 的帧，日志会刷
`lidar loop back, clear buffer`，轨迹直接断在半路。

### 4.4 RViz 里该看到什么

| Display | 话题 | 内容 |
|---|---|---|
| `Cloud` | `/cloud_registered` | 当前帧点云 |
| `Map` | `/Laser_map` | 累积地图 |
| `Path` | `/path` | 轨迹 |
| Odometry | `/aft_mapped_to_init` | 位姿 |

**Fixed Frame = `camera_init`**（Point-LIO 代码里就是这么写的，
`laserMapping.cpp:278` `odomAftMapped.header.frame_id = "camera_init"`）。
项目里的 `pointlio_ros2.rviz` 已经把它写好了，**不用改**。

> 如果 RViz 里什么都看不到，先确认 Fixed Frame 是不是被改成了 `odom` ——
> Point-LIO 没这个坐标系，会报错、画面空。

---

## 5. LIO-SAM

### 5.1 终端 A —— 点云格式转换（同 Point-LIO）

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

/usr/bin/python3 scripts/16_pc2_to_custom.py
```

### 5.2 终端 B —— 算法 + RViz

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/liosam_ws/install/setup.bash

# 注意：这一行必须在 source 之后加，且 ~/.local/lib 要排最前
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"

cd ~/slam_benchmark
ros2 launch scripts/liosam/run_headless.launch.py \
    params_file:=$HOME/slam_benchmark/config/liosam_mid360.yaml \
    rviz:=true \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/liosam_ros2.rviz
```

> 注意：`LD_LIBRARY_PATH` 那一行不做的话，会命中 `/usr/local/lib` 的旧版 GTSAM，
> 报 `libboost_serialization.so.1.82.0: not found` 直接起不来。

会起 4 个节点：

```bash
ros2 node list | grep lio_sam
# /lio_sam_imuPreintegration
# /lio_sam_imageProjection
# /lio_sam_featureExtraction
# /lio_sam_mapOptimization
```

### 5.3 终端 C —— 回放（**不加** `--clock`）

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

ros2 bag play rosbag/04_02 -r 0.7
```

### 5.4 RViz 里该看到什么

| Display | 话题 | 内容 |
|---|---|---|
| 当前帧点云 | `/lio_sam/mapping/cloud_registered` | 配准后的点云，按 z 高度彩虹着色 |
| 全局地图 | `/lio_sam/mapping/map_global` | 越走越密 |
| Path | `/lio_sam/mapping/path` | 红色轨迹 |
| MarkerArray | `/lio_sam/mapping/loop_closure_constraints` | **回环约束**，走回起点附近才突然冒出来 |

> `cloud_registered` 和 `loop_closure_constraints` 这两个话题**要等有数据才会出现**，
> 刚启动时 `ros2 topic list` 里看不到是正常的。

**Fixed Frame = `map`**（LIO-SAM 自己发 `map → odom → livox_frame`，不用改）。

> 上游自带的 `rviz2.rviz` 有个 Display 指向 `/points`，而 LIO-SAM **根本不发这个话题** ——
> 那个 Display 会永远黄色。本项目 `scripts/rviz/liosam_ros2.rviz` 已改成 `map_global`。

**想直观看回环**：盯住 `Path`，走到起点附近时会突然「被拉回去」一下，
同时 `loop_closure_constraints` 里多出几根线 —— 那就是回环约束在起作用。

---

## 6. GLIM

GLIM 没有图形界面。它的 `rviz_viewer` 模块只是个**话题发布器**
（`~/glim_ws/src/glim_ros2/src/glim_ros/rviz_viewer.cpp`），
把结果发到 `/glim_ros/*` 话题上，**RViz 得你自己另开**。

> 有 本仓库已备好两个东西，直接能用，不用手配：
>
> | 文件 | 作用 |
> |---|---|
> | [`scripts/rviz/glim_ros2.rviz`](../scripts/rviz/glim_ros2.rviz) | **预配好的 RViz 配置** —— 5 个 Display 全指好了话题、Fixed Frame = `map`、`/glim_ros/map` 的 QoS 已设成 `Transient Local` |
> | `config/glim_livox_view/` | **慢放用的 GLIM 配置** —— 把 `playback_speed` 从默认的 **100 倍** 改成 **1.0 倍（实时）** |

### 6.1 注意：先说「GLIM 跑太快看不到」

`glim_rosbag` 是离线读 bag，**默认 `playback_speed = 100`** —— 137 秒的包 1.4 秒就放完了，
RViz 根本来不及画。

两个解决办法，任选：

**A. 用慢放配置（推荐，能完整看全程）**

```bash
# 就是 config_path 换成 glim_livox_view（其他文件都是软链接，只覆盖了 playback_speed）
-p config_path:=$HOME/slam_benchmark/config/glim_livox_view
```

日志里出现这行就对了：

```
[glim] [info] playback speed: 1.000x
```

**B. 用键盘暂停（用默认配置时）**

`glim_rosbag` 支持**空格键暂停/继续**（读 `STDIN`，所以在终端里直接按）：

```
playback paused (press space to resume)
```

### 6.2 离线跑 + 预配置 RViz（两个终端）

**终端 A —— GLIM（1.0 倍速）**

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash      # 提供 livox_ros_driver2 的消息类型
source ~/glim_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"   # GTSAM / gtsam_points 在 ~/.local

cd ~/slam_benchmark
$HOME/glim_ws/install/glim_ros/lib/glim_ros/glim_rosbag \
    rosbag/04_02 \
    --ros-args \
    -p config_path:=$HOME/slam_benchmark/config/glim_livox_view \
    -p dump_path:=/tmp/glim_out
```

**终端 B —— 预配好的 RViz**

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/glim_ws/install/setup.bash

cd ~/slam_benchmark
rviz2 -d scripts/rviz/glim_ros2.rviz
```

> **先起 RViz，再起 GLIM**。反过来也行，但 GLIM 已经开始跑了你才打开 RViz，
> 前面那段就看不到了（`/glim_ros/map` 是 Transient Local 的，地图能补上；
> 但逐帧点云补不回来）。

### 6.3 预配置里到底配了什么

打开就能看到画面，不用 Add 任何东西：

| Display | 话题 | 关键设置 |
|---|---|---|
| **TF** | — | 显示 `map → odom → livox_imu → livox_frame` |
| **Grid** | — | 1 m 网格，跟着 Fixed Frame |
| **Trajectory** | `/glim_ros/odom_corrected` | `Keep: 3000` —— **靠它画轨迹**（GLIM 不发 `/path` 话题）|
| **GLIM_Map** | `/glim_ros/map` | **Durability = Transient Local** ← 不改这个收不到地图 |
| **GLIM_PointsCorrected** | `/glim_ros/points_corrected` | Decay 0.5 s，畸变校正后的当前帧 |
| **GLIM_PointsRaw** | `/glim_ros/points` | 默认**关闭**（原始点云，省带宽）|

**Fixed Frame = `map`**（`config_ros.json` 里 `"map_frame_id": "map"`）。

> 注意：**`/glim_ros/map` 的 QoS 是 `TRANSIENT_LOCAL`**（`rviz_viewer.cpp:66`），
> RViz 的订阅端也必须是 `Transient Local`，否则**主题看得见、数据一条没有**。
> 预配置里已经设好了；如果你自己 Add，一定要改 `Durability Policy`。

> 注意：刚启动、还没有数据进来时，`map` 和 `odom` 这两个 TF 帧**还不存在**，
> RViz 会红字报 `Invalid frame ID "map"`。等 GLIM 开始输出后自然就有了。

> 注意：日志里刷 `Publishing global map is computationally demanding and not recommended`
> 是正常的 —— 说明 RViz 订阅了 `/glim_ros/map`，GLIM 在定期重建全局地图。
> 场地只有十几米时完全跟得上（实测 10 Hz）；换成大场景觉得卡，
> 就把 `GLIM_Map` 这个 Display 关掉（`Enabled` 取消勾选）。

### 6.4 其它有用的话题

预配置里没放，需要时自己 `Add → By topic`：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/glim_ros/points` | `PointCloud2` | 原始点云 |
| `/glim_ros/points_corrected` | `PointCloud2` | **畸变校正后**的点云（看它就知道 IMU 有没有起作用）|
| `/glim_ros/aligned_points_corrected` | `PointCloud2` | 配准到地图坐标系的点云 |
| `/glim_ros/odom` | `Odometry` | 里程计位姿（未回环校正）|
| `/glim_ros/odom_corrected` | `Odometry` | **回环校正后**（和上面比就能看出全局优化的效果）|
| `/glim_ros/lidar_odom` | `Odometry` | LiDAR-only 位姿（和 `odom` 比就能看出 IMU 贡献多少）|
| `/glim_ros/pose_corrected_with_cov` | `PoseWithCovarianceStamped` | 带协方差的位姿 |

### 6.5 在线跑（实时订阅，边跑边看）

```bash
# 终端 A：GLIM 在线
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
source ~/glim_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=$HOME/slam_benchmark/config/glim_livox

# 终端 B：RViz
cd ~/slam_benchmark && rviz2 -d scripts/rviz/glim_ros2.rviz

# 终端 C：回放（这里用 bag 自己的倍速，0.5 = 半速，方便看）
source /opt/ros/humble/setup.bash && source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark && ros2 bag play rosbag/04_02 -r 0.5
```

### 6.6 GLIM 的坐标系（容易踩）

`config_ros.json` 里写死成四层：

```
map  →  odom  →  livox_imu  →  livox_frame
                    ↑ IMU         ↑ 点云
```

`imu_frame_id = livox_imu`、`lidar_frame_id = livox_frame` 是**故意分开的** ——
Livox 驱动的 IMU 和点云的 `frame_id` 都叫 `livox_frame`，不分开 GLIM 会报
「TF 父子同名」直接崩。`publish_imu2lidar: true` 会把 `T_lidar_imu`
发成 TF，把 `livox_imu` 和 `livox_frame` 连起来。

所以 RViz 的 Fixed Frame 选 `map` 或 `odom` 都行，**别选 `livox_frame`**
（那样看不到地图和轨迹）。

## 7. RViz 通用操作（从零加 Display）

RViz 打开时是**空壳**，每个图层都要自己加。左下角 **Add** 按钮：

| 想看 | 选 Type | Topic |
|---|---|---|
| 点云 | `PointCloud2` | 见上面各算法的话题表 |
| 轨迹线 | `Path` | `/path` 或 `/lio_sam/mapping/path` |
| 位姿箭头 | `Odometry` | `/Odometry` / `/aft_mapped_to_init` |
| 坐标系树 | `TF` | — |

加完在左侧列表点开调参数：

| 参数 | 建议值 | 说明 |
|---|---|---|
| **Style** | `Points` | 默认 |
| **Size (m)** | `0.05` ~ `0.1` | 点太密就往小调 |
| **Color Transformer** | `AxisColor`（按轴）或 `Intensity` | `AxisColor` 一眼看出方向 |
| **Decay Time** | `0` 看当前帧；`30` 看堆积 | 想看「地图在长」就设大 |

### 鼠标

| 操作 | 效果 |
|---|---|
| 左键拖 | 旋转 |
| 滚轮 | 缩放 |
| 中键拖 | 平移 |
| 左侧底部 `Reset` | 视角复位 |

---

## 8. 怎么确认「数据真的进来了」

RViz 里的 display 图标：

| 图标 | 含义 |
|---|---|
| 绿色圆点 | 话题在且有数据 —— 正常 |
| 黄色圆点 | 话题在但没数据 —— 算法没跑 / bag 没放 |
| 红色圆点 | 话题类型不匹配 —— 删了重加 |

更准的办法是终端里看频率：

```bash
# FAST-LIO2
source /opt/ros/humble/setup.bash && source ~/lio_ws/install/setup.bash
ros2 topic hz /cloud_registered      # ~10 Hz
ros2 topic hz /Laser_map

# Point-LIO
source /opt/ros/humble/setup.bash && source ~/pointlio_ws/install/setup.bash
ros2 topic hz /aft_mapped_to_init    # ~10 Hz（0.35 倍速时约 3.5 Hz）

# LIO-SAM
source /opt/ros/humble/setup.bash && source ~/liosam_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
ros2 topic hz /lio_sam/mapping/odometry     # ~5 Hz

# GLIM
source /opt/ros/humble/setup.bash && source ~/glim_ws/install/setup.bash
ros2 topic hz /glim_ros/odom
```

---

## 9. 收尾（顺序有讲究）

**别用 `kill -9`** —— 算法来不及把最后的位姿落盘，轨迹会缺尾巴。

| 顺序 | 终端 | 操作 |
|---|---|---|
| 1 | 回放终端 | 等它自己结束，或 `Ctrl+C` |
| 2 | 算法终端 | 等 5~10 秒让它把缓冲区算完，再 `Ctrl+C` |
| 3 | 转换终端（Point-LIO / LIO-SAM）| `Ctrl+C` |

算法正常退出时终端会打印 `catch sig 2` 之类的信息，那是正常的 SIGINT。

---

## 10. 故障排除

| 现象 | 原因 | 怎么修 |
|---|---|---|
| **左侧 Displays 面板整块变成报错文字、底部 Time 面板也报错、画面全黑**<br>`The class required for this panel, 'rviz/Displays', could not be loaded` | **把 ROS 1 的 .rviz 配置灌进了 rviz2**（`Class: rviz/Displays` 是 ROS 1 写法）| 用项目里的 `scripts/rviz/{fastlio,pointlio,liosam}_ros2.rviz`；或自己转换：面板 `rviz/*` → `rviz_common/*`，显示插件 `rviz/*` → `rviz_default_plugins/*` |
| **RViz 红字 `Fixed Frame [camera_init] does not exist`，画面全空** | FAST-LIO 自带配置用了 ROS 1 旧名，代码里发的是 `odom` | 用 `scripts/rviz/fastlio_ros2.rviz`，或界面里改成 `odom` |
| **RViz 空且报 `Fixed Frame [odom] does not exist`（Point-LIO）** | Point-LIO 发的是 `camera_init` | Fixed Frame 改成 `camera_init` |
| **算法起来了但一条点都没有** | bag 是 `PointCloud2`，配置却是吃 `CustomMsg` 的 | FAST-LIO 换 `*_pc2.yaml`；Point-LIO/LIO-SAM 起 `16_pc2_to_custom.py` |
| **LIO-SAM 起不来，`libboost_serialization.so.1.82.0: not found`** | `/usr/local` 的旧 GTSAM 抢先 | `export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"` |
| **`No module named 'em'` / `rclpy` 相关报错** | conda 的 python 抢进了 ROS | 重跑第 1 节的第 1 行，或 `conda deactivate` |
| **Point-LIO 轨迹只走了一半** | 回放太快，它追不上 | `-r 0.35` |
| **日志刷 `lidar loop back, clear buffer`** | 同上，帧被丢弃 | 降回放倍率 |
| **FAST-LIO 中途发散（位置跳到 1e5）** | 长包 + 1.0 倍速 | `-r 0.7`；用 `scripts/28_trim_traj.py` 裁掉发散尾巴 |
| **`ros2 topic hz` 报 `message type is invalid`** | 没 source 算法工作区（缺消息定义）| 补上 `source ~/xxx_ws/install/setup.bash` |
| **GLIM 日志刷 `IMU prediction is not good`** | `T_lidar_imu` 不准，或雷达不是按静态起步录的 | 确认用的是对应雷达的配置（`config/glim_livox` / `config/glim_c01`）。静止起步能压住这条警告，走起来才起录就会反复报 |
| **GLIM 一闪就结束，RViz 里啥也没看到** | `glim_rosbag` 默认 `playback_speed=100`（100 倍速）| 用 `-p config_path:=.../config/glim_livox_view`（1.0 倍速），或跑起来后按**空格**暂停 |
| **GLIM 的 `/glim_ros/map` 在 RViz 里一直不出画** | RViz 订阅端 QoS 不是 `Transient Local`，而 GLIM 的 map 是 Transient Local 发布的 | 用 `scripts/rviz/glim_ros2.rviz`；手工加的话把 Display 的 `Durability Policy` 改成 `Transient Local` |
| **多个算法同时跑，结果都乱了** | 都在抢 `/livox/lidar` 这类同名话题 | **一次只跑一个算法**，跑完彻底退出再换 |
| **轨迹路程是真实值的几十倍、时间戳重复** | 别处还有 bag 在回放 → `16_pc2_to_custom.py` 订阅 `/livox/lidar`，会把别人的数据一起收 | 跑之前 `ps -eo pid,args \| grep -E "bag play\|pc2_to_custom"` 确认干净。本项目踩过：C01 的 04_02 首轮 LIO-SAM 因此从 0.4 cm 虚高到 3.7 cm |

---

## 11. 一页速查

```bash
# ── 公共环境（每个终端都要）──
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
cd ~/slam_benchmark

# ── FAST-LIO2（终端 A / B）──
source ~/lio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py rviz:=true \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/fastlio_ros2.rviz \
    use_sim_time:=true config_path:=$HOME/lio_ws/src/fast_lio/config \
    config_file:=c01_pc2.yaml
ros2 bag play rosbag/04_02 --clock -r 0.7                    # 终端 B

# ── Point-LIO（终端 A / B / C）──
/usr/bin/python3 scripts/16_pc2_to_custom.py                 # 终端 A
source ~/pointlio_ws/install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py rviz:=true \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/pointlio_ros2.rviz   # 终端 B
ros2 bag play rosbag/04_02 -r 0.35                            # 终端 C

# ── LIO-SAM（终端 A / B / C）──
/usr/bin/python3 scripts/16_pc2_to_custom.py                 # 终端 A
source ~/liosam_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
ros2 launch scripts/liosam/run_headless.launch.py rviz:=true \
    params_file:=$HOME/slam_benchmark/config/liosam_mid360.yaml \
    rviz_cfg:=$HOME/slam_benchmark/scripts/rviz/liosam_ros2.rviz   # 终端 B
ros2 bag play rosbag/04_02 -r 0.7                             # 终端 C

# ── GLIM（终端 A 跑算法 + 终端 B 跑预配好的 RViz）──
source ~/glim_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"
# 终端 B 先起：rviz2 -d scripts/rviz/glim_ros2.rviz
# 终端 A（glim_livox_view = 1.0 倍速，不加就是 100 倍速看不到）：
$HOME/glim_ws/install/glim_ros/lib/glim_ros/glim_rosbag rosbag/04_02 \
    --ros-args -p config_path:=$HOME/slam_benchmark/config/glim_livox_view \
    -p dump_path:=/tmp/glim_out
```
