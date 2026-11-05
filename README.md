# 开源算法 Benchmark

> **任务**：开源算法 benchmark 调研，构建起本队的 benchmark
> **设备**：Livox MID-360s、SKYLANDX C01
> **8个包怎么来的**：`01_1` `01_2` `02_1` `02_2` 是 **MID-360s** 录的，
> `03_01` `03_02` `04_01` `04_02` 是 **C01** 录的
> **对比对象**：FAST-LIO2 / Point-LIO / LIO-SAM / GLIM
>
> **在线站点**：https://jimaoo0222.github.io/slam_benchmark/
> （[概览](https://jimaoo0222.github.io/slam_benchmark/) · [数据集](https://jimaoo0222.github.io/slam_benchmark/dataset.html)）
> → [**四算法最终总结**](docs/02_四算法最终总结.md)
>
> | 算法 | 回环 | MID-360s 点位 RMS（4 包） | C01 点位 RMS（4 包） | 8 包总平均 | 闭合误差（MID-360s）|
> |---|---|---|---|---|---|
> | **GLIM** | 有 | 0.33 cm | **0.5 ~ 4.7 cm** | **1.12 cm** | 0.44 cm |
> | **Point-LIO** | 无 | **0.31 cm** | 0.4 ~ 7.2 cm | 1.68 cm | **0.43 cm** |
> | LIO-SAM | 有 | 0.77 cm | 0.6 ~ 4.9 cm | 1.39 cm | 1.29 cm |
> | FAST-LIO2 | 无 | 1.90 cm | 0.4 ~ 8.1 cm | 2.43 cm | 5.94 cm |
>
> 外参已按两颗雷达各自官方标定值统一修正，32 次运行 2026-11-05 全部重跑，
> 8 个包同一口径 → [`results/2026-11-05_外参修正/`](results/2026-11-05_外参修正/README.md)

---
## 方案的依据

```
① 公开数据集里的雷达全是旋转式 / 机械式
   —— 逐条查过 12 个主流数据集；GitHub 搜 livox benchmark odometry 结果为 0

② 主流评测框架 evalio 没收录 FAST-LIO2 / Point-LIO
   且其数据接口按「扫描线」设计，装不下 Livox

③ 正常场景下各算法 ATE 无显著差异
   → 必须靠退化场景 + 退化比拉开差距
```

**方案三步**：

```
① 数据   自采（Lidar + 卷尺真值）
② 算法   8~10 个，统一配置，每个跑 ≥3 次
③ 评测   两组误差（不对齐 + 对齐） + 分场景 + 资源消耗
```

→ 评测方法与结果见 [`docs/02_四算法最终总结.md`](docs/02_四算法最终总结.md)

---

## 文档

| 文档 | 什么时候看 |
|---|---|
| [**01_MID360_vs_C01_雷达对比.md**](docs/01_MID360_vs_C01_雷达对比.md) | 两颗雷达的规格 / 实测差异、驱动异同、怎么判断 bag 是哪颗录的 |
| [**02_四算法最终总结.md**](docs/02_四算法最终总结.md) | 四个算法的原理、8 个 bag 的完整结果、选型指南、局限 |
| [**03_四算法手动启动与RViz.md**](docs/03_四算法手动启动与RViz.md) | 手敲命令跑四个算法 + RViz 看效果 |

---
## 目录

```
slam_benchmark/
├── README.md                        ← 本文件
├── docs/                            文档 + GitHub Pages 站点
│   ├── index.html                   站点首页（Pages 入口）
│   ├── dataset.html                 数据集页：下载 / 采集方法 / 真值 / 校验和
│   ├── 01_MID360_vs_C01_雷达对比.md  两颗雷达的规格与实测差异
│   ├── 02_四算法最终总结.md          8 个 bag × 4 算法 的完整结果
│   ├── 03_四算法手动启动与RViz.md    手敲命令跑四个算法 + RViz
│   └── photo/                       现场照片与场地布局图（两个页面的插图）
├── scripts/
│   ├── 26_record_bag.sh             录包（环境 + 话题自检 + 录完核对）
│   ├── 03_livox_discover.py         探测雷达 IP / SN / dev_type
│   ├── 02_get_livox.sh              装驱动 + 写配置（不需要 sudo）
│   ├── 05_run_fastlio_on_bag.sh     一个 bag 跑完 FAST-LIO2
│   ├── 23_run_pointlio_on_bag.sh    一个 bag 跑完 Point-LIO
│   ├── 17_run_liosam_on_bag.sh      一个 bag 跑完 LIO-SAM
│   ├── 12_run_glim_on_bag.sh        一个 bag 跑完 GLIM
│   ├── lib_radar.sh                 按包名判断雷达 → 自动选外参配置
│   ├── 14_eval_on_site.py           绝对精度评测（对卷尺真值）
│   ├── traj_recorder.py             任意 Odometry 话题 → TUM 轨迹
│   └── 27_salvage_zip.py            抢救下载中断的 zip
├── config/
│   ├── site_*.json                  场地真值（5 份，见 docs/02 第 2 节）
│   ├── glim_livox/                  GLIM 配置 — MID-360s 外参
│   ├── glim_c01/                    GLIM 配置 — C01 外参
│   ├── glim_livox_view/             GLIM 配置（1 倍速看 RViz 用）
│   ├── liosam_mid360.yaml           LIO-SAM — MID-360s 外参
│   ├── liosam_c01.yaml              LIO-SAM — C01 外参
│   └── pointlio_mid360.yaml
├── scripts/rviz/                    RViz 配置（每算法一份，ROS 2 格式）
├── rosbag/                          8 个自录 bag
├── results/                         评测产物，每个实验一个目录
└── tutorials/                       入门脚本（点云基础 / AprilTag / OpenGL）
```

---

## 环境

```bash
pip3 install --user evo rosbags      # 轨迹评测 + 读 rosbag
bash scripts/00_setup_env.sh         # 幂等自检与修复（不需要 sudo）
```

**可选：接入 `evalio`**（拿现成的 8 个算法做外部对照）

```bash
pip install -i https://mirrors.aliyun.com/pypi/simple/ evalio
```

> 注意：`evalio` 要求 **Python ≥ 3.11**（系统自带的 3.10 装不了，用 conda 的 python）

---
## 工具

| 文件 | 用途 |
|---|---|
| [`scripts/benchmark_eval.py`](scripts/benchmark_eval.py) | **评测工具**：两组误差 / 分场景 / 退化比 / 找停车点 / 一致性 |
| [`scripts/protocol_eval.py`](scripts/protocol_eval.py) | **协议评测（主力）**：稳定性 / 可重复性 / 点间距 / 闭合误差 —— **不需要绝对真值** |
| [`scripts/run_eval_all.sh`](scripts/run_eval_all.sh) | **一键评测**：对所有 `traj_*.txt` 批量跑并汇总 |
| [`scripts/traj_recorder.py`](scripts/traj_recorder.py) | 把算法的 `/Odometry` 话题录成 TUM 轨迹文件 |
| [`scripts/02_get_livox.sh`](scripts/02_get_livox.sh) | **装 MID-360s 驱动**：SDK → `~/.local`、驱动 → `~/livox_ws`、写配置、编译（**不需要 sudo**）。`--device` 选设备类型（默认 `mid360`，段名/文件名不匹配会被 SDK **静默丢弃**） |
| [`scripts/03_livox_discover.py`](scripts/03_livox_discover.py) | **主动探测雷达**：IP / SN / `dev_type` —— 不用翻机身标签、不用 Livox Viewer；还会提醒端口被谁抢了 |
| [`scripts/01_get_fastlio.sh`](scripts/01_get_fastlio.sh) | 下载并编译 FAST-LIO2（带断点续传） |
| [`scripts/05_run_fastlio_on_bag.sh`](scripts/05_run_fastlio_on_bag.sh) | **一个 bag 跑完 FAST-LIO2**：回放 → 轨迹 → 地图 PCD → 日志，一条命令 |
| [`scripts/06_save_map_pcd.py`](scripts/06_save_map_pcd.py) | 订阅 `/Laser_map` 存 PCD（上游 `pcd_save_en` 失效，只能这样拿地图） |
| [`scripts/07_plot_map.py`](scripts/07_plot_map.py) | 地图 + 轨迹可视化（俯视图 / 侧视图） |
| [`scripts/08_view_map_3d.py`](scripts/08_view_map_3d.py) | **3D 看地图**：生成可交互 HTML（鼠标拖着转）+ 四角度静态图 |
| [`scripts/09_pcd_viewer.py`](scripts/09_pcd_viewer.py) | **点云查看器程序**：双击打开，把 `.pcd` / `.txt` 拖进浏览器就能转着看（离线可用，不用每次生成 HTML） |
| [`scripts/install_pcd_viewer_desktop.sh`](scripts/install_pcd_viewer_desktop.sh) | 装/卸「点云查看器」的桌面图标和应用菜单项 |
| [`scripts/10_pcd_viewer_gl.py`](scripts/10_pcd_viewer_gl.py) | **OpenGL 版点云查看器**：点云一次性进显存，20 万点轻松 60 fps（`pcd-gl map.pcd`）|
| [`tutorials/03_opengl_basics.py`](tutorials/03_opengl_basics.py) | **OpenGL 入门**：60 行画个彩色三角形，讲透渲染管线 |
| [`scripts/11_rviz_demo.sh`](scripts/11_rviz_demo.sh) | **一键 RViz**：自动起 FAST-LIO + RViz + 回放，任何目录都能跑 |
| [`scripts/12_run_glim_on_bag.sh`](scripts/12_run_glim_on_bag.sh) | **一键 GLIM**：直接读 bag 出轨迹，比实时快 |
| [`scripts/13_compare_trajs.py`](scripts/13_compare_trajs.py) | **轨迹一致性对比**：ICP 形状比对，判断是否走了同一条路 |
| [`scripts/14_eval_on_site.py`](scripts/14_eval_on_site.py) | **绝对精度评测**：用量过的场地真值算点位误差 / 尺度偏差 / 闭合误差 |
| [`scripts/18_get_liosam.sh`](scripts/18_get_liosam.sh) | **装 LIO-SAM**（MID-360 ROS2 版，含 4 处必要修补，不需要 sudo）|
| [`scripts/17_run_liosam_on_bag.sh`](scripts/17_run_liosam_on_bag.sh) | **一个 bag 跑完 LIO-SAM**：自动判断点云格式 → 必要时转 CustomMsg → 出轨迹 |
| [`scripts/16_pc2_to_custom.py`](scripts/16_pc2_to_custom.py) | **PointCloud2 → CustomMsg** 转换节点（LIO-SAM 只吃 CustomMsg，格式不对会**静默收不到**）|
| [`scripts/20_liosam_rviz_demo.sh`](scripts/20_liosam_rviz_demo.sh) | **RViz 看 LIO-SAM 建图**：任何目录一条命令，自动转点云格式 + 起 RViz + 回放 |
| [`scripts/rviz/liosam_ros2.rviz`](scripts/rviz/liosam_ros2.rviz) | LIO-SAM 的 RViz 配置（修正上游指向 `/points` 的失效 Display）|
| [`scripts/liosam/run_headless.launch.py`](scripts/liosam/run_headless.launch.py) | LIO-SAM 启动（`rviz:=true` 开界面，默认不开）|
| [`scripts/19_plot_algo_compare.py`](scripts/19_plot_algo_compare.py) | **多算法汇总出图**：柱状对比 + 轨迹叠加 + 停车点误差曲线 |
| [`scripts/22_final_report_plots.py`](scripts/22_final_report_plots.py) | **最终报告出图**：六指标总览 + 误差分布箱线 + 回环效果 |
| [`scripts/25_paper_style_tables.py`](scripts/25_paper_style_tables.py) | **论文格式表**：ATE / 闭合 / 尺度，前三名标色，出 PNG |
| [`scripts/24_get_pointlio.sh`](scripts/24_get_pointlio.sh) | **装 Point-LIO**（clone 上游 + 打 ROS 2 移植补丁 + 编译）|
| [`scripts/23_run_pointlio_on_bag.sh`](scripts/23_run_pointlio_on_bag.sh) | **一个 bag 跑完 Point-LIO**：自动起点云转换 → 出轨迹 |
| [`scripts/29_rerun_all.sh`](scripts/29_rerun_all.sh) | **全量重跑**：8 包 × 4 算法一次跑完，按包名自动选外参配置，断点续跑（`--only` / `--algo` / `--domain` / `--dry`）|
| [`scripts/30_eval_all.py`](scripts/30_eval_all.py) | **批量评测**：每组自动挑一条最干净的轨迹定停车窗口，四个算法共用，出汇总表 |
| [`scripts/lib_radar.sh`](scripts/lib_radar.sh) | 按包名前两位判断是哪颗雷达录的，四个运行脚本 `source` 它自动选配置（`01_`/`02_` → MID-360s，`03_`/`04_` → C01）|
| [`scripts/26_record_bag.sh`](scripts/26_record_bag.sh) | **录包一条命令**：环境 + 话题自检 + 录制 + 录完自动核对点数/频率（`bash scripts/26_record_bag.sh c01_01`）|
| [`scripts/27_salvage_zip.py`](scripts/27_salvage_zip.py) | **抢救半截 zip**：下载中断的包 `unzip` 打不开时，按 CRC 校验救回完整的那些 bag |
| [`patches/`](patches/) | 对第三方算法的必要修复（重装后会丢，需要重新打） |
| [`scripts/00_setup_env.sh`](scripts/00_setup_env.sh) | 环境自检与修复（幂等，无需 sudo） |
| [`results/_TEMPLATE/`](results/_TEMPLATE/) | 实验目录模板（真值 / 场景 / 对比表） |
| [`tutorials/01_pointcloud_basics.py`](tutorials/01_pointcloud_basics.py) | 点云入门（无需硬件，跑完出 3 张图） |
| [`tutorials/02_apriltag_detection.py`](tutorials/02_apriltag_detection.py) | AprilTag 链路自测（无需相机） |

**常用命令**：

```bash
# 装驱动（不需要 sudo）。两颗雷达网段不同，IP 要一起换：
bash scripts/02_get_livox.sh --lidar-ip 192.168.1.3     --device mid360s   # MID-360s
bash scripts/02_get_livox.sh --lidar-ip 192.168.100.200 \
                             --host-ip  192.168.100.150 --net --device mid360  # C01
bash scripts/02_get_livox.sh --net                               # 只配网卡：MID-360s → 192.168.1.50

python3 scripts/03_livox_discover.py                             # 探测雷达 IP + SN + dev_type
ros2 launch livox_ros_driver2 msg_MID360_launch.py               # 起驱动（先关掉 Livox Viewer 2！）
ros2 topic hz /livox/lidar                                       # 应约 10 Hz；imu 约 200 Hz
python3 scripts/benchmark_eval.py --demo                          # 看效果，无需输入
python3 scripts/benchmark_eval.py --find-stops traj.txt           # 自动找停车点 → 生成真值模板
python3 scripts/benchmark_eval.py --gt gt.txt --est traj.txt      # 两组误差（不对齐 + 对齐）
python3 scripts/benchmark_eval.py --gt gt.txt --est t.txt --segments scenes.csv
python3 scripts/benchmark_eval.py --est r1.txt r2.txt --consistency
```

**跑算法的完整流程**（三个终端，换算法只改中间那行）：

```bash
ros2 bag record -o run1 /livox/lidar /livox/imu                     # ① 录「录像带」
# ── 下面三个终端并行 ──
ros2 bag play run1 --clock -r 1.0                                   # ② A：回放
ros2 launch fast_lio mapping.launch.py use_sim_time:=true \
     rviz:=false config_file:=mid360.yaml                           #    B：跑算法
python3 scripts/traj_recorder.py --topic /Odometry \
        --out traj_fastlio2.txt                                     #    C：录轨迹
cd results/2026-10-12_首次跑通 && python3 ../../scripts/protocol_eval.py \
        traj_fastlio2.txt --distances 1.02,3.01,5.03                 # ③ 出排名表（不需要真值）

# 或者一条命令跑完（推荐）：回放 → 算法 → 轨迹 → 地图
bash scripts/05_run_fastlio_on_bag.sh rosbag/你的包 results/日期_场地/map1
#    ↑ 包名 01_/02_ 自动用 mid360_pc2.yaml，03_/04_ 自动用 c01_pc2.yaml
#      （外参不同，选错会让精度白掉；想手动指定就加 --config
python3 scripts/07_plot_map.py results/日期_场地/map1/map.pcd \
        --traj results/日期_场地/map1/traj.txt                    # 出图
```


# slam_benchmark
