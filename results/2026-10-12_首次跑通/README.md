# 2026-10-12 首次跑通（冒烟测试）

> 注意：**这不是一次正式实验。** 雷达放在桌上没动过，四项指标里的
> 「② 可重复性」「③ 尺度偏差」都算不出来。
> 这个目录的唯一作用是**证明「录像带 → 算法 → 轨迹 → 评测」整条链通了**。

## 这次到底做了什么

```
① 驱动接真雷达           → 录成 run1/                     有
② 回放 run1/ 喂给 FAST-LIO2 → 算出轨迹 traj_fastlio2.txt   有
③ 轨迹喂给 protocol_eval.py → 出 eval_fastlio2/            有
```

## 参数

| 项 | 值 |
|---|---|
| 雷达 | MID-360s，SN `47MDMBB0030285`，IP `192.168.1.3`，自报 `dev_type=9` |
| 驱动 | `livox_ros_driver2` v1.2.8，配置 `MID360_config.json` |
| 包 | `run1/` —— 29.8 s，6268 条（雷达 299 帧 @10 Hz / IMU 5969 帧 @200 Hz），116.6 MB |
| 算法 | FAST-LIO2（`andrewliang01/FAST_LIO_ROS2_HUMBLE`，分支 `ros2_humble_support`） |
| 配置 | `fast_lio/config/mid360.yaml` —— **官方默认值，一行未改** |
| 轨迹 | `traj_fastlio2.txt` —— 292 个位姿，跨 29.1 s |

## 结果（只能看数字量级，不能当结论）

| 指标 | 值 | 说明 |
|---|---|---|
| ① 稳定性（最大漂移中位数） | **7.2 mm** | 雷达静止，所以这是算法的**静止噪声地板** |
| ④ 闭合误差（首尾距离） | **5.3 mm** | 静止状态下"走一圈"的距离 ≈ 0 |
| 轨迹总长 | 0.72 m | 雷达没动，这 0.72 m **全是噪声** |
| 最大漂移 | 54 mm | 29 s 内 |

> **这组数字本身有价值**：一个静止的雷达，算法本该输出「一直待在原点」。
> 实测漂了 72 cm / 最大 54 mm —— 这就是 FAST-LIO2 在这台雷达上的**噪声底线**。
> 将来跑真实轨迹时，比这个更小的位移都是噪声，不是运动。

## 怎么复现

```bash
# 1. 录一份新包（雷达要上电、网线插好、Livox Viewer 2 必须关掉）
source /opt/ros/humble/setup.bash
source ~/livox_ws/livox_env.sh
source ~/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py          # 终端 A
ros2 bag record -o run2 /livox/lidar /livox/imu             # 终端 B

# 2. 回放 + 跑算法 + 录轨迹（三个终端）
ros2 bag play run1 --clock -r 1.0                                    # 终端 A
ros2 launch fast_lio mapping.launch.py rviz:=false use_sim_time:=true \
     config_file:=mid360.yaml                                        # 终端 B
python3 ~/slam_benchmark/scripts/traj_recorder.py \
        --topic /Odometry --out traj_fastlio2.txt                    # 终端 C
```

> 注意：用 `--clock` 回放时，算法必须带 `use_sim_time:=true`，否则时间戳对不上。

## 怎么变成一次真实验

1. 按 [02_操作手册.md](../../docs/03_四算法手动启动与RViz.md) 第 3 步的协议采 **3~5 个包**：
   同一路径、每个停车点停稳 5 秒、回到原点
2. 卷尺量相邻停车点距离 → `distances.txt`
3. 每个包都跑一遍算法 → `run1_fastlio2.txt` `run2_fastlio2.txt` …
4. `python3 ../../scripts/protocol_eval.py run1_*.txt run2_*.txt run3_*.txt`
5. 填 `对比表.md`
