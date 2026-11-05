#!/usr/bin/env python3
"""
把 ROS 2 的 Odometry 话题录成 TUM 格式轨迹文件

    todo: 这是「算法 → 评测工具」之间缺的那一环
          FAST-LIO2 / Point-LIO / GLIM 跑完都只发一个 Odometry 话题，
          本脚本负责把它落成文件，才能喂给 benchmark_eval.py

用法
────
    # 终端 A：跑算法（回放 bag 或接真实雷达）
    # 终端 B：录轨迹
    python3 scripts/traj_recorder.py --topic /Odometry --out traj_fastlio2.txt

    # 跑完 Ctrl+C，文件里就是标准 TUM 格式：
    #   timestamp tx ty tz qx qy qz qw

常用话题名
──────────
    FAST-LIO2     /Odometry            （也可能是 /lio/odom 等，看 launch）
    Point-LIO     /Odometry
    GLIM          /glim_ros/odom
    KISS-ICP      /odometry

不确定话题名？另开终端跑：
    ros2 topic list | grep -i odom
"""
import argparse
import sys
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
except ImportError as e:
    sys.exit(
        f"❌ 导入失败: {e}\n"
        "   本脚本需要在 ROS 2 环境里运行：\n"
        "       source /opt/ros/humble/setup.bash"
    )


class TrajRecorder(Node):
    def __init__(self, topic: str, out_path: Path):
        super().__init__("traj_recorder")
        self.out_path = out_path
        self.f = out_path.open("w")
        self.f.write("# timestamp tx ty tz qx qy qz qw\n")
        self.n = 0

        # ⚠️ 订阅端用 BEST_EFFORT 最保险：
        #    reliable 发布 + best_effort 订阅  → 兼容 ✅
        #    best_effort 发布 + reliable 订阅  → 不兼容 ❌（收不到）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(Odometry, topic, self.cb, qos)
        self.get_logger().info(f"订阅 {topic}  →  写入 {self.out_path}")
        self.get_logger().info("按 Ctrl+C 结束并保存")

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(
            f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
            f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
        )
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()
            self.get_logger().info(f"已记录 {self.n} 帧")

    def close(self):
        self.f.flush()
        self.f.close()
        if self.n == 0:
            self.get_logger().error(
                "❌ 一帧都没收到 —— 检查话题名对不对：ros2 topic list | grep -i odom"
            )
        else:
            self.get_logger().info(f"✅ 共记录 {self.n} 帧  →  {self.out_path}")


def main():
    ap = argparse.ArgumentParser(description="Odometry → TUM 轨迹记录器")
    ap.add_argument("--topic", default="/Odometry", help="要订阅的话题（默认 /Odometry）")
    ap.add_argument("--out", required=True, help="输出的 TUM 轨迹文件，例 traj_fastlio2.txt")
    args = ap.parse_args()

    out = Path(args.out)
    if out.suffix != ".txt":
        out = out.with_suffix(".txt")

    rclpy.init()
    node = TrajRecorder(args.topic, out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
