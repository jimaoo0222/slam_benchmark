#!/usr/bin/env python3
"""订阅 FAST-LIO 的累积地图话题，退出时存成 PCD。

为什么需要它
────────────
FAST-LIO2 的 ROS 2 移植版里，`src/laserMapping.cpp:513-540` 那段「累积并保存
PCD」的代码**整段被注释掉了**（注释里写着 pcd save 会拖慢实时性能），于是：

    · pcd_save_en: true  **完全失效**
    · pcl_wait_save 永远是空的 → 退出时那个 if 不成立 → 一个 PCD 都不写

而 `publish_map()` 是活的：它把每帧降采样点云累加进 `pcl_wait_pub` 并发到
`/Laser_map`。所以订阅这个话题拿最后一帧，就是完整地图 —— 不用改别人的源码。

用法
────
    # 常见做法：和算法、回放一起跑（scripts/05_run_fastlio_on_bag.sh 已自动调用）
    python3 scripts/06_save_map_pcd.py --topic /Laser_map --out map.pcd

退出
────
    Ctrl+C。每收到一帧就覆盖写一次，所以哪怕中途崩了，也有一份最新的图。

⚠️ 内存：地图会一直长。326 秒的包大约几百万点（几十 MB），27 GB 内存没问题；
   但如果跑很长（>30 分钟），建议用 --voxel 边收边降采样。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
except ImportError as exc:  # pragma: no cover
    sys.exit(
        f"❌ 导入失败: {exc}\n"
        "   需要在 ROS 2 环境里运行：\n"
        "       source /opt/ros/humble/setup.bash"
    )


def cloud_to_xyzi(msg: PointCloud2, voxel: float = 0.0) -> np.ndarray | None:
    """PointCloud2 → (N,4) float32 数组 [x, y, z, intensity]。"""
    offsets = {f.name: f.offset for f in msg.fields}
    for axis in ("x", "y", "z"):
        if axis not in offsets:
            return None

    n = len(msg.data) // msg.point_step
    if n == 0:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    cols = []
    for name in ("x", "y", "z", "intensity"):
        off = offsets.get(name)
        if off is None:
            cols.append(np.zeros(n, dtype=np.float32))
        else:
            cols.append(raw[:, off:off + 4].copy().view(np.float32).ravel())
    pts = np.stack(cols, axis=1)

    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if voxel > 0 and pts.shape[0] > 0:
        key = np.floor(pts[:, :3] / voxel).astype(np.int64)
        _, keep = np.unique(key, axis=0, return_index=True)
        pts = pts[np.sort(keep)]
    return pts


def write_pcd(path: Path, pts: np.ndarray) -> None:
    """写 binary PCD（PCL 能直接读）。"""
    n = pts.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


class MapSaver(Node):
    def __init__(self, topic: str, out: Path, voxel: float):
        super().__init__("map_saver")
        self.out = out
        self.voxel = voxel
        self.latest: np.ndarray | None = None
        self.n_recv = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.get_logger().info(f"订阅 {topic} → {out}")

    def cb(self, msg: PointCloud2) -> None:
        pts = cloud_to_xyzi(msg, self.voxel)
        if pts is None:
            return
        self.latest = pts
        self.n_recv += 1
        # 每 10 帧落一次盘，防止中途崩溃丢图
        if self.n_recv % 10 == 1:
            write_pcd(self.out, pts)
            self.get_logger().info(
                f"第 {self.n_recv} 帧地图：{pts.shape[0]} 点 → 已写入 {self.out.name}"
            )

    def save(self) -> int:
        if self.latest is None:
            self.get_logger().error(
                "❌ 一帧地图都没收到 —— 检查算法的 publish.map_en 是否为 true"
            )
            return 0
        write_pcd(self.out, self.latest)
        self.get_logger().info(
            f"✅ 最终地图：{self.latest.shape[0]} 点 → {self.out}"
        )
        return self.latest.shape[0]


def main() -> int:
    ap = argparse.ArgumentParser(description="订阅累积地图话题并存成 PCD")
    ap.add_argument("--topic", default="/Laser_map", help="累积地图话题（默认 /Laser_map）")
    ap.add_argument("--out", required=True, help="输出 .pcd 路径")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="可选：边收边按该体素边长降采样（米），默认不降")
    args = ap.parse_args()

    out = Path(args.out)
    if out.suffix.lower() != ".pcd":
        out = out.with_suffix(".pcd")
    out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = MapSaver(args.topic, out, args.voxel)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n = node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
