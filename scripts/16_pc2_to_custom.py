#!/usr/bin/python3
"""PointCloud2 → livox_ros_driver2/CustomMsg 转换节点。

为什么需要：
    `rviz_MID360_launch.py` 录出来的 bag 是 sensor_msgs/PointCloud2，
    而 LIO-SAM / 部分 Livox 系算法只订阅 livox_ros_driver2/msg/CustomMsg。
    回放 bag 时同时跑本节点，把 PointCloud2 转成 CustomMsg 即可。

用法：
    /usr/bin/python3 scripts/16_pc2_to_custom.py                # 默认 /livox/lidar → /livox/lidar_custom
    /usr/bin/python3 scripts/16_pc2_to_custom.py --ros-args \
        -p out_topic:=/livox/lidar_custom -p in_topic:=/livox/lidar

关键点（已实测本机 MID-360s bag）：
    - 每点自带 float64 `timestamp`，单位纳秒，帧跨度约 100 ms
    - 约 0.7% 相邻点存在微回退（最大 352 us），做单调化抹平
    - intensity 实测 0~151，转 uint8 无损
    - line 取值为 0~3，对应 N_SCAN=4
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint

NP_TYPE = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
           5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def _pick_time_unit(ts0_ns_guess, declared_ns):
    """把各字段时间戳统一到纳秒。返回倍数。"""
    if declared_ns <= 0:
        return 1.0
    ratio = ts0_ns_guess / declared_ns
    for mult, lo, hi in ((1e9, 0.5, 2.0),        # 秒
                         (1e6, 0.5, 2.0),        # 毫秒
                         (1e3, 0.5, 2.0),        # 微秒
                         (1.0, 0.5, 2.0)):       # 纳秒
        if lo <= ratio / mult <= hi:
            return mult
    return 1.0


class Pc2ToCustom(Node):
    def __init__(self):
        super().__init__('pc2_to_custom')
        self.declare_parameter('in_topic', '/livox/lidar')
        self.declare_parameter('out_topic', '/livox/lidar_custom')
        self.declare_parameter('lidar_id', 0)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        # 订阅用 BEST_EFFORT：既能收 BEST_EFFORT 也能收 RELIABLE 的 bag
        qos_sub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        # 发布用 RELIABLE：RELIABLE 发布者可满足 BEST_EFFORT(如 LIO-SAM) 与
        # RELIABLE 两类订阅者；反之则不行
        qos_pub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CustomMsg, out_topic, qos_pub)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, qos_sub)

        self.n_frames = 0
        self.n_skipped = 0
        self.warned = set()
        self.get_logger().info(f'转换节点已启动: {in_topic} (PointCloud2) '
                               f'-> {out_topic} (CustomMsg)')

    def _warn_once(self, key, msg):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(msg)

    def on_cloud(self, msg: PointCloud2):
        fmap = {f.name: f for f in msg.fields}
        missing = {'x', 'y', 'z'} - fmap.keys()
        if missing:
            self._warn_once('xyz', f'点云缺少字段 {missing}，无法转换')
            self.n_skipped += 1
            return

        n = msg.width * msg.height
        if n == 0:
            self.n_skipped += 1
            return

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

        def col(name, default=None):
            f = fmap.get(name)
            if f is None:
                return None if default is None else np.full(n, default)
            dt = NP_TYPE.get(f.datatype)
            if dt is None:
                return None if default is None else np.full(n, default)
            nb = np.dtype(dt).itemsize
            return buf[:, f.offset:f.offset + nb].copy().view(dt).ravel()

        x = col('x').astype(np.float32)
        y = col('y').astype(np.float32)
        z = col('z').astype(np.float32)

        inten = col('intensity')
        if inten is None:
            reflect = np.zeros(n, dtype=np.uint8)
        else:
            reflect = np.clip(np.rint(inten), 0, 255).astype(np.uint8)

        tag = col('tag', 0).astype(np.uint8)
        line = col('line', 0).astype(np.uint8)

        hdr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        raw_ts = col('timestamp')
        if raw_ts is None:
            self._warn_once('no_ts',
                            '点云无 `timestamp` 字段，按帧内均匀线性插值生成 offset_time')
            # 无逐点时间戳：用均匀线性（对静止或慢速场景足够）
            offset_ns = (np.arange(n, dtype=np.float64) / max(n - 1, 1) * 1e8)
            timebase = hdr_ns
        else:
            unit = _pick_time_unit(float(raw_ts[0]), float(hdr_ns))
            ts_ns = raw_ts.astype(np.float64) * unit
            rel = ts_ns - ts_ns[0]
            np.maximum(rel, 0.0, out=rel)       # 抹掉微回退
            np.maximum.accumulate(rel, out=rel)  # 强制单调
            span = float(rel[-1]) if n > 1 else 0.0
            if span > 4.0e9:
                self._warn_once('span', f'帧跨度 {span/1e9:.2f}s 超出 uint32 纳秒容量，已截断')
                rel = np.minimum(rel, 4.0e9)
            offset_ns = rel
            timebase = hdr_ns if unit == 1.0 else int(ts_ns[0])

        out = CustomMsg()
        out.header = msg.header
        out.timebase = int(timebase)
        out.point_num = n
        out.lidar_id = self.get_parameter('lidar_id').value
        out.rsvd = [0, 0, 0]

        # 逐点构造（Python 侧唯一瓶颈，n≈2e4 时约 10 ms/帧）
        off = offset_ns.astype(np.uint32).tolist()
        xs, ys, zs = x.tolist(), y.tolist(), z.tolist()
        rs, tg, ln = reflect.tolist(), tag.tolist(), line.tolist()
        # ROS2 生成的 Python 消息类只接受关键字参数
        out.points = [
            CustomPoint(offset_time=o, x=a, y=b, z=c,
                        reflectivity=r, tag=t, line=l)
            for o, a, b, c, r, t, l in zip(off, xs, ys, zs, rs, tg, ln)
        ]

        self.pub.publish(out)
        self.n_frames += 1
        if self.n_frames % 50 == 1:
            self.get_logger().info(
                f'帧 {self.n_frames}: {n} 点, 跨度 {offset_ns[-1]/1e6:.1f} ms, '
                f'reflectivity 0~{int(reflect.max())}, line 0~{int(line.max())}')


def main():
    rclpy.init()
    node = Pc2ToCustom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'退出：转换 {node.n_frames} 帧, 跳过 {node.n_skipped} 帧')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
