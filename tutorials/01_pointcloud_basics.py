#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云数据入门 —— 从 2D 激光扫描到 3D 点云
=========================================

运行方式:
    python3 01_pointcloud_basics.py

输出:
    out_1_2d_scan.png      2D 激光扫描（极坐标 → 直角坐标）
    out_2_3d_cloud.png     3D 点云（16 线雷达）
    out_3_comparison.png   2D vs 3D 对照

本脚本不需要任何硬件、不需要 rosbag、不需要下载数据集。
所有点云都是用几何方法"模拟"出来的，目的是让你看清：
    数字  ←→  图像   的对应关系
"""

import os
import sys
import numpy as np

# ---------------------------------------------------------------------
# 环境修复（Ubuntu 特有坑，先看这里！）
# ---------------------------------------------------------------------
# 症状: ValueError: Unknown projection '3d'
#
# 原因: 系统里 /usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth
#       会在解释器【启动时】就把 mpl_toolkits 塞进 sys.modules，
#       并指向系统的旧版 matplotlib 3.5。
#       于是即使 pip 在 ~/.local 装了新版 matplotlib 3.10，
#       import mpl_toolkits.mplot3d 仍然命中那个旧缓存
#       → 新版 matplotlib 里已删除的 API 被旧代码调用 → 报错。
#
# 修法: 清掉预缓存，让 import 按 sys.path 正常解析
#       （~/.local 在 sys.path 的第 6 位，优先于 dist-packages 的第 8 位）
#
# 注意: 不要卸载系统 matplotlib！ros-humble-rqt-plot 依赖它。
for _m in [m for m in list(sys.modules)
           if m == "mpl_toolkits" or m.startswith("mpl_toolkits.")]:
    del sys.modules[_m]

import matplotlib

# 无显示器也能跑（服务器/SSH 环境）；需要交互窗口时改成 TkAgg
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  注册 '3d' projection

# ---------------------------------------------------------------------
# 中文字体配置（不配的话所有中文都会变成方框 □□□）
# ---------------------------------------------------------------------
# matplotlib 默认字体 DejaVu Sans 不含汉字。
# 系统里装了 noto-cjk，但 matplotlib 只注册了其中的 JP 面，
# 所以这里用「字体列表 + 回退」的方式，逐个尝试直到能渲染。
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP",      # 系统 noto-cjk（质量最好）
    "Droid Sans Fallback",   # 兜底，覆盖全部 CJK
    "AR PL UMing CN",        # 再兜底
    "DejaVu Sans",           # 最后退回默认（至少英文/数字正常）
]
plt.rcParams["font.family"] = "sans-serif"
# 汉字字体里常缺 U+2212(数学减号)，关掉它改用 ASCII 的 '-'，否则负号也是方框
plt.rcParams["axes.unicode_minus"] = False

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
SEP = "=" * 68


# =====================================================================
# 1. 先定义"真实世界"：一个房间
# =====================================================================
# 我们要模拟一个 8m × 6m 的房间，中间有 1 根柱子。
# 激光雷达放在房间中央偏下一点的位置。
#
#   世界坐标系：x 向右，y 向上，雷达在 (0,0)
#
#        ┌──────────────────────────────┐  y = +3
#        │                              │
#        │              ■  ← 柱子 (2,1) │
#        │                              │
#        │           ⊕ ← 雷达在 (0,0)   │
#        │                              │
#        └──────────────────────────────┘  y = -3
#      x = -4                        x = +4

ROOM = {
    "x_min": -4.0, "x_max": 4.0,
    "y_min": -3.0, "y_max": 3.0,
}
PILLAR = {"x": 2.0, "y": 1.0, "radius": 0.3}

# --- 3D 才需要的高度参数（都以雷达自身为 z=0）---
RADAR_HEIGHT_M = 1.0                      # 雷达离地 1.0 m
FLOOR_Z = -RADAR_HEIGHT_M                 # 地板在雷达下方 1.0 m
CEILING_Z = 3.0 - RADAR_HEIGHT_M          # 天花板离地 3.0 m → 相对雷达 +2.0 m
PILLAR_TOP_Z = 2.5 - RADAR_HEIGHT_M       # 柱子高 2.5 m → 相对雷达 +1.5 m


def ray_cast(angle_rad, max_range=12.0, step=0.01):
    """
    模拟一束激光：从原点 (0,0) 沿 angle_rad 方向发射，返回打到障碍物的距离。

    做法：从近到远一步步试探，第一个撞到墙或柱子的位置就是答案。
    真实雷达是测"光往返时间"，这里用几何求交来模拟。
    """
    for r in np.arange(0.1, max_range, step):
        x = r * np.cos(angle_rad)
        y = r * np.sin(angle_rad)

        # 撞到四面墙？
        if x <= ROOM["x_min"] or x >= ROOM["x_max"]:
            return r
        if y <= ROOM["y_min"] or y >= ROOM["y_max"]:
            return r

        # 撞到柱子？（柱子的横截面是圆）
        if np.hypot(x - PILLAR["x"], y - PILLAR["y"]) <= PILLAR["radius"]:
            return r

    return max_range  # 什么都没打到 → 达最大量程


def ray_cast_3d(theta, phi, max_range=12.0, step=0.01):
    """
    3D 射线投射：方向由方位角 theta 和俯仰角 phi 共同决定。

    和 2D 版的唯一区别：方向向量多了 z 分量，并且要额外判断
    「打到地板」和「打到天花板」—— 这是 2D 雷达完全不会遇到的情况。

        2D 方向: (cos θ,  sin θ)              … z 恒为 0
        3D 方向: (cos φ·cos θ, cos φ·sin θ, sin φ)

    做法和 2D 一样：从近到远一步步试探，第一个撞到东西的位置就是答案。
    """
    cos_p, sin_p = np.cos(phi), np.sin(phi)

    for r in np.arange(0.1, max_range, step):
        x = r * cos_p * np.cos(theta)
        y = r * cos_p * np.sin(theta)
        z = r * sin_p

        # (a) 撞到四面墙（无限高）
        if x <= ROOM["x_min"] or x >= ROOM["x_max"]:
            return r
        if y <= ROOM["y_min"] or y >= ROOM["y_max"]:
            return r

        # (b) 撞到柱子（柱子只有 2.5 m 高，高过它就打不到了）
        if (z <= PILLAR_TOP_Z
                and np.hypot(x - PILLAR["x"], y - PILLAR["y"]) <= PILLAR["radius"]):
            return r

        # (c) 撞到地板或天花板 ← 3D 特有
        if z <= FLOOR_Z or z >= CEILING_Z:
            return r

    return max_range


# =====================================================================
# 2. 模拟一次 2D 激光扫描
# =====================================================================
print(SEP)
print("第 1 步：模拟一次 2D 激光扫描（就像 RPLIDAR C1 做的那样）")
print(SEP)

# C1 转一圈大约采 360~500 个点，这里用 1° 分辨率 = 360 个点
N_BEAMS = 360
angles_deg = np.arange(0, 360, 360 / N_BEAMS)
angles_rad = np.deg2rad(angles_deg)

# 对每个方向发射一束激光，得到距离数组
ranges = np.array([ray_cast(a) for a in angles_rad])

print(f"雷达转一圈，采了 {N_BEAMS} 个点。")
print(f"\nranges 数组就是雷达最原始的输出（前 12 个值）：")
print("  ", np.round(ranges[:12], 3))
print(f"\n  长度: {len(ranges)}   （= 一圈采了多少个点）")
print(f"  最小值: {ranges.min():.3f} m")
print(f"  最大值: {ranges.max():.3f} m")
print(f"\n👉 注意：这个数组里【没有 x，没有 y】！只有『距离』。")
print(f"   角度信息是隐含的：第 i 个元素对应角度 {360/N_BEAMS}*i 度。")
print(f"   『距离 + 角度』就是极坐标表示的点云。")

# ---- 极坐标 → 直角坐标 ----
print("\n" + "-" * 68)
print("极坐标 → 直角坐标：x = r·cos(θ),  y = r·sin(θ)")
print("-" * 68)

xs = ranges * np.cos(angles_rad)
ys = ranges * np.sin(angles_rad)

for i in [0, 90, 180]:
    r, th = ranges[i], angles_deg[i]
    print(f"  第 {i:3d} 个点: r={r:6.3f} m, θ={th:5.1f}°"
          f"  →  x={xs[i]:7.3f}, y={ys[i]:7.3f}")

print(f"\n👉 现在每个点都有 (x, y) 了。这就是『二维点云』。")
print(f"   维度: ({len(xs)}, 2) —— 一共 {len(xs)} 个点，每个点 2 个数。")

# ---- 画出来 ----
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# 左图：极坐标视角
ax = axes[0]
ax.set_title("① 雷达看到的世界（极坐标视角）", fontsize=13)
for a in np.deg2rad(np.arange(0, 360, 30)):
    ax.plot([0, 12 * np.cos(a)], [0, 12 * np.sin(a)],
            color="lightgray", lw=0.5, zorder=1)
for r in [2, 4, 6, 8, 10]:
    c = plt.Circle((0, 0), r, fill=False, color="lightgray", lw=0.5, zorder=1)
    ax.add_patch(c)
    ax.text(r, 0.1, f"{r}m", fontsize=7, color="gray")
ax.plot(xs, ys, "o", ms=1.5, color="crimson", zorder=3, label="扫描到的点")
ax.plot(0, 0, "s", ms=12, color="steelblue", zorder=4, label="雷达位置")
for a, lab in [(0, "0°"), (90, "90°"), (180, "180°"), (270, "270°")]:
    ar = np.deg2rad(a)
    ax.text(11 * np.cos(ar), 11 * np.sin(ar), lab,
            fontsize=11, ha="center", color="darkgreen")
ax.set_xlim(-13, 13); ax.set_ylim(-13, 13)
ax.set_aspect("equal"); ax.grid(alpha=0.2)
ax.legend(loc="upper right", fontsize=9)

# 右图：直角坐标视角
ax = axes[1]
ax.set_title("② 同一次扫描（直角坐标视角）", fontsize=13)
ax.plot(xs, ys, "o", ms=2.5, color="crimson", label="点云")
# 画出真实房间轮廓做参照
room_x = [ROOM["x_min"], ROOM["x_max"], ROOM["x_max"], ROOM["x_min"], ROOM["x_min"]]
room_y = [ROOM["y_min"], ROOM["y_min"], ROOM["y_max"], ROOM["y_max"], ROOM["y_min"]]
ax.plot(room_x, room_y, "--", color="gray", lw=1, alpha=0.6, label="真实墙(参照)")
ax.add_patch(plt.Circle((PILLAR["x"], PILLAR["y"]), PILLAR["radius"],
                        color="orange", alpha=0.5, label="柱子"))
ax.plot(0, 0, "s", ms=12, color="steelblue", label="雷达位置")
ax.annotate("", xy=(xs[0], ys[0]), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="green", lw=1.2))
ax.text(xs[0] / 2, 0.25, f"θ={angles_deg[0]:.0f}°\nr={ranges[0]:.2f}m",
        fontsize=8, color="green")
ax.set_xlim(-5, 5); ax.set_ylim(-4, 4)
ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.legend(loc="upper right", fontsize=9)

plt.tight_layout()
p1 = os.path.join(OUT_DIR, "out_1_2d_scan.png")
plt.savefig(p1, dpi=110, bbox_inches="tight")
plt.close()
print(f"\n✅ 已保存图像: {p1}")


# =====================================================================
# 3. 模拟一次 3D 激光扫描
# =====================================================================
print("\n" + SEP)
print("第 2 步：模拟一次 3D 激光扫描（16 线雷达）")
print(SEP)

print("""
2D 雷达只在【一个水平面】上转 → 所有点 z 都相同 → 退化成一个平面。

3D 雷达多了一件事：【垂直方向也扫】。
  机械式（Velodyne/禾赛）: 内部有多个激光器排成不同俯仰角，一起旋转
  固态式（Livox）        : 用棱镜把光束扫成花瓣状轨迹（非重复扫描）

结果：点云从"一个圈" 变成 "一堆叠起来的圈"。

⚠️ 但 3D 的点【不只是 2D 的简单叠加】，有两件全新的事：

  (1) 斜向下的光束会打到【地板】→ 在地板上扫出一圈一圈的同心环
  (2) 斜向上的光束会打到【天花板】
  (3) 垂直方向撞不到东西的光束会"飞出去"，打到很远

  → 所以 3D 点云的"结构感"主要来自地面/天花板，而不只是墙。
  → 这也是为什么 3D 算法特别依赖"有地面"这个假设
     （LeGO-LOAM 就是专门做地面分割的）。
""")

N_RINGS = 16
# 16 线的典型俯仰角分布：从 -15° 到 +15°
elev_deg = np.linspace(-15, 15, N_RINGS)
elev_rad = np.deg2rad(elev_deg)

# 水平分辨率取粗一点，避免点太多（真实雷达 0.2° 左右）
N_AZ = 360
az_deg = np.arange(0, 360, 360 / N_AZ)
az_rad = np.deg2rad(az_deg)

print(f"雷达参数: {N_RINGS} 线，俯仰角范围 [{elev_deg[0]:.1f}°, {elev_deg[-1]:.1f}°]")
print(f"          水平分辨率 {360/N_AZ:.1f}°  →  每圈 {N_AZ} 个点")
print(f"          总点数 = {N_RINGS} × {N_AZ} = {N_RINGS * N_AZ}")

points_3d = []
for phi in elev_rad:
    for th in az_rad:
        # 关键：3D 的射线方向要同时考虑俯仰角和方位角
        r = ray_cast_3d(th, phi)
        x = r * np.cos(phi) * np.cos(th)
        y = r * np.cos(phi) * np.sin(th)
        z = r * np.sin(phi)
        points_3d.append((x, y, z))

points_3d = np.array(points_3d)
print(f"\n三维点云数组的形状: {points_3d.shape}"
      f"  →  {points_3d.shape[0]} 个点，每点 {points_3d.shape[1]} 个数 (x,y,z)")

print(f"\n前 5 个点长这样（每行是一个点的 x y z）：")
for p in points_3d[:5]:
    print(f"    x={p[0]:8.4f}   y={p[1]:8.4f}   z={p[2]:8.4f}")

print(f"\n👉 和 2D 点云的唯一区别：现在每个点有【3】个数，不是 2 个。")
print(f"   z 不再是常数，而是随俯仰角变化。")

# ---- 画 3D 图 ----
fig = plt.figure(figsize=(15, 6))

ax = fig.add_subplot(121, projection="3d")
colors = plt.cm.viridis(np.linspace(0, 1, N_RINGS))
for i in range(N_RINGS):
    seg = points_3d[i * N_AZ:(i + 1) * N_AZ]
    ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], ".",
            ms=1.2, color=colors[i], alpha=0.8)
ax.scatter([0], [0], [0], s=90, color="red", marker="*",
           label="雷达位置", zorder=10)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
ax.set_title(f"③ 3D 点云：{N_RINGS} 线扫描结果\n(颜色 = 不同扫描线)",
             fontsize=12)
ax.view_init(elev=18, azim=-60)
ax.legend(fontsize=9)

# 俯视图：对比 2D 和 3D 投影
ax2 = fig.add_subplot(122)
ax2.plot(xs, ys, ".", ms=3, color="crimson", alpha=0.9,
         label="2D 扫描（1 条线）")
ax2.plot(points_3d[:, 0], points_3d[:, 1], ".", ms=1.2,
         color="steelblue", alpha=0.35,
         label=f"3D 点云俯视投影（{N_RINGS} 条线叠加）")
ax2.plot(0, 0, "s", ms=12, color="black", zorder=5)
ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)")
ax2.set_title("④ 俯视图：3D 点云投影到地面\n"
              "（外墙轮廓与 2D 重合；内部同心环来自地板/天花板）",
              fontsize=11)
ax2.set_aspect("equal"); ax2.grid(alpha=0.3); ax2.legend(fontsize=9)
ax2.set_xlim(-5, 5); ax2.set_ylim(-4, 4)

plt.tight_layout()
p2 = os.path.join(OUT_DIR, "out_2_3d_cloud.png")
plt.savefig(p2, dpi=110, bbox_inches="tight")
plt.close()
print(f"\n✅ 已保存图像: {p2}")


# =====================================================================
# 4. 数字到底是怎么存的？（最重要的一节）
# =====================================================================
print("\n" + SEP)
print("第 3 步：数据在文件/内存里到底是什么样子")
print(SEP)

print("""
【形式 A】ROS 的 LaserScan（2D 雷达用）
    消息里只有:
        angle_min, angle_max, angle_increment   ← 角度信息（元数据，只有一份）
        ranges[]                                ← 距离数组（真正的数据）
    所以 360 个点只占 360 个 float。角度靠"第几个元素"推出来。

【形式 B】ROS 的 PointCloud2（3D 雷达用）
    消息里有一个二进制大 blob（data[]），描述如下:
        FIELDS: x y z intensity
        POINT_STEP: 16          ← 每个点占 16 字节 (4 个 float32)
        WIDTH: 5760             ← 点数
    data 就是 5760 × 16 = 92160 字节的原始内存。
    你必须"按 16 字节一组"去切，才能还原出每个点。

【形式 C】KITTI 的 .bin 文件（最常见的数据集格式）
    干脆连头部都没有，就是裸的 float32:
        [x0,y0,z0,i0, x1,y1,z1,i1, x2,y2,z2,i2, ...]
    x,y,z = 坐标，i = 反射强度
    读法: np.fromfile(path, dtype=np.float32).reshape(-1, 4)
""")

# ---- 真的演示一下这三种形式 ----
print("-" * 68)
print("实际演示：把同一批点用三种方式存一遍")
print("-" * 68)

sample = points_3d[:3].copy()
intensity = np.array([42.0, 43.0, 41.0])
sample_xyzi = np.hstack([sample, intensity.reshape(-1, 1)]).astype(np.float32)

print(f"\n【形式 C】KITTI .bin 格式")
print(f"  3 个点，每点 4 个 float32 → 内存里是 {sample_xyzi.size} 个 float32 "
      f"= {sample_xyzi.nbytes} 字节")
print(f"  展开成数组: {sample_xyzi.ravel()}")
print(f"  还原成点: reshape(-1, 4) →")
for row in sample_xyzi:
    print(f"      x={row[0]:8.4f}  y={row[1]:8.4f}  z={row[2]:8.4f}  "
          f"intensity={row[3]:.0f}")

print(f"\n【形式 A】LaserScan 格式（只存距离）")
print(f"  同一个房间的 2D 扫描，只需要 {len(ranges)} 个 float"
      f" = {ranges.nbytes} 字节")
print(f"")
print(f"  每个点的存储开销对比：")
print(f"    2D LaserScan :  4 字节/点  (只存 1 个距离；角度靠下标推出来，不占空间)")
print(f"    3D .bin(无强度): 12 字节/点 (x,y,z 三个 float32)")
print(f"    3D .bin(带强度): 16 字节/点 (x,y,z,intensity 四个 float32)")
print(f"  → 3D 点云每点比 2D 贵 3~4 倍。再加上 3D 点数本来就多一个量级，")
print(f"     所以 3D 数据集体积通常是 2D 的几十倍（这也是你磁盘吃紧的原因）。")

# ---- 写一个 PCD ASCII 文件让他亲眼看 ----
pcd_path = os.path.join(OUT_DIR, "out_4_example.pcd")
with open(pcd_path, "w") as f:
    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z intensity\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write(f"WIDTH {len(points_3d)}\n")
    f.write("HEIGHT 1\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write(f"POINTS {len(points_3d)}\n")
    f.write("DATA ascii\n")
    for p, inten in zip(points_3d[:200], np.linspace(0, 100, 200)):
        f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {inten:.1f}\n")

print(f"\n✅ 已写出一份真实的 PCD 文本文件: {pcd_path}")
print(f"   用文本编辑器打开它，你会看到【纯文本的 x y z intensity】。")
print(f"   💡 建议现在就 cat 一下看看前 20 行 ——")
print(f"      这是建立直觉最快的方法。")


# =====================================================================
# 5. 对照图
# =====================================================================
fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))

ax = axes[0]
ax.set_title("2D 雷达 (RPLIDAR C1)\n1 条扫描线", fontsize=12)
ax.plot(xs, ys, "o", ms=2, color="crimson")
ax.plot(0, 0, "s", ms=11, color="steelblue")
ax.set_xlim(-5, 5); ax.set_ylim(-4, 4); ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

ax = axes[1]
ax.set_title("16 线 3D 雷达\n16 条扫描线（俯视）", fontsize=12)
for i in range(N_RINGS):
    seg = points_3d[i * N_AZ:(i + 1) * N_AZ]
    ax.plot(seg[:, 0], seg[:, 1], ".", ms=1, alpha=0.5)
ax.plot(0, 0, "s", ms=11, color="steelblue")
ax.set_xlim(-5, 5); ax.set_ylim(-4, 4); ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x (m)")

ax = axes[2]
ax.set_title("64 线 3D 雷达\n64 条扫描线（俯视）", fontsize=12)
for i in range(64):
    phi = np.deg2rad(np.linspace(-24, 2, 64)[i])
    seg = []
    for th in az_rad:
        r = ray_cast_3d(th, phi)
        seg.append((r * np.cos(phi) * np.cos(th), r * np.cos(phi) * np.sin(th)))
    seg = np.array(seg)
    ax.plot(seg[:, 0], seg[:, 1], ".", ms=0.6, alpha=0.35)
ax.plot(0, 0, "s", ms=11, color="steelblue")
ax.set_xlim(-5, 5); ax.set_ylim(-4, 4); ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x (m)")

fig.suptitle("线数越多 → 点越密 → 对环境的描述越完整（但数据量也越大）",
             fontsize=13, y=1.02)
plt.tight_layout()
p3 = os.path.join(OUT_DIR, "out_3_comparison.png")
plt.savefig(p3, dpi=110, bbox_inches="tight")
plt.close()
print(f"✅ 已保存图像: {p3}")


# =====================================================================
# 总结
# =====================================================================
print("\n" + SEP)
print("总结：记住这 5 句话就够了")
print(SEP)
print("""
1. 点云 = 一大堆 (x, y, z) 坐标点。没有别的。

2. 一个点 = 一次激光测距。距离 + 角度 → 用三角函数算出 x, y, z。

3. 2D 雷达 = 只在一个平面扫 → 点的 z 是常数 → 退化成平面点云。
   3D 雷达 = 垂直方向也扫 → 每个点有独立的 z（由俯仰角决定）。

4. 2D 雷达的 LaserScan 只存 ranges[] 一个数组（省内存）；
   3D 雷达必须存每个点的 x,y,z（数据量大得多）。

5. 看不懂格式时，先转成 PCD ASCII 文本文件，用编辑器打开看数字。
   看懂了数字排布，其他格式都是同一件事的不同包装。
""")
print(SEP)
print("生成的 4 个文件（都在本脚本同目录）：")
print("  out_1_2d_scan.png      2D 扫描：极坐标 vs 直角坐标")
print("  out_2_3d_cloud.png     3D 点云：16 线效果 + 俯视投影")
print("  out_3_comparison.png   2D / 16线 / 64线 对照")
print("  out_4_example.pcd      ⭐ 真实 PCD 文本文件，务必打开看看")
print(SEP)
