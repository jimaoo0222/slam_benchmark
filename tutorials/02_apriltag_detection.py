#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag 真值链路自测 —— 不需要相机、不需要打印纸
==================================================

运行方式:
    python3 tutorials/02_apriltag_detection.py

输出（都写在 tutorials/ 目录下，编号接在 01 脚本的 out_1~out_4 之后）:
    out_5_scene.png                          虚拟场景俯视图
    out_6_detection.png                      渲染出的图像 + 检测结果
    out_7_trajectory.png                     解算轨迹 vs 真值轨迹
    out_8_error.png                          逐帧误差 + 统计
    camera_trajectory_estimated_tum.txt      解算出的轨迹（TUM 格式）
    camera_trajectory_groundtruth_tum.txt    真值轨迹（TUM 格式）

配套文档: docs/13_AprilTag实施手册.md

------------------------------------------------------------------------
为什么要有这个脚本？
------------------------------------------------------------------------
AprilTag 方案里最容易出错的**不是打印纸，而是代码**：

    · 内参用对了没有？
    · 角点顺序和 objectPoints 对上了没有？
    · solvePnP 该用哪个模式？
    · 四元数里 qw 是第几位？
    · 输出的 TUM 格式对不对？

这些错了都不会报错，只会得到一条"看起来像那么回事、其实全错"的轨迹。
所以本脚本**自己造数据、自己解算、再和真值对比**：

    造场景 → 渲染图像 → 检测 → 解算 → 对比 → 报误差

如果整条链路正确，最后报出来的误差应该是**毫米级**（因为造数据时
用的是同一个成像模型，剩下的只是像素量化的误差）。
等真实器材到了，只要把"渲染图像"换成"相机拍的照片"就行。
"""

import os
import sys
import pathlib

# ---------------------------------------------------------------------
# 环境准备
# ---------------------------------------------------------------------
# 强制用 Agg 后端：不弹窗、可无人值守（详见 common/plot_setup.py 的注释）
os.environ.setdefault("SLAM_BENCH_HEADLESS", "1")

# 让 "from common import plot_setup" 能找到项目根目录
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common import plot_setup  # noqa: E402  自动完成字体/3D 修复

import numpy as np  # noqa: E402
import cv2  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
SEP = "=" * 68


# =====================================================================
# 参数区 —— 想改实验条件就改这里
# =====================================================================
IMG_W, IMG_H = 1280, 720          # 相机分辨率
FX = FY = 800.0                   # 焦距（像素）
CX, CY = IMG_W / 2, IMG_H / 2     # 主点（光轴打在图像上的位置）
K = np.array([[FX, 0.0, CX],
              [0.0, FY, CY],
              [0.0, 0.0, 1.0]])

# 畸变系数（k1,k2,p1,p2,k3）。真实相机靠标定得到，这里直接给定。
# ⚠️ 故意给一个不为零的 k1：这样才能演示"为什么必须去畸变"。
DIST = np.array([-0.08, 0.01, 0.0, 0.0, 0.0])

TAG_SIZE = 0.20      # ★ marker（黑边外沿）的边长，单位米 —— 就是"卡尺量出来的值"
TAG_PX = 400         # marker 的渲染分辨率（越大越清晰）
PAD_PX = 60          # marker 外面再留一圈白边（quiet zone）
BOARD_PX = TAG_PX + 2 * PAD_PX

N_FRAMES = 180       # 模拟多少帧
TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11
# ⚠️ 实地采集建议换成 tagStandard41h12（检测更稳、ID 更多）。
#    这里用 36h11 是因为 OpenCV 自带，不用装库、不用下载图片。


# =====================================================================
# 0. 造 tag 图片
# =====================================================================
print(SEP)
print("第 0 步：造 AprilTag 图片")
print(SEP)

dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)


def make_tag_image(tag_id: int) -> np.ndarray:
    """
    生成一张 tag 图片，外面加白边。

    关键点（实测确认）:
        cv2.aruco.generateImageMarker(dict, id, P) 生成的 P×P 图像里，
        marker（黑边外沿）【恰好占满整幅图像】。
        所以 "marker 的像素边长 = P"，不需要再换算比例。

    但只有白边=1 个模块宽时，检测器会失败（实测 ids=None）。
    必须自己再加一圈较宽的白边。
    """
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, TAG_PX)
    return cv2.copyMakeBorder(marker, PAD_PX, PAD_PX, PAD_PX, PAD_PX,
                              cv2.BORDER_CONSTANT, value=255)


# 整幅 board 的物理边长（marker + 两侧白边）
BOARD_SIZE = TAG_SIZE * BOARD_PX / TAG_PX

_s = TAG_SIZE / TAG_PX   # 米 / 像素（marker 物理尺寸 ÷ marker 像素尺寸）

# board 图像的 4 个角（渲染时拿它做透视贴图）。
# 顺序固定为【左上 → 右上 → 右下 → 左下】—— 和检测器返回的角点顺序一致（实测确认）。
BOARD_CORNERS_PX = np.array([[0, 0], [BOARD_PX, 0],
                             [BOARD_PX, BOARD_PX], [0, BOARD_PX]], dtype=float)

print(f"tag family   : APRILTAG_36h11  (markerSize={dictionary.markerSize})")
print(f"marker 物理边长 TAG_SIZE = {TAG_SIZE*100:.1f} cm  ← 卡尺量的就是这个")
print(f"marker 渲染分辨率         = {TAG_PX} px")
print(f"整幅图片(含白边) BOARD_SIZE = {BOARD_SIZE*100:.1f} cm")
print(f"换算比例          = {_s*1000:.3f} mm/px")
print("\n👉 记住 TAG_SIZE 的定义：")
print("   【黑边外沿到黑边外沿】，不是图案的外框。量错了整条轨迹尺度就错了。")


# =====================================================================
# 1. 造虚拟场景：一个房间 + 四面墙上的 tag
# =====================================================================
print("\n" + SEP)
print("第 1 步：造虚拟场景（8m × 6m 的房间，四壁贴 tag）")
print(SEP)

ROOM_X = (-4.0, 4.0)
ROOM_Y = (-3.0, 3.0)
TAG_Z = 1.30          # tag 挂在离地 1.3 m（和相机高度一致）

# 每面墙：法向（朝房间内）、起点、水平方向、数量、间距
WALLS = [
    dict(normal=(-1, 0, 0), origin=(4.0, -2.25), direction=(0, 1, 0), n=4),
    dict(normal=(1, 0, 0), origin=(-4.0, -2.25), direction=(0, 1, 0), n=4),
    dict(normal=(0, -1, 0), origin=(-2.25, 3.0), direction=(1, 0, 0), n=4),
    dict(normal=(0, 1, 0), origin=(-2.25, -3.0), direction=(1, 0, 0), n=4),
]
TAG_SPACING = 1.5

def square_corners_world(center, outward, half_size, up_world=(0.0, 0.0, 1.0)):
    """
    算出一个"贴在墙上、正面朝 outward"的正方形，4 个角在世界系里的坐标。

    返回顺序固定为【左上 → 右上 → 右下 → 左下】—— 和 OpenCV 检测器返回的
    角点顺序一致（实测确认）。两边顺序必须一一对应，否则解算结果会整体镜像
    或转 90°，而且**不会报任何错**，只会给你一条看起来正常、其实全错的轨迹。

    ⚠️ 这里有一个极容易踩的坑（本脚本第一版就栽在这上面）：
        相机的图像坐标系是【左手系】（x 向右、y 向下、z 朝向观察者）。
        如果照搬它去构造 tag 的旋转矩阵（旋转矩阵要求行列式 = +1），
        得到的图案是【镜像】的 —— 检测器会"看到候选"但解不出码，
        表现为 detectMarkers 一直返回 ids=None。
        正确做法是从"观察者的右手方向"反推：right = (-outward) × up。
    """
    z_out = np.asarray(outward, float)
    z_out = z_out / np.linalg.norm(z_out)
    up = np.asarray(up_world, float)
    up = up / np.linalg.norm(up)

    # 观察者站在 +z_out 一侧、朝 -z_out 看，那么"右手边"是：
    right = np.cross(-z_out, up)
    right = right / np.linalg.norm(right)

    h = float(half_size)
    return np.array([
        center - h * right + h * up,     # 左上
        center + h * right + h * up,     # 右上
        center + h * right - h * up,     # 右下
        center - h * right - h * up,     # 左下
    ])


TAGS = {}   # {tag_id: dict(center, normal, corners_marker(4x3), img)}
_next_id = 1

for wall in WALLS:
    n_vec = np.array(wall["normal"], float)      # 法向，朝房间内
    origin = np.array(wall["origin"], float)
    d_vec = np.array(wall["direction"], float)

    for k in range(wall["n"]):
        center = np.array([origin[0] + d_vec[0] * TAG_SPACING * k,
                           origin[1] + d_vec[1] * TAG_SPACING * k,
                           TAG_Z])
        TAGS[_next_id] = dict(
            center=center,
            normal=n_vec,
            # 解算时要用 marker（黑边外沿）的 4 个角 —— 检测器返回的就是它们
            corners_marker=square_corners_world(center, n_vec, TAG_SIZE / 2),
            img=make_tag_image(_next_id),
        )
        _next_id += 1

print(f"共布置 {len(TAGS)} 个 tag：")
for tid, tag in TAGS.items():
    c = tag["center"]
    print(f"  ID {tid:2d}  中心=({c[0]:5.2f},{c[1]:5.2f},{c[2]:4.2f})  "
          f"法向={tuple(int(v) for v in tag['normal'])}")


# =====================================================================
# 2. 造虚拟相机轨迹
# =====================================================================
print("\n" + SEP)
print("第 2 步：造一条虚拟相机轨迹（沿椭圆绕场一周）")
print(SEP)

RX, RY = 2.6, 1.9        # 椭圆半径
CAM_Z = TAG_Z            # 与 tag 同高
SKEW_DEG = 25.0          # ★ 视线相对径向偏多少度（故意斜视，见下方说明）

theta = np.linspace(0.0, 2 * np.pi, N_FRAMES, endpoint=False)

gt_pos = np.zeros((N_FRAMES, 3))
gt_rot = np.zeros((N_FRAMES, 3, 3))       # 相机→世界 的旋转

for i, th in enumerate(theta):
    pos = np.array([RX * np.cos(th), RY * np.sin(th), CAM_Z])
    gt_pos[i] = pos

    radial = np.array([np.cos(th), np.sin(th), 0.0])          # 朝墙（向外）
    tangent = np.array([-np.sin(th), np.cos(th), 0.0])        # 沿椭圆

    # ⚠️ 为什么故意斜 25°：
    #    如果相机正对 tag（视线 ⊥ 墙面），位姿解算会【退化】——
    #    距离远一点 + 倾斜一点点，成像几乎一样，分不出来。
    #    实际采集时也要刻意斜着看。
    a = np.deg2rad(SKEW_DEG)
    look = radial * np.cos(a) + tangent * np.sin(a)
    look /= np.linalg.norm(look)

    # lookAt：构造相机坐标系（z 轴朝前、x 向右、y 向下）
    z_cam = look
    x_cam = np.cross(z_cam, np.array([0.0, 0.0, 1.0]))
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    gt_rot[i] = np.column_stack([x_cam, y_cam, z_cam])

print(f"帧数        : {N_FRAMES}")
print(f"椭圆半径    : {RX} m × {RY} m，高度 {CAM_Z} m")
print(f"相机到墙距离: 约 {ROOM_X[1]-RX:.1f} ~ {ROOM_Y[1]-RY:.1f} m")
print(f"视频偏心角  : {SKEW_DEG}°（故意斜视，避免正对退化）")


# =====================================================================
# 3. 渲染：把 3D 场景"拍"成 2D 图像
# =====================================================================
print("\n" + SEP)
print("第 3 步：按真实成像模型渲染每一帧")
print(SEP)


def world_to_cam(P_world, R_wc, t_wc):
    """世界坐标 → 相机坐标。  X_cam = R_wcᵀ (X_world - t_wc)"""
    return (R_wc.T @ (P_world - t_wc).T).T


def render_frame(i: int):
    """
    渲染第 i 帧。

    这里用的是 OpenCV 的 projectPoints —— 它会【自动带上畸变】，
    所以渲染出来的图像是"真实相机看到的样子"。
    """
    R_wc, t_wc = gt_rot[i], gt_pos[i]

    rvec, _ = cv2.Rodrigues(R_wc.T)      # world→camera 的旋转
    tvec = (-R_wc.T @ t_wc).reshape(3, 1)

    # 背景：中灰（纯黑背景会让检测器有奇怪的响应）
    canvas = np.full((IMG_H, IMG_W), 110, dtype=np.uint8)

    visible = []
    # 远的先画、近的后画（简单的画家算法）
    order = sorted(TAGS.items(),
                   key=lambda kv: -np.linalg.norm(kv[1]["center"] - t_wc))

    for tid, tag in order:
        if np.dot(tag["normal"], t_wc - tag["center"]) <= 0:
            continue                                   # 背面，看不见

        # board（含白边的整幅图）的 4 个角在世界系里的坐标
        board_world = square_corners_world(tag["center"], tag["normal"],
                                           BOARD_SIZE / 2)

        # ★ 先判断"是不是在相机前方"。
        #   projectPoints 对 z<0 的点不会报错，而是除以负数给出【翻转】的坐标，
        #   结果就是"相机背后的 tag 也被画到图像上" —— 非常隐蔽的坑。
        board_cam = (R_wc.T @ (board_world - t_wc).T).T
        if np.any(board_cam[:, 2] <= 0.05):
            continue

        # projectPoints 会自动带上畸变（DIST），所以渲染结果 = 真实相机所见
        board_pts, _ = cv2.projectPoints(board_world, rvec, tvec, K, DIST)
        dst = board_pts.reshape(-1, 2)
        if not np.all(np.isfinite(dst)):
            continue
        # 投影点全在画面外就跳过（留一点余量）
        if (dst[:, 0].max() < -50 or dst[:, 0].min() > IMG_W + 50
                or dst[:, 1].max() < -50 or dst[:, 1].min() > IMG_H + 50):
            continue

        # ★ 抗混叠。
        #   源图有 BOARD_PX=520 px，但在图像里可能只有 ~150 px。
        #   直接 warpPerspective 做【缩小】会产生混叠 —— tag 的黑白格子被糊掉，
        #   检测器就认不出来了（实测：errors 从毫米级变成米级）。
        #   正确做法：先按比例用 INTER_AREA 缩到接近目标尺寸，再 warp。
        #   ⚠️ 真实相机拍的照片不存在这个问题，这是"自己渲染虚拟图像"特有的坑。
        dst_w = float(np.hypot(*(dst[1] - dst[0])))
        src_px = BOARD_CORNERS_PX
        tag_img = tag["img"]
        if 0 < dst_w < BOARD_PX:
            sc = max(dst_w / BOARD_PX, 1e-3)
            src_px = BOARD_CORNERS_PX * sc
            tag_img = cv2.resize(tag["img"], None, fx=sc, fy=sc,
                                 interpolation=cv2.INTER_AREA)

        H = cv2.getPerspectiveTransform(
            src_px.astype(np.float32), dst.astype(np.float32))
        warped = cv2.warpPerspective(tag_img, H, (IMG_W, IMG_H),
                                     flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(tag_img, 255), H, (IMG_W, IMG_H),
                                   flags=cv2.INTER_NEAREST,
                                   borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        canvas[mask > 0] = warped[mask > 0]
        visible.append(tid)

    # 传感器噪声
    canvas = np.clip(canvas.astype(np.float32)
                     + np.random.normal(0, 2.5, canvas.shape), 0, 255).astype(np.uint8)
    return canvas, visible


images = []
vis_truth = []
for i in range(N_FRAMES):
    img, vis = render_frame(i)
    images.append(img)
    vis_truth.append(vis)

n_vis = sum(1 for v in vis_truth if v)
print(f"渲染完成：{N_FRAMES} 帧，其中 {n_vis} 帧至少看到一个 tag")

counts = [len(v) for v in vis_truth]
print(f"每帧可见 tag 数：最少 {min(counts)}，最多 {max(counts)}，"
      f"平均 {np.mean(counts):.2f}")
print("\n👉 注意有多少帧【一个 tag 都看不到】：")
print("   那些帧就是『没有真值』的时刻 —— 这正是 AprilTag 方案的本质限制：")
print("   它是离散的锚点，不是连续的轨迹。间隙只能靠其他信息（参考轨迹）补。")


# =====================================================================
# 4. 检测 + 解算 —— 从这里开始就是"真实采集时用的同一套代码"
# =====================================================================
print("\n" + SEP)
print("第 4 步：检测 tag 并解算相机位姿")
print(SEP)

detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

# 上一帧的解，用作下一帧 solvePnP 的初值（提高稳定性）
rvec_guess = None
tvec_guess = None

est_pos = np.full((N_FRAMES, 3), np.nan)
est_rot = np.full((N_FRAMES, 3, 3), np.nan)
n_detected = np.zeros(N_FRAMES, dtype=int)
n_failed = 0

for i, img in enumerate(images):
    corners, ids, _ = detector.detectMarkers(img)
    if ids is None:
        continue

    n_detected[i] = len(ids)

    # ---- 4.1 去畸变 ----
    # 检测出来的角点是【带畸变】的图像坐标，必须先拉直。
    # undistortPoints 默认输出归一化坐标，传 P=K 才会输出像素坐标。
    img_pts, obj_pts_world = [], []
    for k, tid in enumerate(ids.ravel()):
        pts = corners[k].reshape(4, 2).astype(np.float64)   # 左上→右上→右下→左下
        pts_und = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, DIST,
                                      P=K).reshape(-1, 2)
        img_pts.append(pts_und)
        obj_pts_world.append(TAGS[tid]["corners_marker"])   # marker 4 角的世界坐标

    obj_all = np.vstack(obj_pts_world).reshape(-1, 1, 3).astype(np.float64)
    img_all = np.vstack(img_pts).reshape(-1, 1, 2)

    try:
        # ---- 4.2 求解相机位姿 ----
        # 直接用角点的【世界坐标】当 objectPoints，solvePnP 一步就给出
        # 相机在世界系里的位姿，中间不需要再做一次坐标换算（少一处出错机会）。
        #   · 只看到 1 个 tag → 4 个点共面，用平面目标专用求解器 IPPE
        #   · 看到多个 tag   → 点不再共面，用通用迭代求解器
        #     （多个 tag 一起解 = 同时用上所有约束，比"各自解完再平均"更准）
        flags = (cv2.SOLVEPNP_IPPE if len(ids) == 1
                 else cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(obj_all, img_all, K, None, flags=flags)
        R_cw = cv2.Rodrigues(rvec)[0]
        t_cw = tvec.reshape(3)

        if not ok:
            n_failed += 1
            continue

        # ---- 4.3 世界→相机 转成 相机→世界（我们想知道的） ----
        est_rot[i] = R_cw.T
        est_pos[i] = -R_cw.T @ t_cw

    except cv2.error:
        n_failed += 1
        continue

valid = ~np.isnan(est_pos[:, 0])
print(f"有解算结果的帧数 : {valid.sum()} / {N_FRAMES}")
print(f"解算失败次数     : {n_failed}")
print(f"单 tag 帧 / 多 tag 帧: "
      f"{(n_detected[valid] == 1).sum()} / {(n_detected[valid] >= 2).sum()}")


# =====================================================================
# 5. 输出 TUM 格式轨迹
# =====================================================================
print("\n" + SEP)
print("第 5 步：输出 TUM 格式轨迹")
print(SEP)


def write_tum(path, pos, rot, timestamps):
    """
    TUM 格式：每行 'timestamp tx ty tz qx qy qz qw'
    ⚠️ qw 在【最后一位】—— 这是 TUM 的规定，写成第一位是最常见的错。
    """
    with open(path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for t, p, R in zip(timestamps, pos, rot):
            q = Rotation.from_matrix(R).as_quat()      # scipy 返回 [x,y,z,w] ✓
            f.write(f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


FRAME_DT = 1.0 / 20.0            # 假装 20 Hz
stamps_all = np.arange(N_FRAMES) * FRAME_DT

p_est = os.path.join(OUT_DIR, "camera_trajectory_estimated_tum.txt")
p_gt = os.path.join(OUT_DIR, "camera_trajectory_groundtruth_tum.txt")

write_tum(p_est, est_pos[valid], est_rot[valid], stamps_all[valid])
write_tum(p_gt, gt_pos[valid], gt_rot[valid], stamps_all[valid])

print(f"真值轨迹 : {p_gt}")
print(f"解算轨迹 : {p_est}")
print(f"（都只写了有解算结果的 {valid.sum()} 帧）")
print("\n可以用它直接喂给 evo：")
print(f"  evo_ape tum {os.path.relpath(p_gt)} {os.path.relpath(p_est)} -va --align --plot")


# =====================================================================
# 6. 评估：和真值对比
# =====================================================================
print("\n" + SEP)
print("第 6 步：算误差（ATE）")
print(SEP)


def umeyama(src: np.ndarray, dst: np.ndarray):
    """
    求 R, t 使 dst ≈ R·src + t（只含旋转和平移，不含缩放）。

    为什么需要这一步：解算出来的轨迹和真值可能差一个整体坐标系
    （AprilTag 的世界原点是 tag，算法的原点是雷达）。ATE 的常规做法是
    先把这个刚体差异去掉，再比"形状"。
    """
    n = len(src)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / n
    U, _, Vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1.0, 1.0, d]) @ Vt
    return R, mu_d - R @ mu_s


if valid.sum() < 3:
    print("❌ 有效帧不足 3 帧，无法评估。")
    print(f"   调试图像已保存: {os.path.join(OUT_DIR, '_debug_frame0.png')}")
    sys.exit(1)

R_a, t_a = umeyama(est_pos[valid], gt_pos[valid])
aligned = (R_a @ est_pos[valid].T).T + t_a
err = np.linalg.norm(aligned - gt_pos[valid], axis=1)

rmse = float(np.sqrt(np.mean(err ** 2)))
med = float(np.median(err))
q1, q3 = np.percentile(err, [25, 75])
worst = float(err.max())

print(f"位置误差（对齐后，共 {len(err)} 帧）")
print(f"  RMSE            : {rmse*1000:8.2f} mm   ← 会被个别离群点拉高")
print(f"  中位数          : {med*1000:8.2f} mm   ← 更有代表性")
print(f"  四分位距 IQR    : [{(q1)*1000:.2f}, {(q3)*1000:.2f}] mm")
print(f"  最大            : {worst*1000:8.2f} mm   ← 最差的那一帧")
print(f"  (相机到墙约 1.1~1.4 m，tag 边长 {TAG_SIZE*100:.0f} cm)")

# ---- 按"同时看到几个 tag"拆分，看联合解算带来的收益 ----
m_single = n_detected[valid] == 1
if m_single.any() and (~m_single).any():
    print("\n按『同时看到几个 tag』拆分：")
    print(f"  只看到 1 个 tag（{m_single.sum():3d} 帧）: "
          f"中位数 {np.median(err[m_single])*1000:6.2f} mm")
    print(f"  看到 ≥2 个 tag（{(~m_single).sum():3d} 帧）: "
          f"中位数 {np.median(err[~m_single])*1000:6.2f} mm")
    print("  👉 多 tag 联合解算明显更准 —— 所以布场时 tag 要贴密一点。")

print()
print("👉 注意 RMSE 和中位数差了这么多：")
print(f"   RMSE {rmse*1000:.1f} mm  ≫  中位数 {med*1000:.1f} mm")
print("   说明是【少数离群点】把 RMSE 拉高的。这正是 docs/10 规范 3 说的")
print("   『单个离群点就能毁掉 ATE』。所以报告里必须给中位数 + 四分位距，")
print("   只报一个 RMSE 数字会严重低估真实精度。")

if med < 0.03:
    print("\n✅ 中位数在 3 cm 以内 → 整条链路（内参 / 角点顺序 / 坐标系 /")
    print("   solvePnP 模式 / 去畸变）都是对的。等器材到位后，只要把")
    print("   『渲染图像』换成『相机拍的照片』，代码一行都不用改。")
else:
    print("\n❌ 误差偏大。最常见的三个原因：")
    print("   ① objectPoints 和检测角点的【顺序】没对上（会整体镜像或转 90°）")
    print("   ② 忘了去畸变（第 4.1 步）")
    print("   ③ tag 的坐标系建成了镜像的（检测器会只给 rejected，不给 ids）")


# =====================================================================
# 7. 画图
# =====================================================================
print("\n" + SEP)
print("第 7 步：画图")
print(SEP)

# ---------- 图 5：场景俯视图 ----------
fig, ax = plt.subplots(figsize=(8, 6))
ax.add_patch(plt.Rectangle((ROOM_X[0], ROOM_Y[0]), ROOM_X[1] - ROOM_X[0],
                           ROOM_Y[1] - ROOM_Y[0], fill=False, lw=2, ec="0.4"))
for tid, tag in TAGS.items():
    c = tag["center"]
    ax.plot(c[0], c[1], "s", ms=9, color="tab:orange")
    ax.annotate(f"{tid}", (c[0], c[1]), textcoords="offset points",
                xytext=(6, 6), fontsize=8, color="tab:orange")
ax.plot(gt_pos[:, 0], gt_pos[:, 1], "-", lw=2, color="tab:green",
        label="真值轨迹（虚拟）")
ax.plot(est_pos[valid, 0], est_pos[valid, 1], "--", lw=1.5, color="tab:red",
        label="AprilTag 解算轨迹")
ax.set_aspect("equal")
ax.set_xlabel("x / m"); ax.set_ylabel("y / m")
ax.set_title("虚拟场景俯视图：方形=墙上的 tag，两种=两条轨迹")
ax.legend(loc="upper right", fontsize=9)
fig.tight_layout()
plot_setup.save(fig, "out_5_scene.png", OUT_DIR)

# ---------- 图 6：检测结果 ----------
sample_idx = [i for i in range(N_FRAMES) if n_detected[i] >= 2][:4]
if len(sample_idx) < 4:
    sample_idx = [i for i in range(N_FRAMES) if n_detected[i] >= 1][:4]
if len(sample_idx) < 4:      # 一个都没检测到时，也把画面展示出来（便于排查）
    sample_idx = [0, N_FRAMES // 4, N_FRAMES // 2, 3 * N_FRAMES // 4]
print("展示的帧:", sample_idx)

fig, axes = plt.subplots(2, 2, figsize=(13, 7.5))
for ax, i in zip(axes.ravel(), sample_idx):
    corners, ids, _ = detector.detectMarkers(images[i])
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(images[i], corners, ids)
        ax.imshow(images[i])          # 不指定 cmap，保留检测框的颜色
        ax.set_title(f"第 {i} 帧 · 检测到 {len(ids)} 个 tag "
                     f"(ID {'/'.join(map(str, ids.ravel()))})", fontsize=10)
    else:
        ax.imshow(images[i], cmap="gray")
        ax.set_title(f"第 {i} 帧 · 没检测到", fontsize=10)
    ax.set_xticks([]); ax.set_yticks([])
    ax.grid(False)
fig.suptitle("渲染出的『相机图像』与检测结果（真实采集时看到的就是这个）", fontsize=11)
fig.tight_layout()
plot_setup.save(fig, "out_6_detection.png", OUT_DIR)

# ---------- 图 7：轨迹对比 ----------
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
axes[0].plot(gt_pos[:, 0], gt_pos[:, 1], "-", lw=2, color="tab:green", label="真值")
axes[0].plot(est_pos[valid, 0], est_pos[valid, 1], "--", lw=1.5,
             color="tab:red", label="解算")
axes[0].set_aspect("equal"); axes[0].legend(fontsize=9)
axes[0].set_xlabel("x / m"); axes[0].set_ylabel("y / m")
axes[0].set_title("俯视图（整体形状）")

t_axis = stamps_all[valid]
for k, name in enumerate("xyz"):
    axes[1].plot(stamps_all, gt_pos[:, k], "-", lw=2, color="tab:green")
    axes[1].plot(t_axis, est_pos[valid, k], "--", lw=1.2, color="tab:red")
axes[1].set_xlabel("时间 / s"); axes[1].set_ylabel("坐标 / m")
axes[1].set_title("x / y / z 随时间（绿=真值，红=解算）")

axes[2].plot(t_axis, err * 1000, "-", color="tab:red")
axes[2].axhline(med * 1000, ls=":", color="0.3",
                label=f"中位数 {med*1000:.1f} mm")
axes[2].set_xlabel("时间 / s"); axes[2].set_ylabel("位置误差 / mm")
axes[2].set_title("逐帧误差"); axes[2].legend(fontsize=9)
fig.tight_layout()
plot_setup.save(fig, "out_7_trajectory.png", OUT_DIR)

# ---------- 图 8：误差分布 ----------
fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
axes[0].hist(err * 1000, bins=25, color="tab:blue", alpha=0.75, edgecolor="w")
axes[0].axvline(med * 1000, color="tab:red", lw=2,
                label=f"中位数 {med*1000:.1f} mm")
axes[0].axvline(q1 * 1000, color="0.4", ls="--", lw=1)
axes[0].axvline(q3 * 1000, color="0.4", ls="--", lw=1, label="四分位")
axes[0].set_xlabel("位置误差 / mm"); axes[0].set_ylabel("帧数")
axes[0].set_title("误差分布"); axes[0].legend(fontsize=9)

axes[1].axis("off")
axes[1].text(0.02, 0.95, "ATE 统计（对齐后）", fontsize=13, va="top", weight="bold")
stats = [("RMSE", f"{rmse*1000:.2f} mm"),
         ("中位数", f"{med*1000:.2f} mm"),
         ("IQR 下沿", f"{q1*1000:.2f} mm"),
         ("IQR 上沿", f"{q3*1000:.2f} mm"),
         ("最大", f"{worst*1000:.2f} mm"),
         ("有效帧", f"{valid.sum()} / {N_FRAMES}"),
         ("tag 边长", f"{TAG_SIZE*100:.1f} cm"),
         ("相机-tag 距离", "约 1.1 ~ 1.4 m")]
for k, (name, val) in enumerate(stats):
    y = 0.80 - k * 0.085
    axes[1].text(0.04, y, name, fontsize=11, va="top")
    axes[1].text(0.45, y, val, fontsize=11, va="top", weight="bold",
                 color="tab:red")
fig.tight_layout()
plot_setup.save(fig, "out_8_error.png", OUT_DIR)

print(SEP)
print("完成。")
print(SEP)
print("下一步（等真实器材到位后）：")
print("  1. 标定相机内参 → 替换本脚本的 K 和 DIST")
print("  2. 量出 tag 边长   → 替换 TAG_SIZE")
print("  3. 量出 tag 在墙上的位置 → 替换 TAGS 里的 center / normal")
print("  4. 把 render_frame() 换成『读相机照片』")
print("  5. 其余代码（检测、去畸变、solvePnP、TUM 输出、评估）完全不用改")
