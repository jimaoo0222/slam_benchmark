#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RC 2026 定位 Benchmark 评测工具
================================

一条命令算出定位误差，支持「分动作统计」和「无真值的一致性检查」。

------------------------------------------------------------------------
⭐ 输入是什么？—— 是两个【轨迹文件】，不是原始点云
------------------------------------------------------------------------
    --gt    真值     ← 你手工写的（卷尺量的坐标，可以只有 5 行）
    --est   雷达值   ← GLIM 跑出来的轨迹（maps/traj_lidar.txt）

数据流长这样：

    雷达点云 ─→ GLIM ─→ traj_lidar.txt ─┐
                                        ├─→ benchmark_eval.py ─→ 误差
    卷尺量点 ─→ 手写 gt.txt ────────────┘

    ⚠️ 原始点云【不直接】进这个脚本 —— 它先经 GLIM 处理，GLIM 输出轨迹，
       轨迹才进这里。所以本脚本不读 .bag / .pcd，只读 TUM 纯文本。

    两个文件都是纯文本，每行 8 列：时间 x y z qx qy qz qw

------------------------------------------------------------------------
为什么要单独写这个脚本，而不是直接用 evo？
------------------------------------------------------------------------
evo 很好用，但有两个问题：

  1. evo 默认鼓励用 `--align`（整体对齐）。**我们绝不能对齐** ——
     比赛用场地绝对坐标，整体偏移正是最致命的错误（详见 docs/14 第二章）。
     这个脚本默认**不对齐**，而且不做对齐这个选项的默认值。

  2. evo 只给"整条轨迹一个数字"。我们需要的是**按比赛动作分段**的数字：
     直线段 / 旋转段 / 台阶段 / 对准段，各自误差多少。

------------------------------------------------------------------------
用法
------------------------------------------------------------------------
# ① 有真值：算绝对误差（最常用）
python3 scripts/benchmark_eval.py --gt gt.txt --est maps/traj_lidar.txt

# ② 分动作统计（动作时间段写在一个 csv 里）
python3 scripts/benchmark_eval.py --gt gt.txt --est traj.txt --segments actions.csv

# ③ 没有真值：多条轨迹互相比较（一致性检查，衡量"稳不稳"）
python3 scripts/benchmark_eval.py --est run1.txt run2.txt run3.txt --consistency

# ④ 先看效果（自己造数据，不需要任何文件）
python3 scripts/benchmark_eval.py --demo

------------------------------------------------------------------------
输入格式：TUM
------------------------------------------------------------------------
纯文本，每行 8 列：

    timestamp tx ty tz qx qy qz qw

真值可以是**稀疏**的（比如只在关键位置打了 5 个点），脚本会自动处理。

分段文件格式（CSV，带表头）：

    name,start,end
    MOVE_1,1783079896.0,1783079900.5
    STAIR,1783079901.0,1783079906.2
    ALIGN,1783079910.0,1783079912.0
"""

import argparse
import csv
import os
import sys
import pathlib

os.environ.setdefault("SLAM_BENCH_HEADLESS", "1")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common import plot_setup  # noqa: E402

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
SEP = "=" * 70


# =====================================================================
# 1. 读写 TUM 轨迹
# =====================================================================
def load_tum(path):
    """
    读 TUM 格式轨迹。

    返回 (t, xyz, quat)，都是 numpy 数组。四元数顺序 [x, y, z, w]。
    自动跳过注释行（# 开头）和空行。
    """
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(v) for v in parts[:8]])
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"{path} 里没读到有效数据（需要 8 列 TUM 格式）")

    a = np.array(rows, dtype=float)
    order = np.argsort(a[:, 0])          # 按时间排序
    a = a[order]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def align_by_time(t_query, t_src, xyz_src):
    """
    在估计轨迹上按时间插值，取出真值那些时刻的估计位置。

    为什么要插值：真值可能是稀疏的（人工只标了几个关键点），
    而估计轨迹是稠密的（100 Hz）。我们要看的是
    "机器人停在矛头前的那一刻，估计的位置是多少"。

    超出估计轨迹时间范围的查询点会被丢弃（返回 mask）。
    """
    inside = (t_query >= t_src[0]) & (t_query <= t_src[-1])
    out = np.full((len(t_query), 3), np.nan)
    out[inside] = np.column_stack([
        np.interp(t_query[inside], t_src, xyz_src[:, k]) for k in range(3)
    ])
    return out, inside


# =====================================================================
# 2. 指标计算
# =====================================================================
def stats_block(err, threshold=None):
    """算一组误差的统计量。注意：报中位数 + IQR，不只是 RMSE。"""
    d = {}
    d["n"] = len(err)
    if len(err) == 0:
        return d
    d["median"] = float(np.median(err))
    q1, q3 = np.percentile(err, [25, 75])
    d["q1"], d["q3"] = float(q1), float(q3)
    d["iqr"] = float(q3 - q1)
    d["rmse"] = float(np.sqrt(np.mean(err ** 2)))
    d["mean"] = float(np.mean(err))
    d["max"] = float(np.max(err))
    d["min"] = float(np.min(err))
    if threshold is not None:
        d["over_ratio"] = float(np.mean(err > threshold))
        d["threshold"] = threshold
    return d


def print_stats(name, d, unit="mm"):
    if d.get("n", 0) == 0:
        print(f"  {name}: 没有有效数据点")
        return
    k = 1000.0 if unit == "mm" else 1.0
    print(f"  {name}  （{d['n']} 个点）")
    print(f"    中位数       : {d['median']*k:8.2f} {unit}   ← 主要参考")
    print(f"    四分位距 IQR : [{d['q1']*k:.2f}, {d['q3']*k:.2f}] {unit}")
    print(f"    RMSE         : {d['rmse']*k:8.2f} {unit}   ← 会被离群点拉高")
    print(f"    最大         : {d['max']*k:8.2f} {unit}")
    if "over_ratio" in d:
        print(f"    超 {d['threshold']*100:.0f}cm 占比 : "
              f"{d['over_ratio']*100:6.2f} %   ← 对比赛最有用")


def umeyama(src, dst):
    """
    求 R, t 使 dst ≈ R·src + t（只含旋转和平移，**不含缩放**）。

    这就是 evo 的 `--align` 做的事：把整体坐标系差异消掉，只比“轨迹形状”。
    """
    n = len(src)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / n
    U, _, Vt = np.linalg.svd(cov)
    sign = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1.0, 1.0, sign]) @ Vt
    return R, mu_d - R @ mu_s


def print_stats_pair(d_raw, d_ali, unit="mm"):
    """
    并排打印“不对齐”和“对齐”两组统计。

    为什么要两组：
        不对齐 = 部署到实际系统时的表现（含系统性偏移）
        对齐   = 算法本身的轨迹形状精度（学术界对比算法的惯例）
        两者差距大 → 该算法有系统性偏移（标定 / 初始化 / 尺度问题）
    """
    if d_raw.get("n", 0) == 0:
        print("  没有有效数据点")
        return
    k = 1000.0 if unit == "mm" else 1.0
    print(f"  {'':<16s}{'不对齐':>14s}{'对齐 SE(3)':>16s}")
    print(f"  {'中位数':<16s}{d_raw['median']*k:>12.2f}  "
          f"{d_ali['median']*k:>14.2f}  {unit}")
    print(f"  {'IQR 下沿':<16s}{d_raw['q1']*k:>12.2f}  "
          f"{d_ali['q1']*k:>14.2f}  {unit}")
    print(f"  {'IQR 上沿':<16s}{d_raw['q3']*k:>12.2f}  "
          f"{d_ali['q3']*k:>14.2f}  {unit}")
    print(f"  {'RMSE':<16s}{d_raw['rmse']*k:>12.2f}  "
          f"{d_ali['rmse']*k:>14.2f}  {unit}")
    print(f"  {'最大':<16s}{d_raw['max']*k:>12.2f}  "
          f"{d_ali['max']*k:>14.2f}  {unit}")
    if "over_ratio" in d_raw:
        thr = d_raw["threshold"] * 100
        label = f"超 {thr:.0f}cm 占比"
        print(f"  {label:<16s}{d_raw['over_ratio']*100:>12.2f}%  "
              f"{d_ali['over_ratio']*100:>13.2f}%")


# =====================================================================
# 3. 分段
# =====================================================================
def load_segments(path):
    """读分段 CSV。返回 [(name, start, end), ...]"""
    segs = []
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            try:
                segs.append((row["name"].strip(),
                             float(row["start"]), float(row["end"])))
            except (KeyError, ValueError):
                continue
    return segs


def segment_errors(t_gt, err, segments):
    """把误差按时间段归类（只保留落在段内的时间戳）。"""
    out = []
    for name, t0, t1 in segments:
        m = (t_gt >= t0) & (t_gt <= t1)
        out.append((name, err[m], t0, t1))
    return out


# =====================================================================
# 4. 画图
# =====================================================================
def plot_result(t_gt, err, gt_xyz, est_xyz, segments, threshold, outdir,
                err_ali=None):
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1])

    # --- 左上：俯视图 ---
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(est_xyz[:, 0], est_xyz[:, 1], "-", lw=1.0, color="tab:red",
            label="估计轨迹")
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "o", ms=6, color="tab:green",
            label="真值点")
    for name, _, t0, t1 in segments:
        m = (t_gt >= t0) & (t_gt <= t1)
        if m.any():
            ax.plot(gt_xyz[m, 0], gt_xyz[m, 1], "s", ms=9, mfc="none",
                    mec="tab:blue",
                    label=name if name not in ax.get_legend_handles_labels()[1] else None)
    ax.set_aspect("equal")
    ax.set_xlabel("x / m"); ax.set_ylabel("y / m")
    ax.set_title("俯视图")
    ax.legend(fontsize=8)

    # --- 右上：误差随时间 ---
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t_gt - t_gt[0], err * 1000, "-", lw=1.2, color="tab:red",
            label="不对齐")
    if err_ali is not None:
        ax.plot(t_gt - t_gt[0], err_ali * 1000, "--", lw=1.1,
                color="tab:green", label="对齐 SE(3)")
    if threshold:
        ax.axhline(threshold * 1000, ls=":", color="0.4",
                   label=f"阈值 {threshold*100} cm")
    for name, _, t0, t1 in segments:
        ax.axvspan(t0 - t_gt[0], t1 - t_gt[0], alpha=0.12, color="tab:blue")
        ax.text((t0 + t1) / 2 - t_gt[0], ax.get_ylim()[1] * 0.95, name,
                ha="center", va="top", fontsize=8, color="tab:blue")
    ax.set_xlabel("时间 / s（相对起点）"); ax.set_ylabel("位置误差 / mm")
    ax.set_title("逐点误差：实线=不对齐，虚线=对齐")
    ax.legend(fontsize=8)

    # --- 左下：误差分布 ---
    ax = fig.add_subplot(gs[1, 0])
    ax.hist(err * 1000, bins=min(30, max(5, len(err) // 2)),
            color="tab:blue", alpha=0.75, edgecolor="w")
    ax.axvline(np.median(err) * 1000, color="tab:red", lw=2,
               label=f"中位数 {np.median(err)*1000:.1f} mm")
    ax.axvline(np.percentile(err, 25) * 1000, color="0.4", ls="--", lw=1)
    ax.axvline(np.percentile(err, 75) * 1000, color="0.4", ls="--", lw=1,
               label="四分位")
    ax.set_xlabel("位置误差 / mm"); ax.set_ylabel("点数")
    ax.set_title("误差分布"); ax.legend(fontsize=8)

    # --- 右下：分段柱状图 ---
    ax = fig.add_subplot(gs[1, 1])
    if segments:
        names, meds = [], []
        for name, e, _, _ in segments:        # segments 已经是分好的结果
            if len(e):
                names.append(name); meds.append(np.median(e) * 1000)
        if names:
            ax.bar(names, meds, color="tab:purple", alpha=0.8)
            ax.set_ylabel("中位数误差 / mm")
            ax.set_title("分动作对比  ← 这才是有决策价值的一张图")
            ax.tick_params(axis="x", rotation=20)
    if not ax.has_data():
        ax.axis("off")
        ax.text(0.5, 0.5, "（没提供分段文件，\n加 --segments actions.csv 看这张图）",
                ha="center", va="center", fontsize=11, color="0.5")

    fig.tight_layout()
    return plot_setup.save(fig, "benchmark_report.png", outdir)


# =====================================================================
# 5. 无真值模式：一致性检查
# =====================================================================
def consistency_check(paths, threshold=None, outdir=OUT_DIR):
    """
    没有真值时的退路：多条轨迹互相比较。

    能回答："同一段路跑 3 次，结果差多少？" —— 衡量**稳定性**。
    不能回答："准不准？" —— 那必须要有真值。
    """
    runs = []
    for p in paths:
        t, xyz, _ = load_tum(p)
        runs.append((os.path.basename(p), t, xyz))

    print(f"\n{SEP}")
    print("一致性检查（无真值）")
    print(SEP)
    print("⚠️ 注意：这只说明『稳不稳』，不能说明『准不准』。")
    print("   要判断准不准，仍然需要真值。\n")

    # 取所有轨迹时间范围的交集作为公共时间轴
    t0 = max(t[0] for _, t, _ in runs)
    t1 = min(t[-1] for _, t, _ in runs)
    if t1 <= t0:
        print("❌ 这几条轨迹在时间上没有重叠，无法比较。")
        print("   （检查是不是不同场次的数据，或时间戳时钟不一致）")
        return
    t_common = np.linspace(t0, t1, 500)

    interp = []
    for name, t, xyz in runs:
        interp.append((name, np.column_stack(
            [np.interp(t_common, t, xyz[:, k]) for k in range(3)])))

    n = len(interp)
    print(f"两两之间的位置差异（{n} 条轨迹，公共时段 {t1-t0:.1f} s）：")
    allerr = []
    for i in range(n):
        for j in range(i + 1, n):
            d = np.linalg.norm(interp[i][1] - interp[j][1], axis=1)
            allerr.append(d)
            print(f"  {interp[i][0]} vs {interp[j][0]}: "
                  f"中位数 {np.median(d)*1000:7.1f} mm, "
                  f"最大 {np.max(d)*1000:7.1f} mm")

    if allerr:
        cat = np.concatenate(allerr)
        print()
        print_stats("全部两两差异", stats_block(cat, threshold))

    # 画图
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for name, xyz in interp:
        axes[0].plot(xyz[:, 0], xyz[:, 1], "-", lw=1.2, label=name)
    axes[0].set_aspect("equal"); axes[0].legend(fontsize=8)
    axes[0].set_xlabel("x / m"); axes[0].set_ylabel("y / m")
    axes[0].set_title("多条轨迹俯视图（注意：**没有做对齐**）")

    k = 0
    for i in range(n):
        for j in range(i + 1, n):
            axes[1].plot(t_common - t0, allerr[k] * 1000,
                         lw=1, label=f"{interp[i][0]} vs {interp[j][0]}")
            k += 1
    axes[1].set_xlabel("时间 / s"); axes[1].set_ylabel("差异 / mm")
    axes[1].set_title("逐时刻差异"); axes[1].legend(fontsize=8)
    fig.tight_layout()
    plot_setup.save(fig, "benchmark_consistency.png", outdir)


# =====================================================================
# 6. 自动找静止段（停车打点法）
# =====================================================================
def find_stops(t, xyz, speed_thr, min_dur):
    """
    找出轨迹里“停住不动”的片段。

    原理：逐点算速度（相邻两点位移 ÷ 时间差），
          速度低于阈值且持续够长的连续片段，就是一次“停车打点”。

    注意两个工程细节：
      · 只用【水平位移】判断 —— z 方向受振动/俯仰影响大，容易误判
      · 阈值不能太小 —— 真实 SLAM 在静止时也会有缓慢漂移
        （IMU 零偏），所以“速度=0”是拿不到的
    """
    if len(t) < 3:
        return []
    dt = np.diff(t)
    dp = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)   # 只看 xy
    speed = dp / np.maximum(dt, 1e-9)

    still = speed < speed_thr
    stops = []
    i = 0
    while i < len(still):
        if not still[i]:
            i += 1
            continue
        j = i
        while j < len(still) and still[j]:
            j += 1
        jj = min(j, len(t) - 1)          # 静止区间 = 点 i .. jj
        dur = t[jj] - t[i]
        if dur >= min_dur:
            mid = (i + jj) // 2
            stops.append((t[mid], xyz[mid], dur))
        i = j
    return stops


def do_find_stops(path, speed_thr, min_dur, outdir):
    t, xyz, _ = load_tum(path)
    print(f"\n{SEP}")
    print("自动找静止段（停车打点法）")
    print(SEP)
    print(f"文件   : {os.path.basename(path)}")
    print(f"总时长 : {t[-1] - t[0]:.1f} s，{len(t)} 个点，"
          f"平均 {1.0 / np.mean(np.diff(t)):.1f} Hz")
    print(f"判定   : 速度 < {speed_thr} m/s 且持续 > {min_dur} s\n")

    stops = find_stops(t, xyz, speed_thr, min_dur)
    if not stops:
        print("❌ 没找到静止段。试试：")
        print("   --stop-speed 0.05    放宽速度阈值（机器人停不太稳时）")
        print("   --stop-min-dur 1.0   放短持续时间要求")
        return

    print(f"找到 {len(stops)} 段静止：\n")
    print(f"  #   时间戳               位置 (x, y, z)                    持续")
    for k, (tm, p, dur) in enumerate(stops, 1):
        print(f"  {k:<3d} {tm:.5f}    "
              f"({p[0]:8.3f}, {p[1]:8.3f}, {p[2]:8.3f})    {dur:5.1f} s")

    tpl = os.path.join(outdir, "gt_template.txt")
    with open(tpl, "w") as f:
        f.write("# 真值模板 —— 由 benchmark_eval.py --find-stops 生成\n")
        f.write("# ⚠️ 把 tx ty tz 三列换成【卷尺量出来的真实坐标】，时间戳不要改\n")
        f.write("# ⚠️ 坐标必须和被测轨迹用同一套坐标系\n")
        f.write("# ⚠️ 不要直接拿这个文件当结果 —— 它里面填的还是算法自己的位置！\n")
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for tm, p, _, in stops:
            f.write(f"{tm:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} 0.0 0.0 0.0 1.0\n")

    print(f"\n✅ 已生成真值模板: {tpl}")
    print("   把 tx ty tz 换成卷尺量的值，然后：")
    print(f"   python3 {os.path.basename(__file__)} "
          f"--gt {os.path.basename(tpl)} --est {os.path.basename(path)}")


# =====================================================================
# 7. 自造数据演示
# =====================================================================
def make_demo(path_gt, path_est):
    """
    造一段虚拟轨迹：**走走停停**，模拟真实采集的"停车打点"节奏。

    节奏：走 5 s → 停 3 s，重复 4 次（每次转 90°）。
    故意做成这样，是为了让 `--find-stops` 也能演示得出来。
    """
    DT, SPEED, N_CYCLE = 0.1, 0.5, 4
    N_MOVE, N_STOP = int(5.0 / DT), int(3.0 / DT)

    ts, xs, ys = [], [], []
    t_now, x, y = 0.0, 0.0, 0.0

    for k in range(N_CYCLE):
        heading = k * np.pi / 2
        for _ in range(N_MOVE):                       # 走 5 秒
            x += SPEED * DT * np.cos(heading)
            y += SPEED * DT * np.sin(heading)
            ts.append(t_now); xs.append(x); ys.append(y)
            t_now += DT
        for _ in range(N_STOP):                       # 停 3 秒
            ts.append(t_now); xs.append(x); ys.append(y)
            t_now += DT

    t = np.array(ts)
    gt = np.column_stack([xs, ys, 0.02 * np.sin(t)])
    # 估计值 = 真值 + 随时间增长的漂移 + 后半段的台阶跳变
    drift = np.column_stack([0.015 * t, 0.008 * t, 0.002 * t])
    mid = len(t) // 2
    stair = np.where(t > t[mid], 0.05, 0.0)[:, None] * np.array([[1.0, 0.5, 0.1]])
    est = gt + drift + stair

    def dump(path, xyz):
        with open(path, "w") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for i in range(len(t)):
                f.write(f"{t[i]:.9f} {xyz[i,0]:.9f} {xyz[i,1]:.9f} "
                        f"{xyz[i,2]:.9f} 0 0 0 1\n")

    dump(path_est, est)

    # 真值只留每段"停车"的中间时刻 —— 模拟人工在几个点上打点
    cycle = N_MOVE + N_STOP
    keep = [k * cycle + N_MOVE + N_STOP // 2 for k in range(N_CYCLE)]
    with open(path_gt, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in keep:
            f.write(f"{t[i]:.9f} {gt[i,0]:.9f} {gt[i,1]:.9f} {gt[i,2]:.9f} "
                    f"0 0 0 1\n")

    # 分段文件（演示用）
    seg = os.path.join(OUT_DIR, "demo_segments.csv")
    with open(seg, "w") as f:
        f.write("name,start,end\n")
        f.write(f"MOVE_1,{t[0]:.3f},{t[N_MOVE-1]:.3f}\n")
        f.write(f"STAIR,{t[cycle]:.3f},{t[cycle+N_MOVE-1]:.3f}\n")
        f.write(f"ALIGN,{t[2*cycle]:.3f},{t[-1]:.3f}\n")
    return seg


# =====================================================================
# main
# =====================================================================
def main():
    ap = argparse.ArgumentParser(
        description="RC 2026 定位 Benchmark 评测工具（默认不做对齐）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--gt", help="真值轨迹（TUM，可以稀疏）")
    ap.add_argument("--est", nargs="+", help="估计轨迹（TUM），可给多条")
    ap.add_argument("--segments", help="分段 CSV（name,start,end）")
    ap.add_argument("--consistency", action="store_true",
                    help="无真值模式：多条估计轨迹互相比较")
    ap.add_argument("--threshold", type=float, default=0.10,
                    help="误差阈值（米），用于统计超阈值占比。默认 0.10")
    ap.add_argument("--time-offset", type=float, default=0.0,
                    help="给估计轨迹的时间戳加一个偏移（秒），用于不同时钟源")
    ap.add_argument("--outdir", default=OUT_DIR)
    ap.add_argument("--demo", action="store_true", help="自造数据跑一遍")
    ap.add_argument("--find-stops", metavar="TRAJ",
                    help="从一个轨迹里自动找静止段（停车打点法），并生成真值模板")
    ap.add_argument("--stop-speed", type=float, default=0.05,
                    help="静止判定的水平速度阈值 (m/s)，默认 0.05")
    ap.add_argument("--stop-min-dur", type=float, default=2.0,
                    help="静止段最短持续秒数，默认 2.0")
    args = ap.parse_args()

    # ---------- 找静止段 ----------
    if args.find_stops:
        do_find_stops(args.find_stops, args.stop_speed,
                      args.stop_min_dur, args.outdir)
        return

    print(SEP)
    print("RC 2026 定位 Benchmark 评测")
    print(SEP)

    # ---------- demo ----------
    if args.demo:
        print("演示模式：自己造一段数据（真值 + 带漂移的估计值）")
        p_gt = os.path.join(args.outdir, "demo_gt.txt")
        p_est = os.path.join(args.outdir, "demo_est.txt")
        seg = make_demo(p_gt, p_est)
        args.gt, args.est, args.segments = p_gt, [p_est], seg
        print(f"  真值   : {p_gt}（故意只留 20 个稀疏点）")
        print(f"  估计   : {p_est}")
        print(f"  分段   : {seg}")

    if not args.est:
        ap.print_help()
        return

    # ---------- 无真值模式 ----------
    if args.consistency or not args.gt:
        if len(args.est) < 2:
            print("\n❌ 没有给真值（--gt），而且估计轨迹少于 2 条。")
            print("   要么给真值，要么给 ≥2 条轨迹做一致性检查。")
            return
        consistency_check(args.est, args.threshold, args.outdir)
        return

    # ---------- 有真值：主流程 ----------
    est_path = args.est[0]
    t_gt, gt_xyz, _ = load_tum(args.gt)
    t_est, est_xyz, _ = load_tum(est_path)
    t_est = t_est + args.time_offset

    print(f"\n真值     : {os.path.basename(args.gt)}  "
          f"（{len(t_gt)} 个点，时长 {t_gt[-1]-t_gt[0]:.1f} s）")
    print(f"估计     : {os.path.basename(est_path)}  "
          f"（{len(t_est)} 个点，时长 {t_est[-1]-t_est[0]:.1f} s）")

    # 在估计轨迹上插值，取出真值时刻的位置
    est_at_gt, inside = align_by_time(t_gt, t_est, est_xyz)
    if inside.sum() == 0:
        print("\n❌ 两条轨迹在时间上没有重叠。")
        print("   检查：① 时间戳是不是同一时钟 ② 要不要用 --time-offset")
        return

    t_use = t_gt[inside]
    gt_use = gt_xyz[inside]
    est_use = est_at_gt[inside]

    # ---------- 算两组误差 ----------
    # ① 不对齐：部署到实际系统时的表现（含系统性偏移）
    err_raw = np.linalg.norm(est_use - gt_use, axis=1)
    # ② 对齐 SE(3)：算法本身的轨迹形状精度（学术界对比算法的惯例）
    R_a, t_a = umeyama(est_use, gt_use)
    est_ali = (R_a @ est_use.T).T + t_a
    err_ali = np.linalg.norm(est_ali - gt_use, axis=1)

    # ---------- 总体统计 ----------
    print(f"\n{SEP}")
    print(f"位置误差（有效点 {len(err_raw)}/{len(t_gt)}）")
    print(SEP)
    print_stats_pair(stats_block(err_raw, args.threshold),
                     stats_block(err_ali, args.threshold))

    # ---------- 分段统计 ----------
    seg_raw = []
    if args.segments and os.path.exists(args.segments):
        segs = load_segments(args.segments)
        seg_raw = segment_errors(t_use, err_raw, segs)
        seg_ali = segment_errors(t_use, err_ali, segs)
        print(f"\n{SEP}")
        print("分段统计（中位数：不对齐 / 对齐）")
        print(SEP)
        for (name, e_r, _, _), (_, e_a, _, _) in zip(seg_raw, seg_ali):
            if len(e_r) == 0:
                print(f"  {name:<14s} （无数据点）")
                continue
            print(f"  {name:<14s} 中位数 {np.median(e_r)*1000:8.1f} / "
                  f"{np.median(e_a)*1000:8.1f} mm"
                  f"     超{args.threshold*100:.0f}cm "
                  f"{np.mean(e_r > args.threshold)*100:5.1f}% / "
                  f"{np.mean(e_a > args.threshold)*100:5.1f}%")

        for (name, e_r, _, _), (_, e_a, _, _) in zip(seg_raw, seg_ali):
            if name.upper().startswith("STAIR") and len(e_r) >= 2:
                print(f"\n⭐ 台阶漂移增量 Δstair   不对齐 "
                      f"{(e_r[-1]-e_r[0])*1000:+.1f} mm / 对齐 "
                      f"{(e_a[-1]-e_a[0])*1000:+.1f} mm")
    else:
        if args.segments:
            print(f"\n⚠️ 分段文件不存在: {args.segments}")

    # ---------- 画图 ----------
    png = plot_result(t_use, err_raw, gt_use, est_use, seg_raw,
                      args.threshold, args.outdir, err_ali=err_ali)

    # ---------- 结论提示 ----------
    print(f"\n{SEP}")
    print("怎么看这些数字")
    print(SEP)
    d_raw = stats_block(err_raw, args.threshold)
    d_ali = stats_block(err_ali, args.threshold)
    print(f"  不对齐中位数 {d_raw['median']*1000:8.1f} mm   "
          f"← 部署到实际系统时的表现")
    print(f"  对齐中位数   {d_ali['median']*1000:8.1f} mm   "
          f"← 算法本身的轨迹形状精度")

    gap = d_raw["median"] - d_ali["median"]
    if gap > 0.02:
        print(f"\n  ⚠️ 两者相差 {gap*1000:.0f} mm —— 说明该算法有**系统性偏移**")
        print("     （标定 / 初始化 / 尺度问题），不是形状不准。")
    elif d_ali["median"] > d_raw["median"] * 0.8:
        print("\n  ✅ 两者接近 —— 误差主要来自轨迹形状，没有明显的系统性偏移。")

    if d_raw["rmse"] > 2 * d_raw["median"]:
        print(f"\n  ⚠️ RMSE ({d_raw['rmse']*1000:.0f} mm) 远大于中位数 "
              f"({d_raw['median']*1000:.0f} mm)：有少数离群点。")
        print("     **报告时两个都给**，只给 RMSE 会低估真实精度。")

    print(f"\n✅ 图已保存: {png}")
    print(SEP)


if __name__ == "__main__":
    main()
