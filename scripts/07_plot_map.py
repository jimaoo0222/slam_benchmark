#!/usr/bin/env python3
"""把地图 PCD + 轨迹画成图（俯视图 / 侧视图 / 轨迹叠加）。

用法
────
    # 只画地图
    python3 scripts/07_plot_map.py map.pcd

    # 地图 + 轨迹叠加（推荐：一眼看出走到哪、地图对不对）
    python3 scripts/07_plot_map.py map.pcd --traj traj.txt -o map_preview.png

    # 只留量程内的点（去掉远处离群点，MID-360 大约 40 m）
    python3 scripts/07_plot_map.py map.pcd --max-range 40

输出
────
    左：俯视图（XY，按高度 Z 着色）—— 看整体形状和轨迹
    右：侧视图（XZ）             —— 看楼层/地面是否平
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_pcd(path: Path) -> np.ndarray:
    """读 PCD（支持 binary 和 ascii；本项目的 06_save_map_pcd.py 写的是 binary）。"""
    data = path.read_bytes()
    marker = b"DATA binary\n"
    if marker in data:
        i = data.index(marker) + len(marker)
        header = data[:i].decode("ascii", "replace")
        npts = int(re.search(r"POINTS (\d+)", header).group(1))
        fields = re.search(r"FIELDS (.+)", header).group(1).split()
        ncol = len(fields)
        pts = np.frombuffer(data[i:], dtype="<f4").reshape(-1, ncol)
        idx = {"x": 0, "y": 1, "z": 2, "intensity": 3}
        cols = [idx.get(f, 0) for f in ("x", "y", "z", "intensity") if f in fields]
        out = np.zeros((pts.shape[0], 4), dtype=np.float32)
        for k, f in enumerate(("x", "y", "z", "intensity")):
            if f in fields:
                out[:, k] = pts[:, fields.index(f)]
        return out[:npts]

    if b"DATA ascii" in data:
        pts = np.loadtxt(path, skiprows=11)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)
        out = np.zeros((pts.shape[0], 4), dtype=np.float32)
        out[:, :min(4, pts.shape[1])] = pts[:, :4]
        return out

    sys.exit(f"❌ 认不出 {path} 的格式（既不是 DATA binary 也不是 DATA ascii）")


def read_traj(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        rows.append([float(x) for x in line.split()])
    return np.array(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description="地图 PCD + 轨迹可视化")
    ap.add_argument("pcd", help="PCD 文件")
    ap.add_argument("--traj", default="", help="TUM 轨迹文件（可选，叠加到俯视图）")
    ap.add_argument("-o", "--out", default="", help="输出 png（默认 <pcd名>_preview.png）")
    ap.add_argument("--max-range", type=float, default=0.0,
                    help="只保留离原点该距离内的点（米），默认不过滤")
    ap.add_argument("--top", type=float, default=0.0,
                    help="只保留该高度以下的点（米），默认不过滤")
    args = ap.parse_args()

    pcd_path = Path(args.pcd)
    if not pcd_path.exists():
        sys.exit(f"❌ 文件不存在: {pcd_path}")
    pts = read_pcd(pcd_path)
    n0 = pts.shape[0]

    if args.max_range > 0:
        r = np.linalg.norm(pts[:, :3], axis=1)
        pts = pts[r <= args.max_range]
    if args.top != 0:
        pts = pts[pts[:, 2] <= args.top]

    print(f"读入 {n0} 点", end="")
    if pts.shape[0] != n0:
        print(f" → 过滤后 {pts.shape[0]} 点", end="")
    print()
    print(f"  x {pts[:,0].min():+7.1f} ~ {pts[:,0].max():+7.1f} m")
    print(f"  y {pts[:,1].min():+7.1f} ~ {pts[:,1].max():+7.1f} m")
    print(f"  z {pts[:,2].min():+7.1f} ~ {pts[:,2].max():+7.1f} m")

    out = Path(args.out) if args.out else pcd_path.with_name(pcd_path.stem + "_preview.png")

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle(f"FAST-LIO2 map — {pcd_path.name}  ({pts.shape[0]:,} points)",
                 fontsize=13, fontweight="bold")

    # 左：俯视图，按高度着色
    ax = axes[0]
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], s=0.15,
                    cmap="viridis", linewidths=0, alpha=0.8)
    plt.colorbar(sc, ax=ax, label="height z (m)")

    if args.traj:
        traj = read_traj(Path(args.traj))
        if traj.shape[0] > 0:
            ax.plot(traj[:, 1], traj[:, 2], "r-", lw=1.8, alpha=0.9, label="trajectory")
            ax.plot(traj[0, 1], traj[0, 2], "go", ms=9, label="start")
            ax.plot(traj[-1, 1], traj[-1, 2], "r^", ms=9, label="end")
            ax.legend(loc="upper right", fontsize=9)
            span = max(np.ptp(traj[:, 1]), np.ptp(traj[:, 2]))
            print(f"轨迹 {traj.shape[0]} 位姿，跨度 x {np.ptp(traj[:,1]):.2f} m  "
                  f"y {np.ptp(traj[:,2]):.2f} m")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("top view (XY, coloured by Z)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.25)

    # 右：侧视图
    ax = axes[1]
    ax.scatter(pts[:, 0], pts[:, 2], c=pts[:, 2], s=0.15,
               cmap="viridis", linewidths=0, alpha=0.7)
    if args.traj:
        traj = read_traj(Path(args.traj))
        if traj.shape[0] > 0:
            ax.plot(traj[:, 1], traj[:, 3], "r-", lw=1.8, alpha=0.9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("side view (XZ)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"✅ 图已保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
