#!/usr/bin/env python3
"""把建图 PCD 看成 3D —— 生成可交互 HTML（鼠标拖着转）+ 多角度静态图。

用法
────
    python3 scripts/08_view_map_3d.py <map.pcd> [--traj traj.txt] [-o 输出前缀]

产出
────
    <前缀>_3d.html      ⭐ 浏览器打开，鼠标拖动旋转、滚轮缩放
    <前缀>_views.png    四个固定角度的静态图（没有浏览器时看这个）

为什么要降采样
──────────────
浏览器扛不住几十万点。默认按体素降到 6 万点以内，形状几乎不变，旋转却很流畅。
用 --points 调；想看清局部细节就用 --max-range 裁一块近的。
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np


def read_pcd(path: Path) -> np.ndarray:
    data = path.read_bytes()
    marker = b"DATA binary\n"
    if marker not in data:
        sys.exit(f"❌ {path} 不是 binary PCD（本项目 06_save_map_pcd.py 写的是 binary）")
    i = data.index(marker) + len(marker)
    header = data[:i].decode("ascii", "replace")
    fields = re.search(r"FIELDS (.+)", header).group(1).split()
    npts = int(re.search(r"POINTS (\d+)", header).group(1))
    pts = np.frombuffer(data[i:], dtype="<f4").reshape(-1, len(fields))
    out = np.zeros((pts.shape[0], 4), dtype=np.float32)
    for k, f in enumerate(("x", "y", "z", "intensity")):
        if f in fields:
            out[:, k] = pts[:, fields.index(f)]
    return out[:npts]


def read_traj(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        rows.append([float(x) for x in line.split()])
    return np.array(rows)


def voxel_downsample(pts: np.ndarray, target: int) -> np.ndarray:
    """按体素降采样到 target 点以内；自动搜索体素边长。"""
    if pts.shape[0] <= target:
        return pts
    lo = 1e-3
    hi = 5.0
    best = pts
    for _ in range(24):
        voxel = (lo + hi) / 2
        key = np.floor(pts[:, :3] / voxel).astype(np.int64)
        _, keep = np.unique(key, axis=0, return_index=True)
        best = pts[np.sort(keep)]
        if best.shape[0] > target:
            lo = voxel
        else:
            hi = voxel
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description="3D 看建图结果")
    ap.add_argument("pcd")
    ap.add_argument("--traj", default="")
    ap.add_argument("-o", "--out", default="", help="输出前缀（默认与 pcd 同名）")
    ap.add_argument("--points", type=int, default=60000, help="HTML 里最多渲染多少点")
    ap.add_argument("--max-range", type=float, default=0.0, help="只保留该距离内的点")
    ap.add_argument("--voxel", type=float, default=0.0, help="直接指定体素边长，跳过自动搜索")
    args = ap.parse_args()

    pcd_path = Path(args.pcd)
    if not pcd_path.exists():
        sys.exit(f"❌ 文件不存在: {pcd_path}")

    pts = read_pcd(pcd_path)
    n_raw = pts.shape[0]
    if args.max_range > 0:
        r = np.linalg.norm(pts[:, :3], axis=1)
        pts = pts[r <= args.max_range]

    preview = voxel_downsample(pts, args.points) if args.voxel <= 0 else \
        pts[np.sort(np.unique(np.floor(pts[:, :3] / args.voxel).astype(np.int64),
                              axis=0, return_index=True)[1])]

    prefix = Path(args.out) if args.out else pcd_path.with_suffix("")
    prefix = Path(str(prefix).removesuffix(".pcd"))

    print(f"原始 {n_raw:,} 点 → 渲染 {preview.shape[0]:,} 点"
          f"（{preview.shape[0]/max(n_raw,1)*100:.1f}%）")
    print(f"  x {preview[:,0].min():+7.1f} ~ {preview[:,0].max():+7.1f} m")
    print(f"  y {preview[:,1].min():+7.1f} ~ {preview[:,1].max():+7.1f} m")
    print(f"  z {preview[:,2].min():+7.1f} ~ {preview[:,2].max():+7.1f} m")

    traj = read_traj(Path(args.traj)) if args.traj else None

    # ── 1. 可交互 HTML ────────────────────────────────────────
    try:
        import plotly.graph_objects as go
    except ImportError:
        go = None

    if go is not None:
        fig = go.Figure()
        fig.add_trace(go.Scatter3d(
            x=preview[:, 0], y=preview[:, 1], z=preview[:, 2],
            mode="markers",
            marker=dict(
                size=1.1,
                color=preview[:, 2],
                colorscale="Viridis",
                opacity=0.85,
                colorbar=dict(title="height<br>z (m)", thickness=12),
            ),
            name="map",
            hovertemplate="x=%{x:.1f} y=%{y:.1f} z=%{z:.1f}<extra></extra>",
        ))
        if traj is not None and traj.shape[0] > 0:
            fig.add_trace(go.Scatter3d(
                x=traj[:, 1], y=traj[:, 2], z=traj[:, 3],
                mode="lines",
                line=dict(color="red", width=6),
                name="trajectory",
            ))
            fig.add_trace(go.Scatter3d(
                x=[traj[0, 1]], y=[traj[0, 2]], z=[traj[0, 3]],
                mode="markers+text", text=["START"], textposition="top center",
                marker=dict(size=6, color="lime"), name="start",
            ))
            fig.add_trace(go.Scatter3d(
                x=[traj[-1, 1]], y=[traj[-1, 2]], z=[traj[-1, 3]],
                mode="markers+text", text=["END"], textposition="top center",
                marker=dict(size=6, color="red"), name="end",
            ))

        fig.update_layout(
            title=dict(
                text=f"FAST-LIO2 map — {pcd_path.name}  "
                     f"({preview.shape[0]:,} / {n_raw:,} points)",
                font=dict(size=16),
            ),
            scene=dict(
                xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)",
                aspectmode="data",
                bgcolor="#101014",
                camera=dict(
                    eye=dict(x=-1.6, y=-1.6, z=0.9),   # 初始斜俯视，一眼看到楼层
                    center=dict(x=0, y=0, z=0),
                    up=dict(x=0, y=0, z=1),
                ),
                xaxis=dict(gridcolor="#303040", color="#cccccc"),
                yaxis=dict(gridcolor="#303040", color="#cccccc"),
                zaxis=dict(gridcolor="#303040", color="#cccccc"),
            ),
            paper_bgcolor="#101014",
            font=dict(color="#dddddd"),
            legend=dict(x=0.01, y=0.99, bgcolor="rgba(20,20,25,0.7)"),
            margin=dict(l=0, r=0, t=44, b=0),
        )  # 拖一下就能转
        html = prefix.parent / (prefix.name + "_3d.html")
        html.parent.mkdir(parents=True, exist_ok=True)
        fig.write_html(html, include_plotlyjs=True, config={  # 内联 js，离线也能开
            "scrollZoom": True,
            "displaylogo": False,
        })
        print(f"✅ 可交互 3D: {html}")
        print("   浏览器打开后：左键拖动旋转、滚轮缩放、双击复位")

    # ── 2. 四角度静态图（没有浏览器时看这个）────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(17, 13), facecolor="#101014")
    views = [
        ("perspective (30°, -60°)", 30, -60),
        ("front (10°, -90°)", 10, -90),
        ("side (10°, 0°)", 10, 0),
        ("top (89°, -90°)", 89, -90),
    ]
    # 按高度着色，和 HTML 一致
    zmin, zmax = preview[:, 2].min(), preview[:, 2].max()
    cnorm = (preview[:, 2] - zmin) / max(zmax - zmin, 1e-6)

    for idx, (title, elev, azim) in enumerate(views, start=1):
        ax = fig.add_subplot(2, 2, idx, projection="3d", facecolor="#101014")
        ax.scatter(preview[:, 0], preview[:, 1], preview[:, 2],
                   c=cnorm, cmap="viridis", s=0.35, linewidths=0, alpha=0.75)
        if traj is not None and traj.shape[0] > 0:
            ax.plot(traj[:, 1], traj[:, 2], traj[:, 3], color="red", lw=2.2, zorder=10)
            ax.scatter([traj[0, 1]], [traj[0, 2]], [traj[0, 3]],
                       c="lime", s=70, depthshade=False, zorder=11)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title, color="#dddddd", fontsize=11)
        ax.set_xlabel("x (m)", color="#999999", fontsize=8)
        ax.set_ylabel("y (m)", color="#999999", fontsize=8)
        ax.set_zlabel("z (m)", color="#999999", fontsize=8)
        ax.tick_params(colors="#777777", labelsize=7)
        for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
            pane.pane.set_facecolor("#15151a")
            pane.pane.set_edgecolor("#333344")

    fig.suptitle(f"FAST-LIO2 map — {preview.shape[0]:,} points",
                 color="#eeeeee", fontsize=15, fontweight="bold")
    png = prefix.parent / (prefix.name + "_views.png")
    fig.savefig(png, dpi=120, bbox_inches="tight", facecolor="#101014")
    print(f"✅ 四角度静态图: {png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
