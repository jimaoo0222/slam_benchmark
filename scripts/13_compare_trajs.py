#!/usr/bin/env python3
"""比较两条（或多条）轨迹是否走了同一条路。

为什么不能直接按时间逐点对比
────────────────────────────
两次采集的**行走速度不同**（比如 212 s vs 251 s 走同一条路）。
按时间对齐会把「走得快」的点对上「走得慢」的点，得出完全错误的大误差。
所以这里**丢掉时间信息**，只比路径的几何形状。

方法
────
1. ICP 迭代最近点，把轨迹 A 刚体对齐到轨迹 B
2. 对齐后算双向最近邻距离（Chamfer），看形状差多少

判读
────
    平均/95% 很小（几 cm）   → 同一条路
    95% 明显变大（几十 cm）  → 不同路，或其中一次走偏了

用法
────
    # 两条对比
    python3 scripts/13_compare_trajs.py a.txt b.txt

    # 多条两两全比（找分组）
    python3 scripts/13_compare_trajs.py t1.txt t2.txt t3.txt t4.txt

    # 只看 XY 平面（忽略高度漂移）
    python3 scripts/13_compare_trajs.py a.txt b.txt --plane xy
"""
from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def load(path: Path, plane: str = "xyz") -> np.ndarray:
    """读 TUM 轨迹 → (N,3) 位置。"""
    rows = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split()
        if len(p) < 4:
            continue
        try:
            rows.append((float(p[1]), float(p[2]), float(p[3])))
        except ValueError:
            continue
    if not rows:
        sys.exit(f"❌ {path} 里没读到轨迹点（需要 TUM 8 列格式）")
    a = np.array(rows, dtype=np.float64)
    if plane == "xy":
        a[:, 2] = 0.0
    return a


def _umeyama_pairs(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """已知一一对应时，求最优刚体变换 (R, t) 使 R@a+t ≈ b。"""
    mu_a, mu_b = a.mean(0), b.mean(0)
    a0, b0 = a - mu_a, b - mu_b
    cov = b0.T @ a0 / len(a)
    u, _, vt = np.linalg.svd(cov)
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_b - r @ mu_a


def icp(src: np.ndarray, dst: np.ndarray, iters: int = 40) -> tuple[np.ndarray, np.ndarray]:
    """把 src 刚体对齐到 dst（无初始对应）。返回 (R, t) 和最终最近邻距离。"""
    tree = cKDTree(dst)
    r_total, t_total = np.eye(3), np.zeros(3)
    cur = src.copy()
    for _ in range(iters):
        dist, idx = tree.query(cur)
        keep = dist < np.percentile(dist, 90)      # 去掉 10% 外点，抗噪
        if keep.sum() < 3:
            break
        r, t = _umeyama_pairs(cur[keep], dst[idx[keep]])
        cur = (r @ cur.T).T + t
        r_total, t_total = r @ r_total, r @ t_total + t
    dist, _ = tree.query(cur)
    return r_total, t_total, dist


def compare(fa: Path, fb: Path, plane: str = "xyz") -> dict:
    a, b = load(fa, plane), load(fb, plane)
    _, _, d_ab = icp(a, b)                          # a → b
    r, t, _ = icp(a, b)
    a2 = (r @ a.T).T + t
    d_ba, _ = cKDTree(a2).query(b)                  # b → a
    return {
        "mean": float((d_ab.mean() + d_ba.mean()) / 2),
        "p95": float(max(np.percentile(d_ab, 95), np.percentile(d_ba, 95))),
        "max": float(max(d_ab.max(), d_ba.max())),
    }


def path_len(p: Path, plane: str = "xyz") -> float:
    a = load(p, plane)
    return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())


def main() -> int:
    ap = argparse.ArgumentParser(description="比较轨迹形状（不看时间，只看几何）")
    ap.add_argument("trajs", nargs="+", help="2 个以上 TUM 轨迹文件")
    ap.add_argument("--plane", choices=["xyz", "xy"], default="xyz",
                    help="只用 XY 平面可忽略高度漂移（默认 xyz）")
    args = ap.parse_args()

    paths = [Path(p).expanduser() for p in args.trajs]
    for p in paths:
        if not p.is_file():
            sys.exit(f"❌ 文件不存在: {p}")

    print(f"路径长度：")
    for p in paths:
        print(f"  {p.parent.name:22s} {path_len(p, args.plane):7.2f} m   ({p.name})")

    if len(paths) < 2:
        return 0

    print()
    print(f"{'A':<22}{'B':<22}{'平均':>8}{'95%':>9}{'最大':>9}   判读")
    print("-" * 88)
    for pa, pb in itertools.combinations(paths, 2):
        r = compare(pa, pb, args.plane)
        verdict = "同一条路 ✅" if r["p95"] < 0.20 else ("差得较多 ⚠️" if r["p95"] < 0.40 else "不是同一条路 ❌")
        print(f"{pa.parent.name:<22}{pb.parent.name:<22}"
              f"{r['mean']*100:7.1f}cm{r['p95']*100:8.1f}cm{r['max']*100:8.1f}cm   {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
