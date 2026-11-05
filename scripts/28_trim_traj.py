#!/usr/bin/env python3
"""裁掉轨迹末尾/中间的「发散毛刺」。

有的算法在 bag 放完后还有几次迭代会给出天文数字的位姿（比如 x=2.2e5 m），
或者时间戳倒流。这些点会污染：
  · 停车点检测（速度 → 无穷大，静止段全被打断）
  · 闭合误差、总路径长

做法：保留「时间戳单调递增 + 相邻位移 < 阈值」的最长前缀。
不修改原文件，输出 <名字>_clean.txt。

用法：
    python3 scripts/28_trim_traj.py <traj.txt> [更多 traj.txt ...]
    python3 scripts/28_trim_traj.py --max-step 2.0 --inplace traj.txt
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def trim(path: Path, max_step: float):
    rows, header = [], []
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            header.append(line)
            continue
        rows.append([float(x) for x in s.split()])
    d = np.array(rows)
    if d.ndim != 2 or d.shape[0] < 2:
        return None, header, 0

    t = d[:, 0]
    step = np.linalg.norm(np.diff(d[:, 1:4], axis=0), axis=1)
    good = (np.diff(t) > 0) & (step < max_step)
    bad = np.nonzero(~good)[0]
    if bad.size:
        keep = d[: bad[0] + 1]
        return keep, header, d.shape[0] - keep.shape[0]
    return d, header, 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trajs", nargs="+")
    ap.add_argument("--max-step", type=float, default=2.0, help="相邻位姿最大位移（米），默认 2.0")
    ap.add_argument("--inplace", action="store_true", help="直接覆盖原文件")
    a = ap.parse_args()

    for p in a.trajs:
        path = Path(p)
        keep, header, cut = trim(path, a.max_step)
        if keep is None:
            print(f"⚠️  {path} 读不出轨迹，跳过")
            continue
        if cut == 0:
            print(f"✅ {path}：{len(keep)} 个位姿，无毛刺")
            continue
        out = path if a.inplace else path.with_name(path.stem + "_clean.txt")
        with open(out, "w") as f:
            for h in header or ["# timestamp x y z qx qy qz qw"]:
                f.write(h + "\n")
            np.savetxt(f, keep, fmt="%.6f")
        print(f"✂️  {path}：{keep.shape[0]} 个位姿（丢掉尾部 {cut} 个发散点）→ {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
