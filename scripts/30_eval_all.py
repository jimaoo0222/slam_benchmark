#!/usr/bin/env python3
"""把一次全量重跑的所有轨迹，按包分组统一评测。

为什么要分组、还要共用停车窗口
────────────────────────────
每组 4 个算法走的是**同一段路、同样的停车时刻**（人什么时候站住是数据的
属性，跟算法无关）。但算法噪声不同，静止时的单步位移可以从 0.04 cm 到
0.7 cm —— 用同一个绝对速度阈值去找停车点，噪声大的算法会被少找到几个，
误差立刻虚高几十倍。所以每组固定用**一条最干净的轨迹**定停车窗口，
其余三个算法共用。

「最干净」怎么定
────────────────
按 GLIM → FAST-LIO2 → Point-LIO → LIO-SAM 的顺序试，取第一条
「用默认阈值能正好检出真值数量停车点」的轨迹。

用法
────
    python3 scripts/30_eval_all.py                       # 自动找最新的结果目录
    python3 scripts/30_eval_all.py results/2026-11-05_外参修正
    python3 scripts/30_eval_all.py --md-out 总表.md       # 另存 Markdown
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from importlib import import_module
site_eval = import_module("14_eval_on_site")

# 包 → 场地真值。两套场地、四条路线，见 docs/02 第 2 节。
SITES = {
    "01_1": "config/site_mid360_demo.json",
    "01_2": "config/site_mid360_demo.json",
    "02_1": "config/site_mid360_demo_path2.json",
    "02_2": "config/site_mid360_demo_path2.json",
    "03_01": "config/site_c01_rect.json",
    "03_02": "config/site_c01_rect_rev_open.json",
    "04_01": "config/site_mid360_demo_path2.json",
    "04_02": "config/site_mid360_path1_open.json",
}

RADAR = {
    "01_1": "MID-360s", "01_2": "MID-360s", "02_1": "MID-360s", "02_2": "MID-360s",
    "03_01": "C01", "03_02": "C01", "04_01": "C01", "04_02": "C01",
}

ALGOS = ["glim", "fastlio", "pointlio", "liosam"]
LABEL = {"glim": "GLIM", "fastlio": "FAST-LIO2",
         "pointlio": "Point-LIO", "liosam": "LIO-SAM"}

# 定停车窗口的优先顺序
REF_ORDER = ["glim", "fastlio", "pointlio", "liosam"]


def traj_of(d: Path) -> Path | None:
    for name in ("traj_glim.txt", "traj.txt"):
        p = d / name
        if p.is_file() and p.stat().st_size > 0:
            return p
    return None


def pick_reference(bag, paths):
    """挑一条能当「停车时钟」的轨迹。

    只看停车点数量是不够的 —— 数量对上但**位置挤在一起**时，说明把同一次
    停顿拆成了好几段（噪声大的算法常见），拿去当参考会让所有算法一起算错。
    所以这里做**自校验**：先用这条轨迹独立跑一次对齐，尺度必须接近 1
    （≈ 形状与真值一致），才认为它的停车时刻可信。
    """
    site = json.loads((ROOT / SITES[bag]).read_text())
    want = len(site["order"])
    best = (None, None, 9e9)
    for algo in REF_ORDER:
        p = paths.get(algo)
        if not p:
            continue
        r = site_eval.evaluate(p, site, 0.08, 2.0, None)
        if r is None:
            n = len(site_eval.find_stops(site_eval.load_traj(p), 0.08, 2.0))
            print(f"    {LABEL[algo]:<10} 检出 {n} 个停车点（真值 {want}），无法对齐 —— 跳过")
            continue
        dev = abs(r["align_scale"] - 1.0)
        verdict = "可用" if dev < 0.03 else "尺度偏太多，不用"
        print(f"    {LABEL[algo]:<10} {len(r['stops'])} 个停车点，"
              f"自校验尺度 {r['align_scale']:.4f}（偏差 {dev*100:.1f}%）—— {verdict}")
        if dev < 0.03 and dev < best[2]:
            best = (algo, p, dev)
    if best[0]:
        print(f"    参考轨迹 = {LABEL[best[0]]}")
        return best[0], best[1]
    return None, None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("results_dir", nargs="?", default="",
                    help="结果目录，默认取 results/ 下最新的 *_外参修正")
    ap.add_argument("--md-out", default="", help="把汇总表另存为 Markdown")
    ap.add_argument("--eval-dir", default="",
                    help="每条轨迹的详细报告存哪，默认 <结果目录>/eval")
    args = ap.parse_args()

    if args.results_dir:
        out_root = Path(args.results_dir).expanduser().resolve()
    else:
        cands = sorted(ROOT.glob("results/*_外参修正"))
        if not cands:
            sys.exit("找不到 results/*_外参修正，请显式指定结果目录")
        out_root = cands[-1]
    if not out_root.is_dir():
        sys.exit(f"不是目录: {out_root}")

    eval_dir = Path(args.eval_dir).expanduser() if args.eval_dir else out_root / "eval"
    eval_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 74)
    print(f" 统一评测  {out_root}")
    print("=" * 74)

    rows = []          # (bag, algo, rms, max, closure, scale_pct, n_poses)
    missing = []

    for bag in SITES:
        paths = {}
        for algo in ALGOS:
            p = traj_of(out_root / f"{bag}_{algo}")
            if p:
                paths[algo] = p
        print(f"\n── {bag}（{RADAR[bag]}）  找到 {len(paths)}/4 条轨迹 ──")
        if not paths:
            missing.append(bag)
            continue

        ref_algo, ref_path = pick_reference(bag, paths)
        if ref_path is None:
            print("    没有一条轨迹的停车点数对得上真值 —— 该组只用自动检测（结果需人工核对）")

        cmd = [sys.executable, str(ROOT / "scripts/14_eval_on_site.py"),
               *[str(paths[a]) for a in ALGOS if a in paths],
               "--site", SITES[bag],
               "--json-out", str(eval_dir / f"{bag}.json")]
        if ref_path:
            cmd += ["--stop-from", str(ref_path)]
        log = eval_dir / f"{bag}.log"
        r = subprocess.run(cmd, capture_output=True, text=True)
        log.write_text(r.stdout + r.stderr)
        if r.returncode != 0:
            print(f"    评测失败，见 {log}")
            print("    " + (r.stdout + r.stderr).strip().splitlines()[-1][:120])
            continue

        data = json.loads((eval_dir / f"{bag}.json").read_text())
        by_name = {Path(d["name"]).parent.name: d for d in data}
        for algo in ALGOS:
            key = f"{bag}_{algo}"
            d = by_name.get(key)
            if not d:
                continue
            rows.append(dict(
                bag=bag, radar=RADAR[bag], algo=algo,
                rms=d["err_rms"] * 100, mx=d["err_max"] * 100,
                closure=d["closure"] * 100,
                scale=(sum(d["seg_err_pct"]) / len(d["seg_err_pct"])) if d["seg_err_pct"] else 0.0,
                poses=d["poses"],
            ))

    # ── 汇总表 ────────────────────────────────────────────
    lines = []
    lines.append("| 包 | 雷达 | 算法 | 点位 RMS | 点位最大 | 闭合误差 | 尺度偏差 |")
    lines.append("|---|---|---|---|---|---|---|")
    for bag in SITES:
        group = sorted([r for r in rows if r["bag"] == bag], key=lambda x: x["rms"])
        for k, r in enumerate(group):
            best = "**(-)**" if k == 0 and len(group) > 1 else ""
            bag_cell = f"`{bag}`" if k == 0 else ""
            radar_cell = r["radar"] if k == 0 else ""
            closure = "—" if len(json.loads((ROOT / SITES[bag]).read_text())["order"]) < 2 or \
                json.loads((ROOT / SITES[bag]).read_text())["order"][0] != \
                json.loads((ROOT / SITES[bag]).read_text())["order"][-1] else f"{r['closure']:.1f} cm"
            lines.append(f"| {bag_cell} | {radar_cell} | {LABEL[r['algo']]}{best} | "
                         f"{r['rms']:.1f} cm | {r['mx']:.1f} cm | {closure} | {r['scale']:+.2f}% |")

    md = "\n".join(lines)
    print("\n" + "=" * 74)
    print(" 汇总")
    print("=" * 74)
    print(md)

    if missing:
        print(f"\n缺数据的包：{', '.join(missing)}")

    if args.md_out:
        Path(args.md_out).write_text(md + "\n")
        print(f"\n已存 {args.md_out}")
    (eval_dir / "summary.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False))
    print(f"明细 {eval_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
