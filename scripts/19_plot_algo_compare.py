#!/usr/bin/python3
"""三个算法 × 多个 bag 的绝对精度对比图。

用法：
    /usr/bin/python3 scripts/19_plot_algo_compare.py \
        --site config/site_mid360_demo.json config/site_mid360_demo_path2.json \
        --alg "GLIM:results/2026-10-21_对比/{bag}_glim/traj_glim.txt" \
        --alg "FAST-LIO2:results/2026-10-21_对比/{bag}_flio/traj.txt" \
        --alg "LIO-SAM:results/2026-10-22_LIO-SAM/{bag}/traj.txt" \
        --bags 01_1 01_2 02_1 02_2 \
        --out results/2026-10-22_三算法对比

每个 --alg 是 `名字:轨迹路径模板`，模板里的 {bag} 会被替换。
各 bag 对应的真值 JSON 由 --bag-site 指定（默认：01_* / 02_* 自动选）。
"""
import argparse
import importlib.util
import io
import contextlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys_setup = ROOT / "common" / "plot_setup.py"
if sys_setup.exists():
    import sys
    sys.path.insert(0, str(ROOT))
    import common.plot_setup  # noqa: F401

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_eval_module():
    spec = importlib.util.spec_from_file_location(
        "eval_on_site", ROOT / "scripts" / "14_eval_on_site.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", nargs="+", required=True,
                    help="真值 JSON，可给多个；用 --bag-site 关联")
    ap.add_argument("--alg", action="append", required=True,
                    help="名字:路径模板，模板含 {bag}")
    ap.add_argument("--bags", nargs="+", required=True)
    ap.add_argument("--bag-site", nargs="+", default=None,
                    help="与 --bags 一一对应的真值 JSON；省略则按 01_*/02_* 前缀自动选")
    ap.add_argument("--speed", type=float, default=0.08)
    ap.add_argument("--min-dur", type=float, default=2.0)
    ap.add_argument("--out", default="results/compare")
    args = ap.parse_args()

    ev = load_eval_module()
    sites = [json.load(open(p)) for p in args.site]
    if args.bag_site:
        bag_site = [json.load(open(p)) for p in args.bag_site]
    else:
        bag_site = [sites[0] if b.startswith("01") else sites[-1] for b in args.bags]

    algs = []
    for spec in args.alg:
        name, tmpl = spec.split(":", 1)
        algs.append((name, tmpl))

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for bag, site in zip(args.bags, bag_site):
        for name, tmpl in algs:
            p = ROOT / tmpl.replace("{bag}", bag)
            if not p.exists():
                print(f"  ! 缺 {name} {bag}: {p}")
                continue
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    r = ev.evaluate(p, site, args.speed, args.min_dur)
            except Exception as e:  # noqa: BLE001
                print(f"  ! {name} {bag} 评测异常: {e}")
                continue
            if r is None:
                print(f"  ! {name} {bag} 停车点对不上")
                continue
            e = np.asarray(r["err"])
            rows.append(dict(
                bag=bag, alg=name, rms=float(np.sqrt((e ** 2).mean())) * 100,
                mx=float(e.max()) * 100, close=r["closure"] * 100,
                scale=(r["align_scale"] - 1) * 100,
                aligned=r["aligned"], truth=r["truth"]))

    if not rows:
        print("  没有任何可用数据")
        return

    # ── 表 ────────────────────────────────────────────────────
    print(f"\n  {'bag':6s} {'算法':11s} {'RMS':>8s} {'最大':>8s} {'尺度偏差':>10s} {'闭合':>8s}")
    print("  " + "-" * 58)
    for r in rows:
        print(f"  {r['bag']:6s} {r['alg']:11s} {r['rms']:7.2f}cm {r['mx']:7.2f}cm "
              f"{r['scale']:+9.3f}% {r['close']:7.2f}cm")

    hull = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    if not hull.exists():  # matplotlib 自带 DejaVu
        pass

    names = [n for n, _ in algs]
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(names), 3)))

    # ── 图 1：每个 bag 的误差柱状对比 ─────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for ax, key, title in ((axes[0], "rms", "点位 RMS 误差（越小越好）"),
                           (axes[1], "close", "闭合误差（走一圈回到起点）")):
        x = np.arange(len(args.bags))
        w = 0.8 / len(names)
        for k, nm in enumerate(names):
            vals = [next((r[key] for r in rows if r["bag"] == b and r["alg"] == nm), np.nan)
                    for b in args.bags]
            ax.bar(x + k * w - 0.4 + w / 2, vals, w, label=nm, color=colors[k])
        ax.set_xticks(x)
        ax.set_xticklabels(args.bags)
        ax.set_ylabel("cm")
        ax.set_title(title)
        ax.legend(fontsize=8)
    fig.suptitle("三算法绝对精度对比", fontsize=13)
    fig.tight_layout()
    f1 = out_dir / "精度对比.png"
    fig.savefig(f1)
    plt.close(fig)
    print(f"\n  ✅ {f1}")

    # ── 图 2：对齐后的轨迹 vs 真值（2×2 布局，场地 6.3×1.8 m 没法并排 4 栏）──
    n = len(args.bags)
    ncol = min(2, n)
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.6 * ncol, 3.2 * nrow))
    axes = np.atleast_1d(axes).ravel()
    for ax, bag in zip(axes, args.bags):
        for k, nm in enumerate(names):
            r = next((r for r in rows if r["bag"] == bag and r["alg"] == nm), None)
            if r is None:
                continue
            ax.plot(r["aligned"][:, 0], r["aligned"][:, 1], "-", lw=1.3,
                    color=colors[k], label=nm, alpha=0.85)
        tr = next((r["truth"] for r in rows if r["bag"] == bag), None)
        if tr is not None:
            ax.plot(tr[:, 0], tr[:, 1], "k--o", lw=1.8, ms=6, label="真值", zorder=5)
        ax.set_aspect("equal")
        ax.set_title(bag, fontsize=11)
        ax.set_xlabel("x (m)", fontsize=9)
        ax.set_ylabel("y (m)", fontsize=9)
        ax.tick_params(labelsize=8)
    for ax in axes[n:]:
        ax.axis("off")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("对齐后轨迹 vs 场地真值（三条彩线几乎完全重合 = 都准）", fontsize=12)
    fig.tight_layout()
    f2 = out_dir / "轨迹对比.png"
    fig.savefig(f2)
    plt.close(fig)
    print(f"  ✅ {f2}")

    # ── 图 3：每个停车点的误差增广曲线 ───────────────────────
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for bag in args.bags:
        for k, nm in enumerate(names):
            r = next((r for r in rows if r["bag"] == bag and r["alg"] == nm), None)
            if r is None:
                continue
            # 停车点误差在 rows 里没存，用对齐点重算
            err_pts = np.linalg.norm(r["aligned"] - r["truth"], axis=1) * 100
            ax.plot(np.arange(len(err_pts)), err_pts, marker="o", ms=4, lw=1.2,
                    color=colors[k], alpha=0.6 if bag.endswith("_2") else 1.0,
                    ls="--" if bag.endswith("_2") else "-",
                    label=f"{nm} {bag}")
    ax.set_xlabel("停车点序号", fontsize=10)
    ax.set_ylabel("定位误差 (cm)", fontsize=10)
    ax.set_title("各停车点定位误差", fontsize=12)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    f3 = out_dir / "停车点误差.png"
    fig.savefig(f3)
    plt.close(fig)
    print(f"  ✅ {f3}")

    with open(out_dir / "metrics.json", "w") as f:
        json.dump([{k: v for k, v in r.items() if k not in ("aligned", "truth")}
                   for r in rows], f, ensure_ascii=False, indent=1)
    print(f"  ✅ {out_dir/'metrics.json'}")


if __name__ == "__main__":
    main()
