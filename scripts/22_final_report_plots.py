#!/usr/bin/python3
"""最终 Benchmark 报告专用综合图。

产出 3 张图（存到 --out 目录）：
    1. 综合指标总览.png    六指标分面柱状图（点位 RMS / 最大 / 尺度 / 闭合 / 漂移率 / 稳定性）
    2. 误差分布.png        12 次运行的逐停车点误差箱线图 + 散点
    3. 回环效果.png        带回环 vs 无回环的误差累积对比（最能说明问题的一张）

数据来源同 scripts/19_plot_algo_compare.py。

用法：
    /usr/bin/python3 scripts/22_final_report_plots.py \
        --site config/site_mid360_demo.json config/site_mid360_demo_path2.json \
        --alg "GLIM:results/2026-10-21_对比/{bag}_glim/traj_glim.txt" \
        --alg "FAST-LIO2:results/2026-10-21_对比/{bag}_flio/traj.txt" \
        --alg "LIO-SAM:results/2026-10-22_LIO-SAM/{bag}/traj.txt" \
        --bags 01_1 01_2 02_1 02_2 \
        --out results/最终Benchmark
"""
import argparse
import contextlib
import importlib.util
import io
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
try:
    import common.plot_setup  # noqa: F401  (中文字体等)
except Exception:  # noqa: BLE001
    pass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_eval():
    spec = importlib.util.spec_from_file_location(
        "eval_on_site", ROOT / "scripts" / "14_eval_on_site.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# 固定配色（与前面几张图保持一致，便于穿插使用）
COLORS = {"GLIM": "#1f77b4", "FAST-LIO2": "#8c564b", "LIO-SAM": "#17becf",
          "Point-LIO": "#2ca02c", "LIO-SAM ": "#17becf"}
HAS_LOOP = {"GLIM": True, "FAST-LIO2": False, "LIO-SAM": True, "Point-LIO": False}


def color_of(name):
    return COLORS.get(name, "#7f7f7f")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", nargs="+", required=True)
    ap.add_argument("--alg", action="append", required=True)
    ap.add_argument("--bags", nargs="+", required=True)
    ap.add_argument("--speed", type=float, default=0.08)
    ap.add_argument("--min-dur", type=float, default=2.0)
    ap.add_argument("--stop-from", default="",
                    help="参考轨迹模板（含 {bag}），用它的停车窗口；推荐用最干净的算法")
    ap.add_argument("--out", default="results/最终Benchmark")
    args = ap.parse_args()

    ev = load_eval()
    sites = [json.load(open(p)) for p in args.site]
    bag_site = [sites[0] if b.startswith("01") else sites[-1] for b in args.bags]
    algs = [s.split(":", 1) for s in args.alg]
    names = [n for n, _ in algs]

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows = []
    for bag, site in zip(args.bags, bag_site):
        ref_windows = None
        if args.stop_from:
            rp = ROOT / args.stop_from.replace("{bag}", bag)
            if rp.exists():
                rw = ev.find_stops(ev.load_traj(rp), args.speed, args.min_dur)
                if len(rw) == len(site["order"]):
                    ref_windows = rw
                else:
                    print(f"  ! 参考 {bag} 只找到 {len(rw)} 个停车点，退回自动检测")
            else:
                print(f"  ! 参考轨迹缺失: {rp}")
        for name, tmpl in algs:
            p = ROOT / tmpl.replace("{bag}", bag)
            if not p.exists():
                print(f"  ! 缺 {name} {bag}")
                continue
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                r = ev.evaluate(p, site, args.speed, args.min_dur, ref_windows)
            if r is None:
                print(f"  ! {name} {bag} 停车点对不上")
                continue
            e = np.asarray(r["err"]) * 100        # cm
            rows.append(dict(
                bag=bag, alg=name, poses=r["poses"],
                rms=float(np.sqrt((e ** 2).mean())),
                mean=float(e.mean()), mx=float(e.max()),
                scale=(r["align_scale"] - 1) * 100,
                close=r["closure"] * 100,
                drift=float(np.sqrt((e ** 2).mean())) / float(r["path_len_gt"]),
                per_stop=e,
                cum=None,
                aligned=r["aligned"],
                truth=r["truth"],
            ))
            # 累积误差曲线：按时间归一化到 0~1 的路径进度
            a = ev.load_traj(p)
            t = a[:, 0] - a[0, 0]
            t = t / t[-1] if t[-1] > 0 else t
            rows[-1]["cum"] = (t, e * 1.0)  # 这里 e 是停车点误差，另算逐点

    if not rows:
        print("  无数据")
        return

    print(f"\n  {'bag':5s} {'算法':11s} {'RMS':>7s} {'mean':>7s} {'max':>7s} "
          f"{'尺度%':>7s} {'闭合':>7s} {'漂移cm/m':>9s}")
    print("  " + "-" * 68)
    for r in rows:
        print(f"  {r['bag']:5s} {r['alg']:11s} {r['rms']:6.2f}c {r['mean']:6.2f}c "
              f"{r['mx']:6.2f}c {r['scale']:+6.3f} {r['close']:6.2f}c {r['drift']:8.3f}")

    # ── 图 1：六指标分面总览 ──────────────────────────────────
    agg = {}
    for n in names:
        s = [r for r in rows if r["alg"] == n]
        if not s:
            continue
        agg[n] = dict(
            rms=np.mean([r["rms"] for r in s]),
            rms_std=np.std([r["rms"] for r in s]),
            mx=np.max([r["mx"] for r in s]),
            scale=np.mean([abs(r["scale"]) for r in s]),
            close=np.mean([r["close"] for r in s]),
            drift=np.mean([r["drift"] for r in s]),
            spread=np.max([r["rms"] for r in s]) - np.min([r["rms"] for r in s]),
        )

    panels = [
        ("rms",      "① 点位 RMS 误差 (cm)",        "越小越好", None),
        ("mx",       "② 最差单点误差 (cm)",         "越小越好", None),
        ("close",    "③ 闭合误差 (cm)",             "走一圈回到原点", None),
        ("drift",    "④ 漂移率 (cm/m 路径)",        "每走 1 m 累积的误差", None),
        ("scale",    "⑤ 尺度偏差 绝对值 (%)",       "越小越好", None),
        ("spread",   "⑥ 稳定性：重复跑之间的极差 (cm)", "越小越好", None),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.6))
    for ax, (key, title, sub, _) in zip(axes.ravel(), panels):
        vals = [agg[n][key] for n in names if n in agg]
        lbls = [n for n in names if n in agg]
        bars = ax.bar(lbls, vals, color=[color_of(n) for n in lbls], width=0.62)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.2f}",
                    ha="center", va="bottom", fontsize=9)
        ax.set_title(f"{title}\n({sub})", fontsize=10)
        ax.set_ylabel(key if key != "drift" else "cm/m", fontsize=9)
        ax.tick_params(labelsize=9)
        ax.set_ylim(0, max(vals) * 1.22 if max(vals) > 0 else 1)
    fig.suptitle(f"{len(names)} 个算法综合指标总览（{len(args.bags)} 个 bag 统计）", fontsize=14)
    fig.tight_layout()
    fig.savefig(out / "综合指标总览.png")
    plt.close(fig)
    print(f"\n  ✅ {out/'综合指标总览.png'}")

    # ── 图 2：逐停车点误差箱线 + 散点 ────────────────────────
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    data, lbls, cols = [], [], []
    for n in names:
        vals = np.concatenate([[v for v in r["per_stop"]] for r in rows if r["alg"] == n])
        if len(vals) == 0:
            continue
        data.append(vals); lbls.append(n); cols.append(color_of(n))
    bp = ax.boxplot(data, tick_labels=lbls, patch_artist=True, widths=0.5,
                    medianprops=dict(color="black", lw=1.6))
    for patch, c in zip(bp["boxes"], cols):
        patch.set_facecolor(c); patch.set_alpha(0.45)
    rng = np.random.default_rng(0)
    for i, (vals, c) in enumerate(zip(data, cols), start=1):
        ax.scatter(rng.normal(i, 0.06, len(vals)), vals, s=16, color=c,
                   alpha=0.75, zorder=3, edgecolors="none")
    ax.set_ylabel("停车点定位误差 (cm)", fontsize=10)
    ax.set_title(f"逐停车点误差分布（每算法 {len(args.bags)} 次运行 × 6 个停车点）", fontsize=11)
    ax.tick_params(labelsize=10)
    fig.tight_layout()
    fig.savefig(out / "误差分布.png")
    plt.close(fig)
    print(f"  ✅ {out/'误差分布.png'}")

    # ── 图 3：回环效果（闭合误差 vs 路径，最直观）────────────
    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    x = np.arange(len(args.bags))
    w = 0.8 / len(names)
    for k, n in enumerate(names):
        vals = [next((r["close"] for r in rows if r["bag"] == b and r["alg"] == n), np.nan)
                for b in args.bags]
        tag = "（带回环）" if HAS_LOOP.get(n) else "（无回环）"
        ax.bar(x + k * w - 0.4 + w / 2, vals, w, label=n + tag, color=color_of(n))
        for xi, v in zip(x + k * w - 0.4 + w / 2, vals):
            if not np.isnan(v):
                ax.text(xi, v, f"{v:.1f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(args.bags)
    ax.set_ylabel("闭合误差 (cm)", fontsize=10)
    ax.set_title("闭合误差：走一圈回到原点的偏差\n（注意 Point-LIO 无回环却最好 —— 回环不是唯一因素）", fontsize=10)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "回环效果.png")
    plt.close(fig)
    print(f"  ✅ {out/'回环效果.png'}")

    # ── 图 4：对齐后轨迹 vs 真值（2×2 布局，场地 6.3×1.8 m 没法并排）──
    nb = len(args.bags)
    ncol = min(2, nb)
    nrow = int(np.ceil(nb / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.8 * ncol, 3.4 * nrow))
    axes = np.atleast_1d(axes).ravel()
    for ax, bag in zip(axes, args.bags):
        for k, n in enumerate(names):
            r = next((r for r in rows if r["bag"] == bag and r["alg"] == n), None)
            if r is None or "aligned" not in r:
                continue
            ax.plot(r["aligned"][:, 0], r["aligned"][:, 1], "-", lw=1.4,
                    color=color_of(n), label=n, alpha=0.85)
        tr = next((r["truth"] for r in rows if r["bag"] == bag), None)
        if tr is not None:
            ax.plot(tr[:, 0], tr[:, 1], "k--o", lw=1.8, ms=6, label="真值", zorder=5)
        ax.set_aspect("equal")
        ax.set_title(bag, fontsize=11)
        ax.set_xlabel("x (m)", fontsize=9)
        ax.set_ylabel("y (m)", fontsize=9)
        ax.tick_params(labelsize=8)
    for ax in axes[nb:]:
        ax.axis("off")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("对齐后轨迹 vs 场地真值（彩线越贴近黑虚线越准）", fontsize=12)
    fig.tight_layout()
    fig.savefig(out / "轨迹对比.png")
    plt.close(fig)
    print(f"  ✅ {out/'轨迹对比.png'}")

    # ── 图 5：误差放大图（把各算法相对真值的偏差单独放大画出来）──
    fig, axes = plt.subplots(nrow, ncol, figsize=(5.2 * ncol, 3.8 * nrow))
    axes = np.atleast_1d(axes).ravel()
    for ax, bag in zip(axes, args.bags):
        for k, n in enumerate(names):
            r = next((r for r in rows if r["bag"] == bag and r["alg"] == n), None)
            if r is None or "aligned" not in r:
                continue
            e = (r["aligned"] - r["truth"]) * 100.0   # cm
            ax.plot(e[:, 0], e[:, 1], "o-", ms=5, lw=1.1, color=color_of(n),
                    label=f"{n} (RMS {np.sqrt((e**2).sum(1).mean()):.2f}cm)", alpha=0.9)
        ax.axhline(0, color="k", lw=0.6, alpha=0.4)
        ax.axvline(0, color="k", lw=0.6, alpha=0.4)
        ax.set_aspect("equal")
        ax.set_title(bag, fontsize=11)
        ax.set_xlabel("x 方向误差 (cm)", fontsize=9)
        ax.set_ylabel("y 方向误差 (cm)", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.legend(fontsize=7, loc="best")
    for ax in axes[nb:]:
        ax.axis("off")
    fig.suptitle("各停车点定位误差放大图（坐标单位 cm，越靠近原点越准）", fontsize=12)
    fig.tight_layout()
    fig.savefig(out / "误差放大.png")
    plt.close(fig)
    print(f"  ✅ {out/'误差放大.png'}")

    # ── 机器可读结果 ────────────────────────────────────────
    with open(out / "metrics.json", "w") as f:
        json.dump(dict(
            per_run=[{k: v for k, v in r.items() if k not in ("per_stop", "cum", "aligned", "truth")} for r in rows],
            summary=agg,
        ), f, ensure_ascii=False, indent=1)
    print(f"  ✅ {out/'metrics.json'}")

    # ── 控制台汇总 ──────────────────────────────────────────
    print("\n  ── 分算法汇总 ──")
    for n in names:
        if n in agg:
            a = agg[n]
            print(f"  {n:11s} RMS {a['rms']:.2f}±{a['rms_std']:.2f}cm  "
                  f"闭合 {a['close']:.2f}cm  漂移 {a['drift']:.3f}cm/m  "
                  f"尺度 {a['scale']:.3f}%  稳定性 {a['spread']:.2f}cm")


if __name__ == "__main__":
    main()
