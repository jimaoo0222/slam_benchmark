#!/usr/bin/python3
"""按论文（GLIM Table IV/V）格式生成 benchmark 表。

产出三张同结构的表（行=算法，列=bag，末列 Average，前三名标色）：

    表 1  序列信息            ← 对应论文 Table IV（时长 / 路径长 / 停车点数）
    表 2  绝对轨迹误差 ATE    ← 对应论文 Table V
    表 3  闭合误差            ← 本项目特有：能否走回原点
    表 4  尺度偏差            ← 本项目特有：相对量的核心

> 论文 Table VI 的 RTE（相对轨迹误差）**本项目无法计算**：
> 它需要**逐帧真值**（论文用 mocap / 高精度参考系统），
> 而我们的真值只有 6 个卷尺标定的停车点。硬算出来的数字没有物理意义，
> 所以改用「闭合误差 + 尺度偏差」这两个同样反映「相对形变」的指标替代。

用法：
    /usr/bin/python3 scripts/25_paper_style_tables.py \
        --site config/site_mid360_demo.json config/site_mid360_demo_path2.json \
        --alg "GLIM:results/2026-10-21_对比/{bag}_glim/traj_glim.txt" \
        --alg "FAST-LIO2:results/2026-10-21_对比/{bag}_flio/traj.txt" \
        --alg "LIO-SAM:results/2026-10-22_LIO-SAM/{bag}/traj.txt" \
        --alg "Point-LIO:results/2026-10-25_Point-LIO/{bag}/traj.txt" \
        --bags 01_1 01_2 02_1 02_2 \
        --out results/最终Benchmark
"""
from __future__ import annotations

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
    import common.plot_setup  # noqa: F401   中文字体
except Exception:  # noqa: BLE001
    pass

HAS_LOOP = {"GLIM": True, "LIO-SAM": True, "CLINS": True,
            "Point-LIO": False, "FAST-LIO2": False, "DLO": False, "LINS": False}

PATH_ORDER = {"01": "0→1→2→3→4→0", "02": "0→2→3→4→1→0"}


def load_eval():
    spec = importlib.util.spec_from_file_location(
        "eval_on_site", ROOT / "scripts" / "14_eval_on_site.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def rank_marks(values: list[float]) -> list[str]:
    """越小越好 → 前三名返回 🥇🥈🥉"""
    valid = [(v, i) for i, v in enumerate(values) if not np.isnan(v)]
    order = [i for _, i in sorted(valid)]
    marks = [""] * len(values)
    for rank, i in enumerate(order[:3]):
        marks[i] = ["🥇", "🥈", "🥉"][rank]
    return marks


def build_md_table(title: str, note: str, data: dict, names: list[str],
                   bags: list[str], scale: float, nd: int = 2,
                   suffix: str = "") -> str:
    col_marks = {b: rank_marks([data[(n, b)] for n in names]) for b in bags}
    avg = {n: float(np.nanmean([data[(n, b)] for b in bags])) for n in names}
    avg_marks = rank_marks([avg[n] for n in names])

    L = [title, ""]
    if note:
        L += [note, ""]
    L.append("| 方法 | Loop closure | " + " | ".join(bags) + " | Average |")
    L.append("|---|---|" + "---|" * (len(bags) + 1))
    for i, n in enumerate(names):
        cells = [f"{data[(n, b)]*scale:.{nd}f}{suffix}{col_marks[b][i]}" for b in bags]
        L.append(f"| **{n}** | {'✓' if HAS_LOOP.get(n) else ''} | "
                 + " | ".join(cells)
                 + f" | **{avg[n]*scale:.{nd}f}{suffix}**{avg_marks[i]} |")
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", nargs="+", required=True)
    ap.add_argument("--alg", action="append", required=True)
    ap.add_argument("--bags", nargs="+", required=True)
    ap.add_argument("--stop-from", default="", help="参考轨迹模板（定停车窗口）")
    ap.add_argument("--out", default="results/最终Benchmark")
    args = ap.parse_args()

    ev = load_eval()
    sites = [json.load(open(p)) for p in args.site]
    bag_site = [sites[0] if b.startswith("01") else sites[-1] for b in args.bags]
    algs = [s.split(":", 1) for s in args.alg]
    names = [n for n, _ in algs]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    ref_tmpl = args.stop_from or algs[0][1]
    windows = {}
    for bag in args.bags:
        rp = ROOT / ref_tmpl.replace("{bag}", bag)
        if rp.exists():
            windows[bag] = ev.find_stops(ev.load_traj(rp), 0.08, 2.0)

    ate, close, scale_dev, info = {}, {}, {}, {}
    for bag, site in zip(args.bags, bag_site):
        for name, tmpl in algs:
            p = ROOT / tmpl.replace("{bag}", bag)
            if not p.exists():
                print(f"  ! 缺 {name} {bag}")
                for D in (ate, close, scale_dev):
                    D[(name, bag)] = float("nan")
                continue
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                r = ev.evaluate(p, site, 0.08, 2.0, windows.get(bag))
            if r is None:
                for D in (ate, close, scale_dev):
                    D[(name, bag)] = float("nan")
                continue
            e = np.asarray(r["err"])
            ate[(name, bag)] = float(np.sqrt((e ** 2).mean()))
            close[(name, bag)] = float(r["closure"])
            scale_dev[(name, bag)] = abs(float(r["align_scale"]) - 1.0)
            info[(name, bag)] = dict(poses=r["poses"], dur=r["dur"],
                                     plen=r["path_len_slam"])
            print(f"  {bag} {name:11s} ATE {ate[(name,bag)]*100:6.2f}cm  "
                  f"闭合 {close[(name,bag)]*100:6.2f}cm  尺度 {scale_dev[(name,bag)]*100:6.3f}%")

    # ── 表 1：序列信息 ──
    L = ["### 表 1 · 测试序列信息（对应论文 Table IV）", "",
         "| 序列 | 行走顺序 | 时长 (s) | 真值路径长 (m) | 停车点数 | 算法位姿数 |",
         "|---|---|---|---|---|---|"]
    for bag, site in zip(args.bags, bag_site):
        pts = np.asarray(site["points"], dtype=float)
        o = site["order"]
        gt = sum(float(np.linalg.norm(pts[o[i + 1]] - pts[o[i]])) for i in range(len(o) - 1))
        inf = next((info[(n, bag)] for n in names if (n, bag) in info), None)
        pref = bag.split("_")[0]
        L.append(f"| `{bag}` | {PATH_ORDER.get(pref, '—')} | "
                 f"{inf['dur']:.0f} | {gt:.3f} | {len(o)} | {inf['poses']} |")
    t_info = "\n".join(L)

    t_ate = build_md_table(
        "### 表 2 · 绝对轨迹误差 ATE（停车点定位 RMS，cm）—— 对应论文 Table V",
        "> 整条轨迹经 Umeyama（旋转+平移+尺度）对齐后，6 个停车点位置误差的均方根。",
        ate, names, args.bags, 100.0)

    t_close = build_md_table(
        "### 表 3 · 闭合误差（走一圈回到原点的偏差，cm）",
        "> 真值路径首尾同点，所以「终点离起点多远」直接反映累积漂移。**这是最能拉开差距的指标。**",
        close, names, args.bags, 100.0)

    t_scale = build_md_table(
        "### 表 4 · 尺度偏差绝对值（%，相对量）",
        "> 对应论文 RTE 想表达的「相对形变」精神：反映里程计尺度估计准不准，与全局位姿无关。",
        scale_dev, names, args.bags, 100.0, nd=3, suffix="%")

    md = "\n\n".join([
        "# Benchmark 结果表（论文格式）",
        "",
        "> 场地：室内货架仓库 · 4 个自录 bag（2 条路径 × 各 2 次重复）· 16 次运行",
        "> 停车窗口统一取自参考轨迹（`--stop-from`），消除速度阈值对不同噪声算法的不公平",
        "",
        t_info, "", t_ate, "", t_close, "", t_scale, "",
        "---", "",
        "**图例**：🥇 最好 · 🥈 次好 · 🥉 第三",
        "",
        "**单位说明**：论文原表用 m（Newer College 场景误差在分米级）；",
        "本项目路径仅 15 m、误差在毫米级，故用 **cm** 以便阅读（1 cm = 0.01 m）。",
        "",
        "**关于 RTE（论文 Table VI）**：需要**逐帧真值**才能计算（论文用 mocap），",
        "本项目真值只有 6 个卷尺标定的停车点，无法计算等效指标 —— ",
        "改用「闭合误差」（表 3）和「尺度偏差」（表 4）表达同类信息，",
        "不为凑表格格式而给出没有物理意义的数字。",
    ])
    (out / "论文格式表.md").write_text(md)
    print(f"\n  ✅ {out/'论文格式表.md'}")

    # ── 渲染图片 ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        RANK_BG = {"🥇": "#cfe8ff", "🥈": "#ffd6d6", "🥉": "#e8e8e8", "": "#ffffff"}

        def render(ax, title, note, data, scale, nd, suffix=""):
            labels = ["Method", "Loop\nclosure"] + args.bags + ["Average"]
            col_marks = {b: rank_marks([data[(n, b)] for n in names]) for b in args.bags}
            avg = {n: float(np.nanmean([data[(n, b)] for b in args.bags])) for n in names}
            avg_marks = rank_marks([avg[n] for n in names])
            cells, colors = [], []
            for i, n in enumerate(names):
                row = [n, "✓" if HAS_LOOP.get(n) else ""]
                cc = ["#ffffff", "#ffffff"]
                for b in args.bags:
                    row.append(f"{data[(n,b)]*scale:.{nd}f}{suffix}")
                    cc.append(RANK_BG[col_marks[b][i]])
                row.append(f"{avg[n]*scale:.{nd}f}{suffix}")
                cc.append(RANK_BG[avg_marks[i]])
                cells.append(row)
                colors.append(cc)
            tb = ax.table(cellText=cells, colLabels=labels, cellColours=colors,
                          loc="center", cellLoc="center")
            tb.auto_set_font_size(False)
            tb.set_fontsize(9.5)
            tb.scale(1, 2.0)
            for j in range(len(labels)):
                tb[0, j].set_facecolor("#d0d0d0")
                tb[0, j].set_text_props(weight="bold")
            for i in range(1, len(cells) + 1):
                tb[i, 0].set_text_props(weight="bold")
            ax.set_title(f"{title}\n{note}" if note else title,
                         fontsize=11, weight="bold", pad=16, linespacing=1.7)
            ax.axis("off")

        # ── 序列信息表（对应论文 Table IV）──
        fig0, ax0 = plt.subplots(figsize=(9.2, 2.2))
        rows0, bags0 = [], []
        for bag, site in zip(args.bags, bag_site):
            pts = np.asarray(site["points"], dtype=float)
            o = site["order"]
            gt = sum(float(np.linalg.norm(pts[o[i + 1]] - pts[o[i]]))
                     for i in range(len(o) - 1))
            inf = next((info[(n, bag)] for n in names if (n, bag) in info), None)
            bags0.append(bag)
            rows0.append([f"{inf['dur']:.0f}" if inf else "—",
                          f"{gt:.3f}", str(len(o)),
                          PATH_ORDER.get(bag.split("_")[0], "—")])
        cols0 = bags0
        tbl = ax0.table(cellText=[[r[i] for r in rows0] for i in range(4)],
                        rowLabels=["Duration (s)", "Path length (m)",
                                   "Stops", "Walk order"],
                        colLabels=cols0, loc="center", cellLoc="center")
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(9.5)
        tbl.scale(1, 1.7)
        for j in range(len(cols0)):
            tbl[0, j].set_facecolor("#d0d0d0")
            tbl[0, j].set_text_props(weight="bold")
        for i in range(1, 5):
            tbl[i, -1].set_text_props(weight="bold")
        # rowLabels 用第 -1 列（matplotlib 的 rowLabels 在 col -1）
        for i in range(1, 5):
            tbl[i, -1].set_facecolor("#d0d0d0")
            tbl[i, -1].set_text_props(weight="bold")
        ax0.set_title("Table 0 · 测试序列信息（对应论文 Table IV）",
                      fontsize=11, weight="bold", pad=14)
        ax0.axis("off")
        fig0.tight_layout()
        fig0.savefig(out / "论文格式表_序列信息.png", dpi=150, bbox_inches="tight")
        plt.close(fig0)
        print(f"  ✅ {out/'论文格式表_序列信息.png'}")

        fig, axes = plt.subplots(3, 1, figsize=(11.5, 9.2))
        render(axes[0], "Table A · 绝对轨迹误差 ATE（停车点定位 RMS, cm）",
               "对应论文 Table V   |   蓝=最好  红=次好  灰=第三", ate, 100.0, 2)
        render(axes[1], "Table B · 闭合误差（走一圈回到原点的偏差, cm）",
               "真值首尾同点 → 直接反映累积漂移", close, 100.0, 2)
        render(axes[2], "Table C · 尺度偏差绝对值（%）",
               "反映里程计尺度估计，与全局位姿无关", scale_dev, 100.0, 3, "%")
        fig.tight_layout(h_pad=2.2)
        fig.savefig(out / "论文格式表.png", dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  ✅ {out/'论文格式表.png'}")
    except Exception as e:  # noqa: BLE001
        print(f"  ⚠️ 渲染图片失败: {e}")

    print("\n" + md)
    return 0


if __name__ == "__main__":
    sys.exit(main())
