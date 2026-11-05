#!/usr/bin/env python3
"""
协议评测 —— 不依赖高精度绝对真值

    ⭐ 核心思路（学长的方案）：
       不测「每个点的绝对坐标」（尺子量不准），
       改测「相对量」和「重复性」。

    三类指标，全都不需要绝对真值：

      ① 稳定性   机器人在固定点停稳时，算法输出应该【原地不动】。
                 实际漂移的量 = 稳定性指标。完全不需要真值。

      ② 可重复性 同一路径录多个包，同一个停车点的坐标应该一致。
                 离散多少 = 可重复性。完全不需要真值。
                 （也可用同一包跑多次→测算法自身随机性）

      ③ 点间距   卷尺量的是【两点之间的距离】（1~3 米，相对误差小），
                 对比算法给的距离 → 尺度准不准、比例是否一致。
                 只需要量距离，不需要量坐标。

      ④ 闭合误差 走一圈回到原点时，终点应该 ≈ 起点。
                 偏差 = 累计漂移。完全不需要真值。

用法
────
    # 单条轨迹（算稳定性 + 点间距 + 闭合误差）
    python3 scripts/protocol_eval.py traj_glim.txt

    # 带上卷尺量到的距离（按顺序，和停车点一一对应）
    python3 scripts/protocol_eval.py traj_glim.txt --distances 3.02,2.98,3.03

    # 不同次采集对比 → ② 可重复性（⭐ 主力用法）
    python3 scripts/protocol_eval.py bag1_glim.txt bag2_glim.txt bag3_glim.txt

参数
────
    --speed      判定静止的速度阈值（米/秒），默认 0.05
    --min-dur    静止段最短时长（秒），默认 2.0
    --distances  卷尺量到的相邻停车点距离，逗号分隔（米）
    --outdir     图片输出目录
"""
import argparse
import sys
from pathlib import Path

import numpy as np

# 复用已有的读取逻辑
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from benchmark_eval import load_tum
except ImportError as e:
    sys.exit(f"❌ 无法导入 benchmark_eval: {e}\n   本脚本要放在 scripts/ 目录下")

# 中文字体等绘图环境
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from common import plot_setup  # noqa: F401
except Exception:
    pass
import matplotlib.pyplot as plt  # noqa: E402


# ══════════════════════════════════════════════════════════════#  找静止段（返回【索引区间】，才能分析段内稳定性）
# ════════════════════════════════════════════════════════════
def find_stop_ranges(t, xyz, speed_thr=0.05, min_dur=2.0):
    """
    找出“停住不动”的片段，返回 [(i0, i1), ...]（i1 取不到）。

    和 benchmark_eval.find_stops 的区别：
      那个只返回每个停车点的一个代表值（用于和真值对应），
      本函数返回【整个静止区间】的索引 —— 因为要分析“停稳期间漂了多少”。
    """
    if len(t) < 3:
        return []
    dt = np.diff(t)
    dp = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)   # 只看水平
    speed = dp / np.maximum(dt, 1e-9)
    still = speed < speed_thr

    out = []
    n = len(still)
    i = 0
    while i < n:
        if not still[i]:
            i += 1
            continue
        j = i
        while j < n and still[j]:
            j += 1
        # speed[k] 描述的是 点k → 点k+1，所以静止区间含 点 j
        i0, i1 = i, min(j + 1, len(t))
        if t[i1 - 1] - t[i0] >= min_dur:
            out.append((i0, i1))
        i = j
    return out


# ════════════════════════════════════════════════════════════#  指标 ① 稳定性：静止段内算法输出漂不漂
# ══════════════════════════════════════════════════════════════
def stability_of_stop(t, xyz, i0, i1):
    """分析 [i0, i1) 这段静止数据"""
    seg_t = t[i0:i1]
    seg_p = xyz[i0:i1, :2]          # 只看水平面 xy

    mean = seg_p.mean(axis=0)
    d = np.linalg.norm(seg_p - mean, axis=1)

    dur = float(seg_t[-1] - seg_t[0]) if len(seg_t) > 1 else 0.0
    return {
        "n": len(seg_p),
        "dur": dur,
        "mean": mean,
        "std": float(d.std()),               # 相对均值的离散程度
        "max_drift": float(d.max()),          # 最远漂了多远
        "xy_std": np.sqrt(seg_p.var(axis=0)), # 各轴标准差
        # 漂移率：漂移量 ÷ 停留时长（mm/s）—— 越小越好
        "drift_rate": float(d.max() / dur) if dur > 0 else float("nan"),
        "t_mid": float((seg_t[0] + seg_t[-1]) / 2),
    }


# ══════════════════════════════════════════════════════════════
#  指标 ③ 闭合误差：回到起点没
# ══════════════════════════════════════════════════════════════
def closure_error(stops):
    """第一个停车点 vs 最后一个停车点（如果路径是闭合的）"""
    if len(stops) < 2:
        return None
    a, b = stops[0]["mean"], stops[-1]["mean"]
    return float(np.linalg.norm(b - a))


# ══════════════════════════════════════════════════════════════
#  指标 ② 点间距：算法给的距离 vs 卷尺量到的距离
# ══════════════════════════════════════════════════════════════
def spacings(stops):
    """相邻停车点之间的距离（米）"""
    out = []
    for a, b in zip(stops[:-1], stops[1:]):
        out.append(float(np.linalg.norm(b["mean"] - a["mean"])))
    return out


def compare_spacings(alg, meas):
    """算法距离 vs 实测距离"""
    n = min(len(alg), len(meas))
    if n == 0:
        return None
    a = np.array(alg[:n])
    m = np.array(meas[:n])
    ratio = m / a                                    # 每段的比值
    # 线性拟合 m = k*a + b，k 就是尺度因子
    if n >= 2 and a.std() > 1e-6:
        k, b = np.polyfit(a, m, 1)
        pred = k * a + b
        ss_res = float(((m - pred) ** 2).sum())
        ss_tot = float(((m - m.mean()) ** 2).sum())
        r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    else:
        k, b, r2 = float("nan"), float("nan"), float("nan")
    return {
        "n": n, "alg": a, "meas": m, "ratio": ratio,
        "k": float(k), "b": float(b), "r2": float(r2),
        "err": m - a, "abs_err": np.abs(m - a),
    }


# ══════════════════════════════════════════════════════════════
#  打印
# ══════════════════════════════════════════════════════════════
def bar(ch="═", n=70):
    print(ch * n)


def report_one(name, t, xyz, stops, meas_dist):
    bar()
    print(f"  轨迹: {name}")
    print(f"  总帧数 {len(t)}   时间跨度 {t[-1]-t[0]:.1f} s   停车点 {len(stops)} 个")
    bar()

    if not stops:
        print("  ⚠️ 没找到静止段 —— 跑数据时要在每个点【停稳 5 秒】")
        return None

    # ── ① 稳定性 ─────────────────────────────────────────
    print()
    print("① 点的稳定性（停稳期间输出漂了多少）")
    print("─" * 70)
    print(f"  {'#':<3} {'停留':>6} {'标准差':>9} {'最大漂移':>10} {'漂移率':>10}")
    print(f"  {'':<3} {'(s)':>6} {'(mm)':>9} {'(mm)':>10} {'(mm/s)':>10}")
    for i, s in enumerate(stops, 1):
        print(f"  {i:<3} {s['dur']:>6.1f} {s['std']*1000:>9.1f} "
              f"{s['max_drift']*1000:>10.1f} {s['drift_rate']*1000:>10.2f}")

    drifts = np.array([s["max_drift"] for s in stops]) * 1000
    print("─" * 70)
    print(f"  最大漂移的中位数: {np.median(drifts):.1f} mm"
          f"   ← 越小越稳（理想是 0）")

    # ── ② 点间距 ─────────────────────────────────────────
    alg_d = spacings(stops)
    print()
    print("③ 点间距一致性")
    print("─" * 70)
    for i, d in enumerate(alg_d, 1):
        extra = ""
        if meas_dist and i <= len(meas_dist):
            m = meas_dist[i - 1]
            extra = f"   实测 {m:.3f} m   比值 {m/d:.4f}   差 {(m-d)*1000:+.0f} mm"
        print(f"  点{i} → 点{i+1}:  算法 {d:.3f} m{extra}")

    cmp = None
    if meas_dist:
        cmp = compare_spacings(alg_d, meas_dist)
        if cmp:
            print("─" * 70)
            r = cmp["ratio"]
            print(f"  比值（实测/算法）: 均值 {r.mean():.4f}   波动 {r.std():.4f}"
                  f"   {'← 各段越接近说明尺度越一致' if len(r) > 1 else ''}")
            if not np.isnan(cmp["k"]):
                print(f"  线性拟合 实测 = {cmp['k']:.4f} × 算法"
                      f"   R² = {cmp['r2']:.5f}")
                if cmp["r2"] >= 0.9:
                    print(f"  ⭐ 尺度因子 {cmp['k']:.4f}"
                          f" → 偏差 {(cmp['k']-1)*100:+.2f}%")
                else:
                    print(f"  ⚠️ R² 只有 {cmp['r2']:.3f} —— 拟合不可靠，别看那个斜率")
                    print( "     原因：各段距离彼此太接近，没有区分度")
                    print( "     建议：让停车点之间的距离拉开（例如 1 m / 3 m / 5 m）")
                    print( "     → 这种情况下请只看【比值的波动】：")
                    print(f"       比值均值 {r.mean():.4f}（偏离 1 多少 = 综合尺度误差），"
                          f"波动 {r.std():.4f}（越小 = 越一致）")
    else:
        print()
        print("  （没给 --distances，跳过实测对比）")

    # ── ③ 闭合误差 ───────────────────────────────────────
    ce = closure_error(stops)
    print()
    print("④ 闭合误差（第一个点 vs 最后一个点）")
    print("─" * 70)
    if ce is None:
        print("  停车点不足 2 个，算不了")
    else:
        print(f"  首尾距离: {ce*1000:.1f} mm")
        print("  （前提：路径是「回到原点」的闭合回路，否则这个数没有意义）")

    return {"stops": stops, "spacings": alg_d, "closure": ce, "cmp": cmp}


def report_repeatability(results):
    """⭐ 指标 ② 可重复性：同一个点，在不同次采集/运行里一致吗

    传多个包的轨迹  → 测【采集间一致性】（真实可重复性）
    传同一包跑多次  → 测【算法自身随机性】
    """
    names = list(results.keys())
    ns = [len(results[n]["stops"]) for n in names]
    if len(set(ns)) > 1:
        print(f"  ⚠️ 各次找到的停车点数不一致: {dict(zip(names, ns))}")
    k = min(ns)
    if k == 0:
        bar(); print("  ⚠️ 没有可用的停车点，算不了可重复性"); bar(); return

    bar()
    print("  ② 可重复性 —— 同一个点，在不同次采集/运行里一致吗")
    print("  （传多个包的轨迹 → 测采集间一致性）")
    print("  （传同一包跑多次 → 测算法自身随机性）")
    bar()
    print()
    print(f"  {'点':<4} {'最大离散':>10}     各次相对均值的偏离 (mm)")
    print("─" * 70)
    allspread = []
    for i in range(k):
        pts = np.array([results[n]["stops"][i]["mean"] for n in names])
        spread = np.linalg.norm(pts - pts.mean(axis=0), axis=1)
        allspread.append(spread.max())
        print(f"  {i+1:<4} {spread.max()*1000:>8.1f} mm     "
              f"{'  '.join(f'{v*1000:+7.1f}' for v in spread)}")
    print("─" * 70)
    print(f"  最大离散的中位数: {np.median(allspread)*1000:.1f} mm"
          f"   ← 越接近 0 = 可重复性越好")
    print(f"  运行次数: {len(names)}")

    # 顺便把①稳定性跨运行汇总，方便直接抄进对比表
    print()
    print("  ① 稳定性跨运行汇总（每条轨迹的漂移中位数，mm）")
    print("─" * 70)
    for n in names:
        st = results[n]["stops"]
        if not st:
            continue
        d = np.array([s["max_drift"] for s in st]) * 1000
        print(f"  {n:<26} {np.median(d):>8.1f}")


def plot_result(results, outdir):
    names = list(results.keys())
    n = len(names)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle("协议评测：稳定性 / 点间距 / 闭合误差",
                 fontsize=13, fontweight="bold")

    # 左：停车点位置
    ax = axes[0]
    for k, name in enumerate(names):
        st = results[name]["stops"]
        if not st:
            continue
        p = np.array([s["mean"] for s in st])
        p = p - p[0]                                  # 相对第一个点
        ax.plot(p[:, 0], p[:, 1], "o-", label=name, markersize=7, alpha=0.85)
        for i, (x, y) in enumerate(p, 1):
            ax.annotate(str(i), (x, y), textcoords="offset points",
                        xytext=(6, 6), fontsize=9)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("各次运行的停车点位置（相对第 1 点）")
    ax.axis("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # 右：每个点的稳定性
    ax = axes[1]
    for name in names:
        st = results[name]["stops"]
        if not st:
            continue
        ax.plot(range(1, len(st) + 1), [s["max_drift"] * 1000 for s in st],
                "o-", label=name, markersize=7, alpha=0.85)
    ax.set_xlabel("停车点序号"); ax.set_ylabel("最大漂移 (mm)")
    ax.set_title("① 点的稳定性（越小越稳，理想 0）")
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    plt.tight_layout()
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    p = out / "protocol_report.png"
    plt.savefig(p, dpi=130, bbox_inches="tight")
    print()
    print(f"✅ 图已保存: {p}")


def main():
    ap = argparse.ArgumentParser(
        description="协议评测：用可重复性和相对量代替绝对真值",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("trajs", nargs="+", help="一条或多条 TUM 轨迹文件")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="判定静止的速度阈值 m/s（默认 0.05）")
    ap.add_argument("--min-dur", type=float, default=2.0,
                    help="静止段最短时长 s（默认 2.0）")
    ap.add_argument("--distances", type=str, default="",
                    help="卷尺量到的相邻停车点距离，逗号分隔，如 3.02,2.98,3.03")
    ap.add_argument("--outdir", type=str, default="protocol_eval")
    args = ap.parse_args()

    meas = None
    if args.distances.strip():
        try:
            meas = [float(x) for x in args.distances.replace("，", ",").split(",") if x.strip()]
        except ValueError:
            sys.exit("❌ --distances 格式不对，应该是 3.02,2.98,3.03")

    print()
    bar()
    print("  协议评测 —— 不依赖绝对真值")
    print("  （稳定性 / 点间距一致性 / 闭合误差）")
    bar()

    results = {}
    for path in args.trajs:
        p = Path(path)
        if not p.exists():
            print(f"  ⚠️ 跳过不存在的文件: {path}")
            continue
        t, xyz, _quat = load_tum(str(p))
        segs = find_stop_ranges(t, xyz, args.speed, args.min_dur)
        stops = [stability_of_stop(t, xyz, i0, i1) for i0, i1 in segs]
        name = p.stem
        results[name] = report_one(name, t, xyz, stops, meas)

    if len(results) > 1:
        report_repeatability(results)
    elif results:
        print()
        print("  💡 想看【② 可重复性】，把多次采集（多个包）的轨迹一起传进来：")
        print("       python3 protocol_eval.py bag1_glim.txt bag2_glim.txt bag3_glim.txt")

    if results:
        try:
            plot_result(results, args.outdir)
        except Exception as e:
            print(f"  ⚠️ 画图失败: {e}")

    print()
    bar()
    print("  怎么看这些数")
    bar()
    print("  ① 点的稳定性   —— 停稳时漂得越少越好。理想接近 0 mm")
    print("                    （单次运行内；看算法本身抖不抖）")
    print("  ② 可重复性     —— 同一个点，在不同次采集/运行里的离散程度")
    print("                    （越接近 0 越好；要 3 次以上才有意义）")
    print("                    ↳ 传多个包的轨迹 → 测采集一致性")
    print("                    ↳ 传同一包跑多次 → 测算法自身随机性")
    print("  ③ 点间距比值   —— 各段比值应该【差不多】（说明尺度一致）；")
    print("                    比值均值偏离 1 多少 = 尺度误差")
    print("  ④ 闭合误差     —— 走一圈回到原点的偏差 = 累计漂移")
    print()
    print("  这三项都不需要高精度绝对真值 ✅")
    print()


if __name__ == "__main__":
    main()
