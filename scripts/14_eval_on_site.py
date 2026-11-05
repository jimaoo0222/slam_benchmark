#!/usr/bin/env python3
"""用场地真值评测 SLAM 轨迹（真正的绝对精度）。

为什么需要它
────────────
`protocol_eval.py` 只能看「重复性」和「相对量」——它不知道真实距离。
但如果你量过场地的停车点位置（卷尺 / 图纸），就能算出**绝对误差**：

    · 每个停车点偏了多少 cm
    · 算法给的点间距 vs 卷尺量到的（= ③ 尺度偏差）
    · 走一圈回到原点的闭合误差

场地真值从哪来
──────────────
用 `--site` 指定一个 JSON，格式见 `config/site_mid360_demo.json`：

    {
      "points": [[0, 0], [0, 1.80], [-1.80, 0], [-4.05, 0], [-6.30, 0]],
      "order":  [0, 1, 2, 3, 4, 0]
    }

`points` 是停车点坐标（米），`order` 是行走顺序（首尾相同 = 闭合回路）。

用法
────
    # 单车评测
    python3 scripts/14_eval_on_site.py traj.txt --site config/site_mid360_demo.json

    # 多算法对比
    python3 scripts/14_eval_on_site.py a.txt b.txt --site config/site_mid360_demo.json

    # 停车点没停稳/没停 5 秒时，放宽判定
    python3 scripts/14_eval_on_site.py traj.txt --site ... --speed 0.12 --min-dur 1.5

方法
────
1. 从轨迹里找「静止段」（速度低于阈值且持续够久）→ 这些就是停车点
2. 用 Umeyama 求最优刚体变换，把 SLAM 坐标系对齐到场地坐标系
3. 对齐后算每个停车点的位置误差
4. 算相邻停车点的间距误差（尺度）
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def load_traj(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        p = s.split()
        if len(p) >= 4:
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
            except ValueError:
                continue
    if not rows:
        sys.exit(f"❌ {path} 里没读到轨迹（需要 TUM 8 列格式）")
    a = np.array(rows, dtype=np.float64)
    return a[np.argsort(a[:, 0])]


def find_stops(a: np.ndarray, speed: float, min_dur: float,
               merge_same_place: bool = True) -> list[dict]:
    """找静止段。相邻段间隔 < 3 s 的合并（同一次停顿被噪声打断）。

    merge_same_place：位置相距 < 15 cm 也当作同一次停顿合并。
        闭环路径（如 0→1→2→3→4→0）的**起点与终点是同一个物理位置**，
        开着这个开关会把首尾两段误合并成一段，导致停车点数少 1。
        调用方检测到这种情况时会用 merge_same_place=False 重试。
    """
    t, p = a[:, 0], a[:, 1:4]
    dt = np.diff(t)
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(dt, 1e-9)
    still = v < speed

    raw: list[tuple[int, int]] = []
    i, n = 0, len(still)
    while i < n:
        if still[i]:
            j = i
            while j + 1 < n and still[j + 1]:
                j += 1
            if t[j + 1] - t[i] >= min_dur:
                raw.append((i, j + 1))
            i = j + 1
        else:
            i += 1

    merged: list[tuple[int, int]] = []
    for a0, b0 in raw:
        if merged:
            pa = p[merged[-1][0]:merged[-1][1] + 1].mean(0)
            pb = p[a0:b0 + 1].mean(0)
            # 时间间隔 < 3 s，或位置很近（< 15 cm）→ 认为是同一次停顿
            same_place = merge_same_place and np.linalg.norm(pa - pb) < 0.15
            if t[a0] - t[merged[-1][1]] < 3.0 or same_place:
                merged[-1] = (merged[-1][0], b0)
                continue
        merged.append((a0, b0))

    return [
        dict(t0=float(t[a0]), dur=float(t[b0] - t[a0]), mean=p[a0:b0 + 1].mean(0))
        for a0, b0 in merged
    ]


def sample_at_windows(a: np.ndarray, windows: list[dict]) -> list[dict]:
    """按**给定的时间窗口**取位置，而不是自己找静止段。

    为什么需要：不同算法输出噪声不同（静止时单步位移可从 0.04cm 到 0.7cm），
    用同一个**绝对速度阈值**判定「是否在停车」会对噪声大的算法不公平。
    更严谨的做法是：停车时刻是**数据的属性**（人什么时候站住了），
    与算法无关 —— 所以用一条干净轨迹（GLIM）定出停车窗口，所有算法共用。

    参数 windows: [{'t0': 绝对起始时间, 'dur': 时长}, ...]
    返回同 find_stops 的结构，取窗口内位姿均值，保证口径一致。
    """
    t, p = a[:, 0], a[:, 1:4]
    out = []
    for w in windows:
        t0, t1 = w["t0"], w["t0"] + w["dur"]
        m = (t >= t0) & (t <= t1)
        if not m.any():
            # 窗口内没有样本（该算法起止时间不同）→ 取最近的一帧
            i = int(np.argmin(np.abs(t - (t0 + t1) / 2)))
            pos = p[i]
            t_use = float(t[i])
        else:
            pos = p[m].mean(0)
            t_use = float(t[m][0])
        out.append(dict(t0=t_use, dur=float(w["dur"]), mean=np.asarray(pos)))
    return out


def umeyama_2d(src: np.ndarray, dst: np.ndarray):
    """求最优 (R, t, s) 使 s*R@src + t ≈ dst（允许尺度，便于看尺度偏差）。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, dv, vt = np.linalg.svd(cov)
    d = np.eye(2)
    d[1, 1] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    scale = float(np.trace(np.diag(dv) @ d) / max((s0 ** 2).sum() / len(src), 1e-12))
    t = mu_d - scale * (r @ mu_s)
    return r, t, scale


def umeyama_rigid(src: np.ndarray, dst: np.ndarray):
    """刚性版（不允许缩放），用来看纯粹的对齐误差。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.eye(2)
    d[1, 1] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def apply(r, t, pts, scale=1.0):
    return scale * (r @ pts.T).T + t


def evaluate(traj_path: Path, site: dict, speed: float, min_dur: float,
             ref_windows: list[dict] | None = None) -> dict | None:
    a = load_traj(traj_path)
    if ref_windows is not None:
        stops = sample_at_windows(a, ref_windows)
    else:
        stops = find_stops(a, speed, min_dur)
    gt_pts = np.array(site["points"], dtype=np.float64)
    order = site["order"]

    # 停车点数对不上时自动补救：
    #   · 闭环路径（order 首尾同点）首尾停车位位置相同，会被「按位置合并」误并成一段
    #   · 有的停车只停了 1~2 秒，被 min_dur 卡掉
    # → 依次尝试「关掉位置合并」「放宽 min_dur」
    if ref_windows is None and len(stops) != len(order):
        for msp in (False, True):
            for md in (max(min_dur * 0.75, 0.6), 1.5, 1.0, 0.6):
                if md >= min_dur and msp is False:
                    continue
                retry = find_stops(a, speed, md, merge_same_place=msp)
                if len(retry) == len(order):
                    stops = retry
                    print(f"  ℹ️ 自动重试匹配成功：min_dur {min_dur}→{md}s，"
                          f"按位置合并={'关' if not msp else '开'}")
                    break
            if len(stops) == len(order):
                break

    if len(stops) != len(order):
        print(f"  ⚠️ 检测到 {len(stops)} 个停车点，真值有 {len(order)} 个 —— 无法一一对应")
        print(f"     检测到的位置：")
        for k, s in enumerate(stops):
            m = s["mean"]
            print(f"       #{k}  t={s['t0']-a[0,0]:6.1f}s  ({m[0]:+.2f}, {m[1]:+.2f}, {m[2]:+.2f})")
        return None

    # ⚠️ order 是【场地点编号】，不是【停车点索引】。
    #    第 k 个停车点 stops[k] 对应 场地 points[order[k]]。
    #    （早先这里写成 stops[order[k]]，当 order 恰好是 0,1,2,3,4,0 时碰巧正确，
    #      换成 0,2,3,4,1,0 就全乱了。）
    slam = np.stack([stops[k]["mean"][:2] for k in range(len(order))])
    truth = gt_pts[order]

    r, t, scale = umeyama_2d(slam, truth)
    aligned = apply(r, t, slam, scale)
    err = np.linalg.norm(aligned - truth, axis=1)

    r2, t2 = umeyama_rigid(slam, truth)
    aligned2 = apply(r2, t2, slam)
    err_rigid = np.linalg.norm(aligned2 - truth, axis=1)

    seg_names, seg_slam, seg_gt = [], [], []
    for i in range(len(order) - 1):
        seg_names.append(f"{order[i]}→{order[i+1]}")
        seg_slam.append(float(np.linalg.norm(aligned[i + 1] - aligned[i])))
        seg_gt.append(float(np.linalg.norm(truth[i + 1] - truth[i])))

    seg_slam = np.array(seg_slam)
    seg_gt = np.array(seg_gt)

    return dict(
        path=traj_path, poses=len(a), dur=float(a[-1, 0] - a[0, 0]),
        n_stops=len(stops), stops=stops,
        align_scale=scale, align_angle_deg=float(np.degrees(np.arctan2(r[1, 0], r[0, 0]))),
        err=err, err_rigid=err_rigid, aligned=aligned, truth=truth,
        seg_names=seg_names, seg_slam=seg_slam, seg_gt=seg_gt,
        closure=float(np.linalg.norm(aligned[-1] - aligned[0])),
        path_len_slam=float(np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1).sum()),
        path_len_gt=float(seg_gt.sum()),
    )


def report(res: dict) -> None:
    d = res
    print(f"\n{'='*72}")
    print(f" {d['path'].parent.name} / {d['path'].name}")
    print(f"{'='*72}")
    print(f"  轨迹     {d['poses']} 位姿 / {d['dur']:.0f}s   停车点 {d['n_stops']} 个 ✅")
    print(f"  对齐     旋转 {d['align_angle_deg']:+.1f}°   尺度 {d['align_scale']:.5f}"
          f"  (偏差 {(d['align_scale']-1)*100:+.3f}%)")
    print()
    print(f"  ── 每个停车点的定位误差 ──")
    for k, s in enumerate(d["stops"]):
        al, tr = d["aligned"][k], d["truth"][k]
        print(f"    点{s['t0']-d['stops'][0]['t0']:6.1f}s  "
              f"算法 ({al[0]:+6.2f},{al[1]:+6.2f})  "
              f"真值 ({tr[0]:+6.2f},{tr[1]:+6.2f})  "
              f"误差 {d['err'][k]*100:6.1f} cm")
    print(f"    平均 {d['err'].mean()*100:.1f} cm   "
          f"最大 {d['err'].max()*100:.1f} cm   "
          f"RMS {np.sqrt((d['err']**2).mean())*100:.1f} cm")
    print()
    print(f"  ── 相邻停车点间距（③ 尺度）──")
    print(f"    {'段':<8}{'算法':>9}{'真值':>9}{'差':>9}{'误差%':>9}")
    for nm, ss, gg in zip(d["seg_names"], d["seg_slam"], d["seg_gt"]):
        print(f"    {nm:<8}{ss*100:8.1f}cm{gg*100:8.1f}cm{(ss-gg)*100:+8.1f}cm{(ss/gg-1)*100:+8.2f}%")
    rel = (d["seg_slam"] / d["seg_gt"] - 1) * 100
    print(f"    尺度偏差 平均 {rel.mean():+.2f}%   最大 |{np.abs(rel).max():.2f}%|")
    print()
    print(f"  ── 闭合误差（走一圈回到起点）──")
    print(f"    对齐后首尾距离  {d['closure']*100:.1f} cm")
    print(f"  路径长  算法 {d['path_len_slam']:.2f} m   真值 {d['path_len_gt']:.2f} m"
          f"   ({(d['path_len_slam']/d['path_len_gt']-1)*100:+.1f}%)")


def main() -> int:
    ap = argparse.ArgumentParser(description="用场地真值评测 SLAM 轨迹")
    ap.add_argument("trajs", nargs="+", help="TUM 轨迹文件")
    ap.add_argument("--site", required=True, help="场地真值 JSON")
    ap.add_argument("--speed", type=float, default=0.08, help="判定静止的速度阈值 m/s（默认 0.08）")
    ap.add_argument("--min-dur", type=float, default=2.0, help="静止段最短时长 s（默认 2.0）")
    ap.add_argument("--json-out", default="", help="把结果另存为 JSON")
    ap.add_argument("--stop-from", default="",
                    help="用这条参考轨迹定停车窗口，所有待评测轨迹共用（推荐：取最干净的那条）")
    args = ap.parse_args()

    site = json.loads(Path(args.site).read_text())

    ref_windows = None
    if args.stop_from:
        rp = Path(args.stop_from).expanduser()
        if not rp.is_file():
            print(f"❌ 参考轨迹不存在: {rp}")
            return 1
        ra = load_traj(rp)
        want = len(site["order"])
        # 找停车窗口。默认阈值未必够 —— 有的停车只站了 1~2 秒，会被 min_dur 卡掉。
        # 所以和 evaluate() 里一样依次放宽，并且**用对齐尺度自校验**：
        # 窗口一旦配错（把同一次停顿拆成几段），尺度会明显偏离 1。
        attempts = [(args.min_dur, True), (args.min_dur, False)]
        for md_try in (max(args.min_dur * 0.75, 0.6), 1.5, 1.0, 0.6, 2.5, 3.0):
            attempts += [(md_try, True), (md_try, False)]
        seen = set()
        for md_try, msp in attempts:
            if (md_try, msp) in seen:
                continue
            seen.add((md_try, msp))
            rw = find_stops(ra, args.speed, md_try, merge_same_place=msp)
            if len(rw) != want:
                continue
            chk = evaluate(rp, site, args.speed, md_try, rw)
            if chk is None or abs(chk["align_scale"] - 1.0) > 0.03:
                continue
            ref_windows = rw
            relaxed = (md_try != args.min_dur) or (not msp)
            if relaxed:
                print(f"   （参考轨迹放宽判定：min_dur={md_try}s，"
                      f"按位置合并={'开' if msp else '关'}，"
                      f"自校验尺度 {chk['align_scale']:.4f}）")
            break
        if ref_windows is None:
            print(f"⚠️ 参考轨迹找不到 {want} 个可信的停车点，退回自动检测模式")
        else:
            rw = ref_windows
            print(f"📌 停车窗口取自参考轨迹: {rp.name}"
                  f"（{len(rw)} 个："
                  + ", ".join(f"{w['t0']-rw[0]['t0']:.0f}s" for w in rw) + "）")
            print("   所有算法共用同一组时刻 —— 消除速度阈值对不同噪声水平算法的不公平\n")

    results = []
    for t in args.trajs:
        p = Path(t).expanduser()
        if not p.is_file():
            print(f"❌ 文件不存在: {p}")
            continue
        r = evaluate(p, site, args.speed, args.min_dur, ref_windows)
        if r:
            report(r)
            results.append(r)

    if not results:
        return 1

    if len(results) > 1:
        print(f"\n{'='*72}")
        print(" 汇总")
        print(f"{'='*72}")
        print(f"  {'轨迹':<24}{'点位RMS':>10}{'点位最大':>10}{'尺度偏差':>10}{'闭合':>10}")
        for r in results:
            name = f"{r['path'].parent.name}"
            print(f"  {name:<24}"
                  f"{np.sqrt((r['err']**2).mean())*100:9.1f}cm"
                  f"{r['err'].max()*100:9.1f}cm"
                  f"{((r['seg_slam']/r['seg_gt']-1).mean())*100:+9.2f}%"
                  f"{r['closure']*100:9.1f}cm")

    if args.json_out:
        out = [dict(name=str(r["path"]), poses=r["poses"], dur=r["dur"],
                    n_stops=r["n_stops"], scale=r["align_scale"],
                    err_mean=float(r["err"].mean()), err_max=float(r["err"].max()),
                    err_rms=float(np.sqrt((r["err"] ** 2).mean())),
                    seg_err_pct=[float(x) for x in (r["seg_slam"] / r["seg_gt"] - 1) * 100],
                    closure=r["closure"])
               for r in results]
        Path(args.json_out).write_text(json.dumps(out, indent=2, ensure_ascii=False))
        print(f"\n✅ 结果已存 {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
