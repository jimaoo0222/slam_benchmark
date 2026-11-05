#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  外参修正后，把 8 个 bag × 4 个算法 = 32 次全部重跑一遍
#
#  为什么需要重跑
#  ──────────────
#  之前 32 次运行的 IMU↔LiDAR 外参是错的：
#    · C01 的 4 个包借用了 MID-360s 的外参
#    · LIO-SAM 干脆填 [0,0,0]
#    · Point-LIO 的符号还是反的
#  现在四份配置都已按官方值改正（见 docs/01 第 5 节）。重跑才能让
#  8 个包的数字是同一个口径，横向比较才有意义。
#
#  用法
#  ────
#     bash scripts/29_rerun_all.sh                    # 全部 32 次
#     bash scripts/29_rerun_all.sh --only 03_01       # 只跑一个包
#     bash scripts/29_rerun_all.sh --only c01         # 只跑 C01 的 4 个包
#     bash scripts/29_rerun_all.sh --only mid360      # 只跑 MID-360s 的 4 个包
#     bash scripts/29_rerun_all.sh --algo glim        # 只跑某个算法
#     bash scripts/29_rerun_all.sh --dry              # 只打印计划
#
#  断点续跑
#  ────────
#  每个组合跑完会在输出目录留一个 `.done` 标记。再跑一次会跳过它。
#  想强制重跑某个组合，删掉对应的 `.done` 就行。
#
#  回放倍率为什么不一样
#  ────────────────────
#  C01 每帧 21.7k 点，比 MID-360s 的 20k 多 8%，但这台机器上
#  Point-LIO / LIO-SAM 跑不到实时，1.0 倍速会丢帧、轨迹断在半路。
#  所以 C01 上 Point-LIO 用 0.35、LIO-SAM 用 0.7（见 docs/02 第 4 节）。
#  MID-360s 的 4 个包三个算法都能 1.0。
#
#  ⚠️ 必须一次只跑一个：四个算法都订阅 /livox/lidar，
#     同时回放会让 16_pc2_to_custom.py 把别人的数据一起收进去。
#     本脚本串行执行，不会踩这个坑。
# ══════════════════════════════════════════════════════════════
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAMP="$(date +%Y-%m-%d)"
OUT_ROOT="$ROOT/results/${STAMP}_外参修正"

ONLY=""
ALGO_ONLY=""
DOMAIN=""
DRY=0

while [ $# -gt 0 ]; do
    case "$1" in
        --only)   ONLY="$2";      shift 2 ;;
        --algo)   ALGO_ONLY="$2"; shift 2 ;;
        --out)    OUT_ROOT="$2";  shift 2 ;;
        --domain) DOMAIN="$2";    shift 2 ;;
        --dry)    DRY=1;          shift 1 ;;
        -h|--help) sed -n '2,45p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 并行跑多个组合时必须给每路一个不同的 ROS_DOMAIN_ID：
# 四个算法都订阅 /livox/lidar，同 domain 会互相收到别人的点云。
if [ -n "$DOMAIN" ]; then
    export ROS_DOMAIN_ID="$DOMAIN"
    echo "ROS_DOMAIN_ID = $DOMAIN（话题与其他并行任务隔离）"
fi

G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; B='\033[0;34m'; N='\033[0m'
ok(){ echo -e "  ${G}OK${N} $*"; }
warn(){ echo -e "  ${Y}!!${N} $*"; }
err(){ echo -e "  ${R}ERR${N} $*"; }
step(){ echo; echo -e "${B}======== $* ========${N}"; }

# ── 包 → 雷达 / 回放倍率 的对应表 ─────────────────────────
# 格式：包名:雷达
BAGS="01_1:mid360 01_2:mid360 02_1:mid360 02_2:mid360
      03_01:c01 03_02:c01 04_01:c01 04_02:c01"

# 每个算法的回放倍率：<算法>:<mid360 倍率>:<c01 倍率>
RATES="glim:-:- fastlio:1.0:1.0 pointlio:1.0:0.35 liosam:1.0:0.7"

# 个别包需要单独降速（1.0 时发散或丢帧）：<包>:<算法>:<倍率>
OVERRIDES="03_02:fastlio:0.7"

rate_for() {   # rate_for <algo> <radar> <bag>
    local algo="$1" radar="$2" bag="$3" r
    for r in $OVERRIDES; do
        [ "${r%%:*}" = "$bag" ] || continue
        local rest="${r#*:}"
        [ "${rest%%:*}" = "$algo" ] && { echo "${rest#*:}"; return; }
    done
    for r in $RATES; do
        [ "${r%%:*}" = "$algo" ] || continue
        local rest="${r#*:}"
        local m360="${rest%%:*}" c01="${rest#*:}"
        [ "$radar" = "c01" ] && echo "$c01" || echo "$m360"
        return
    done
    echo "1.0"
}

run_one() {   # run_one <bag> <radar> <algo>
    local bag="$1" radar="$2" algo="$3"
    local out="$OUT_ROOT/${bag}_${algo}"
    local rate; rate="$(rate_for "$algo" "$radar" "$bag")"

    if [ -f "$out/.done" ]; then
        warn "跳过 ${bag} × ${algo}（已有 .done）"
        return 0
    fi

    # 上一轮留下的半成品先挪开，避免脚本误判「已跑完」
    [ -d "$out" ] && mv "$out" "${out}.bak_$(date +%H%M%S)"

    local t0; t0=$(date +%s)
    mkdir -p "$out"

    local rc=0
    case "$algo" in
        glim)
            bash "$ROOT/scripts/12_run_glim_on_bag.sh" \
                 "$ROOT/rosbag/$bag" "$out" > "$out/run.log" 2>&1 || rc=$?
            ;;
        fastlio)
            bash "$ROOT/scripts/05_run_fastlio_on_bag.sh" \
                 "$ROOT/rosbag/$bag" "$out" --rate "$rate" >> "$out/run.log" 2>&1 || rc=$?
            ;;
        pointlio)
            bash "$ROOT/scripts/23_run_pointlio_on_bag.sh" \
                 "$ROOT/rosbag/$bag" "$out" --rate "$rate" >> "$out/run.log" 2>&1 || rc=$?
            ;;
        liosam)
            bash "$ROOT/scripts/17_run_liosam_on_bag.sh" \
                 "$ROOT/rosbag/$bag" "$out" --rate "$rate" >> "$out/run.log" 2>&1 || rc=$?
            ;;
    esac

    # 有没有拿到轨迹，比退出码更可信（脚本内部会兜住信号）
    local traj=""
    for c in "$out/traj.txt" "$out/traj_glim.txt"; do
        [ -s "$c" ] && traj="$c" && break
    done

    local dt=$(( $(date +%s) - t0 ))
    if [ -n "$traj" ]; then
        local n; n=$(grep -cve '^\s*$' "$traj")
        ok "${bag} × ${algo}  ${n} 位姿  ${dt}s  (倍率 ${rate})"
        echo "$bag $radar $algo $rate $n $dt" >> "$SUMMARY"
        touch "$out/.done"
        return 0
    fi

    err "${bag} × ${algo} 没拿到轨迹（${dt}s，退出码 $rc）—— 日志尾部："
    tail -12 "$out/run.log" 2>/dev/null | sed 's/^/      /'
    echo "$bag $radar $algo $rate FAIL $dt" >> "$SUMMARY"
    return 1
}

# ── 主流程 ─────────────────────────────────────────────────
mkdir -p "$OUT_ROOT"
SUMMARY="$OUT_ROOT/_summary${DOMAIN:+_d$DOMAIN}.txt"
: > "$SUMMARY"

echo "======================================================================"
echo " 外参修正后全量重跑"
echo "   输出  : $OUT_ROOT"
echo "   范围  : ${ONLY:-全部 8 个包} / ${ALGO_ONLY:-全部 4 个算法}"
echo "   开始  : $(date '+%F %T')"
echo "======================================================================"

FAIL=0
N=0
for entry in $BAGS; do
    bag="${entry%%:*}"
    radar="${entry##*:}"

    case "$ONLY" in
        "")      ;;
        mid360)  [ "$radar" = "mid360" ] || continue ;;
        c01)     [ "$radar" = "c01" ]    || continue ;;
        *)       [ "$ONLY" = "$bag" ]    || continue ;;
    esac

    for algo in glim fastlio pointlio liosam; do
        [ -n "$ALGO_ONLY" ] && [ "$ALGO_ONLY" != "$algo" ] && continue
        N=$((N+1))
        step "[$N] $bag ($radar) × $algo"
        if [ "$DRY" = "1" ]; then
            echo "  plan: rate=$(rate_for "$algo" "$radar" "$bag")  out=$OUT_ROOT/${bag}_${algo}"
            continue
        fi
        run_one "$bag" "$radar" "$algo" || FAIL=$((FAIL+1))
    done
done

echo
echo "======================================================================"
if [ "$DRY" = "1" ]; then
    echo " 干跑结束，共 $N 个组合"
else
    echo " 重跑结束  $(date '+%F %T')   失败 $FAIL / $N"
    echo " 汇总: $SUMMARY"
fi
echo "======================================================================"
exit $([ "$FAIL" -gt 0 ] && echo 1 || echo 0)
