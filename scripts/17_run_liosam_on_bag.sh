#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  对一个 bag 跑 LIO-SAM（MID-360 ROS2 版），产出轨迹 + 日志
#
#  用法：
#     bash scripts/17_run_liosam_on_bag.sh <bag目录> <输出目录> [选项]
#
#  例：
#     bash scripts/17_run_liosam_on_bag.sh \
#          rosbag/01_1 results/2026-10-22_LIO-SAM/01_1
#
#  选项：
#     --rate 1.0        回放倍率（默认 1.0）
#     --params <file>   LIO-SAM 参数文件（默认 config/liosam_mid360.yaml）
#     --topic <t>       轨迹话题（默认 /lio_sam/mapping/odometry）
#
#  它会做：
#     1. 若 bag 的点云是 PointCloud2 → 自动起 16_pc2_to_custom.py 转成 CustomMsg
#        （LIO-SAM 只订阅 livox_ros_driver2/msg/CustomMsg）
#     2. 起 LIO-SAM 四个节点（无 RViz）
#     3. 起 traj_recorder 录轨迹 → traj.txt（TUM 格式）
#     4. 回放 bag
#     5. 优雅退出并汇总
#
#  ⚠️ 三个必须知道的点：
#     · **`~/.local/lib` 必须排在 LD_LIBRARY_PATH 最前**，否则会命中
#       `/usr/local/lib/libgtsam.so.4`（旧版，用 conda 的 boost 1.82 编译），
#       报 `libboost_serialization.so.1.82.0: not found` 而启动失败
#     · LIO-SAM 需要 IMU 200Hz 且时间戳连续；bag 用 --clock 反而会乱，
#       本脚本**不加 --clock**（LIO-SAM 用 header.stamp，不依赖 /clock）
#     · 算法不能 kill -9，会丢最后的位姿
# ══════════════════════════════════════════════════════════════
set -uo pipefail

BAG="${1:-}"
OUT="${2:-}"
shift 2 2>/dev/null || true

RATE="1.0"
PARAMS=""            # 留空 = 按 bag 名字自动选（见下方 lib_radar.sh）
TOPIC="/lio_sam/mapping/odometry"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

while [ $# -gt 0 ]; do
    case "$1" in
        --rate)   RATE="$2";   shift 2 ;;
        --params) PARAMS="$2"; shift 2 ;;
        --topic)  TOPIC="$2";  shift 2 ;;
        -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 没显式指定参数文件时按 bag 前缀选：01_/02_ → MID-360s，03_/04_ → C01
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/lib_radar.sh"
RADAR="$(radar_of_bag "$BAG")"
if [ -z "$PARAMS" ]; then
    case "$RADAR" in
        c01)    PARAMS="$ROOT/config/liosam_c01.yaml" ;;
        *)      PARAMS="$ROOT/config/liosam_mid360.yaml" ;;
    esac
fi

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

if [ -z "$BAG" ] || [ -z "$OUT" ]; then
    err "缺参数"; sed -n '2,32p' "$0"; exit 1
fi
[ -d "$BAG" ] || { err "bag 目录不存在: $BAG"; exit 1; }
[ -f "$PARAMS" ] || { err "参数文件不存在: $PARAMS"; exit 1; }
[ -f "$HOME/liosam_ws/install/setup.bash" ] || { err "LIO-SAM 未编译，先跑 scripts/18_get_liosam.sh"; exit 1; }

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 记录本次启动前已有的 lio_sam 进程，收尾时只杀新增的（支持多 bag 并行跑）
LIO_PAT="lio_sam_(imageProjection|featureExtraction|mapOptimization|imuPreintegration)"
BEFORE_PIDS=$(ps -eo pid,args --no-headers 2>/dev/null \
    | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}' | sort | tr '\n' ' ')

echo "======================================================================"
echo " LIO-SAM × bag"
echo "   bag    : $BAG"
echo "   输出   : $OUT"
echo "   参数   : $PARAMS（倍率 $RATE）"
echo "======================================================================"

# conda 的 python 会顶掉 ROS 的 python3.10
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for f in "$HOME/livox_ws/install/setup.bash" "$HOME/liosam_ws/install/setup.bash"; do
    [ -f "$f" ] && source "$f"
done
set -u
# ⚠️ 必须放在 source 之后（ROS 会往后追加）
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

rm -f "$OUT/traj.txt"

# ── 0. 判断 bag 点云类型 ──────────────────────────────────────
echo
echo "── 0. 检查 bag 的点云类型 ──"
BAG_TYPE=$(ros2 bag info "$BAG" 2>/dev/null \
    | grep -E "Topic: /livox/lidar " | grep -oE "(CustomMsg|PointCloud2)" | head -1)
if [ "$BAG_TYPE" = "PointCloud2" ]; then
    ok "PointCloud2 → 需要转换节点（CustomMsg）"
    NEED_CONV=1
elif [ "$BAG_TYPE" = "CustomMsg" ]; then
    ok "已经是 CustomMsg → 无需转换"
    NEED_CONV=0
else
    warn "未识别点云类型（$BAG_TYPE），按 PointCloud2 处理"
    NEED_CONV=1
fi

# ── 1. 起转换节点 ─────────────────────────────────────────────
CONV_PID=""
if [ "$NEED_CONV" = "1" ]; then
    echo
    echo "── 1. 起 PointCloud2 → CustomMsg 转换节点 ──"
    /usr/bin/python3 "$SCRIPT_DIR/16_pc2_to_custom.py" > "$OUT/conv.log" 2>&1 &
    CONV_PID=$!
    sleep 3
    if ps -p "$CONV_PID" > /dev/null 2>&1; then
        ok "转换节点就绪（pid $CONV_PID）"
    else
        err "转换节点没起来"; tail -15 "$OUT/conv.log"; exit 1
    fi
fi

# ── 2. 起 LIO-SAM ────────────────────────────────────────────
echo
echo "── 2. 起 LIO-SAM（4 节点，无 RViz）──"
ros2 launch "$SCRIPT_DIR/liosam/run_headless.launch.py" params_file:="$PARAMS" \
    > "$OUT/lio.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 40); do
    N=$(ros2 node list 2>/dev/null | grep -c "^/lio_sam_" || true)
    [ "${N:-0}" -ge 4 ] && break
    sleep 0.5
done
N=$(ros2 node list 2>/dev/null | grep -c "^/lio_sam_" || true)
if [ "${N:-0}" -lt 4 ]; then
    err "LIO-SAM 只起来 ${N:-0}/4 个节点，日志："
    tail -20 "$OUT/lio.log"
    [ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
    kill -INT "$LAUNCH_PID" 2>/dev/null
    exit 1
fi
ok "四个节点就绪"

# ── 3. 起轨迹记录 ────────────────────────────────────────────
echo
echo "── 3. 起 traj_recorder（订阅 $TOPIC）──"
/usr/bin/python3 "$SCRIPT_DIR/traj_recorder.py" --topic "$TOPIC" --out "$OUT/traj.txt" \
    > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 2
ok "轨迹记录器就绪"

# ── 4. 回放 ──────────────────────────────────────────────────
echo
echo "── 4. 回放 bag（-r $RATE，不加 --clock）──"
PLAY_START=$(date +%s)
ros2 bag play "$BAG" -r "$RATE" > "$OUT/play.log" 2>&1
PLAY_SECS=$(( $(date +%s) - PLAY_START ))
ok "回放结束（耗时 ${PLAY_SECS}s）"

echo
echo "── 5. 等算法处理完剩余数据 ──"
sleep 8

# ── 6. 优雅退出 ──────────────────────────────────────────────
echo
echo "── 6. 停止各进程 ──"
kill -INT "$REC_PID" 2>/dev/null
sleep 2
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 4
[ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
# ros2 launch 常常不把 SIGINT 传给子节点，逐个清掉「本次新增」的 lio_sam 进程
for p in $(ps -eo pid,args --no-headers 2>/dev/null \
           | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}'); do
    case " $BEFORE_PIDS " in
        *" $p "*) ;;                       # 本次运行前就有 → 不是我的
        *) kill -INT "$p" 2>/dev/null ;;   # 本次起的 → 收掉
    esac
done
sleep 3
LEFT=$(ps -eo pid,args --no-headers 2>/dev/null | grep -E "$LIO_PAT" | grep -v grep \
       | awk '{print $1}' | while read -r p; do
           case " $BEFORE_PIDS " in *" $p "*) ;; *) echo "$p" ;; esac
       done | wc -l)
[ "${LEFT:-0}" -gt 0 ] && warn "还有 $LEFT 个节点进程未退出" || ok "已停止"

# ── 7. 汇总 ──────────────────────────────────────────────────
echo
echo "── 7. 产物 ──"
if [ -s "$OUT/traj.txt" ]; then
    NP=$(grep -cv '^#' "$OUT/traj.txt" 2>/dev/null || echo 0)
    ok "轨迹 $OUT/traj.txt（${NP} 个位姿）"
else
    warn "没生成轨迹 —— 检查 $OUT/recorder.log 与 $OUT/lio.log"
fi
[ -f "$OUT/conv.log" ] && ok "转换日志 $OUT/conv.log"
ok "算法日志 $OUT/lio.log"
echo
echo "下一步（算绝对精度）："
echo "  /usr/bin/python3 scripts/14_eval_on_site.py \\"
echo "      --traj \"$OUT/traj.txt\" \\"
echo "      --truth config/site_mid360_demo.json"
