#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  对一个 bag 跑 FAST-LIO2，产出轨迹 + 建图 PCD + 日志
#
#  用法：
#     bash scripts/05_run_fastlio_on_bag.sh <bag目录> <输出目录> [选项]
#
#  例：
#     bash scripts/05_run_fastlio_on_bag.sh \
#          rosbag/my_static_map1 results/2026-10-19_建图测试/map1
#
#  选项：
#     --rate 1.0          回放倍率（默认 1.0；2.0 大约快一倍，但可能丢帧）
#     --config mid360_pc2.yaml   fast_lio 的配置文件名
#     --topic /Odometry   算法发轨迹的话题
#
#  它会做：
#     1. 起 FAST-LIO2（use_sim_time=true）
#     2. 起 traj_recorder 录 /Odometry → traj.txt
#     3. 回放 bag（--clock）
#     4. 回放结束后停掉所有进程
#     5. 汇总 PCD / 轨迹 / 日志
#
#  ⚠️ 三个必须知道的点：
#     · bag 是 PointCloud2（rviz_MID360_launch.py 录的）→ 要用 mid360_pc2.yaml
#       （lidar_type: 4）；CustomMsg（msg_MID360_launch.py 录的）→ mid360.yaml
#     · **pcd_save_en 在这个 ROS 2 移植版里是失效的**（源码那段被注释掉了），
#       所以地图由 06_save_map_pcd.py 订阅 /Laser_map 另存
#     · 算法不能 kill -9，会丢最后的处理结果
# ══════════════════════════════════════════════════════════════
set -uo pipefail

BAG="${1:-}"
OUT="${2:-}"
shift 2 2>/dev/null || true

RATE="1.0"
CONFIG=""            # 留空 = 按 bag 名字自动选（见下方 lib_radar.sh）
TOPIC="/Odometry"

while [ $# -gt 0 ]; do
    case "$1" in
        --rate)   RATE="$2";   shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
        --topic)  TOPIC="$2";  shift 2 ;;
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 没显式指定 config 时按 bag 前缀选：01_/02_ → MID-360s，03_/04_ → C01
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/lib_radar.sh"
RADAR="$(radar_of_bag "$BAG")"
if [ -z "$CONFIG" ]; then
    case "$RADAR" in
        c01)    CONFIG="c01_pc2.yaml" ;;
        *)      CONFIG="mid360_pc2.yaml" ;;
    esac
fi

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

if [ -z "$BAG" ] || [ -z "$OUT" ]; then
    err "缺参数"
    sed -n '2,26p' "$0"
    exit 1
fi
[ -d "$BAG" ] || { err "bag 目录不存在: $BAG"; exit 1; }

LIO_SRC="$HOME/lio_ws/src/fast_lio"
CFG_DIR="$LIO_SRC/config"
[ -f "$CFG_DIR/$CONFIG" ] || { err "找不到配置 $CFG_DIR/$CONFIG"; exit 1; }

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

echo "======================================================================"
echo " FAST-LIO2 × bag"
echo "   bag    : $BAG"
echo "   输出   : $OUT"
echo "   配置   : $CONFIG（倍率 $RATE）"
echo "======================================================================"

# conda 的 python 会顶掉 ROS 的 python3.10
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for f in "$HOME/livox_ws/livox_env.sh" "$HOME/livox_ws/install/setup.bash" "$HOME/lio_ws/install/setup.bash"; do
    # shellcheck disable=SC1090,SC1091
    [ -f "$f" ] && source "$f"
done
set -u

# 清掉上次残留，免得把旧结果当这次的
rm -f "$OUT/traj.txt" "$OUT/map.pcd"

# ── 1. 起算法 ──────────────────────────────────────────────
echo
echo "── 1. 起 FAST-LIO2 ──"
ros2 launch fast_lio mapping.launch.py \
    rviz:=false use_sim_time:=true \
    config_path:="$CFG_DIR" config_file:="$CONFIG" \
    > "$OUT/lio.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 30); do
    grep -q "Node init finished" "$OUT/lio.log" 2>/dev/null && break
    sleep 0.5
done
if ! grep -q "Node init finished" "$OUT/lio.log" 2>/dev/null; then
    err "算法没起来，日志："; tail -15 "$OUT/lio.log"; kill -INT "$LAUNCH_PID" 2>/dev/null; exit 1
fi
ok "算法就绪（lidar_type=$(grep -oE 'lidar_type [0-9]+' "$OUT/lio.log" | head -1 | grep -oE '[0-9]+')）"

# ── 2. 起录轨迹 + 地图保存 ─────────────────────────────────
echo
echo "── 2. 起 traj_recorder（订阅 $TOPIC）+ 地图保存 ──"
python3 "$(dirname "$0")/traj_recorder.py" --topic "$TOPIC" --out "$OUT/traj.txt" \
    > "$OUT/recorder.log" 2>&1 &
REC_PID=$!

# ⚠️ FAST-LIO 的 ROS 2 移植版把「累积存 PCD」那段注释掉了（pcd_save_en 失效），
#    但 publish_map() 是活的，/Laser_map 就是累积好的完整地图 —— 订阅它即可
python3 "$(dirname "$0")/06_save_map_pcd.py" --topic /Laser_map --out "$OUT/map.pcd" \
    > "$OUT/mapsaver.log" 2>&1 &
MAP_PID=$!
sleep 2
ok "轨迹记录器 + 地图保存就绪"

# ── 3. 回放 ────────────────────────────────────────────────
echo
echo "── 3. 回放 bag（--clock -r $RATE）──"
PLAY_START=$(date +%s)
ros2 bag play "$BAG" --clock -r "$RATE" > "$OUT/play.log" 2>&1
PLAY_SECS=$(( $(date +%s) - PLAY_START ))
ok "回放结束（耗时 ${PLAY_SECS}s）"

# 等算法把缓冲区的数据算完
echo
echo "── 4. 等算法处理完剩余数据 ──"
sleep 6

# ── 5. 优雅退出 ────────────────────────────────────────────
echo
echo "── 5. 停止算法与保存节点 ──"
NODE_PID=$(ps -eo pid,args --no-headers | awk '/fastlio_mapping/ && !/awk/ {print $1; exit}')
if [ -n "${NODE_PID:-}" ]; then
    kill -INT "$NODE_PID" 2>/dev/null
    for _ in $(seq 1 40); do
        ps -p "$NODE_PID" > /dev/null 2>&1 || break
        sleep 0.5
    done
    ok "算法已退出"
else
    warn "没找到 fastlio_mapping 进程（可能已自行退出）"
fi
kill -INT "$REC_PID" 2>/dev/null
kill -INT "$MAP_PID" 2>/dev/null
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 4

# ── 6. 收集产物 ────────────────────────────────────────────
echo
echo "── 6. 收集产物 ──"
if [ -s "$OUT/map.pcd" ]; then
    NPTS=$(sed -n 's/^POINTS //p' "$OUT/map.pcd" | head -1)
    ok "地图 $OUT/map.pcd（${NPTS:-?} 点，$(du -h "$OUT/map.pcd" | cut -f1)）"
else
    warn "没生成地图 —— 检查算法日志里 publish.map_en 是否为 true"
fi

NPOSE=$(grep -vc '^#' "$OUT/traj.txt" 2>/dev/null || echo 0)
if [ "$NPOSE" -gt 0 ]; then ok "轨迹 $NPOSE 个位姿"; else err "轨迹是空的！检查 $OUT/lio.log"; fi

# 只统计真正的错误，忽略 reflectivity 那种字段名警告
ERRS=$(grep -icE "error|failed to (init|find topic)" "$OUT/lio.log" 2>/dev/null || echo 0)
WARN_REFL=$(grep -c "Failed to find match for field 'reflectivity'" "$OUT/lio.log" 2>/dev/null || echo 0)

echo
echo "======================================================================"
echo " 完成 → $OUT"
echo "======================================================================"
echo "   traj.txt    $NPOSE 个位姿"
echo "   map*.pcd    建图点云"
echo "   lio.log     算法日志"
if [ "$WARN_REFL" -gt 0 ]; then
    echo "   （日志里有 $WARN_REFL 条 reflectivity 警告 —— 包是 PointCloud2 格式，正常，不影响定位）"
fi
echo
echo "看轨迹最快的方式："
echo "   python3 $(dirname "$0")/protocol_eval.py \"$OUT/traj.txt\""
