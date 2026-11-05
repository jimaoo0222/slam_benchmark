#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  对一个 bag 跑 Point-LIO（本项目自移植的 ROS 2 版），产出轨迹 + 日志
#
#  用法：
#     bash scripts/23_run_pointlio_on_bag.sh <bag目录> <输出目录> [选项]
#
#  例：
#     bash scripts/23_run_pointlio_on_bag.sh rosbag/01_1 results/2026-10-25_Point-LIO/01_1
#
#  选项：
#     --rate 1.0        回放倍率
#     --params <file>   参数文件（默认 ~/pointlio_ws/install/.../config/mid360.yaml）
#     --topic <t>       轨迹话题（默认 /aft_mapped_to_init）
#
#  它会做：
#     1. 起 16_pc2_to_custom.py（Point-LIO 走 CustomMsg 路径）
#     2. 起 point_lio（无 RViz）
#     3. 起 traj_recorder 录轨迹 → traj.txt
#     4. 回放 bag（不加 --clock）
#     5. 优雅退出并汇总
# ══════════════════════════════════════════════════════════════
set -uo pipefail

BAG="${1:-}"; OUT="${2:-}"; shift 2 2>/dev/null || true
RATE="1.0"; TOPIC="/aft_mapped_to_init"
PARAMS=""            # 留空 = 按 bag 名字自动选（见下方 lib_radar.sh）
PCFG="$HOME/pointlio_ws/install/point_lio/share/point_lio/config"

while [ $# -gt 0 ]; do
    case "$1" in
        --rate)   RATE="$2";   shift 2 ;;
        --params) PARAMS="$2"; shift 2 ;;
        --topic)  TOPIC="$2";  shift 2 ;;
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 没显式指定参数文件时按 bag 前缀选：01_/02_ → MID-360s，03_/04_ → C01
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/lib_radar.sh"
RADAR="$(radar_of_bag "$BAG")"
if [ -z "$PARAMS" ]; then
    case "$RADAR" in
        c01)    PARAMS="$PCFG/c01.yaml" ;;
        *)      PARAMS="$PCFG/mid360.yaml" ;;
    esac
fi

G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; B='\033[0;34m'; N='\033[0m'
ok(){ echo -e "  ${G}✅${N} $*"; }; warn(){ echo -e "  ${Y}⚠️ ${N} $*"; }
err(){ echo -e "  ${R}❌${N} $*"; }; step(){ echo; echo -e "${B}── $* ──${N}"; }

[ -n "$BAG" ] && [ -n "$OUT" ] || { err "用法: $0 <bag> <输出目录>"; exit 1; }
[ -d "$BAG" ] || { err "bag 不存在: $BAG"; exit 1; }
[ -f "$PARAMS" ] || { err "参数文件不存在: $PARAMS（先跑 scripts/24_get_pointlio.sh）"; exit 1; }

mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"

echo "======================================================================"
echo " Point-LIO × bag"
echo "   bag  : $BAG"
echo "   输出 : $OUT"
echo "   参数 : $(basename $PARAMS)（倍率 $RATE）"
echo "======================================================================"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
source /opt/ros/humble/setup.bash
for f in "$HOME/livox_ws/install/setup.bash" "$HOME/pointlio_ws/install/setup.bash"; do
    [ -f "$f" ] && source "$f"
done
set -u

LIO_PAT="pointlio_mapping"
BEFORE=$(ps -eo pid,args --no-headers 2>/dev/null | grep "$LIO_PAT" | grep -v grep | awk '{print $1}' | tr '\n' ' ')

rm -f "$OUT/traj.txt"

step "1. 起点云转换节点"
BAG_TYPE=$(ros2 bag info "$BAG" 2>/dev/null | grep -E "Topic: /livox/lidar " | grep -oE "(CustomMsg|PointCloud2)" | head -1)
if [ "$BAG_TYPE" = "CustomMsg" ]; then
    ok "bag 已是 CustomMsg，无需转换"; NEED_CONV=0; LID_TOPIC="/livox/lidar"
    sed -i 's|^\(\s*\)lid_topic:.*|\1lid_topic: "/livox/lidar"|' "$PARAMS" 2>/dev/null || true
else
    /usr/bin/python3 "$SCRIPT_DIR/16_pc2_to_custom.py" > "$OUT/conv.log" 2>&1 &
    CONV_PID=$!; NEED_CONV=1; sleep 3
    ps -p "$CONV_PID" >/dev/null 2>&1 && ok "转换节点就绪" || { err "转换节点失败"; tail -10 "$OUT/conv.log"; exit 1; }
fi

step "2. 起 Point-LIO"
ros2 launch point_lio mapping_mid360.launch.py rviz:=false config_file:="$PARAMS" \
    > "$OUT/plio.log" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 60); do
    ros2 node list 2>/dev/null | grep -q "^/laserMapping" && break
    sleep 0.5
done
ros2 node list 2>/dev/null | grep -q "^/laserMapping" \
    && ok "Point-LIO 就绪" \
    || { err "启动失败"; tail -20 "$OUT/plio.log"; kill -INT "$LAUNCH_PID" 2>/dev/null; exit 1; }

step "3. 起轨迹记录（订阅 $TOPIC）"
/usr/bin/python3 "$SCRIPT_DIR/traj_recorder.py" --topic "$TOPIC" --out "$OUT/traj.txt" \
    > "$OUT/recorder.log" 2>&1 &
REC_PID=$!; sleep 2; ok "记录器就绪"

step "4. 回放 bag（-r $RATE）"
T0=$(date +%s)
ros2 bag play "$BAG" -r "$RATE" > "$OUT/play.log" 2>&1
ok "回放结束（$(( $(date +%s) - T0 ))s）"

step "5. 等处理完剩余数据"
sleep 8

step "6. 停止"
kill -INT "$REC_PID" 2>/dev/null; sleep 2
kill -INT "$LAUNCH_PID" 2>/dev/null; sleep 4
for p in $(ps -eo pid,args --no-headers 2>/dev/null | grep "$LIO_PAT" | grep -v grep | awk '{print $1}'); do
    case " $BEFORE " in *" $p "*) ;; *) kill -INT "$p" 2>/dev/null ;; esac
done
[ "${NEED_CONV:-0}" = "1" ] && kill -INT "$CONV_PID" 2>/dev/null
sleep 2; ok "已停止"

step "7. 产物"
if [ -s "$OUT/traj.txt" ]; then
    ok "轨迹 $OUT/traj.txt（$(grep -cv '^#' "$OUT/traj.txt") 个位姿）"
else
    warn "没生成轨迹，看 $OUT/recorder.log 与 $OUT/plio.log"
fi
echo
echo "算精度："
echo "  /usr/bin/python3 scripts/14_eval_on_site.py \"$OUT/traj.txt\" --site config/site_mid360_demo.json"
