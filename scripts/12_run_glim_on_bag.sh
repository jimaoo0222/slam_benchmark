#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  一键：用 GLIM 跑一个 bag，产出轨迹
#
#  用法（任何目录都行）：
#      bash ~/slam_benchmark/scripts/12_run_glim_on_bag.sh
#      bash ~/slam_benchmark/scripts/12_run_glim_on_bag.sh rosbag/my_static_map1
#      bash ~/slam_benchmark/scripts/12_run_glim_on_bag.sh rosbag/my_static_map2 results/xxx
#
#  参数：
#      第 1 个   bag 目录（默认 rosbag/my_static_map1）
#      第 2 个   输出目录（默认 results/日期_GLIM/<包名>）
#
#  产出：
#      traj_glim.txt     TUM 轨迹，可直接喂 protocol_eval.py
#      traj_imu.txt      以 IMU 为基准的轨迹
#      glim.log          完整日志
#      submaps/          各子图点云
#
#  ⚠️ 环境坑（脚本已处理）：
#      · GLIM 装在 ~/glim_ws，GTSAM/gtsam_points 在 ~/.local，都要 source
#      · conda 的 python/boost 必须从 PATH/LD_LIBRARY_PATH 里剔除
# ══════════════════════════════════════════════════════════════
set -uo pipefail

ROOT="$HOME/slam_benchmark"
GLIM_WS="$HOME/glim_ws"

BAG="${1:-rosbag/my_static_map1}"
case "$BAG" in
    /*) ;;
    *)  BAG="$ROOT/$BAG" ;;
esac
NAME="$(basename "$BAG")"
OUT="${2:-$ROOT/results/$(date +%Y-%m-%d)_GLIM/$NAME}"

# 按 bag 前缀选配置：01_/02_ → MID-360s，03_/04_ → C01（两颗雷达外参不同）
source "$(cd "$(dirname "$0")" && pwd)/lib_radar.sh"
RADAR="$(radar_of_bag "$BAG")"
case "$RADAR" in
    c01)    CFG="$ROOT/config/glim_c01"  ;;
    *)      CFG="$ROOT/config/glim_livox" ;;
esac

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

echo "======================================================================"
echo " GLIM × bag"
echo "   bag : $BAG"
echo "   雷达: $(radar_desc "$RADAR")"
echo "   配置: $CFG"
echo "   输出: $OUT"
echo "======================================================================"

# ── 检查 ───────────────────────────────────────────────────
[ -d "$BAG" ] || { err "bag 目录不存在: $BAG"; exit 1; }
[ -x "$GLIM_WS/install/glim_ros/lib/glim_ros/glim_rosbag" ] || {
    err "GLIM 没装好。看 docs/03_四算法手动启动与RViz.md 第 6 节"; exit 1; }
[ -d "$CFG" ] || { err "缺配置目录: $CFG"; exit 1; }

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
DUMP="$(mktemp -d)"
trap 'rm -rf "$DUMP"' EXIT

# ── 环境 ───────────────────────────────────────────────────
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for f in "$HOME/livox_ws/livox_env.sh" "$HOME/livox_ws/install/setup.bash" \
         "$HOME/lio_ws/install/setup.bash" "$GLIM_WS/install/setup.bash"; do
    # shellcheck disable=SC1090,SC1091
    [ -f "$f" ] && source "$f"
done
set -u
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
ok "环境就绪"

# ── 跑 ─────────────────────────────────────────────────────
echo
echo "── 运行 GLIM（比实时快，不用等回放）──"
"$GLIM_WS/install/glim_ros/lib/glim_ros/glim_rosbag" "$BAG" \
    --ros-args \
    -p config_path:="$CFG" \
    -p auto_quit:=true \
    -p dump_path:="$DUMP" \
    > "$OUT/glim.log" 2>&1
RET=$?

# ── 收集 ───────────────────────────────────────────────────
echo
echo "── 收集产物 ──"
if [ -f "$DUMP/traj_lidar.txt" ]; then
    cp "$DUMP/traj_lidar.txt" "$OUT/traj_glim.txt"
    cp "$DUMP/traj_imu.txt"   "$OUT/traj_imu.txt" 2>/dev/null || true
    [ -d "$DUMP/000000" ] && cp -r "$DUMP/000000" "$OUT/submaps"
    NP=$(grep -vc '^#' "$OUT/traj_glim.txt" 2>/dev/null || echo 0)
    ok "轨迹 $OUT/traj_glim.txt（$NP 个位姿）"
    [ -d "$OUT/submaps" ] && ok "子图 $(ls "$OUT/submaps" | wc -l) 个"
else
    err "没有产出轨迹（退出码 $RET）—— 日志尾部："
    grep -viE "TF_SELF_TRANSFORM" "$OUT/glim.log" | tail -15
    exit 1
fi

# 统计
if command -v python3 >/dev/null 2>&1; then
    python3 - "$OUT/traj_glim.txt" <<'PY'
import sys
import numpy as np
a = np.loadtxt(sys.argv[1], comments="#")
if a.ndim == 1: a = a.reshape(1, -1)
d = np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1).sum()
span = a[:, 1:4].max(axis=0) - a[:, 1:4].min(axis=0)
print(f"   时长 {a[-1,0]-a[0,0]:.1f}s   路径长 {d:.2f} m")
print(f"   跨度 x{span[0]:.2f} y{span[1]:.2f} z{span[2]:.2f} m")
PY
fi

echo
echo "======================================================================"
echo " 完成 → $OUT"
echo "======================================================================"
echo "  看轨迹： python3 $ROOT/scripts/protocol_eval.py \"$OUT/traj_glim.txt\""
echo "  看点云： 见 docs/03_四算法手动启动与RViz.md 第 7 节（RViz 加 Display）"
echo
warn "IMU 校验日志（判断 IMU 是否真起作用）："
echo "     grep 'IMU better ratios' \"$OUT/glim.log\" | tail -1"
