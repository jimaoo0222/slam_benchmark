#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  一键：回放 bag + 跑 FAST-LIO + 开 RViz
#
#  用法（在【任何目录】都行）：
#      bash ~/slam_benchmark/scripts/11_rviz_demo.sh
#      bash ~/slam_benchmark/scripts/11_rviz_demo.sh rosbag/my_static_map1
#      bash ~/slam_benchmark/scripts/11_rviz_demo.sh rosbag/my_static_map2 2.0
#
#  参数：
#      第 1 个   bag 目录（默认 rosbag/my_static_map1）
#      第 2 个   回放倍率（默认 1.0；2.0 快一倍但可能丢帧）
#
#  它会自动：
#      · 判断 bag 是 CustomMsg 还是 PointCloud2 → 选对配置
#      · 清掉 conda 的 python（否则 ROS 起不来）
#      · source 驱动 + 算法的环境
#      · 起 FAST-LIO + RViz（用修好 Fixed Frame 的配置）
#      · 回放 bag
#      · 回放结束后 RViz 保持打开，按 Enter 退出
# ══════════════════════════════════════════════════════════════
set -uo pipefail

ROOT="$HOME/slam_benchmark"
cd "$ROOT" || { echo "❌ 找不到 $ROOT"; exit 1; }

BAG="${1:-rosbag/my_static_map1}"
RATE="${2:-1.0}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

echo "======================================================================"
echo " 一键 RViz 演示"
echo "   bag : $BAG"
echo "   倍率: $RATE"
echo "======================================================================"

# ── 1. 检查 ────────────────────────────────────────────────
[ -d "$BAG" ] || { err "bag 目录不存在: $ROOT/$BAG"; echo "   可用的："; ls -d rosbag/*/ 2>/dev/null | sed 's/^/     /'; exit 1; }
[ -x /usr/bin/python3 ] || { err "找不到 /usr/bin/python3"; exit 1; }

# conda 的 python 会顶掉 ROS 的 3.10
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
for f in "$HOME/livox_ws/livox_env.sh" "$HOME/livox_ws/install/setup.bash" "$HOME/lio_ws/install/setup.bash"; do
    # shellcheck disable=SC1090,SC1091
    [ -f "$f" ] && source "$f"
done
set -u
ok "环境已就绪"

# ── 2. 判断 bag 格式，选配置 ────────────────────────────────
TOPIC_TYPE=$(ros2 bag info "$BAG" 2>/dev/null | grep "livox/lidar" | grep -oE "CustomMsg|PointCloud2" | head -1)
if [ "$TOPIC_TYPE" = "CustomMsg" ]; then
    CFG="mid360.yaml"
elif [ "$TOPIC_TYPE" = "PointCloud2" ]; then
    CFG="mid360_pc2.yaml"
else
    err "读不出 bag 里 /livox/lidar 的类型"; exit 1
fi
ok "bag 是 $TOPIC_TYPE → 用配置 $CFG"

RVCFG="$ROOT/scripts/rviz/fastlio_ros2.rviz"
[ -f "$RVCFG" ] || { err "缺 rviz 配置: $RVCFG"; exit 1; }

# ── 3. 起算法 + RViz（后台）────────────────────────────────
echo
echo "── 启动 FAST-LIO + RViz（RViz 窗口马上会弹出来）──"
ros2 launch fast_lio mapping.launch.py \
    rviz:=true \
    rviz_cfg:="$RVCFG" \
    use_sim_time:=true \
    config_path:="$HOME/lio_ws/src/fast_lio/config" \
    config_file:="$CFG" \
    > /tmp/rviz_demo_lio.log 2>&1 &
LIO_PID=$!

# 等算法就绪
for _ in $(seq 1 40); do
    grep -q "Initialize the map kdtree\|IMU Initial Done" /tmp/rviz_demo_lio.log 2>/dev/null && break
    sleep 0.5
done
if ! ps -p "$LIO_PID" > /dev/null 2>&1; then
    err "算法没起来，日志："
    tail -15 /tmp/rviz_demo_lio.log
    exit 1
fi
ok "算法 + RViz 已启动（pid $LIO_PID）"

# ── 4. 回放 ────────────────────────────────────────────────
echo
echo "── 开始回放（时长 $(ros2 bag info "$BAG" 2>/dev/null | grep -oE 'Duration:[[:space:]]+[0-9.]+' | grep -oE '[0-9.]+') 秒）──"
echo "   RViz 里应该能看到点云一帧帧堆成建筑"
echo
ros2 bag play "$BAG" --clock -r "$RATE"
ok "回放结束"

# ── 5. 收尾 ────────────────────────────────────────────────
echo
echo "======================================================================"
echo " 回放完了，RViz 还开着 —— 现在可以："
echo "   左键拖 = 转    滚轮 = 缩放    中键拖 = 平移"
echo "======================================================================"
warn "RViz 里如果一片空白：检查左侧 Global Options → Fixed Frame 是否为 odom"
echo
if [ -t 0 ]; then
    read -r -p "按 Enter 关闭 RViz 并退出… " _ || true
fi
# 先停子进程（节点、rviz），再停 launch 本身
for pid in $(ps -eo pid,ppid --no-headers | awk -v p="$LIO_PID" '$2==p {print $1}'); do
    kill -INT "$pid" 2>/dev/null
done
kill -INT "$LIO_PID" 2>/dev/null
sleep 2
ok "已退出"
