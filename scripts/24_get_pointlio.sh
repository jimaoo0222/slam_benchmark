#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  一键装 Point-LIO（本项目自移植的 ROS 2 版）
#
#  背景：官方 Point-LIO 只有 ROS 1 (Noetic) 版，本机是 ROS 2 Humble。
#        本项目已完成 ROS 2 移植（11k 行代码，算法核心未改），
#        补丁见 patches/0003-pointlio-ros2-port.patch
#
#  用法：
#     bash scripts/24_get_pointlio.sh            # 装到 ~/pointlio_ws
#     bash scripts/24_get_pointlio.sh --check    # 只检查现状
#
#  前置：ROS 2 Humble + ~/livox_ws 里编译好的 livox_ros_driver2
# ══════════════════════════════════════════════════════════════
set -uo pipefail

WS="$HOME/pointlio_ws"
SRC="$WS/src/point_lio"
PATCH="$(cd "$(dirname "$0")/.." && pwd)/patches/0003-pointlio-ros2-port.patch"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; B='\033[0;34m'; N='\033[0m'
ok(){ echo -e "  ${G}✅${N} $*"; }; warn(){ echo -e "  ${Y}⚠️ ${N} $*"; }
err(){ echo -e "  ${R}❌${N} $*"; }; step(){ echo; echo -e "${B}── $* ──${N}"; }

if [ "$CHECK" = "1" ]; then
    step "检查 Point-LIO (ROS 2) 安装状态"
    [ -d "$SRC" ] && ok "源码 $SRC" || warn "源码缺失"
    [ -f "$WS/install/point_lio/lib/point_lio/pointlio_mapping" ] && ok "已编译" || warn "未编译"
    [ -f "$HOME/livox_ws/install/setup.bash" ] && ok "livox_ros_driver2 存在" || warn "livox_ros_driver2 缺失"
    [ -f "$SRC/config/mid360.yaml" ] && ok "MID-360 配置存在" || warn "缺 mid360.yaml"
    exit 0
fi

echo "======================================================================"
echo " 安装 Point-LIO（ROS 2 移植版）"
echo "   工作区: $WS"
echo "======================================================================"

step "0. 前置检查"
[ -f /opt/ros/humble/setup.bash ] || { err "没找到 ROS 2 Humble"; exit 1; }
[ -f "$HOME/livox_ws/install/setup.bash" ] || { err "缺 livox_ros_driver2，先跑 scripts/02_get_livox.sh"; exit 1; }
[ -f "$PATCH" ] || { err "缺补丁文件: $PATCH"; exit 1; }
ok "前置齐全"

step "1. 取上游源码"
if [ -d "$SRC/.git" ] || [ -d "$SRC/src" ]; then
    ok "已存在，跳过"
else
    mkdir -p "$WS/src"
    git clone --depth 1 https://github.com/hku-mars/Point-LIO.git "$SRC" 2>&1 | tail -3
    [ -d "$SRC/src" ] || { err "clone 失败"; exit 1; }
    ok "已下载"
fi

step "2. 应用 ROS 2 移植补丁"
if [ -f "$SRC/include/ros2_compat.h" ]; then
    ok "补丁已应用（检测到 ros2_compat.h）"
else
    ( cd "$SRC" && patch -p1 --no-backup-if-mismatch < "$PATCH" ) 2>&1 | tail -8
    [ -f "$SRC/include/ros2_compat.h" ] && ok "补丁应用成功" || { err "补丁失败"; exit 1; }
fi

step "3. 准备日志目录（上游会往里写 CSV）"
mkdir -p "$SRC/Log" && ok "$SRC/Log"

step "4. 编译"
cd "$WS" || exit 1
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
set -u
colcon build --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -8

step "5. 验证"
if [ -f "$WS/install/point_lio/lib/point_lio/pointlio_mapping" ]; then
    ok "编译产物就绪"
    echo
    echo "  跑一个 bag："
    echo "    bash scripts/23_run_pointlio_on_bag.sh rosbag/01_1 results/pointlio_test"
else
    err "编译失败，看上面的错误"
    exit 1
fi
