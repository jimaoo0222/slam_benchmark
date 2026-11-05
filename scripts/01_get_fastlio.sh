#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  获取并编译 FAST-LIO2（ROS 2 Humble 版）
#
#  用法：
#     bash scripts/01_get_fastlio.sh [工作区路径]
#     默认工作区：~/lio_ws
#
#  做完这些事：
#     1. 下载 FAST-LIO2 源码（支持断点续传 + 重试）
#     2. 解压到 <工作区>/src/fast_lio
#     3. colcon build
#     4. 打印运行提示
#
#  ⚠️ 网络提示：
#     github.com 直连不通，本脚本走 codeload.github.com（实测可用）。
#     源码约 130 MB，网速不稳 —— 脚本会重试续传，中断了再跑一次即可。
# ══════════════════════════════════════════════════════════════
set -uo pipefail

WS="${1:-$HOME/lio_ws}"
SRC="$WS/src"
NAME="fast_lio"
REPO="andrewliang01/FAST_LIO_ROS2_HUMBLE"
BRANCH="ros2_humble_support"
TARBALL="/tmp/fastlio_${BRANCH}.tar.gz"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

echo "======================================================================"
echo " FAST-LIO2 获取与编译"
echo "  工作区: $WS"
echo "======================================================================"

mkdir -p "$SRC"

# ── 1. 已经解压过就跳过下载 ────────────────────────────────
if [ -d "$SRC/$NAME" ] && [ -f "$SRC/$NAME/package.xml" ]; then
    ok "源码已存在: $SRC/$NAME"
else
    echo
    echo "── 1. 下载源码（约 130 MB，支持断点续传）──"
    URL="https://codeload.github.com/${REPO}/tar.gz/refs/heads/${BRANCH}"

    for i in 1 2 3 4 5; do
        # -C - 续传；失败则下一轮接着下
        timeout 120 curl -C - -sL -o "$TARBALL" "$URL" || true
        SZ=$(stat -c%s "$TARBALL" 2>/dev/null || echo 0)
        printf "  第 %d 轮: %.1f MB\n" "$i" "$(echo "$SZ/1048576" | bc -l 2>/dev/null || echo 0)"

        if tar tzf "$TARBALL" >/dev/null 2>&1; then
            ok "下载完整（$(echo "$SZ/1048576" | bc -l 2>/dev/null | cut -c1-5) MB）"
            break
        fi
        [ "$i" = 5 ] && { err "5 轮仍未完整 —— 网络太慢，稍后再跑一次即可（会接着下）"; exit 1; }
    done

    echo
    echo "── 2. 解压 ──"
    rm -rf "$SRC/$NAME.tmp"
    mkdir -p "$SRC/$NAME.tmp"
    tar xzf "$TARBALL" -C "$SRC/$NAME.tmp" --strip-components=1
    mv "$SRC/$NAME.tmp" "$SRC/$NAME"
    ok "解压到 $SRC/$NAME"
fi

echo
echo "── 3. 编译 ──"
cd "$WS" || exit 1
if [ ! -f /opt/ros/humble/setup.bash ]; then
    err "找不到 /opt/ros/humble —— 先装 ROS 2 Humble"
    exit 1
fi
# conda 的 python3 会顶掉 ROS 的 python3.10，rosidl 就找不到 empy
# （症状：ModuleNotFoundError: No module named 'em' —— conda 的 python 抢进了 ROS）
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
# ROS 的 setup.bash 会引用未定义变量，先关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# ⭐ FAST-LIO2 的 CMakeLists.txt:63 是 find_package(livox_ros_driver2 REQUIRED)，
#    所以必须先 source Livox 驱动的工作区，否则 CMake 找不到
#    livox_ros_driver2Config.cmake，直接报错退出。
LIVOX_WS="${LIVOX_WS:-$HOME/livox_ws}"
if [ -f "$LIVOX_WS/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091,SC1090
    [ -f "$LIVOX_WS/livox_env.sh" ] && source "$LIVOX_WS/livox_env.sh"
    # shellcheck disable=SC1091
    source "$LIVOX_WS/install/setup.bash"
    set -u
    ok "已 source Livox 驱动工作区（$LIVOX_WS）"
else
    err "找不到 $LIVOX_WS/install/setup.bash —— FAST-LIO2 依赖 livox_ros_driver2 这个包"
    echo "     先装驱动："
    echo "       bash ~/slam_benchmark/scripts/02_get_livox.sh --lidar-ip <雷达IP>"
    exit 1
fi

if colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    ok "编译完成"
else
    err "编译失败 —— 常见原因："
    echo "     · 缺依赖: sudo apt install ros-humble-pcl-ros ros-humble-eigen3-cmake-module"
    echo "     · conda 干扰: 先 conda deactivate 再重试"
    exit 1
fi

echo
echo "======================================================================"
echo " 完成！"
echo "======================================================================"
echo
echo "运行方式（三个终端）："
echo
echo "  终端 A  回放数据 / 接真实雷达"
echo "          source $WS/install/setup.bash"
echo "          ros2 launch fast_lio <某个 launch 文件>"
echo
echo "  终端 B  回放 bag"
echo "          ros2 bag play <你的实验目录>/run/"
echo
echo "  终端 C  录轨迹（喂给评测工具）"
echo "          source /opt/ros/humble/setup.bash"
echo "          python3 ~/slam_benchmark/scripts/traj_recorder.py \\"
echo "                  --topic /Odometry --out traj_fastlio2.txt"
echo
echo "  ⚠️ MID-360 的配置要从 lavox 配置改："
echo "     · lid_topic  = /livox/lidar"
echo "     · imu_topic  = /livox/imu"
echo "     · timestamp_unit 要对（本雷达是 float64 秒）"
echo "     详细见 docs/03_四算法手动启动与RViz.md"
echo
