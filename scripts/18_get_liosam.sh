#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  一键装 LIO-SAM（MID-360 ROS2 版）
#
#  用法：
#     bash scripts/18_get_liosam.sh              # 装到 ~/liosam_ws
#     bash scripts/18_get_liosam.sh --check      # 只检查现状，不装
#
#  来源：https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG
#        （集成仓库，内含 autorccar_interfaces / livox_ros_driver2）
#
#  前置（必须已经做好）：
#     · ROS 2 Humble
#     · ~/livox_ws 里编译好的 livox_ros_driver2（scripts/02_get_livox.sh）
#     · ~/.local 里的 GTSAM（GLIM 装的时候已经装了；本项目实测 4.3a1 可用）
#
#  它会做 4 处必要的本地修补（上游代码原样是编不过的）：
#     1. autorccar_interfaces 里 BoundingBoxes.msg 依赖 vision_msgs（系统未装）
#        → 这个 msg 只用于发边界框，LIO-SAM 完全不用 → 裁掉，免 sudo 装包
#     2. 仓库自带的 livox_ros_driver2 与 ~/livox_ws 里的同名包冲突
#        → 加 COLCON_IGNORE，复用已编译好的那份
#     3. `boost::shared_ptr<gtsam::PreintegrationParams> p = ...::MakeSharedU(...)`
#        → GTSAM 4.2+ 的 MakeSharedU 返回 std::shared_ptr，改成 auto
#     4. 编译时强制用系统 boost 1.74（conda 的 boost 1.82 会混进来）
#
#  ⚠️ 运行时必须把 ~/.local/lib 放在 LD_LIBRARY_PATH 最前面，否则会命中
#     /usr/local/lib/libgtsam.so.4（旧版，boost 1.82 编译）→ 启动即挂
# ══════════════════════════════════════════════════════════════
set -uo pipefail

WS="$HOME/liosam_ws"
REPO="$WS/src/LIO-SAM_MID360_ROS2_PKG"
SRC="$REPO/ros2/src"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }
step() { echo; echo -e "${BLUE}── $* ──${NC}"; }

# ── 检查 ─────────────────────────────────────────────────────
if [ "$CHECK_ONLY" = "1" ]; then
    step "检查 LIO-SAM 安装状态"
    [ -d "$REPO" ] && ok "源码 $REPO" || warn "源码缺失 $REPO"
    [ -f "$WS/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization" ] \
        && ok "已编译" || warn "未编译"
    [ -f "$HOME/.local/lib/libgtsam.so.4.3a1" ] && ok "GTSAM 4.3a1 存在" || warn "GTSAM 缺失"
    [ -f "$HOME/livox_ws/install/setup.bash" ] && ok "livox_ros_driver2 存在" || warn "livox_ros_driver2 缺失"
    if [ -f "$WS/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization" ]; then
        # 必须 source 工作区，否则 liblio_sam__rosidl_typesupport_cpp.so 会被误报为找不到
        set +u
        [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
        [ -f "$HOME/livox_ws/install/setup.bash" ] && source "$HOME/livox_ws/install/setup.bash"
        [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
        set -u
        BAD=$(LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}" \
              ldd "$WS/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization" 2>&1 \
              | grep -E "not found|1\.82" | grep -v "rosidl_typesupport" | wc -l)
        [ "${BAD:-0}" -eq 0 ] && ok "动态库依赖干净" || warn "有 $BAD 个库找不到（见 docs/12）"
    fi
    exit 0
fi

echo "======================================================================"
echo " 安装 LIO-SAM（MID-360 ROS2 版）"
echo "   工作区: $WS"
echo "======================================================================"

# ── 0. 前置检查 ──────────────────────────────────────────────
step "0. 前置检查"
[ -f /opt/ros/humble/setup.bash ] || { err "没找到 ROS 2 Humble"; exit 1; }
[ -f "$HOME/livox_ws/install/setup.bash" ] || {
    err "缺 livox_ros_driver2，先跑：bash scripts/02_get_livox.sh"; exit 1; }
[ -d "$HOME/.local/lib/cmake/GTSAM" ] || {
    err "缺 GTSAM（GLIM 装的时候装过），先看 docs/03_四算法手动启动与RViz.md"; exit 1; }
ok "前置齐全"

# ── 1. 取源码 ────────────────────────────────────────────────
step "1. 取源码"
if [ -d "$REPO/.git" ]; then
    ok "已存在，跳过 clone（要更新请手动 git pull）"
else
    mkdir -p "$WS/src"
    git clone --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/UV-Lab/LIO-SAM_MID360_ROS2_PKG.git "$REPO" 2>&1 | tail -5
    [ -d "$REPO/ros2/src/LIO-SAM_MID360_ROS2" ] || { err "clone 失败"; exit 1; }
    ok "已下载"
fi

# ── 2. 修补 ①：裁掉 vision_msgs 依赖 ─────────────────────────
step "2. 修补 ①  裁掉 vision_msgs 依赖"
AI="$SRC/autorccar_interfaces"
if [ -f "$AI/msg/BoundingBoxes.msg" ]; then
    mkdir -p "$AI/disabled"
    mv "$AI/msg/BoundingBoxes.msg" "$AI/disabled/"
    sed -i 's|^\(\s*\)"msg/BoundingBoxes.msg"|\1# "msg/BoundingBoxes.msg"   # 需 vision_msgs，LIO-SAM 不用|' \
        "$AI/CMakeLists.txt"
    sed -i 's|DEPENDENCIES std_msgs geometry_msgs vision_msgs|DEPENDENCIES std_msgs geometry_msgs|' \
        "$AI/CMakeLists.txt"
    sed -i 's|^find_package(vision_msgs REQUIRED)|# find_package(vision_msgs REQUIRED)   # 见上|' \
        "$AI/CMakeLists.txt"
    sed -i 's|<depend>vision_msgs</depend>|<!-- vision_msgs 已裁掉，仅 BoundingBoxes.msg 需要 -->|' \
        "$AI/package.xml"
    ok "已裁掉（原文件移到 disabled/）"
else
    ok "已处理过"
fi

# ── 3. 修补 ②：忽略仓库自带的 livox_ros_driver2 ──────────────
step "3. 修补 ②  忽略仓库自带的 livox_ros_driver2"
if [ ! -f "$SRC/livox_ros_driver2/COLCON_IGNORE" ]; then
    touch "$SRC/livox_ros_driver2/COLCON_IGNORE"
    ok "已加 COLCON_IGNORE（复用 ~/livox_ws 里已编译的那份）"
else
    ok "已处理过"
fi

# ── 4. 修补 ③：GTSAM 4.2+ 的 shared_ptr ──────────────────────
step "4. 修补 ③  GTSAM 4.2+ shared_ptr 兼容"
IP="$SRC/LIO-SAM_MID360_ROS2/src/imuPreintegration.cpp"
if grep -q "boost::shared_ptr<gtsam::PreintegrationParams>" "$IP" 2>/dev/null; then
    sed -i 's|boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);|auto p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);  // GTSAM 4.2+ 返回 std::shared_ptr|' "$IP"
    ok "已改为 auto"
else
    ok "已处理过"
fi

# ── 5. 编译 ──────────────────────────────────────────────────
step "5. 编译（系统 boost 1.74）"
cd "$WS" || exit 1
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
set -u
export CMAKE_PREFIX_PATH="$HOME/.local:/opt/ros/humble"

colcon build --packages-up-to lio_sam --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBoost_NO_BOOST_CMAKE=ON \
    -DBOOST_ROOT=/usr \
    -DBOOST_INCLUDEDIR=/usr/include \
    -DBOOST_LIBRARYDIR=/usr/lib/x86_64-linux-gnu \
    2>&1 | tail -15

step "6. 验证动态库"
EXE="$WS/install/lio_sam/lib/lio_sam/lio_sam_mapOptimization"
if [ -f "$EXE" ]; then
    set +u; source "$WS/install/setup.bash"; set -u
    BAD=$(LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}" ldd "$EXE" 2>&1 \
          | grep -cE "1\.82|not found")
    if [ "${BAD:-1}" -eq 0 ]; then
        ok "全部依赖可解析"
    else
        warn "$BAD 个库有问题，多半是 /usr/local/lib/libgtsam.so.4 抢了先"
        echo "    运行时务必： export LD_LIBRARY_PATH=\"\$HOME/.local/lib:\$LD_LIBRARY_PATH\""
    fi
else
    err "编译产物缺失，看上面的错误"
    exit 1
fi

echo
echo "======================================================================"
ok "装好了"
echo
echo "  跑一个 bag："
echo "    bash scripts/17_run_liosam_on_bag.sh rosbag/01_1 results/liosam_test"
echo "======================================================================"
