#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  在 RViz 里看 LIO-SAM 建图过程（任何目录都能跑）
#
#  用法：
#     bash ~/slam_benchmark/scripts/20_liosam_rviz_demo.sh
#     bash ~/slam_benchmark/scripts/20_liosam_rviz_demo.sh rosbag/01_1
#     bash ~/slam_benchmark/scripts/20_liosam_rviz_demo.sh rosbag/01_1 2.0   # 2 倍速
#
#  它会做：
#     1. 判断 bag 点云是 PointCloud2 还是 CustomMsg（前者自动起转换节点）
#     2. 起 LIO-SAM 四个节点 + RViz（用项目内的 scripts/rviz/liosam_ros2.rviz）
#     3. 回放 bag，你在 RViz 里看着走
#     4. Ctrl+C 收尾
#
#  RViz 里会看到 4 个东西（左侧 Displays 面板）：
#     · 当前帧点云  /lio_sam/mapping/cloud_registered   按 z 高度彩虹着色
#     · 全局地图    /lio_sam/mapping/map_global         越走越密
#     · Path        /lio_sam/mapping/path              红色轨迹线
#     · MarkerArray /lio_sam/mapping/loop_closure_constraints  回环约束（走回起点时才出现）
#
#  ⚠️ 两个必须知道的事：
#     · `~/.local/lib` 必须在 LD_LIBRARY_PATH 最前面，否则 LIO-SAM 起不来
#       （/usr/local 有个旧 GTSAM 用 conda boost 1.82 编译，见 docs/12 坑③）
#     · 不要加 `--clock`：LIO-SAM 用 header.stamp，不依赖 /clock
# ══════════════════════════════════════════════════════════════
set -uo pipefail

ROOT="$HOME/slam_benchmark"
BAG="${1:-rosbag/01_1}"
RATE="${2:-1.0}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; BLUE='\033[0;34m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }
step() { echo; echo -e "${BLUE}── $* ──${NC}"; }

# 转换节点是 Python 实现，单帧约 10~20 ms；超过 2 倍速会跟不上而丢帧
# （实测 3 倍速只转出约 200/710 帧）。评测请用 17 脚本的 1 倍速。
if awk "BEGIN{exit !($RATE > 2.0)}"; then
    warn "${RATE} 倍速会丢帧（转换节点跟不上），演示够用、评测请用 1 倍速 + scripts/17"
fi

# ── 1. 检查 ──────────────────────────────────────────────────
step "1. 检查环境"
cd "$ROOT" || { err "找不到 $ROOT"; exit 1; }
[ -d "$BAG" ] || { err "bag 不存在: $BAG（当前目录 $PWD）"; exit 1; }
[ -n "${DISPLAY:-}" ] || { err "DISPLAY 未设置 —— RViz 需要图形界面"; exit 1; }
[ -f "$HOME/liosam_ws/install/setup.bash" ] || {
    err "LIO-SAM 未编译，先跑 bash scripts/18_get_liosam.sh"; exit 1; }
RVCFG="$ROOT/scripts/rviz/liosam_ros2.rviz"
[ -f "$RVCFG" ] || { err "缺 RViz 配置: $RVCFG"; exit 1; }
ok "环境 OK（DISPLAY=$DISPLAY，倍速 $RATE）"

# ── 2. 环境准备 ──────────────────────────────────────────────
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/liosam_ws/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

# 记录启动前已有的 lio_sam 进程，收尾时只杀新增的
LIO_PAT="lio_sam_(imageProjection|featureExtraction|mapOptimization|imuPreintegration)"
BEFORE_PIDS=$(ps -eo pid,args --no-headers 2>/dev/null \
    | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}' | sort | tr '\n' ' ')

# ── 3. 判断点云格式 ──────────────────────────────────────────
step "2. 检查 bag 的点云类型"
BAG_TYPE=$(ros2 bag info "$BAG" 2>/dev/null \
    | grep -E "Topic: /livox/lidar " | grep -oE "(CustomMsg|PointCloud2)" | head -1)
if [ "$BAG_TYPE" = "CustomMsg" ]; then
    ok "已是 CustomMsg → 无需转换"; NEED_CONV=0
else
    ok "PointCloud2 → 自动起转换节点"; NEED_CONV=1
fi

CONV_PID=""
if [ "$NEED_CONV" = "1" ]; then
    /usr/bin/python3 "$ROOT/scripts/16_pc2_to_custom.py" > /tmp/liosam_rviz_conv.log 2>&1 &
    CONV_PID=$!
    sleep 3
    ps -p "$CONV_PID" > /dev/null 2>&1 \
        && ok "转换节点就绪" \
        || { err "转换节点没起来"; tail -10 /tmp/liosam_rviz_conv.log; exit 1; }
fi

# ── 4. 起算法 + RViz ────────────────────────────────────────
step "3. 起 LIO-SAM + RViz"
echo "      （RViz 窗口大约 8 秒后出现，请稍等）"
ros2 launch "$ROOT/scripts/liosam/run_headless.launch.py" \
    params_file:="$ROOT/config/liosam_mid360.yaml" \
    rviz:=true rviz_cfg:="$RVCFG" \
    > /tmp/liosam_rviz_launch.log 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 50); do
    N=$(ros2 node list 2>/dev/null | grep -c "^/lio_sam_" || true)
    [ "${N:-0}" -ge 4 ] && break
    sleep 0.5
done
N=$(ros2 node list 2>/dev/null | grep -c "^/lio_sam_" || true)
if [ "${N:-0}" -lt 4 ]; then
    err "LIO-SAM 只起来 ${N:-0}/4 个节点"
    tail -20 /tmp/liosam_rviz_launch.log
    [ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
    kill -INT "$LAUNCH_PID" 2>/dev/null
    exit 1
fi
ok "四个节点 + RViz 就绪"

# ── 5. 回放 ──────────────────────────────────────────────────
DUR=$(ros2 bag info "$BAG" 2>/dev/null | grep -oE 'Duration:[[:space:]]+[0-9.]+' | grep -oE '[0-9.]+')
step "4. 开始回放（${DUR:-?} 秒，${RATE} 倍速）"
echo "      看完按 Ctrl+C 结束"
echo
ros2 bag play "$BAG" -r "$RATE" || warn "回放异常结束"
ok "回放结束"

# ── 6. 收尾 ──────────────────────────────────────────────────
step "5. 收尾"
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 4
for p in $(ps -eo pid,args --no-headers 2>/dev/null \
           | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}'); do
    case " $BEFORE_PIDS " in *" $p "*) ;; *) kill -INT "$p" 2>/dev/null ;; esac
done
[ -n "$CONV_PID" ] && kill -INT "$CONV_PID" 2>/dev/null
sleep 3
LEFT=$(ps -eo pid,args --no-headers 2>/dev/null | grep -E "$LIO_PAT" | grep -v grep \
       | awk '{print $1}' | while read -r p; do
           case " $BEFORE_PIDS " in *" $p "*) ;; *) echo "$p" ;; esac
       done | wc -l)
[ "${LEFT:-0}" -gt 0 ] && warn "还有 $LEFT 个节点未退出" || ok "已全部退出"
echo
echo "  想把这个 bag 的结果留下来算精度？跑："
echo "    bash scripts/17_run_liosam_on_bag.sh $BAG results/liosam_\$(basename $BAG)"
