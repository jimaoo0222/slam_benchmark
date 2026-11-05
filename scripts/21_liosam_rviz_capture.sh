#!/usr/bin/env bash
# 跑 LIO-SAM + RViz，并在过程中截图（用于文档 / 出报告）
# 用法： bash scripts/21_liosam_rviz_capture.sh <bag> <输出目录> [倍速]
set -uo pipefail

ROOT="$HOME/slam_benchmark"
BAG="${1:-rosbag/01_1}"
OUT="${2:-/tmp/liosam_shots}"
RATE="${3:-2.0}"

mkdir -p "$OUT"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX
set +u
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/liosam_ws/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

LIO_PAT="lio_sam_(imageProjection|featureExtraction|mapOptimization|imuPreintegration)"
BEFORE=$(ps -eo pid,args --no-headers 2>/dev/null | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}' | tr '\n' ' ')

cd "$ROOT" || exit 1
/usr/bin/python3 scripts/16_pc2_to_custom.py > /tmp/cap_conv.log 2>&1 &
CONV=$!
sleep 3

ros2 launch scripts/liosam/run_headless.launch.py \
    params_file:="$ROOT/config/liosam_mid360.yaml" \
    rviz:=true rviz_cfg:="$ROOT/scripts/rviz/liosam_ros2.rviz" \
    > /tmp/cap_launch.log 2>&1 &
LAUNCH=$!

for _ in $(seq 1 60); do
    [ "$(ros2 node list 2>/dev/null | grep -c '^/lio_sam_')" -ge 4 ] && \
    [ "$(ros2 node list 2>/dev/null | grep -c '^/rviz2')" -ge 1 ] && break
    sleep 0.5
done
echo "  节点就绪: $(ros2 node list 2>/dev/null | grep -cE 'lio_sam_|rviz2') 个"

ros2 bag play "$BAG" -r "$RATE" > /tmp/cap_play.log 2>&1 &

# 定时截图：等 RViz 建立渲染再开始
sleep 12
for i in 1 2 3 4 5 6; do
    # 只截 RViz 主窗口
    #  · 必须用 -tree：RViz 主窗口嵌在容器窗口里，-children 看不到
    #  · WM_CLASS 是小写 ("rviz2" "rviz2")，且标题以 " - RViz" 结尾
    #  · 不能只 grep 'rviz' —— VS Code 标题里可能也有 "RViz" 字样
    RVWID=$(xwininfo -root -tree 2>/dev/null \
        | grep -E '"rviz2" "rviz2"' | grep -E '\- RViz"' \
        | sed -E 's/^\s*(0x[0-9a-f]+).*[^0-9]([0-9]+)x([0-9]+)\+.*/\1 \2 \3/' \
        | awk 'NF==3 {if ($2*$3 > best) {best=$2*$3; wid=$1}} END {print wid}')
    if [ -n "$RVWID" ]; then
        timeout 20 xwd -id "$RVWID" -silent 2>/dev/null > "$OUT/raw_$i.xwd"
    else
        timeout 20 xwd -root -silent 2>/dev/null > "$OUT/raw_$i.xwd"
    fi
    echo "  截图 $i: $(du -h "$OUT/raw_$i.xwd" 2>/dev/null | cut -f1)  (窗口 ${RVWID:-root})"
    # 同时记录建图进度
    N=$(ros2 topic echo /lio_sam/mapping/path --once --field poses 2>/dev/null | grep -c "position" || echo "?")
    echo "    path 位姿数 ≈ $N" >> /tmp/cap_progress.txt
    sleep 14
done

wait %3 2>/dev/null
sleep 3
kill -INT "$LAUNCH" 2>/dev/null
sleep 4
for p in $(ps -eo pid,args --no-headers 2>/dev/null | grep -E "$LIO_PAT" | grep -v grep | awk '{print $1}'); do
    case " $BEFORE " in *" $p "*) ;; *) kill -INT "$p" 2>/dev/null ;; esac
done
kill -INT "$CONV" 2>/dev/null
sleep 2
echo "  完成，图在 $OUT"
