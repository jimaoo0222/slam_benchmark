#!/usr/bin/env bash
# 录包一条命令：环境 → 话题自检 → 录 → 录完自动核对计数。
#
#   bash scripts/26_record_bag.sh c01_01
#   bash scripts/26_record_bag.sh c01_01 /livox/lidar /livox/imu /camera/image
#
# 不带包名时用时间戳。Ctrl+C 停止录包，核对表照常打印。
set -o pipefail

WS="$HOME/livox_ws"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_NAME="${1:-bag_$(date +%m%d_%H%M%S)}"
shift || true
TOPICS=("$@")
[ ${#TOPICS[@]} -eq 0 ] && TOPICS=(/livox/lidar /livox/imu)

BAG_DIR="$ROOT/rosbag/$BAG_NAME"

fail() { echo "❌ $*" >&2; exit 1; }

# ── 环境（剔掉 conda 的 python，否则 ROS 会去用 conda 的 3.13）──────────────
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
source /opt/ros/humble/setup.bash || fail "source /opt/ros/humble/setup.bash 失败"
[ -f "$WS/livox_env.sh" ] && source "$WS/livox_env.sh"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# ── 话题自检：驱动没起来就别录出一个空包 ──────────────────────────────────
echo "── 检查话题 ──"
AVAILABLE=$(ros2 topic list 2>/dev/null)
for t in "${TOPICS[@]}"; do
  grep -qx -- "$t" <<< "$AVAILABLE" || fail "话题 $t 不存在。驱动起了吗？（先跑 docs/06 或 docs/15 的启动步骤）"
  echo "  ✅ $t"
done

[ -e "$BAG_DIR" ] && fail "$BAG_DIR 已存在，换个包名"

# ── 录 ────────────────────────────────────────────────────────────────────
mkdir -p "$ROOT/rosbag"
echo
echo "── 开始录制 → rosbag/$BAG_NAME ──"
echo "   Ctrl+C 停止"
echo
ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}"
rc=$?
[ $rc -ne 0 ] && fail "ros2 bag record 退出码 $rc"

# ── 核对 ──────────────────────────────────────────────────────────────────
echo
echo "── 录制结果 ──"
python3 - "$BAG_DIR" <<'PY'
import sys, os
try:
    import yaml
except ImportError:
    yaml = None

bag = sys.argv[1]
meta = os.path.join(bag, "metadata.yaml")
if not os.path.isfile(meta):
    print("⚠️ 没有 metadata.yaml，包里可能一个消息都没有")
    sys.exit(0)

if yaml is None:
    os.system(f"ros2 bag info {bag}")
    sys.exit(0)

info = yaml.safe_load(open(meta))
node = (info.get("rosbag2_bagfile_information") or {})
dur_ns = node.get("duration", {}).get("nanoseconds", 0)
dur = dur_ns / 1e9
counts = {t.get("topic_metadata", {}).get("name"): t.get("message_count", 0)
          for t in node.get("topics_with_message_count", [])}
size_mb = os.path.getsize(os.path.join(bag, node.get("relative_file_paths", ["-"])[0])) / 1e6 \
    if node.get("relative_file_paths") else 0.0

print(f"   时长   {dur:.1f} s")
print(f"   体积   {size_mb:.1f} MB  （约 {size_mb/max(dur,1e-9):.1f} MB/s）")
print()
print(f"   {'话题':<24}{'消息数':>9}{'实测 Hz':>11}")
for t, c in counts.items():
    hz = c / dur if dur > 0 else 0
    flag = ""
    if t.endswith("/imu") and not 150 < hz < 250:
        flag = "  ⚠️ 期望 ~200 Hz"
    if t.endswith("/lidar") and not 8 < hz < 12:
        flag = "  ⚠️ 期望 ~10 Hz"
    print(f"   {t:<24}{c:>9}{hz:>11.1f}{flag}")
PY
