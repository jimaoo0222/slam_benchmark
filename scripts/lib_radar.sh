#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  按 bag 名字判断是哪个雷达录的，供各算法运行脚本 source
#
#  约定（本项目 8 个包）：
#     01_1 01_2 02_1 02_2  → mid360  （Livox MID-360s）
#     03_01 03_02 04_01 04_02 → c01   （SKYLANDX C01）
#
#  用法：
#     source scripts/lib_radar.sh
#     radar="$(radar_of_bag "$BAG")"      # 输出 mid360 / c01 / 空
#
#  判断依据是包的**前缀编号**，不依赖目录结构，所以
#  `rosbag/03_01`、`/data/20261102/03_01`、`rosbag/03_01.db3` 都能认。
# ══════════════════════════════════════════════════════════════

# 从路径里取出「包名」：末段若是 *.db3/*.mcap 或 metadata.yaml 就退到上一级
_bag_basename() {
    local p="${1%/}"
    case "$(basename "$p")" in
        *.db3|*.mcap|metadata.yaml|*.yaml) p="$(dirname "$p")" ;;
    esac
    basename "$p"
}

# 输出 mid360 / c01；认不出来输出空串（调用方自己决定回退到哪个配置）
radar_of_bag() {
    local name
    name="$(_bag_basename "${1:-}")"
    case "$name" in
        01_*|02_*) echo "mid360" ;;
        03_*|04_*) echo "c01"    ;;
        *)         echo ""       ;;
    esac
}

# 给日志用的一句人话
radar_desc() {
    case "${1:-}" in
        mid360) echo "Livox MID-360s" ;;
        c01)    echo "SKYLANDX C01"   ;;
        *)      echo "未知雷达"        ;;
    esac
}
