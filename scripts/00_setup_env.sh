#!/usr/bin/env bash
# =====================================================================
# 环境一次性修复脚本（团队成员在新机器上跑这个）
# =====================================================================
# 用法:
#     bash ~/slam_benchmark/scripts/00_setup_env.sh
#
# 幂等：可以重复运行，已修好的会跳过。
# 不需要 sudo。
# =====================================================================
set -u

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }
hdr()  { echo; echo "── $* ──"; }

echo "======================================================================"
echo " SLAM Benchmark 环境修复"
echo "======================================================================"

# ---------------------------------------------------------------------
hdr "1. Python 依赖"
# ---------------------------------------------------------------------
if python3 -c "import evo" 2>/dev/null; then
    ok "evo 已安装"
else
    warn "evo 未安装，正在安装..."
    pip3 install --user evo
fi

if python3 -c "import rosbags" 2>/dev/null; then
    ok "rosbags 已安装"
else
    warn "rosbags 未安装，正在安装..."
    pip3 install --user rosbags
fi

# ---------------------------------------------------------------------
hdr "2. conda 干扰"
# ---------------------------------------------------------------------
# conda base 自动激活会抢走 ROS 2 的 python，导致 rclpy / ament 导入失败。
if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    warn "当前处于 conda 环境 '${CONDA_DEFAULT_ENV}' —— 跑 ROS 2 前请先 conda deactivate"
    warn "永久关闭自动激活: conda config --set auto_activate_base false"
else
    ok "未激活 conda"
fi

# ---------------------------------------------------------------------
hdr "3. mpl_toolkits 冲突（导致 3D 绘图失效）"
# ---------------------------------------------------------------------
# 症状: ValueError: Unknown projection '3d'
# 原因: 系统的 matplotlib-*-nspkg.pth 在解释器启动时就把 mpl_toolkits 缓存
#       到了系统旧版路径。修法是让 ~/.local 的 mpl_toolkits 变成普通包。
FIXED=0
for d in "$HOME"/.local/lib/python3*/site-packages/mpl_toolkits; do
    [ -d "$d" ] || continue
    if [ -f "$d/__init__.py" ]; then
        ok "$d/__init__.py 已存在"
    else
        touch "$d/__init__.py" && ok "已创建 $d/__init__.py" && FIXED=1
    fi
done
[ "$FIXED" = 0 ] && ok "无需修复"

# ---------------------------------------------------------------------
hdr "4. 中文字体"
# ---------------------------------------------------------------------
CNT=$(fc-list :lang=zh 2>/dev/null | wc -l)
if [ "$CNT" -gt 0 ]; then
    ok "系统中文字体 $CNT 个"
else
    err "没有中文字体，图表中文会变方框"
    warn "安装: sudo apt install fonts-noto-cjk"
fi

# ---------------------------------------------------------------------
hdr "5. 自检"
# ---------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/../common/plot_setup.py"

# ---------------------------------------------------------------------
hdr "6. 磁盘"
# ---------------------------------------------------------------------
AVAIL=$(df -h "$HOME" | tail -1 | awk '{print $4}')
ok "可用空间: $AVAIL"
warn "数据集预算 64 GB 以内，注意定期清理中间产物"

# ---------------------------------------------------------------------
hdr "7. ROS 2 环境"
# ---------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
    ok "ROS 2 Humble 已安装"
else
    err "未找到 /opt/ros/humble"
fi

for p in slam-toolbox cartographer-ros; do
    if dpkg -s "ros-humble-$p" >/dev/null 2>&1; then
        ok "ros-humble-$p 已安装"
    else
        warn "ros-humble-$p 未安装 → sudo apt install ros-humble-$p"
    fi
done

# ---------------------------------------------------------------------
hdr "8. evalio（可选：现成的 8 个算法 + 12 个数据集）"
# ---------------------------------------------------------------------
# ⚠️ evalio 要求 Python >= 3.11；Ubuntu 22.04 自带的 3.10 装不了！
#    系统 python 还要留给 evo / rosbags，所以这里优先找一个高版本解释器。
EVALIO_PY=""
for cand in "$HOME/anaconda3/bin/python3" python3.13 python3.12 python3.11; do
    if command -v "$cand" >/dev/null 2>&1 && \
       "$cand" -c 'import sys; sys.exit(0 if sys.version_info >= (3,11) else 1)' 2>/dev/null; then
        EVALIO_PY="$cand"
        break
    fi
done

if python3 -c "import evalio" 2>/dev/null; then
    ok "evalio 已安装（当前 python3）"
elif [ -n "$EVALIO_PY" ] && "$EVALIO_PY" -c "import evalio" 2>/dev/null; then
    ok "evalio 已安装（$EVALIO_PY）"
elif [ -n "$EVALIO_PY" ]; then
    warn "evalio 未安装，用 $EVALIO_PY 安装..."
    "$EVALIO_PY" -m pip install --progress-bar off \
        -i https://mirrors.aliyun.com/pypi/simple/ evalio
else
    warn "没找到 Python >= 3.11 的解释器，evalio 装不了"
    warn "  evalio 是可选组件 —— 不装也能跑通全流程，只是少 8 个现成算法"
fi

echo
echo "======================================================================"
echo " 完成。"
echo "======================================================================"
echo
echo "提醒：所有画图脚本请这样开头 ——"
echo
echo "    import sys, pathlib"
echo "    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))"
echo "    from common import plot_setup"
echo "    import matplotlib.pyplot as plt"
echo
