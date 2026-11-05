#!/usr/bin/env bash
# 点云查看器启动器 —— 双击这个文件（或在终端跑）就能开。
#
# 它负责挑一个「装了 plotly 的 python」再启动查看器，
# 免得桌面双击时 PATH 里没有 conda、结果报 ModuleNotFoundError。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 候选解释器：把自己也带上，做到「哪个有 plotly 用哪个」
CANDIDATES=(
    "$HOME/anaconda3/bin/python3"
    "$(command -v python3 2>/dev/null || true)"
    "/usr/bin/python3"
)

PY=""
for p in "${CANDIDATES[@]}"; do
    [ -n "$p" ] && [ -x "$p" ] || continue
    if "$p" -c "import plotly" >/dev/null 2>&1; then
        PY="$p"
        break
    fi
done

if [ -z "$PY" ]; then
    echo "❌ 找不到装了 plotly 的 python3"
    echo "   装一下：  python3 -m pip install --user plotly"
    echo
    read -r -p "按回车关闭…" _ || true
    exit 1
fi

echo "使用解释器: $PY"
# -u：不缓冲输出，双击启动时能立刻看到地址
if "$PY" -c "import sys; sys.exit(0 if sys.version_info >= (3,7) else 1)" 2>/dev/null; then
    exec "$PY" -u "$HERE/09_pcd_viewer.py" "$@"
else
    exec "$PY" "$HERE/09_pcd_viewer.py" "$@"
fi
