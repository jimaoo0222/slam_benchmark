#!/usr/bin/env bash
# OpenGL 点云查看器启动器
#
#   bash scripts/10_pcd_viewer_gl.sh map.pcd [traj.txt]
#
# ⚠️ 两个必须由它代劳的事：
#   1. 用【系统 python3】，不是 conda 的
#      —— conda 的老 libstdc++ 缺 GLIBCXX_3.4.30，mesa 驱动 radeonsi_dri.so
#         加载失败，OpenGL 直接不可用（报 "GLXBadFBConfig"）
#   2. 清掉 LD_LIBRARY_PATH
#      —— source 过 ROS 之后里面有 rviz 的 OGRE 库，会抢系统 GL 库
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=/usr/bin/python3

[ -x "$PY" ] || { echo "❌ 找不到 $PY"; exit 1; }

if ! "$PY" -c "import OpenGL, glfw" >/dev/null 2>&1; then
    echo "❌ 系统 python3 缺依赖，装一下（约 2 MB，装到 ~/.local）："
    echo "   /usr/bin/python3 -m pip install --user PyOpenGL glfw"
    exit 1
fi

# 关键：清掉 LD_LIBRARY_PATH，否则 ROS 的 OGRE 库会干扰 GL
exec env -u LD_LIBRARY_PATH "$PY" -u "$HERE/10_pcd_viewer_gl.py" "$@"
