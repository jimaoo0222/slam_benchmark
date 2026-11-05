#!/usr/bin/env bash
# 安装「点云查看器」的桌面图标 + 应用菜单项。
#
#   bash scripts/install_pcd_viewer_desktop.sh          # 安装
#   bash scripts/install_pcd_viewer_desktop.sh --remove # 卸载
#
# ⚠️ .desktop 的 Exec 字段不会展开 $HOME（由 desktop 规范决定），
#    所以这里必须写入【完全展开的绝对路径】。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$HERE/09_pcd_viewer.sh"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
APPS_DIR="$HOME/.local/share/applications"
TARGET_DESKTOP="$DESKTOP_DIR/点云查看器.desktop"
TARGET_APPS="$APPS_DIR/pcd-viewer.desktop"

if [ "${1:-}" = "--remove" ]; then
    rm -f "$TARGET_DESKTOP" "$TARGET_APPS"
    update-desktop-database "$APPS_DIR" 2>/dev/null || true
    echo "✅ 已卸载"
    exit 0
fi

[ -x "$LAUNCHER" ] || chmod +x "$LAUNCHER"
mkdir -p "$APPS_DIR" "$DESKTOP_DIR"

tmp="$(mktemp)"
cat > "$tmp" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=点云查看器
Name[en]=PCD Viewer
GenericName=Point Cloud Viewer
Comment=拖入 .pcd / .txt 即可 3D 查看点云和轨迹
Exec=bash "$LAUNCHER"
Path=$HERE/..
Icon=applications-graphics
Terminal=true
Categories=Graphics;
Keywords=pcd;pointcloud;lidar;slam;3d;点云;
StartupNotify=true
DESKTOP

install -m 755 "$tmp" "$TARGET_DESKTOP"
install -m 644 "$tmp" "$TARGET_APPS"
rm -f "$tmp"
update-desktop-database "$APPS_DIR" 2>/dev/null || true

echo "✅ 已安装"
echo "   桌面:     $TARGET_DESKTOP"
echo "   应用菜单: $TARGET_APPS"
echo
echo "   首次双击桌面图标如果提示「不受信任」，右键 → 允许启动"
