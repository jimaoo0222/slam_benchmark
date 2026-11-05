#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════
#  Livox MID-360s 驱动一键安装（ROS 2 Humble）
#
#  用法：
#     bash scripts/02_get_livox.sh --lidar-ip 192.168.1.1XX
#     bash scripts/02_get_livox.sh --lidar-ip 192.168.1.1XX --net   # 顺便配网卡
#     bash scripts/02_get_livox.sh --lidar-ip 192.168.1.3 --device mid360s
#
#  ⚠️ 两颗雷达的网段不同，切雷达时 IP 要一起换：
#     MID-360s  主机 192.168.1.50   雷达 192.168.1.3      （Livox 出厂）
#     C01       主机 192.168.100.150 雷达 192.168.100.200 （官方驱动默认）
#     端口号两侧完全相同（56300->56301 / 56400->56401）。
#
#     C01 的两条命令：
#       bash scripts/02_get_livox.sh --lidar-ip 192.168.100.200 \
#                                    --host-ip  192.168.100.150 --net
#       # 若驱动已装好、只想换 IP，可加 --skip-build
#
#  做完这些事：
#     1. 下载 Livox-SDK2  → ~/livox_sdk2
#     2. 编译安装 SDK     → ~/.local（不需要 sudo）
#     3. 下载驱动         → ~/livox_ws/src/livox_ros_driver2
#     4. 写 <设备>_config.json（host_ip / 雷达 ip）
#     5. colcon build
#     6. 生成 ~/livox_ws/livox_env.sh（CMAKE_PREFIX_PATH / LD_LIBRARY_PATH）
#
#  ⚠️ 三个已知坑（本脚本已处理，手工装的话一定会遇到）：
#     · conda 的 python 抢进 colcon → rosidl 报 "No module named 'em'"
#       脚本编译时会临时把 anaconda 从 PATH 里摘掉
#     · SDK 装在 ~/.local，不在 ldconfig 路径里 → 跑之前必须
#       source ~/livox_ws/livox_env.sh，否则组件加载失败
#     · **设备类型必须和雷达自报的 dev_type 一致**（段名 MID360=9 / Mid360s=35）。
#       对不上时 SDK 在 device_manager.cpp 里直接 return，**一行日志都不打**，
#       日志就停在 `Init lds lidar success!`，看起来像网线没插好。
#       用 scripts/03_livox_discover.py 查真实类型（会打印 dev_type）。
#
#  ⚠️ 跑之前务必关掉 Livox Viewer 2：它占着 UDP 56000/56301/56401，而 SDK 建 socket 时
#     带了 SO_REUSEADDR，于是「绑定成功但一个包都收不到」—— 同样是完全静默。
# ══════════════════════════════════════════════════════════════
set -uo pipefail

WS="$HOME/livox_ws"
SDK_SRC="$HOME/livox_sdk2"
PREFIX="$HOME/.local"
HOST_IP="192.168.1.50"          # 手册规定的 PC 侧静态 IP
LIDAR_IP=""
IFACE="eno1"
DO_NET=0
SKIP_BUILD=0
DEVICE="mid360"                 # 见下面 case：mid360 / mid360s / avia2 / mid360l / hap

while [ $# -gt 0 ]; do
    case "$1" in
        --lidar-ip)  LIDAR_IP="$2"; shift 2 ;;
        --host-ip)   HOST_IP="$2";  shift 2 ;;
        --iface)     IFACE="$2";    shift 2 ;;
        --ws)        WS="$2";       shift 2 ;;
        --prefix)    PREFIX="$2";   shift 2 ;;
        --device)    DEVICE="$2";   shift 2 ;;
        --net)       DO_NET=1;      shift ;;
        --skip-build) SKIP_BUILD=1; shift ;;   # 只改 IP / 网卡，不重新编译
        -h|--help)   sed -n '2,40p' "$0"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 段名和文件名大小写不一致（AVIA2_config.json 里的段名是 Avia2），所以分开写
case "$DEVICE" in
    mid360)  CONF_FILE="MID360_config.json";  SECTION="MID360";  LAUNCH="msg_MID360_launch.py" ;;
    mid360s) CONF_FILE="MID360s_config.json"; SECTION="Mid360s"; LAUNCH="msg_MID360s_launch.py" ;;
    avia2)   CONF_FILE="AVIA2_config.json";   SECTION="Avia2";   LAUNCH="msg_AVIA2_launch.py" ;;
    mid360l) CONF_FILE="MID360l_config.json"; SECTION="Mid360l"; LAUNCH="msg_MID360l_launch.py" ;;
    hap)     CONF_FILE="HAP_config.json";     SECTION="HAP";     LAUNCH="msg_HAP_launch.py" ;;
    *) echo "未知 --device: $DEVICE（可选 mid360 / mid360s / avia2 / mid360l / hap）"; exit 1 ;;
esac

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "  ${GREEN}✅${NC} $*"; }
warn() { echo -e "  ${YELLOW}⚠️ ${NC} $*"; }
err()  { echo -e "  ${RED}❌${NC} $*"; }

echo "======================================================================"
echo " Livox MID-360s 驱动安装"
echo "   工作区: $WS     SDK: $SDK_SRC     prefix: $PREFIX"
echo "======================================================================"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    err "找不到 /opt/ros/humble —— 先装 ROS 2 Humble"
    exit 1
fi

# ── 0. 配网卡（可选）────────────────────────────────────────
if [ "$DO_NET" = 1 ]; then
    echo
    echo "── 0. 配网卡 $IFACE = $HOST_IP/24（需要 sudo）──"
    # profile 名带网段 —— MID-360(192.168.1.x) 和 C01(192.168.100.x) 各自一个，互不覆盖
    PROF="livox-$(echo "$HOST_IP" | cut -d. -f1-3 | tr . -)"
    if nmcli device status 2>/dev/null | grep -q "^$IFACE"; then
        sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$PROF" \
             ipv4.method manual ipv4.addresses "$HOST_IP/24" \
             ipv4.never-default yes connection.autoconnect yes 2>/dev/null \
            || sudo nmcli connection modify "$PROF" \
                 ipv4.method manual ipv4.addresses "$HOST_IP/24"
        # 别的有线 profile 会来抢网卡，关掉它们的自动连接
        while IFS=: read -r name type; do
            [ "$type" = "802-3-ethernet" ] || continue
            [ "$name" = "$PROF" ] || case "$name" in livox-*) continue ;; esac
            sudo nmcli connection modify "$name" connection.autoconnect no \
                && warn "已关闭「$name」的自动连接"
        done < <(nmcli -t -f NAME,TYPE connection show)
        ok "已建 profile $PROF（插上网线自动生效）"
        sudo nmcli connection up "$PROF" >/dev/null 2>&1 \
            && ok "已启用（网线未插时显示不可用是正常的）" \
            || warn "网线未插 —— 插上后会自动生效"
    else
        sudo ip addr add "$HOST_IP/24" dev "$IFACE" 2>/dev/null
        sudo ip link set "$IFACE" up
        ok "已用 ip 命令配置（重启失效）"
    fi
fi

# ── 1. SDK 源码 ────────────────────────────────────────────
echo
echo "── 1. Livox-SDK2 源码 ──"
if [ -f "$SDK_SRC/CMakeLists.txt" ]; then
    ok "已存在: $SDK_SRC"
else
    rm -rf "$SDK_SRC"
    for i in 1 2 3; do
        git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$SDK_SRC" && break
        warn "第 $i 次 clone 失败，重试..."
        rm -rf "$SDK_SRC"; sleep 2
    done
fi
[ -f "$SDK_SRC/CMakeLists.txt" ] || { err "SDK 下载失败"; exit 1; }

# ── 2. 编译安装 SDK ────────────────────────────────────────
echo
echo "── 2. 编译安装 SDK → $PREFIX ──"
if [ -f "$PREFIX/lib/liblivox_lidar_sdk_shared.so" ]; then
    ok "SDK 已安装，跳过"
else
    mkdir -p "$SDK_SRC/build"
    ( cd "$SDK_SRC/build" \
      && cmake .. -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_BUILD_TYPE=Release >/dev/null \
      && make -j"$(nproc)" >/dev/null \
      && make install >/dev/null ) \
        && ok "已安装 liblivox_lidar_sdk_shared.so + 头文件" \
        || { err "SDK 编译失败"; exit 1; }
fi

# ── 3. 驱动源码 ────────────────────────────────────────────
echo
echo "── 3. livox_ros_driver2 源码 ──"
mkdir -p "$WS/src"
if [ -f "$WS/src/livox_ros_driver2/package_ROS2.xml" ]; then
    ok "已存在: $WS/src/livox_ros_driver2"
else
    rm -rf "$WS/src/livox_ros_driver2"
    for i in 1 2 3; do
        git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
            "$WS/src/livox_ros_driver2" && break
        warn "第 $i 次 clone 失败，重试..."
        rm -rf "$WS/src/livox_ros_driver2"; sleep 2
    done
fi
DRIVER="$WS/src/livox_ros_driver2"
[ -f "$DRIVER/package_ROS2.xml" ] || { err "驱动下载失败"; exit 1; }
cp -f "$DRIVER/package_ROS2.xml" "$DRIVER/package.xml"
ok "package.xml 已就位（ROS2 版）"

# ── 4. 写配置 ──────────────────────────────────────────────
echo
echo "── 4. 写 $CONF_FILE（设备类型 $DEVICE → 段名 $SECTION）──"
CFG="$DRIVER/config/$CONF_FILE"
if [ ! -f "$CFG" ]; then
    err "找不到 $CFG —— 驱动版本可能不含这个设备的配置"
    exit 1
fi
if [ -n "$LIDAR_IP" ]; then
    if python3 - "$CFG" "$LIDAR_IP" "$HOST_IP" "$SECTION" <<'PY'
import json, sys
path, lidar_ip, host_ip, section = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
with open(path) as f:
    cfg = json.load(f)

# 段名 = 设备类型，大小写敏感。对不上时 SDK 会静默丢弃雷达（一行日志都不打），
# 所以这里顺手把写错的段名改回来。
for key in ("MID360", "Mid360s", "Avia2", "Mid360l", "HAP"):
    val = cfg.get(key)
    if key != section and isinstance(val, dict) and "host_net_info" in val:
        cfg.pop(key)
        cfg[section] = val
        print(f"  段名 {key} → {section}")
        break

section_cfg = cfg.get(section)
if not isinstance(section_cfg, dict) or "host_net_info" not in section_cfg:
    sys.exit(f"配置文件里找不到 {section} 段")

hni = section_cfg["host_net_info"]
if isinstance(hni, list):          # 新版格式：host_net_info 是列表
    for item in hni:
        item["host_ip"] = host_ip
else:                              # 老版格式：每个流一个 *._ip 字段
    for k in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        hni[k] = host_ip

if cfg.get("lidar_configs"):
    cfg["lidar_configs"][0]["ip"] = lidar_ip

with open(path, "w") as f:
    json.dump(cfg, f, indent=2)
PY
    then
        ok "配置已写入（雷达 $LIDAR_IP / PC $HOST_IP）"
    else
        err "写配置失败"; exit 1
    fi
    # 直接同步到 install 目录，省得为了改 IP 重新编译
    if [ -d "$WS/install/livox_ros_driver2/share/livox_ros_driver2/config" ]; then
        cp -f "$CFG" "$WS/install/livox_ros_driver2/share/livox_ros_driver2/config/"
        ok "已同步到 install/"
    fi
else
    warn "没给 --lidar-ip —— 配置里还是示例 IP，雷达连不上"
    warn "先跑：python3 scripts/03_livox_discover.py  找到雷达 ip 和设备类型，再带 --lidar-ip 重跑本脚本"
fi

# ── 5. 生成 env ────────────────────────────────────────────
cat > "$WS/livox_env.sh" <<EOF
# source 这个文件后再跑驱动（SDK 装在 $PREFIX，不在 ldconfig 路径里）
export CMAKE_PREFIX_PATH="$PREFIX:\$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PREFIX/lib:\$LD_LIBRARY_PATH"
EOF
ok "已生成 $WS/livox_env.sh"

# ── 6. colcon build ────────────────────────────────────────
if [ "$SKIP_BUILD" = 1 ]; then
    echo
    echo "── 5. 编译驱动（--skip-build，跳过）──"
    warn "已跳过 colcon build。配置已同步到 install/，可直接起驱动"
    exit 0
fi
echo
echo "── 5. 编译驱动 ──"
# conda 的 python3 会顶掉 ROS 的 python3.10，rosidl 就找不到 empy
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd:)
# ROS 的 setup.bash 自己会引用未定义变量，先关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export CMAKE_PREFIX_PATH="$PREFIX:${CMAKE_PREFIX_PATH:-}"
cd "$WS" || exit 1
if colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release >/tmp/livox_build.log 2>&1; then
    ok "编译完成（日志 /tmp/livox_build.log）"
else
    err "编译失败，日志尾部："
    tail -20 /tmp/livox_build.log
    exit 1
fi

echo
echo "======================================================================"
echo " 完成！"
echo "======================================================================"
cat <<EOF

接硬件（一次）：
  1. 网线插 $IFACE，雷达上电（9~27 V）
  2. 查雷达 ip + 设备类型：
       python3 ~/slam_benchmark/scripts/03_livox_discover.py
       # 输出里的 dev_type 要和这里的 --device $DEVICE 对上
  3. 忘了带 --lidar-ip？或者 dev_type 对不上？重跑：
       bash ~/slam_benchmark/scripts/02_get_livox.sh --lidar-ip 192.168.1.3
       bash ~/slam_benchmark/scripts/02_get_livox.sh --lidar-ip 192.168.1.3 --device mid360s

跑起来（两个终端）：

  终端 A  驱动 + 看话题
      conda deactivate
      source /opt/ros/humble/setup.bash
      source $WS/livox_env.sh
      source $WS/install/setup.bash
      ros2 launch livox_ros_driver2 $LAUNCH
      # 另开终端： ros2 topic hz /livox/lidar        ← 应是 10 Hz
      #           ros2 topic hz /livox/imu          ← 应是 200 Hz

  终端 B  录包（评测要的就是它）
      ros2 bag record -o run1 /livox/lidar /livox/imu

  ⚠️ 跑驱动前先关掉 Livox Viewer 2！（它占着 UDP 56000/56301/56401，
     会让 SDK「绑定成功但收不到包」，日志停在 Init lds lidar success! 就没下文了）

  ⚠️ msg_ 开头那个 launch 发的是 livox_ros_driver2/CustomMsg（FAST-LIO 要的格式）；
     rviz_ 开头那个发 PointCloud2，方便肉眼看点云。

想让每个新终端都自动带上环境（可选）：
      echo "source $WS/livox_env.sh" >> ~/.bashrc

EOF
