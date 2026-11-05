#!/usr/bin/env python3
"""找出网线那头的 Livox 雷达：IP、序列号、设备类型（主动探测）。

原理：Livox SDK 每秒从 <host_ip>:56000 往 255.255.255.255:56000 广播一个
`kCommandIDLidarSearch`(0x0000) 包；雷达收到后从自己的 :56000 回一个 48 字节应答，
里面带 **SN / 设备类型 / 自己的 IP / 命令端口**。

本脚本直接复刻这个包（包头 18 字节做 CRC16-CCITT-FALSE），所以：
    · 不需要驱动在跑，也不需要 Livox Viewer
    · 能直接读出「设备类型」—— 这一步很关键，见下面的坑

⚠️ 三个会让你白忙一小时的坑，本脚本已全部处理（手工排查时一定会撞上）：
    1. 端口被抢：Livox Viewer 2 或上一次没退干净的驱动占着 56000/56301/56401 时，
       SDK 因为设了 SO_REUSEADDR 会**绑定成功但一个包都收不到**，日志还停在
       `Init lds lidar success!` —— 完全静默。
    2. 设备类型对不上：驱动配置的段名（`MID360` → 9，`Mid360s` → 35）必须和雷达
       自报的 dev_type 一致，否则 SDK 在 device_manager.cpp 里直接 `return`，
       连一行日志都不打。
    3. 广播走错网卡 + 应答是广播：发包必须绑网卡 IP（绑 0.0.0.0 时去
       255.255.255.255 的路由会选中默认路由 = WiFi）；而雷达的应答是**广播**回的，
       所以收包得绑 255.255.255.255（绑具体 IP 收不到广播）。

用法：
    python3 scripts/03_livox_discover.py                  # 主动探测 10 秒
    python3 scripts/03_livox_discover.py -t 20 -i eno1
    python3 scripts/03_livox_discover.py --host-ip 192.168.1.50
    python3 scripts/03_livox_discover.py --passive        # 只被动听（不发包）

前置条件：
    · 网线插好、雷达上电（9~27 V）
    · PC 网卡已设成静态 IP 192.168.1.50/24（bash scripts/02_get_livox.sh --net）
"""

from __future__ import annotations

import argparse
import re
import select
import socket
import subprocess
import sys
import time

DETECTION_PORT = 56000
SOF = 0xAA
SDK_VERSION = 0  # kSdkVer0
PACKET_LEN = 24  # data_len(0) + sizeof(SdkPacket) - 1

# 设备类型（sdk_core/comm/define.h 的 LivoxLidarDeviceType）
DEV_TYPES = {
    0: "Hub", 1: "Mid40", 2: "Tele", 3: "Horizon", 6: "Mid70", 7: "Avia",
    9: "MID360", 10: "IndustrialHAP", 15: "HAP", 16: "PA",
    35: "Mid360s", 40: "Avia2", 41: "Mid360l",
}
# 设备类型 → 驱动配置段名（= config/<段名>_config.json 的顶层 key，大小写敏感！）
DEV_SECTION = {9: "MID360", 35: "Mid360s", 40: "Avia2", 41: "Mid360l", 15: "HAP", 10: "HAP"}
# 设备类型 → 02_get_livox.sh --device 的取值
DEV_ARG = {9: "mid360", 35: "mid360s", 40: "avia2", 41: "mid360l", 15: "hap", 10: "hap"}

# 被动模式监听的端口（雷达侧端口 = 主机侧端口 - 1）
PASSIVE_PORTS = {
    56000: "设备发现 / 命令",
    56200: "雷达推送信息",
    56201: "PC 收推送信息",
    56300: "雷达点云",
    56301: "PC 收点云",
    56400: "雷达 IMU",
    56401: "PC 收 IMU",
}


def crc16_ccitt_false(data: bytes) -> int:
    """CRC16-CCITT-FALSE（poly=0x1021, init=0xFFFF），对应 SDK 的 FastCRC ccitt()。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_search_packet(seq: int) -> bytes:
    """复刻 DeviceManager::Detection() 发的那个包。"""
    pkt = bytearray(PACKET_LEN)
    pkt[0] = SOF
    pkt[1] = SDK_VERSION
    pkt[2:4] = PACKET_LEN.to_bytes(2, "little")
    pkt[4:8] = (seq & 0xFFFF).to_bytes(4, "little")
    pkt[8:10] = (0).to_bytes(2, "little")   # kCommandIDLidarSearch
    pkt[10] = 0                             # kCommandTypeCmd
    pkt[11] = 0                             # kHostSend
    # 12..17 = rsvd(6) = 0
    pkt[18:20] = crc16_ccitt_false(bytes(pkt[:18])).to_bytes(2, "little")
    return bytes(pkt)


def parse_reply(data: bytes) -> dict | None:
    """解析雷达的 48 字节应答（DetectionData）。"""
    if len(data) < PACKET_LEN or data[0] != SOF:
        return None
    total_len = int.from_bytes(data[2:4], "little")
    cmd_id = int.from_bytes(data[8:10], "little")
    if cmd_id != 0 or total_len < PACKET_LEN:
        return None
    body = data[PACKET_LEN:total_len]
    if len(body) < 24:
        return None
    return {
        "ret_code": body[0],
        "dev_type": body[1],
        "sn": body[2:18].split(b"\x00")[0].decode("ascii", "replace"),
        "ip": ".".join(str(b) for b in body[18:22]),
        "cmd_port": int.from_bytes(body[22:24], "little"),
        "seq": int.from_bytes(data[4:8], "little"),
        "ack": data[10] == 1,
    }


def port_holder(port: int) -> str:
    """谁占着这个 UDP 端口？（用来揪出 Livox Viewer / 没退干净的驱动）"""
    try:
        out = subprocess.run(["ss", "-unap"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ""
    for line in out.splitlines():
        if f":{port} " not in line and not line.rstrip().endswith(f":{port}"):
            continue
        m = re.search(r'users:\(\("([^"]+)",pid=(\d+)', line)
        if m:
            return f"{m.group(1)} (pid {m.group(2)})"
    return ""


def iface_ipv4(iface: str) -> str:
    """取网卡的 IPv4 地址（发包时要用它当源地址，否则广播会走默认路由=WiFi）。"""
    try:
        out = subprocess.run(["ip", "-4", "-brief", "addr", "show", iface],
                             capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ""
    match = re.search(r"(\d+\.\d+\.\d+\.\d+)/", out)
    return match.group(1) if match else ""


def make_socket(bind_ip: str) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((bind_ip, DETECTION_PORT))
    sock.setblocking(False)
    return sock


def check_port_free() -> bool:
    """严格绑定一次：能绑上说明真没人占；绑不上说明有程序在抢（静默失败的元凶）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("255.255.255.255", DETECTION_PORT))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def open_sockets(host_ip: str) -> list[socket.socket]:
    """照抄 SDK 的双 socket 结构（device_manager.cpp:264/273）：

        · 255.255.255.255:56000  —— 收雷达应答。雷达是**广播**回的，
                                     绑具体 IP 的 socket 收不到广播！
        · <host_ip>:56000        —— 发搜索包。绑具体 IP 才能让路由走对网卡
                                     （绑 0.0.0.0 时 255.255.255.255 会走默认路由 = WiFi）。
    """
    socks = [make_socket("255.255.255.255")]
    try:
        socks.append(make_socket(host_ip))
    except OSError as exc:
        print(f"  ⚠️ 绑不上 {host_ip}:{DETECTION_PORT}（{exc}），改用广播 socket 发包")
    return socks


def probe(seconds: float, host_ip: str) -> list[dict]:
    """主动探测：每秒发一个搜索包，收集所有应答。"""
    try:
        socks = open_sockets(host_ip)
    except OSError as exc:
        print(f"  ❌ 绑不上 UDP {DETECTION_PORT}：{exc}")
        return []
    tx = socks[-1]

    print(f"  📤 每 1 秒从 {host_ip}:{DETECTION_PORT} 往 255.255.255.255 发一次搜索包，"
          f"最多 {seconds:.0f} 秒")
    found: dict[str, dict] = {}
    seq = 1
    next_send = 0.0
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                send_search_packet(tx, seq)
                seq += 1
                next_send = now + 1.0
            ready, _, _ = select.select(socks, [], [], 0.2)
            for sock in ready:
                data, addr = sock.recvfrom(4096)
                if addr[1] != DETECTION_PORT:
                    continue
                info = parse_reply(data)
                if not info or info["ip"] in found:
                    continue
                found[info["ip"]] = info
                name = DEV_TYPES.get(info["dev_type"], "未知类型")
                print(f"  📡 {info['ip']}  SN={info['sn']}  设备类型={info['dev_type']}({name})")
    except KeyboardInterrupt:
        print("  (手动中断)")
    finally:
        for sock in socks:
            close_quietly(sock)
    return list(found.values())


def send_search_packet(sock: socket.socket, seq: int) -> None:
    try:
        sock.sendto(build_search_packet(seq), ("255.255.255.255", DETECTION_PORT))
    except OSError as exc:
        print(f"  发送失败: {exc}")


def listen_passively(ports: list[int], seconds: float) -> dict[str, list[tuple[int, int, str]]]:
    socks: dict[socket.socket, int] = {}
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.bind(("", port))
        except OSError as exc:
            print(f"  端口 {port} 占用，跳过（{exc}）")
            close_quietly(sock)
            continue
        sock.setblocking(False)
        socks[sock] = port

    if not socks:
        print("  ❌ 一个端口都没绑上")
        return {}

    print(f"  👂 监听 {len(socks)} 个端口，最多等 {seconds:.0f} 秒（Ctrl+C 可提前结束）")
    found: dict[str, list[tuple[int, int, str]]] = {}
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            ready, _, _ = select.select(list(socks), [], [], 0.5)
            for sock in ready:
                data, addr = sock.recvfrom(4096)
                entry = (socks[sock], len(data), data[:16].hex(" "))
                found.setdefault(addr[0], []).append(entry)
                if len(found[addr[0]]) == 1:
                    print(f"  📡 {addr[0]}:{addr[1]} → 端口 {socks[sock]}"
                          f"（{PASSIVE_PORTS.get(socks[sock], '?')}，{len(data)} 字节）")
    except KeyboardInterrupt:
        print("  (手动中断)")
    finally:
        for sock in socks:
            close_quietly(sock)
    return found


def close_quietly(sock: socket.socket) -> None:
    try:
        sock.close()
    except OSError:
        pass


def my_ips() -> set[str]:
    try:
        out = subprocess.run(["ip", "-4", "-brief", "addr"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return set()
    return set(re.findall(r"(\d+\.\d+\.\d+\.\d+)/\d+", out))


def show_neighbors(iface: str) -> None:
    cmd = ["ip", "-4", "neigh", "show", "dev", iface] if iface else ["ip", "-4", "neigh", "show"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return
    if out:
        print("\n── 内核邻居表（网线那头的东西）──")
        for line in out.splitlines():
            print("  " + line)


def report(lidars: list[dict]) -> int:
    if not lidars:
        return 1
    print()
    print("=" * 70)
    print(f"✅ 找到 {len(lidars)} 台雷达")
    for info in lidars:
        name = DEV_TYPES.get(info["dev_type"], "未知类型")
        print(f"   雷达 IP    {info['ip']}")
        print(f"   序列号     {info['sn']}")
        print(f"   设备类型   {info['dev_type']}  ({name})")
        print(f"   命令端口   {info['cmd_port']}")
    lidar = lidars[0]
    dev_arg = DEV_ARG.get(lidar["dev_type"])
    extra = "" if dev_arg in (None, "mid360") else f" --device {dev_arg}"
    launch = f"msg_{(DEV_SECTION.get(lidar['dev_type']) or 'MID360')}_launch.py"
    print()
    print("→ 写进驱动配置（不用重新编译，脚本会自动同步到 install/）：")
    print()
    print(f"   bash ~/slam_benchmark/scripts/02_get_livox.sh --lidar-ip {lidar['ip']}{extra}")
    print()
    print("→ 然后启动驱动（另开一个终端看话题）：")
    print()
    print("   ros2 launch livox_ros_driver2 " + launch)
    print("   ros2 topic hz /livox/lidar      # 应约 10 Hz")
    if dev_arg is None:
        print()
        print(f"⚠️ 设备类型 {lidar['dev_type']} 不在已知表里，请手动确认驱动配置的段名。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="发现网线那头的 Livox 雷达（IP / SN / 设备类型）")
    ap.add_argument("-t", "--seconds", type=float, default=10.0, help="探测时长（默认 10 秒）")
    ap.add_argument("-i", "--iface", default="eno1", help="网卡名（默认 eno1），用它的 IP 当发包源地址")
    ap.add_argument("--host-ip", default="", help="PC 侧 IP，默认从 --iface 自动取")
    ap.add_argument("--passive", action="store_true", help="只被动听，不主动发搜索包")
    args = ap.parse_args()

    host_ip = args.host_ip or iface_ipv4(args.iface) or "255.255.255.255"

    print("=" * 70)
    print(" Livox 雷达发现" + ("（被动监听）" if args.passive else "（主动探测）"))
    print("=" * 70)

    if not check_port_free() and not args.passive:
        holder = port_holder(DETECTION_PORT)
        print(f"  ⚠️ UDP {DETECTION_PORT} 已经被占用：{holder or '未知进程'}")
        print("     赶紧关掉它 —— 抢着这个端口时 SDK 会「绑定成功但收不到包」，")
        print("     日志停在 `Init lds lidar success!` 然后一片死寂。")
        print("     最常见的是 Livox Viewer 2，或上一次没退干净的驱动：")
        print("         ss -unap | grep 56          # 看谁占着")
        print("         kill <上面的 pid>")
        return 1

    if args.passive:
        found = listen_passively(list(PASSIVE_PORTS), args.seconds)
        mine = my_ips()
        lidars = [{"ip": ip} for ip in sorted(found) if ip not in mine]
        if lidars:
            print()
            print("✅ 收到了来自这些地址的数据：")
            for ip in sorted(found):
                if ip in mine:
                    continue
                print(f"   {ip}   端口 {sorted({p for p, _, _ in found[ip]})}")
            print()
            print("⚠️ 被动模式只看到 IP，拿不到设备类型。去掉 --passive 再跑一次拿完整的：")
            print("   python3 ~/slam_benchmark/scripts/03_livox_discover.py")
            return 0
    else:
        if host_ip == "255.255.255.255":
            print(f"  ⚠️ {args.iface} 上没有 IPv4 地址 —— 广播会走默认路由（多半是 WiFi），"
                  f"雷达收不到")
            print(f"     先配好网卡：bash ~/slam_benchmark/scripts/02_get_livox.sh --net")
        lidars = probe(args.seconds, host_ip)
        if lidars:
            return report(lidars)

    print()
    print("=" * 70)
    print("❌ 没有收到任何应答。逐条检查：")
    print("   1. 雷达上电了吗（9~27 V，航插拧到底）")
    print(f"   2. 网线插在 PC 上吗： cat /sys/class/net/{args.iface}/carrier   ← 应为 1")
    print(f"   3. PC 网卡设成 192.168.1.50 了吗： ip -brief addr show {args.iface}")
    print("   4. 雷达 IP 被改到别的网段了？被动听一下：")
    print("        python3 ~/slam_benchmark/scripts/03_livox_discover.py --passive")
    print("   5. 还是不行 → 用官方 Livox Viewer 2 打开看看（但看完记得关掉，它会抢端口）")
    show_neighbors(args.iface)
    return 1


if __name__ == "__main__":
    sys.exit(main())
