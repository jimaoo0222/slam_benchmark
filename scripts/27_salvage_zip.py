#!/usr/bin/env python3
"""抢救「没有中央目录」的 zip —— 下载/拷贝中断留下的半截包。

普通 unzip 会直接报 `End-of-central-directory signature not found`，
因为 zip 的目录（central directory）写在文件**末尾**，传输一断就永远拿不到。
但每个文件自己的头部（local file header）在文件**开头**，压缩数据是连续的，
所以能顺着偏移一个个解出来 —— 只要那条流在中断点之前就结束了。

做法：读 local header → 流式 deflate 直到流自己结束 → 读数据描述符(PK\x07\x08)
      → 比对 crc32 和大小 → 一致才落盘，不一致就删掉（宁缺毋滥）。
      流没结束就说明这条是半截的，报出来、跳过、继续找下一条。

用法：
    python3 scripts/27_salvage_zip.py <残缺.zip> [输出目录]

例：
    python3 scripts/27_salvage_zip.py ~/Downloads/20261102rosbag.zip ~/Downloads/recovered
"""

from __future__ import annotations

import os
import struct
import sys
import zlib

LOCAL_SIG = b"PK\x03\x04"
DESC_SIG = b"PK\x07\x08"
EOCD_SIG = b"PK\x05\x06"


def human(n: int) -> str:
    for unit in ("B", "KiB", "MiB", "GiB"):
        if abs(n) < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TiB"


def read_local_header(f, offset: int):
    f.seek(offset)
    raw = f.read(30)
    if len(raw) < 30 or raw[:4] != LOCAL_SIG:
        return None
    (_sig, _ver, flag, method, _mt, _md, _crc,
     _csize, usize, nlen, elen) = struct.unpack("<IHHHHHIIIHH", raw)
    name = f.read(nlen).decode("utf-8", "replace")
    f.read(elen)
    return {
        "offset": offset,
        "name": name,
        "flag": flag,
        "method": method,
        "usize_hdr": usize,
        "data_start": offset + 30 + nlen + elen,
    }


def salvage_entry(f, entry: dict, out_dir: str, file_size: int) -> dict:
    """解一条。返回 {'status': ok|truncated|unsupported, ...}"""
    if entry["method"] not in (0, 8):
        return {"status": "unsupported", "reason": f"压缩方法 {entry['method']}"}

    target = os.path.join(out_dir, entry["name"])
    if entry["name"].endswith("/"):
        os.makedirs(target, exist_ok=True)
        return {"status": "ok", "bytes": 0, "dir": True}

    os.makedirs(os.path.dirname(target) or ".", exist_ok=True)
    tmp = target + ".salvage"
    f.seek(entry["data_start"])

    crc = 0
    total = 0
    dec = zlib.decompressobj(-15) if entry["method"] == 8 else None

    with open(tmp, "wb") as out:
        while dec is not None and not dec.eof:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            try:
                data = dec.decompress(chunk)
            except zlib.error as exc:
                out.close()
                os.remove(tmp)
                return {"status": "truncated", "bytes": total,
                        "reason": f"解压中断（{exc}）"}
            if data:
                out.write(data)
                crc = zlib.crc32(data, crc)
                total += len(data)
        if dec is not None and dec.eof:
            rest = dec.flush()
            if rest:
                out.write(rest)
                crc = zlib.crc32(rest, crc)
                total += len(rest)

    if dec is not None and not dec.eof:
        os.remove(tmp)
        return {"status": "truncated", "bytes": total}

    # 流完整结束 → 校验数据描述符
    stream_end = f.tell() - (len(dec.unused_data) if dec else 0)
    f.seek(stream_end)
    desc = f.read(16)
    if desc[:4] == DESC_SIG:
        d_crc, d_csize, d_usize = struct.unpack("<III", desc[4:16])
        desc_len = 16
        # zip64 数据描述符：crc 仍是 4 字节，size 变 8 字节 → 共 24 字节
        if d_csize == 0xFFFFFFFF or d_usize == 0xFFFFFFFF:
            f.seek(stream_end)
            full = f.read(24)
            d_crc, = struct.unpack("<I", full[4:8])
            d_csize, d_usize = struct.unpack("<QQ", full[8:24])
            desc_len = 24
        if d_crc != crc or (d_usize and d_usize != total):
            os.remove(tmp)
            return {"status": "corrupt", "bytes": total,
                    "reason": f"crc {crc:08x} != {d_crc:08x} 或大小 {total} != {d_usize}"}
        next_offset = stream_end + desc_len
    else:
        # 没有描述符（flag bit3 未置位）→ 依赖头里的 crc/usize
        if entry["usize_hdr"] and entry["usize_hdr"] != total:
            os.remove(tmp)
            return {"status": "corrupt", "bytes": total,
                    "reason": f"大小 {total} != 头里 {entry['usize_hdr']}"}
        next_offset = stream_end

    os.replace(tmp, target)
    return {"status": "ok", "bytes": total, "next": next_offset, "crc": crc}


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = os.path.expanduser(sys.argv[1])
    out_dir = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else \
        os.path.join(os.path.dirname(path), "recovered")

    if not os.path.isfile(path):
        print(f"❌ 找不到 {path}")
        return 1

    size = os.path.getsize(path)
    with open(path, "rb") as f:
        f.seek(max(0, size - 65536))
        has_eocd = EOCD_SIG in f.read()

    print("=" * 70)
    print(f" 抢救 {path}")
    print(f" 大小 {human(size)}（{size} 字节）")
    print(f" 中央目录：{'有 ✅ 可以直接 unzip' if has_eocd else '缺失 ❌ 归档被截断'}")
    print(f" 输出到 {out_dir}")
    print("=" * 70)

    if has_eocd:
        print("\n这个包其实是完整的，直接 unzip 就行：")
        print(f"  unzip {path} -d {out_dir}")
        return 0

    offset = 0
    ok_files = []
    bad = []
    f = open(path, "rb")
    try:
        while True:
            entry = read_local_header(f, offset)
            if entry is None:
                break
            print(f"\n── {entry['name']}")
            res = salvage_entry(f, entry, out_dir, size)
            if res["status"] == "ok":
                if res.get("dir"):
                    print("   ✅ 目录")
                else:
                    print(f"   ✅ 完整  {human(res['bytes'])}  crc={res['crc']:08x}")
                    ok_files.append((entry["name"], res["bytes"]))
                offset = res.get("next", entry["data_start"])
            else:
                print(f"   ❌ {res['status']}：{res.get('reason', '数据流在文件末尾被切断')}")
                print(f"      已解出 {human(res['bytes'])}（未落盘）")
                bad.append((entry["name"], res["status"]))
                break
    finally:
        f.close()

    print("\n" + "=" * 70)
    print(f" ✅ 完整救回 {len(ok_files)} 个文件，共 {human(sum(b for _, b in ok_files))}")
    for n, b in ok_files:
        print(f"    {human(b):>10}  {n}")
    if bad:
        print(f" ❌ 丢失 {len(bad)} 条（在截断点之后）")
        for n, s in bad:
            print(f"    {s:<10} {n}")
    print("=" * 70)
    return 0 if ok_files else 1


if __name__ == "__main__":
    sys.exit(main())
