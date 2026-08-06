#!/usr/bin/env python3
"""Probe the live SSH -X proxy and emit a multi-entry Xauthority blob.

背景：本机曾更名 (tegra-ubuntu -> vision) 且 SSH 会话反复开关，宿主
~/.Xauthority 里同一 display 号会同时残留多条不同 cookie
(vision/unix:N 为 FamilyLocal、tegra-ubuntu:N 为 FamilyInternet)。sshd
的 X 代理 (TCP 601x，本机无对应 unix socket) 只接受其中一条，且哪条是
live 的会随会话切换而翻转。按名字 `xauth extract` 会随机抽到 live 或过期
那条，导致容器内 "X11 connection rejected because of wrong authentication"
或 cv2/libX11 连不上。

本脚本：读出该 display 的全部候选 cookie，逐条做真实 X11 连接握手探测，
取服务器实际接受 (status 1/2) 的 live cookie，写成多条 MIT-MAGIC-COOKIE-1
条目输出到 stdout：
  - FamilyLocal(hostname)  命中 preview_camera._is_headless 的精确 _read_xauth
    (其 TCP 分支用 FamilyLocal 码 0x0100 + addr=host 查找，否则会落到挂载的
    宿主 ~/.Xauthority 命中 vision/unix:N 过期 cookie)。
  - FamilyWild             兜底，被 _read_xauth_any_display 与 cv2/libX11 命中。
  - FamilyInternet(host IP) 让 libX11 的 TCP (family,address) 查找精确命中。
无论连接走 TCP 还是 unix socket、地址是 127.0.0.1 还是 127.0.1.1 都能用同一
个 live cookie。
"""
import os
import socket
import struct
import subprocess
import sys


def main() -> None:
    display = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("DISPLAY", "")
    if not display:
        return
    host = display.split(":", 1)[0] if ":" in display else ""
    rest = display.split(":", 1)[1] if ":" in display else display
    try:
        dnum = int(rest.split(".")[0])
    except ValueError:
        return
    disp = str(dnum).encode()

    try:
        out = subprocess.check_output(
            ["xauth", "list"], text=True, stderr=subprocess.DEVNULL
        )
    except Exception:
        return

    seen, cands = set(), []
    for line in out.splitlines():
        p = line.split()
        if len(p) < 3 or p[1] != "MIT-MAGIC-COOKIE-1":
            continue
        # 条目名形如 vision/unix:10、tegra-ubuntu:10、:0 等 -> 取冒号后的 display 号
        if p[0].split(":")[-1].split(".")[0] != str(dnum):
            continue
        try:
            data = bytes.fromhex(p[2])
        except ValueError:
            continue
        if data in seen:
            continue
        seen.add(data)
        cands.append(data)
    if not cands:
        return

    def probe(cookie: bytes) -> bytes:
        try:
            if host:
                s = socket.create_connection((host, 6000 + dnum), 2.0)
            else:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect(f"/tmp/.X11-unix/X{dnum}")
        except OSError:
            return b""
        name = b"MIT-MAGIC-COOKIE-1"

        def pad(x: bytes) -> bytes:
            return x + b"\x00" * ((4 - len(x) % 4) % 4)

        setup = (
            b"\x6c\x00"  # byte-order little-endian + unused
            + struct.pack("<HH", 11, 0)  # protocol 11.0
            + struct.pack("<HH", len(name), len(cookie))
            + b"\x00\x00"
            + pad(name)
            + pad(cookie)
        )
        try:
            s.sendall(setup)
            r = s.recv(1)
        except OSError:
            r = b""
        s.close()
        return r

    live = None
    for c in cands:
        r = probe(c)
        if len(r) == 1 and r[0] in (1, 2):  # 1=Success, 2=Authenticate
            live = c
            break
    if live is None:
        live = cands[0]  # 探测全失败时退回第一个候选 (best effort)

    # 把 live cookie 写成多条 Xauthority 条目，覆盖所有查找路径：
    #   1) FamilyLocal(hostname)  -- preview_camera._is_headless 的 TCP 分支
    #      误用 FamilyLocal 码 0x0100 + addr=host 做精确 _read_xauth 查找，
    #      若 /tmp/.xauth-docker 没有匹配项，它会落到挂载的宿主 ~/.Xauthority
    #      里命中 vision/unix:10 (过期 cookie) -> wrong authentication。
    #   2) FamilyWild              -- 兜底，被 _read_xauth_any_display 与
    #      cv2/libX11 的 (family,address) 查找命中。
    #   3) FamilyInternet(host IP) -- 让 libX11 的 TCP 查找精确命中。
    name = b"MIT-MAGIC-COOKIE-1"
    hostname = socket.gethostname().encode()

    def entry(fam: int, addr: bytes) -> bytes:
        return (
            struct.pack(">H", fam)
            + struct.pack(">H", len(addr)) + addr
            + struct.pack(">H", len(disp)) + disp
            + struct.pack(">H", len(name)) + name
            + struct.pack(">H", len(live)) + live
        )

    blob = entry(0x0100, hostname) + entry(0xFFFF, b"")
    if host:
        try:
            infos = socket.getaddrinfo(host, None, socket.AF_INET)
            for _f, _s, _p, _c, sockaddr in infos:
                blob += entry(0x0000, socket.inet_aton(sockaddr[0]))
                break
        except OSError:
            pass
    sys.stdout.buffer.write(blob)


if __name__ == "__main__":
    main()
