#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""预备位 / 站立位的整段播放，带定时抓拍。

原先的 ``/tmp/dex_play_ready.py``、``/tmp/dex_play_home.py`` 固化成
``play`` 子命令：走完整段四段预备动作（或倒放回站立位），同时按固定间隔把
头部相机画面存成图片，便于事后逐帧核对路径——尤其是 2/4、3/4 两个绕桌航点，
它们被速度检查整条丢掉过。

用法::

    # 直接从当前位置播一遍预备动作，抓拍存 /tmp/ready_seq
    python3 tienkung_dex_controller.py play ready

    # 先分段回站立位再播（真机刚上电、姿态不确定时用）
    python3 tienkung_dex_controller.py play ready --from-home

    # 四段倒放回站立位
    python3 tienkung_dex_controller.py play home

    # 来回各两遍，抓拍间隔 0.2 s
    python3 tienkung_dex_controller.py play ready --rounds 2 --snap-interval 0.2

也可直接跑本文件：``python3 utils/pose_play.py ready``。
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

_UTILS_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.dirname(_UTILS_DIR)
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import constants as C  # noqa: E402
from dex_controller import (  # noqa: E402
    DexController,
    _add_common_args,
    _make_controller,
    _shutdown,
)
from utils.camera import DexCamera  # noqa: E402

# 每遍结束后对照的目标姿态（home 只比上半身，腿默认不动）
POSE_TARGETS = {"ready": C.READY_POSE, "home": C.HOME_UPPER_POSE}
POSE_LABELS = {"ready": "预备动作", "home": "回站立位"}


def _clear_snaps(out_dir: str) -> None:
    """清掉上次抓拍的 ``f*.png``，目录里的其他文件不动。"""
    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(out_dir):
        if name.startswith("f") and name.endswith(".png"):
            os.remove(os.path.join(out_dir, name))


def _grab_loop(cam: DexCamera, out_dir: str, interval: float, stop: threading.Event) -> int:
    """每 ``interval`` 秒存一帧头部 RGB，返回保存帧数。"""
    import cv2

    count = 0
    while not stop.is_set():
        rgb, _ = cam.snapshot()
        if rgb is not None:
            cv2.imwrite(
                os.path.join(out_dir, f"f{count:03d}.png"),
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            )
            count += 1
        stop.wait(interval)
    return count


def _print_worst_errors(controller: DexController, pose: dict[str, float], top: int = 6) -> None:
    """打印偏差最大的若干个关节（目标 / 实测 / 误差）。"""
    current = controller.get_current_positions()
    rows = [(name, float(target), current.get(name)) for name, target in pose.items()]
    rows.sort(
        key=lambda row: abs(row[2] - row[1]) if row[2] is not None else float("inf"),
        reverse=True,
    )
    for name, target, actual in rows[:top]:
        if actual is None:
            print(f"   {name:<28} 目标 {target:+8.3f} 实测 —")
        else:
            print(f"   {name:<28} 目标 {target:+8.3f} 实测 {actual:+8.3f} 误差 {actual - target:+8.3f}")


def _play_once(
    controller: DexController, pose: str, duration_sec: float, include_legs: bool
) -> tuple[bool, float]:
    t0 = time.time()
    if pose == "ready":
        ok = controller.move_to_ready_pose(duration_sec=duration_sec)
    else:
        ok = controller.home(duration_sec=duration_sec, include_legs=include_legs)
    return ok, time.time() - t0


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="播放预备位 / 回站立位，可选定时抓拍")
    parser.add_argument(
        "pose", nargs="?", choices=("ready", "home"), default="ready",
        help="ready=四段预备动作（默认），home=四段倒放回站立位",
    )
    _add_common_args(parser)
    parser.add_argument("--duration", type=float, default=10.0, help="整段总时长（秒）")
    parser.add_argument(
        "--from-home", action="store_true",
        help="ready 之前先分段回站立位（默认直接从当前位置播）",
    )
    # 兼容旧命令行：跳过 home 现在是默认行为
    parser.add_argument("--skip-home", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--rounds", type=int, default=1, help="重复遍数；每遍之间自动回另一端")
    parser.add_argument("--snap-dir", default=None, help="抓拍目录（默认 /tmp/<pose>_seq）")
    parser.add_argument("--snap-interval", type=float, default=0.4, help="抓拍间隔（秒）")
    parser.add_argument("--no-snapshots", action="store_true", help="不抓拍，只看运动")
    cli = parser.parse_args(list(args) if args is not None else None)

    rounds = max(1, int(cli.rounds))
    snap_dir = cli.snap_dir or f"/tmp/{cli.pose}_seq"

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_play", enable_ik=False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    cam = None
    if not cli.no_snapshots:
        cam = DexCamera()
        executor.add_node(cam)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop = threading.Event()
    grab_count: list[int] = []
    grab_thread = None
    try:
        if not controller.wait_for_state(timeout=5.0):
            return
        if cli.pose == "ready" and cli.from_home:
            print("=== 先回站立位 ===")
            print("home 返回", controller.home(duration_sec=cli.duration, include_legs=cli.include_legs))
            time.sleep(1.0)

        if cam is not None:
            if cam.wait_for_rgb(timeout=5.0):
                _clear_snaps(snap_dir)
                grab_thread = threading.Thread(
                    target=lambda: grab_count.append(
                        _grab_loop(cam, snap_dir, cli.snap_interval, stop)
                    ),
                    daemon=True,
                )
                grab_thread.start()
            else:
                print("没有相机画面（桥接没起或相机没发布），本次不抓拍")

        for i in range(rounds):
            print(
                f"=== 播放{POSE_LABELS[cli.pose]}（{cli.duration:.0f}s 分段）"
                f"第 {i + 1}/{rounds} 遍 ==="
            )
            ok, elapsed = _play_once(controller, cli.pose, cli.duration, cli.include_legs)
            time.sleep(1.0)
            print(f"=== 返回 {ok}，耗时 {elapsed:.1f}s ===")
            _print_worst_errors(controller, POSE_TARGETS[cli.pose])
            if i + 1 < rounds:
                other = "home" if cli.pose == "ready" else "ready"
                print(f"   走{POSE_LABELS[other]}，准备重放…")
                _play_once(controller, other, cli.duration, cli.include_legs)
                time.sleep(0.5)

        if grab_thread is not None:
            stop.set()
            grab_thread.join(timeout=2.0)
            print(f"抓拍 {grab_count[0] if grab_count else 0} 帧 → {snap_dir}")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        _shutdown(controller, executor, spin_thread)


if __name__ == "__main__":
    main()
