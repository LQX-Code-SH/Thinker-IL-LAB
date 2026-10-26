#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 仿真数据采集入口（HDF5 单集录制）。

前置条件：仿真已通过 ``scripts/start_sim.sh`` 启动（含 ROS2-ZMQ 桥接），
且遥操作端在发布 ``/tienkung_dex/*_command``。

用法（容器内）::

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 teleoperation/control/tienkung_dex/save_data.py \
        --output-dir /ubt_sim/dataset/tienkung_dex --capture-hz 15

交互：``Enter`` 开始/停止，``s`` 保存并进入下一集，``q`` 退出（自动保存当前集）。
也可以用 ``--auto-start`` + ``--duration`` 做无人值守采集。
"""

from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from utils.recorder import DEFAULT_CAPTURE_HZ, TienkungDexDataRecorder  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "dataset" / "tienkung_dex"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="TienKung Dex HDF5 data collection")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="HDF5 输出目录")
    parser.add_argument("--capture-hz", type=float, default=DEFAULT_CAPTURE_HZ, help="写入频率")
    parser.add_argument("--no-depth", action="store_true", help="不录制深度图")
    parser.add_argument("--auto-start", action="store_true", help="启动后立即开始录制")
    parser.add_argument("--duration", type=float, default=0.0, help="单集时长（秒），0=手动停止")
    parser.add_argument("--episodes", type=int, default=0, help="自动采集集数，0=不限制")
    parser.add_argument("--task-name", default="", help="写入 HDF5 attrs 的任务描述")
    parser.add_argument("--variant", default="inspire", choices=["inspire", "brainco"], help="手部变体")
    return parser.parse_args()


def _stdin_listener(commands: queue.Queue) -> None:
    for line in sys.stdin:
        commands.put(line.strip().lower())
    commands.put("q")


def main() -> int:
    args = _parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = TienkungDexDataRecorder(
        capture_hz=args.capture_hz, record_depth=not args.no_depth, variant=args.variant
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    commands: queue.Queue = queue.Queue()
    threading.Thread(target=_stdin_listener, args=(commands,), daemon=True).start()

    episode_index = 1
    saved = 0
    started_at = 0.0
    running = True

    def save_current() -> None:
        nonlocal episode_index, saved
        if node.recorder.num_frames == 0:
            print("[WARN] 没有收到任何帧，跳过保存")
            return
        path = output_dir / f"episode_{episode_index:04d}.hdf5"
        node.save_episode(str(path), attrs={"task_name": args.task_name, "episode_index": episode_index})
        episode_index += 1
        saved += 1

    try:
        if args.auto_start:
            node.start_episode()
            started_at = time.time()
            print("[INFO] auto-start：开始录制")

        print(
            "[INFO] 控制：Enter=开始/停止，s=保存并下一集，q=退出\n"
            f"[INFO] 输出目录：{output_dir}（variant={args.variant}）"
        )

        while running:
            try:
                command = commands.get(timeout=0.1)
            except queue.Empty:
                command = ""

            if command in ("q", "quit", "exit"):
                if node.recorder.recording:
                    node.stop_episode()
                if node.recorder.num_frames > 0:
                    save_current()
                break

            if command in ("", "t", "toggle"):
                if node.recorder.recording:
                    node.stop_episode()
                else:
                    node.start_episode()
                    started_at = time.time()

            if command in ("s", "save"):
                if node.recorder.recording:
                    node.stop_episode()
                save_current()
                if args.episodes and saved >= args.episodes:
                    break
                if args.auto_start:
                    node.start_episode()
                    started_at = time.time()

            if args.duration > 0 and node.recorder.recording and (time.time() - started_at) >= args.duration:
                node.stop_episode()
                save_current()
                if args.episodes and saved >= args.episodes:
                    break
                if args.auto_start:
                    node.start_episode()
                    started_at = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if node.recorder.recording:
            node.stop_episode()
        if node.recorder.num_frames > 0:
            save_current()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[INFO] 结束，共保存 {saved} 集到 {output_dir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
