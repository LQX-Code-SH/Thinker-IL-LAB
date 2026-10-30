#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 仿真数据采集入口（HDF5 单集录制）。

前置条件：仿真已通过 ``scripts/start_sim.sh`` 启动（含 ROS2-ZMQ 桥接），
且遥操作端在发布与真机一致的关节指令（``/arm|head|waist|leg/cmd`` + 灵巧手）。
默认 0（与真机同域），隔离时用 146，靠 ``ROS_DOMAIN_ID`` 区分。

用法（容器内）::

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0            # 默认与真机同域
    /usr/bin/python3 teleoperation/control/tienkung_dex/save_data.py \
        --output-dir /ubt_sim/dataset/tienkung_dex --capture-hz 15

交互：``Enter`` 开始/停止，``s`` 保存并进入下一集，``q`` 退出（自动保存当前集）。
也可以用 ``--auto-start`` + ``--duration`` 做无人值守采集。

落盘位置与 tienkung_pro / walker_s2 一致，一集一个时间戳目录::

    dataset/tienkung_dex/<epoch秒>/trajectory.hdf5

目录 0777、文件 0666（容器内 root 采集、宿主用户也能读）。
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

from utils.recorder import (  # noqa: E402
    DEFAULT_CAPTURE_HZ,
    HDF5_REL_PATH,
    TienkungDexDataRecorder,
    episode_path,
    prepare_episode_file,
)

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


def _stdin_listener(commands: queue.Queue, quit_on_eof: bool) -> None:
    """把 stdin 的每行转成命令；EOF 时按需退出。

    非交互运行（``docker exec`` 不带 ``-t``、nohup、stdin 重定向）会立刻 EOF，
    如果无条件当作 ``q``，无人值守采集会在第一集刚开录时就被停掉。
    """
    for line in sys.stdin:
        commands.put(line.strip().lower())
    if quit_on_eof:
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
    # --auto-start = 无人值守：结束条件交给 --duration / --episodes，EOF 不算退出
    threading.Thread(
        target=_stdin_listener, args=(commands, not args.auto_start), daemon=True
    ).start()

    episode_index = 1
    saved = 0
    started_at = 0.0
    running = True

    def save_current() -> None:
        nonlocal episode_index, saved
        if node.recorder.num_frames == 0:
            print("[WARN] 没有收到任何帧，跳过保存")
            return
        # 时间戳目录名取 ROS 时钟（与 tienkung_pro / walker_s2 同源）
        epoch_seconds = node.get_clock().now().seconds_nanoseconds()[0]
        path = prepare_episode_file(episode_path(output_dir, epoch_seconds))
        node.save_episode(
            str(path),
            attrs={
                "task": args.task_name,
                "task_name": args.task_name,
                "episode_index": episode_index,
            },
        )
        node.recorder.clear()   # 保存后清缓存，否则退出时 finally 会把上一集再存一遍
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
                # None = 这一拍没有键盘输入；不能和 ""（用户按了 Enter）混为一谈，
                # 否则空闲时会把录制每 100ms 开/关一次
                command = None

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
        print(f"[INFO] 结束，共保存 {saved} 集到 {output_dir}/<时间戳>/{HDF5_REL_PATH}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
