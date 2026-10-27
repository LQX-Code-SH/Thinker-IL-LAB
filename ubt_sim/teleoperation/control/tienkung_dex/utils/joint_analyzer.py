#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节响应分析：阶跃 / 正弦 / 纯监听，输出指标与 CSV。

用途：改完仿真的 ``stiffness``/``damping``、或上真机前核对驱动器跟随特性。
指令走 ``DexController.send_body_position``（原始单帧目标，不插值），
采样直接取控制器缓存的状态，因此测的是"指令 -> 实际"的闭环响应。

用法::

    # 阶跃：把右肩俯仰从当前值阶跃到 0.2 rad，记录 5 s
    /usr/bin/python3 utils/joint_analyzer.py --joint shoulder_pitch_r_joint \\
        --step 0.2 --duration 5 --csv /tmp/step.csv

    # 正弦辨识
    /usr/bin/python3 utils/joint_analyzer.py --joint elbow_pitch_r_joint \\
        --sine --amplitude 0.2 --period 4 --duration 12

    # 只观察别人下发的运动
    /usr/bin/python3 utils/joint_analyzer.py --joint head_yaw_joint --listen --duration 5

指标：阶跃给稳态误差/上升时间(10-90%)/超调/整定时间(±5%)；正弦给
跟随 RMS、幅值比与相位滞后。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

_UTILS_DIR = os.path.dirname(os.path.abspath(__file__))
_CONTROL_DIR = os.path.dirname(_UTILS_DIR)
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

import constants as C  # noqa: E402
from dex_controller import DexController, _add_common_args, _make_controller  # noqa: E402

SAMPLE_HZ = 100.0


# ============================================================================
# 采样与指标
# ============================================================================
class SampleRecorder:
    """按固定频率记录 ``(t, target, actual)``。"""

    def __init__(self, controller: DexController, joint: str, hand_side: str | None = None):
        self.controller = controller
        self.joint = joint
        self.hand_side = hand_side
        self.samples: list[tuple[float, float, float]] = []
        self.current_target: float | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _actual(self) -> float | None:
        if self.hand_side is None:
            return self.controller.get_joint_position(self.joint)
        return self.controller.get_hand_joint_position(self.hand_side, self.joint)

    def _loop(self) -> None:
        t0 = time.monotonic()
        period = 1.0 / SAMPLE_HZ
        while self._running:
            actual = self._actual()
            target = self.current_target
            if actual is not None and target is not None:
                self.samples.append((time.monotonic() - t0, target, actual))
            time.sleep(period)

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.samples:
            return np.zeros(0), np.zeros(0), np.zeros(0)
        data = np.asarray(self.samples, dtype=float)
        return data[:, 0], data[:, 1], data[:, 2]


def step_metrics(t: np.ndarray, target: np.ndarray, actual: np.ndarray) -> dict[str, Any]:
    """阶跃响应指标（指令本身是一次跳变，逐点 target 只用于定基准）。"""
    if t.size < 5:
        return {"error": "样本太少"}
    initial = float(actual[: max(3, int(0.05 * actual.size))].mean())
    final_target = float(target[-1])
    step = final_target - initial
    if abs(step) < 1e-6:
        return {"error": "阶跃幅度过小"}

    tail = actual[-max(5, int(0.1 * actual.size)):]
    steady = float(tail.mean())
    steady_err = steady - final_target

    normalized = (actual - initial) / step
    def _cross(level: float) -> float | None:
        idx = np.where(normalized >= level)[0]
        return float(t[idx[0]]) if idx.size else None

    rise_10, rise_90 = _cross(0.1), _cross(0.9)
    overshoot = float((normalized.max() - 1.0) * 100.0)

    band = 0.05 * abs(step)
    outside = np.where(np.abs(actual - final_target) > band)[0]
    settle_time = float(t[outside[-1] - 1]) if outside.size and outside[-1] > 0 else 0.0

    return {
        "initial": initial,
        "target": final_target,
        "steady": steady,
        "steady_error": steady_err,
        "steady_error_pct": steady_err / abs(step) * 100.0,
        "rise_time_10_90": None if rise_10 is None or rise_90 is None else rise_90 - rise_10,
        "overshoot_pct": overshoot,
        "settle_time_5pct": settle_time,
        "samples": int(t.size),
    }


def sine_metrics(t: np.ndarray, target: np.ndarray, actual: np.ndarray) -> dict[str, Any]:
    """正弦跟随：RMS 误差、幅值比、相位滞后（对稳态段做最小二乘拟合）。"""
    if t.size < 20:
        return {"error": "样本太少"}
    quarter = max(1, t.size // 4)
    t_fit, tgt_fit, act_fit = t[quarter:], target[quarter:], actual[quarter:]

    def _fit(signal: np.ndarray, freq: float) -> tuple[float, float]:
        A = np.column_stack([np.sin(2 * np.pi * freq * t_fit), np.cos(2 * np.pi * freq * t_fit),
                             np.ones_like(t_fit)])
        (a, b, c), *_ = np.linalg.lstsq(A, signal, rcond=None)
        return float(np.hypot(a, b)), float(np.arctan2(b, a))

    # 用指令段估计频率
    diffs = np.diff(tgt_fit)
    signs = np.sign(diffs)
    crossings = np.where(np.diff(signs) != 0)[0]
    if crossings.size >= 2:
        half_periods = np.diff(t_fit[crossings])
        freq = 1.0 / (2.0 * float(np.median(half_periods)))
    else:
        freq = 0.25

    amp_t, phase_t = _fit(tgt_fit, freq)
    amp_a, phase_a = _fit(act_fit, freq)
    phase_lag = float(np.degrees(np.angle(np.exp(1j * (phase_t - phase_a)))))
    rms = float(np.sqrt(np.mean((act_fit - tgt_fit) ** 2)))

    return {
        "freq_hz": freq,
        "rms_error": rms,
        "amplitude_ratio": amp_a / amp_t if amp_t > 1e-9 else None,
        "phase_lag_deg": phase_lag,
        "samples": int(t.size),
    }


def write_csv(path: str, t: np.ndarray, target: np.ndarray, actual: np.ndarray, joint: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    data = np.column_stack([t, target, actual, actual - target])
    header = f"time_s,{joint}_target,{joint}_actual,error"
    np.savetxt(path, data, delimiter=",", header=header, comments="")
    print(f"CSV 已写入 {path}（{data.shape[0]} 行）")


# ============================================================================
# 运行
# ============================================================================
def _target_source(controller: DexController, joint: str, hand_side: str | None,
                   value: float) -> None:
    if hand_side is None:
        controller.send_body_position({joint: value})
    else:
        controller.send_hand_position(hand_side, {joint: value})


def run(args: argparse.Namespace) -> int:
    joint = C.normalize_joint_name(args.joint)
    if joint is None:
        print(f"未知关节: {args.joint}", file=sys.stderr)
        return 2
    left_hand, right_hand = C.hand_joints(args.variant)
    hand_side = None
    if joint in left_hand:
        hand_side = "left"
    elif joint in right_hand:
        hand_side = "right"
    elif joint not in C.BODY_JOINT_LIMITS:
        print(f"关节 {joint} 既不在身体也不在 {args.variant} 手上", file=sys.stderr)
        return 2

    rclpy.init()
    controller = _make_controller(args, "tienkung_dex_analyzer", enable_ik=False)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            return 1
        if hand_side is None and joint in controller.lock_joints:
            print(f"{joint} 被锁定；加 --include-legs 或 --no-lock 再试", file=sys.stderr)
            return 2

        actual = (
            controller.get_joint_position(joint)
            if hand_side is None
            else controller.get_hand_joint_position(hand_side, joint)
        )
        if actual is None:
            print(f"没有 {joint} 的当前读数", file=sys.stderr)
            return 1

        recorder = SampleRecorder(controller, joint, hand_side)
        recorder.current_target = float(actual)
        recorder.start()
        duration = float(args.duration)
        start = time.monotonic()
        print(f"[{joint}] 起始 {actual:+.4f} rad，运行 {duration:.1f}s ...")

        if args.listen:
            while time.monotonic() - start < duration:
                time.sleep(max(0.05, 1.0 / max(args.hz, 1.0)))
                if not args.quiet:
                    now = (controller.get_joint_position(joint) if hand_side is None
                           else controller.get_hand_joint_position(hand_side, joint))
                    print(f"  t={time.monotonic() - start:5.2f}s  actual={now:+.4f}")
        elif args.step is not None:
            recorder.current_target = float(args.step)
            _target_source(controller, joint, hand_side, float(args.step))
            print(f"  阶跃 -> {args.step:+.4f} rad")
            while time.monotonic() - start < duration:
                time.sleep(max(0.05, 1.0 / max(args.hz, 1.0)))
                if not args.quiet:
                    now = (controller.get_joint_position(joint) if hand_side is None
                           else controller.get_hand_joint_position(hand_side, joint))
                    err = (now - args.step) if now is not None else float("nan")
                    print(f"  t={time.monotonic() - start:5.2f}s  actual={now:+.4f}  err={err:+.4f}")
        else:
            period = max(float(args.period), 0.2)
            freq = 1.0 / period
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= duration:
                    break
                value = args.offset + args.amplitude * np.sin(2 * np.pi * freq * elapsed)
                recorder.current_target = float(value)
                _target_source(controller, joint, hand_side, float(value))
                if not args.quiet and int(elapsed * max(args.hz, 1.0)) % 5 == 0:
                    now = (controller.get_joint_position(joint) if hand_side is None
                           else controller.get_hand_joint_position(hand_side, joint))
                    print(f"  t={elapsed:5.2f}s  target={value:+.4f}  actual={now:+.4f}")
                time.sleep(1.0 / SAMPLE_HZ)

        recorder.stop()
        t, target, measured = recorder.arrays()

        if args.listen:
            print(f"\n采样 {t.size} 点；范围 {measured.min():+.4f} ~ {measured.max():+.4f} rad")
            metrics = {"min": float(measured.min()), "max": float(measured.max()),
                       "mean": float(measured.mean()), "samples": int(t.size)}
        elif args.step is not None:
            metrics = step_metrics(t, target, measured)
        else:
            metrics = sine_metrics(t, target, measured)

        print("\n=== 结果 ===")
        for key, value in metrics.items():
            print(f"  {key:>18}: {value if not isinstance(value, float) else f'{value:+.5f}'}")
        if args.csv:
            write_csv(args.csv, t, target, measured, joint)
        if args.json:
            with open(args.json, "w") as f:
                json.dump(metrics, f, indent=2, ensure_ascii=False)
            print(f"JSON 已写入 {args.json}")
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        controller.stop()
        try:
            executor.shutdown(timeout_sec=1.0)
        except TypeError:
            executor.shutdown()
        spin_thread.join(timeout=2.0)
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="TienKung Dex 关节响应分析")
    _add_common_args(parser)
    parser.add_argument("--joint", required=True, help="关节名（身体或该变体的灵巧手关节）")
    parser.add_argument("--step", type=float, default=None, help="阶跃目标角（rad）")
    parser.add_argument("--sine", action="store_true", help="正弦激励（默认模式）")
    parser.add_argument("--amplitude", type=float, default=0.3, help="正弦振幅（rad）")
    parser.add_argument("--period", type=float, default=4.0, help="正弦周期（s）")
    parser.add_argument("--offset", type=float, default=0.0, help="正弦偏置（rad）")
    parser.add_argument("--listen", action="store_true", help="只听不发")
    parser.add_argument("--duration", type=float, default=10.0, help="运行时长（s）")
    parser.add_argument("--csv", default=None, help="导出 CSV 路径")
    parser.add_argument("--json", default=None, help="导出指标 JSON 路径")
    parser.add_argument("--hz", type=float, default=5.0, help="终端打印频率（Hz）")
    parser.add_argument("--quiet", action="store_true", help="不打印过程，只给汇总")
    cli = parser.parse_args(args)
    sys.exit(run(cli))


if __name__ == "__main__":
    main()
