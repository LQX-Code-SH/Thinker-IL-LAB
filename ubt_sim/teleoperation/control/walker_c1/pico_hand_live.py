#!/usr/bin/env python3
"""Retarget live PICO hand skeletons to safe C1 simulation commands.

The default ``preview`` mode has no ROS dependency and never publishes robot
commands.  Simulation publishing is a separate, explicit mode with a second
``--enable-command`` acknowledgement and an interactive keyboard arm gate.  The
default ``hand`` control remains finger-only.  The explicit ``hand-arm`` mode
maps only the right wrist to the right arm while keeping the same 11DoF finger
mapping.  This program has no real-robot mode.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
from typing import Mapping

import numpy as np

try:
    from .pico_hand_offline_demo import (
        DEFAULT_SIM_ROS_DOMAIN_ID,
        _RosSimulationPublisher,
        require_sim_ros_domain,
        selected_sides,
    )
    from .pico_hand_retarget import C1HandRetargeter, RetargetResult
    from .pico_hand_arm import (
        ArmRetargetResult,
        C1WristArmRetargeter,
        RosHandArmSimulationIO,
    )
    from .pico_source import PicoSource
except ImportError:
    from pico_hand_offline_demo import (
        DEFAULT_SIM_ROS_DOMAIN_ID,
        _RosSimulationPublisher,
        require_sim_ros_domain,
        selected_sides,
    )
    from pico_hand_retarget import C1HandRetargeter, RetargetResult
    from pico_hand_arm import (
        ArmRetargetResult,
        C1WristArmRetargeter,
        RosHandArmSimulationIO,
    )
    from pico_source import PicoSource


_THIS_DIR = Path(__file__).resolve().parent
DEFAULT_PICO_CONFIG = _THIS_DIR / "pico_teleop_config.json"


def _load_tracking_basis(path: str | Path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)["tracking_to_robot"]


@dataclass(frozen=True)
class GateUpdate:
    allow_publish: bool
    just_armed: bool
    just_disarmed: bool


class SimulationCommandGate:
    """Explicit keyboard arm latch plus selected-hand tracking interlock."""

    def __init__(
        self,
        sides: tuple[str, ...],
        *,
        valid_dwell_s: float = 0.5,
        tracking_timeout_s: float = 0.30,
    ) -> None:
        if not sides:
            raise ValueError("at least one hand side is required")
        if valid_dwell_s < 0.0 or tracking_timeout_s < 0.0:
            raise ValueError("gate timing values must be non-negative")
        self.sides = tuple(sides)
        self.valid_dwell_s = float(valid_dwell_s)
        self.tracking_timeout_s = float(tracking_timeout_s)
        self.state = "disarmed"
        self._valid_since: float | None = None
        self._invalid_since: float | None = None

    def _all_valid(self, tracking: Mapping[str, bool]) -> bool:
        return all(bool(tracking.get(side, False)) for side in self.sides)

    def request_arm(self, tracking: Mapping[str, bool], now_s: float) -> bool:
        if not self._all_valid(tracking):
            return False
        self.state = "pending"
        self._valid_since = float(now_s)
        self._invalid_since = None
        return True

    def disarm(self) -> bool:
        changed = self.state != "disarmed"
        self.state = "disarmed"
        self._valid_since = None
        self._invalid_since = None
        return changed

    def update(self, tracking: Mapping[str, bool], now_s: float) -> GateUpdate:
        now_s = float(now_s)
        all_valid = self._all_valid(tracking)
        if self.state == "disarmed":
            return GateUpdate(False, False, False)

        if self.state == "pending":
            if not all_valid:
                self.disarm()
                return GateUpdate(False, False, True)
            if self._valid_since is None:
                self._valid_since = now_s
            if now_s - self._valid_since >= self.valid_dwell_s:
                self.state = "armed"
                self._invalid_since = None
                return GateUpdate(True, True, False)
            return GateUpdate(False, False, False)

        if self.state != "armed":
            raise RuntimeError(f"unknown command gate state {self.state!r}")
        if all_valid:
            self._invalid_since = None
            return GateUpdate(True, False, False)
        if self._invalid_since is None:
            self._invalid_since = now_s
        if now_s - self._invalid_since >= self.tracking_timeout_s:
            self.disarm()
            return GateUpdate(False, False, True)
        # Never publish even during the short timeout grace period.
        return GateUpdate(False, False, False)


class TerminalKeyReader:
    """Non-blocking single-key reader that restores the terminal on exit."""

    def __init__(self, stream=None) -> None:
        self.stream = stream or sys.stdin
        self.fd: int | None = None
        self._saved = None

    def __enter__(self):
        if not self.stream.isatty():
            raise RuntimeError("simulation command mode requires an interactive TTY (-it)")
        self.fd = self.stream.fileno()
        self._saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def read(self) -> str | None:
        if self.fd is None:
            return None
        ready, _, _ = select.select([self.fd], [], [], 0.0)
        if not ready:
            return None
        return os.read(self.fd, 1).decode("utf-8", errors="ignore").lower()

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.fd is not None and self._saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)


def retarget_live_hands(
    hands: Mapping[str, object],
    sides: tuple[str, ...],
    retargeter: C1HandRetargeter,
    dt_s: float,
) -> dict[str, RetargetResult]:
    return {
        side: retargeter.retarget_result(hands[side], dt_s=dt_s)
        for side in sides
    }


def results_to_payload(
    results: Mapping[str, RetargetResult],
) -> dict[str, dict[str, float]]:
    """Convert only valid live results into full named 11DoF payload groups."""
    if not results or not all(result.used_tracking for result in results.values()):
        return {}
    return {
        f"{side}_hand": {
            name: float(value)
            for name, value in zip(result.joint_names, result.robot_hand_q)
        }
        for side, result in results.items()
    }


def initial_open_hand_ready(
    results: Mapping[str, RetargetResult],
    sides: tuple[str, ...],
    threshold: float,
) -> tuple[bool, str]:
    """Require a visibly open hand before capturing a wrist/arm anchor."""
    for side in sides:
        result = results[side]
        if not result.used_tracking:
            return False, f"{side} hand tracking is invalid"
        values = {
            finger: float(result.features.get(f"{finger}_total_flexion", 1.0))
            for finger in ("thumb", "index", "middle", "ring", "little")
        }
        opposition = float(result.features.get("thumb_opposition", 1.0))
        maximum = max([opposition, *values.values()])
        if not np.isfinite(maximum) or maximum > float(threshold):
            return False, (
                f"{side} hand must be open before arm: max normalized flexion/opposition "
                f"{maximum:.2f} > {float(threshold):.2f}"
            )
    return True, ""


def format_live_results(
    results: Mapping[str, RetargetResult],
    *,
    mode: str,
    gate_state: str = "disabled",
) -> str:
    lines = [f"mode={mode} gate={gate_state}"]
    for side, result in results.items():
        title = "Left" if side == "left" else "Right"
        if not result.used_tracking:
            lines.append(f"  {title}: tracking invalid; command not publishable")
            continue
        q_text = ", ".join(f"{value:+.3f}" for value in result.robot_hand_q)
        pinch = result.features.get("thumb_index_distance_m", float("nan"))
        flexions = ", ".join(
            f"{finger}={result.features.get(f'{finger}_total_flexion', float('nan')):.2f}"
            for finger in ("thumb", "index", "middle", "ring", "little")
        )
        lines.append(f"  {title} q11_rad=[{q_text}]")
        lines.append(f"    pinch_m={pinch:.4f}; normalized flexion: {flexions}")
    return "\n".join(lines)


def format_arm_result(
    result: ArmRetargetResult | None,
    *,
    ready: bool,
    reason: str,
) -> str:
    if result is not None and result.valid:
        lines = []
        for side, command in result.commands.items():
            compact = ", ".join(f"{value:+.3f}" for value in command)
            title = "Left" if side == "left" else "Right"
            lines.append(f"  {title} arm q7_rad=[{compact}]")
        return "\n".join(lines)
    if result is not None and result.rejection_reasons:
        details = "; ".join(
            f"{side}: {message}"
            for side, message in result.rejection_reasons.items()
        )
        return "  Arm command invalid: " + details
    return "  Arm ready for anchor" if ready else "  Arm not ready: " + reason


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("preview", "sim"), default="preview")
    parser.add_argument(
        "--control",
        choices=("hand", "hand-arm"),
        default="hand",
        help="hand controls fingers only; hand-arm also maps wrist pose to the matching arm",
    )
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--enable-command", action="store_true")
    parser.add_argument("--pico-config", default=str(DEFAULT_PICO_CONFIG))
    parser.add_argument("--retarget-config", default=None)
    parser.add_argument("--arm-config", default=None)
    parser.add_argument("--urdf", default=None)
    parser.add_argument("--read-hz", type=float, default=30.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--valid-dwell-s", type=float, default=0.5)
    parser.add_argument("--tracking-timeout-s", type=float, default=0.30)
    parser.add_argument("--robot-state-timeout-s", type=float, default=0.50)
    parser.add_argument("--initial-open-threshold", type=float, default=0.45)
    parser.add_argument(
        "--record",
        action="store_true",
        help="record A-started, Space-completed 36D hand-arm HDF5 episodes",
    )
    parser.add_argument(
        "--record-root",
        default="/ubt_sim/dataset/walker_c1_pico_hand",
    )
    parser.add_argument(
        "--camera-topic",
        default="/sensor/camera/head/color/raw",
    )
    parser.add_argument("--record-hz", type=float, default=30.0)
    parser.add_argument("--sim-domain-id", type=int, default=DEFAULT_SIM_ROS_DOMAIN_ID)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> tuple[str, ...]:
    if args.read_hz <= 0.0 or args.print_hz <= 0.0 or args.record_hz <= 0.0:
        raise ValueError("--read-hz, --print-hz, and --record-hz must be positive")
    if args.valid_dwell_s < 0.0 or args.tracking_timeout_s < 0.0:
        raise ValueError("gate timing values must be non-negative")
    if args.robot_state_timeout_s <= 0.0:
        raise ValueError("--robot-state-timeout-s must be positive")
    if not 0.0 < args.initial_open_threshold <= 1.0:
        raise ValueError("--initial-open-threshold must be in (0, 1]")
    if args.mode == "preview" and args.enable_command:
        raise ValueError("--enable-command is only valid with --mode sim")
    if args.mode == "sim":
        if not args.enable_command:
            raise ValueError("--mode sim requires explicit --enable-command")
        require_sim_ros_domain(args.sim_domain_id)
    if args.control == "hand-arm":
        if args.mode != "sim":
            raise ValueError("--control hand-arm requires --mode sim")
        if args.side != "right":
            raise ValueError("first hand-arm version is right-side only; pass --side right")
    if args.record and args.control != "hand-arm":
        raise ValueError("--record requires --control hand-arm")
    return selected_sides(args.side)


def run(args: argparse.Namespace) -> int:
    sides = validate_args(args)
    basis = _load_tracking_basis(args.pico_config)
    source = PicoSource(hand_tracking_enabled=True, tracking_to_output=basis)
    retargeter = C1HandRetargeter(args.retarget_config)
    publisher = None
    hand_arm_io = None
    arm_retargeter = None
    key_reader = None
    gate = None
    recorder = None
    if args.control == "hand-arm":
        try:
            arm_retargeter = C1WristArmRetargeter(
                sides,
                config_path=args.arm_config,
                urdf_path=args.urdf,
            )
            hand_arm_io = RosHandArmSimulationIO(sides, arm_retargeter)
        except Exception:
            source.close()
            raise
    elif args.mode == "sim":
        try:
            publisher = _RosSimulationPublisher(sides)
        except Exception:
            source.close()
            raise
    if args.mode == "sim":
        gate = SimulationCommandGate(
            sides,
            valid_dwell_s=args.valid_dwell_s,
            tracking_timeout_s=args.tracking_timeout_s,
        )
        key_reader = TerminalKeyReader()
    if args.record:
        try:
            try:
                from .pico_episode_recorder import PicoEpisodeRecorder
            except ImportError:
                from pico_episode_recorder import PicoEpisodeRecorder

            def disarm_for_recording(reason: str) -> None:
                gate.disarm()
                arm_retargeter.disarm()
                retargeter.reset()
                hand_arm_io.node.get_logger().info(f"DISARMED: {reason}")

            recorder = PicoEpisodeRecorder(
                hand_arm_io.node,
                args.record_root,
                args.camera_topic,
                args.record_hz,
                body_positions=arm_retargeter.joint_positions,
                hand_joint_names={
                    side: retargeter.joint_names(side)
                    for side in ("left", "right")
                },
                hand_state_topics={
                    "left": "/sim/left_hand/joint_states_full",
                    "right": "/sim/right_hand/joint_states_full",
                },
                disarm_callback=disarm_for_recording,
                reset_callback=lambda: hand_arm_io.reset_scene_and_robot(
                    spin_after_publish=False
                ),
                task="walker_c1_pico_hand_tracking_right_pick_place",
                recording_source="pico_hand_tracking",
                ready_label="[a]",
            )
        except Exception:
            source.close()
            if hand_arm_io is not None:
                hand_arm_io.close()
            raise

    print(f"PICO live control started: control={args.control}")
    print("joint order: thumb_cmp, thumb_mpp, thumb_ip, index_mpp, index_ip, "
          "middle_mpp, middle_ip, ring_mpp, ring_ip, little_mpp, little_ip")
    if args.mode == "preview":
        print("PREVIEW: robot commands are disabled")
    else:
        controls = "[a] arm, [d] disarm, [q] quit"
        if args.control == "hand-arm":
            if recorder is None:
                controls += ", [space] reset scene + robot"
            else:
                controls += ", [space] save episode + reset"
        print(f"SIMULATION COMMAND: starts DISARMED; {controls}")
        if args.control == "hand-arm":
            print(
                "Before [a]: look forward, keep the right wrist still in a comfortable "
                "middle pose, and open the fingers"
            )
        if recorder is not None:
            print(
                f"RECORDING ENABLED: a successful [a] starts an episode; [space] "
                f"saves to {args.record_root} and resets"
            )

    read_period = 1.0 / args.read_hz
    print_period = 1.0 / args.print_hz
    next_print = 0.0
    last_tick = time.monotonic()
    try:
        if publisher is not None:
            publisher.warmup()
        if hand_arm_io is not None:
            hand_arm_io.warmup()
        context = key_reader if key_reader is not None else _NullContext()
        with context as keys:
            while True:
                started = time.monotonic()
                dt_s = max(1e-4, started - last_tick)
                last_tick = started
                headset_pose = None
                if hand_arm_io is not None:
                    hand_arm_io.spin_once(timeout_sec=0.0)
                    if recorder is not None and recorder.finishing:
                        time.sleep(min(read_period, 0.02))
                        continue
                    hand_frame = source.read_hand_frame()
                    headset_pose = hand_frame.headset_pose
                    left_hand = hand_frame.left_hand
                    right_hand = hand_frame.right_hand
                else:
                    left_hand, right_hand = source.read_hands()
                hands = {"left": left_hand, "right": right_hand}
                results = retarget_live_hands(hands, sides, retargeter, dt_s)

                tracking = {
                    side: bool(results[side].used_tracking)
                    for side in sides
                }
                arm_result = None
                arm_ready = True
                arm_reason = ""
                if arm_retargeter is not None:
                    if not hand_arm_io.state_fresh(args.robot_state_timeout_s):
                        arm_ready = False
                        arm_reason = "waiting for fresh /mc/sdk/robot_state"
                    else:
                        arm_ready, arm_reason = arm_retargeter.input_ready(
                            headset_pose, hands
                        )
                    if arm_ready and (gate is None or gate.state != "armed"):
                        arm_ready, arm_reason = initial_open_hand_ready(
                            results,
                            sides,
                            args.initial_open_threshold,
                        )
                    if (
                        arm_ready
                        and recorder is not None
                        and gate is not None
                        and gate.state != "armed"
                    ):
                        arm_ready, arm_reason = recorder.inputs_ready()
                    if gate is not None and gate.state == "armed" and arm_ready:
                        arm_result = arm_retargeter.solve(headset_pose, hands)
                        arm_ready = bool(arm_result.valid)
                        if not arm_ready:
                            arm_reason = "; ".join(
                                f"{side}: {reason}"
                                for side, reason in arm_result.rejection_reasons.items()
                            )
                    if not arm_ready:
                        tracking = {side: False for side in sides}

                if gate is not None:
                    key = keys.read()
                    if key == "q":
                        break
                    if key == " " and hand_arm_io is not None:
                        if recorder is not None and recorder.active:
                            if recorder.request_complete("Space key"):
                                print(
                                    "SAVING: episode closed; writing HDF5 and resetting "
                                    "scene + robot ..."
                                )
                            last_tick = time.monotonic()
                            next_print = 0.0
                            continue
                        gate.disarm()
                        arm_retargeter.disarm()
                        retargeter.reset()
                        print(
                            "RESETTING: teleoperation disarmed; resetting Isaac scene "
                            "and moving robot to task pose ..."
                        )
                        try:
                            hand_arm_io.reset_scene_and_robot()
                        except Exception as exc:
                            print(f"RESET FAILED; remaining DISARMED: {exc}")
                        else:
                            print(
                                "RESET COMPLETE: scene and task pose restored; "
                                "open the right hand and press [a] to arm again"
                            )
                        last_tick = time.monotonic()
                        next_print = 0.0
                        continue
                    if key == "d":
                        if gate.disarm():
                            print("DISARMED: operator request")
                        if arm_retargeter is not None:
                            arm_retargeter.disarm()
                    elif key == "a":
                        if not gate.request_arm(tracking, started):
                            suffix = f": {arm_reason}" if arm_reason else ""
                            print("arm refused: safety preconditions are not satisfied" + suffix)
                    update = gate.update(tracking, started)
                    if update.just_armed:
                        # Start every command session from the configured open pose,
                        # then let the USD velocity limits ramp toward the live pose.
                        retargeter.reset()
                        results = retarget_live_hands(hands, sides, retargeter, dt_s)
                        if arm_retargeter is not None:
                            try:
                                arm_retargeter.capture_anchors(headset_pose, hands)
                                arm_result = arm_retargeter.solve(headset_pose, hands)
                            except (TypeError, ValueError) as exc:
                                arm_result = None
                                arm_reason = str(exc)
                            if arm_result is None or not arm_result.valid:
                                gate.disarm()
                                arm_retargeter.disarm()
                                print(f"DISARMED: cannot capture safe arm anchor: {arm_reason}")
                                update = GateUpdate(False, False, True)
                            else:
                                print(
                                    "ARMED: right wrist controls right arm; "
                                    "right skeleton controls 11DoF hand"
                                )
                                if recorder is not None:
                                    already_active = recorder.active
                                    recorder.start_episode()
                                    recording_state = "resumed" if already_active else "started"
                                    print(
                                        f"RECORDING {recording_state.upper()}: press [space] "
                                        "to save and reset"
                                    )
                        else:
                            print("ARMED: publishing selected hand commands to simulation")
                    elif update.just_disarmed:
                        print("DISARMED: tracking interlock or operator request")
                        if arm_retargeter is not None:
                            arm_retargeter.disarm()
                    if update.allow_publish:
                        payload = results_to_payload(results)
                        if hand_arm_io is not None:
                            if payload and arm_result is not None and arm_result.valid:
                                hand_arm_io.publish(arm_result.commands, payload)
                                if recorder is not None:
                                    body_command = {
                                        name: float(value)
                                        for side, values in arm_result.commands.items()
                                        for name, value in zip(
                                            arm_retargeter.solvers[side].joint_names,
                                            values,
                                        )
                                    }
                                    recorder.note_body_command(body_command)
                                    for group, values in payload.items():
                                        recorder.note_hand_command(
                                            group.removesuffix("_hand"), values
                                        )
                        elif payload:
                            publisher.publish(payload)

                if started >= next_print:
                    next_print = started + print_period
                    state = gate.state if gate is not None else "disabled"
                    print(format_live_results(results, mode=args.mode, gate_state=state))
                    if arm_retargeter is not None:
                        print(
                            format_arm_result(
                                arm_result,
                                ready=arm_ready,
                                reason=arm_reason,
                            )
                        )
                time.sleep(max(0.0, read_period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    except Exception:
        if gate is not None:
            gate.disarm()
        if arm_retargeter is not None:
            arm_retargeter.disarm()
        raise
    finally:
        try:
            source.close()
        finally:
            if publisher is not None:
                publisher.close()
            if hand_arm_io is not None:
                hand_arm_io.close()
    print("PICO live hand retarget stopped; no further commands will be published")
    return 0


class _NullContext:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        return run(args)
    except (RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    raise SystemExit(main())
