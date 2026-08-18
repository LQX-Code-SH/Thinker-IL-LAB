#!/usr/bin/env python3
"""Retarget live PICO hand skeletons to C1 11DoF preview or safe simulation commands.

The default ``preview`` mode has no ROS dependency and never publishes robot
commands.  Simulation publishing is a separate, explicit mode with a second
``--enable-command`` acknowledgement and an interactive keyboard arm gate.
This program never controls the C1 arms and has no real-robot mode.
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

try:
    from .pico_hand_offline_demo import (
        DEFAULT_SIM_ROS_DOMAIN_ID,
        _RosSimulationPublisher,
        require_sim_ros_domain,
        selected_sides,
    )
    from .pico_hand_retarget import C1HandRetargeter, RetargetResult
    from .pico_source import PicoSource
except ImportError:
    from pico_hand_offline_demo import (
        DEFAULT_SIM_ROS_DOMAIN_ID,
        _RosSimulationPublisher,
        require_sim_ros_domain,
        selected_sides,
    )
    from pico_hand_retarget import C1HandRetargeter, RetargetResult
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


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("preview", "sim"), default="preview")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--enable-command", action="store_true")
    parser.add_argument("--pico-config", default=str(DEFAULT_PICO_CONFIG))
    parser.add_argument("--retarget-config", default=None)
    parser.add_argument("--read-hz", type=float, default=30.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--valid-dwell-s", type=float, default=0.5)
    parser.add_argument("--tracking-timeout-s", type=float, default=0.30)
    parser.add_argument("--sim-domain-id", type=int, default=DEFAULT_SIM_ROS_DOMAIN_ID)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> tuple[str, ...]:
    if args.read_hz <= 0.0 or args.print_hz <= 0.0:
        raise ValueError("--read-hz and --print-hz must be positive")
    if args.valid_dwell_s < 0.0 or args.tracking_timeout_s < 0.0:
        raise ValueError("gate timing values must be non-negative")
    if args.mode == "preview" and args.enable_command:
        raise ValueError("--enable-command is only valid with --mode sim")
    if args.mode == "sim":
        if not args.enable_command:
            raise ValueError("--mode sim requires explicit --enable-command")
        require_sim_ros_domain(args.sim_domain_id)
    return selected_sides(args.side)


def run(args: argparse.Namespace) -> int:
    sides = validate_args(args)
    basis = _load_tracking_basis(args.pico_config)
    source = PicoSource(hand_tracking_enabled=True, tracking_to_output=basis)
    retargeter = C1HandRetargeter(args.retarget_config)
    publisher = None
    key_reader = None
    gate = None
    if args.mode == "sim":
        try:
            publisher = _RosSimulationPublisher(sides)
        except Exception:
            source.close()
            raise
        gate = SimulationCommandGate(
            sides,
            valid_dwell_s=args.valid_dwell_s,
            tracking_timeout_s=args.tracking_timeout_s,
        )
        key_reader = TerminalKeyReader()

    print("PICO live hand retarget started")
    print("joint order: thumb_cmp, thumb_mpp, thumb_ip, index_mpp, index_ip, "
          "middle_mpp, middle_ip, ring_mpp, ring_ip, little_mpp, little_ip")
    if args.mode == "preview":
        print("PREVIEW: robot commands are disabled")
    else:
        print("SIMULATION COMMAND: starts DISARMED; [a] arm, [d] disarm, [q] quit")

    read_period = 1.0 / args.read_hz
    print_period = 1.0 / args.print_hz
    next_print = 0.0
    last_tick = time.monotonic()
    try:
        if publisher is not None:
            publisher.warmup()
        context = key_reader if key_reader is not None else _NullContext()
        with context as keys:
            while True:
                started = time.monotonic()
                dt_s = max(1e-4, started - last_tick)
                last_tick = started
                left_hand, right_hand = source.read_hands()
                hands = {"left": left_hand, "right": right_hand}
                results = retarget_live_hands(hands, sides, retargeter, dt_s)

                if gate is not None:
                    tracking = {
                        side: bool(results[side].used_tracking)
                        for side in sides
                    }
                    key = keys.read()
                    if key == "q":
                        break
                    if key == "d":
                        if gate.disarm():
                            print("DISARMED: operator request")
                    elif key == "a":
                        if not gate.request_arm(tracking, started):
                            print("arm refused: every selected hand must be valid")
                    update = gate.update(tracking, started)
                    if update.just_armed:
                        # Start every command session from the configured open pose,
                        # then let the USD velocity limits ramp toward the live pose.
                        retargeter.reset()
                        results = retarget_live_hands(hands, sides, retargeter, dt_s)
                        print("ARMED: publishing selected hand commands to simulation")
                    elif update.just_disarmed:
                        print("DISARMED: tracking interlock or operator request")
                    if update.allow_publish:
                        payload = results_to_payload(results)
                        if payload:
                            publisher.publish(payload)

                if started >= next_print:
                    next_print = started + print_period
                    state = gate.state if gate is not None else "disabled"
                    print(format_live_results(results, mode=args.mode, gate_state=state))
                time.sleep(max(0.0, read_period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    except Exception:
        if gate is not None:
            gate.disarm()
        raise
    finally:
        try:
            source.close()
        finally:
            if publisher is not None:
                publisher.close()
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
