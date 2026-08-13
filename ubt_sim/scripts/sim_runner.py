# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run ubt_sim simulation environments."""

import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import signal
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="UBT Sim simulation environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="UBTSim-TienkungPro-Parlor-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument("--perf_stats", action="store_true", help="Print performance statistics.")
parser.add_argument("--load_only", action="store_true", help="Load and render the environment without ROS control.")
# Walker S2 specific (ignored for other robots)
parser.add_argument("--zmq_cmd_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_CMD_PORT", 5655)))
parser.add_argument("--zmq_status_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_STATUS_PORT", 5656)))
parser.add_argument("--zmq_image_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_IMAGE_PORT", 5657)))
parser.add_argument("--zmq_jpeg_image_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_JPEG_IMAGE_PORT", 5658)))
parser.add_argument(
    "--physics_device",
    type=str,
    default=os.environ.get("UBT_SIM_WALKER_S2_PHYSICS_DEVICE", "cpu"),
    help=(
        "Device for Isaac Lab physics tensors. Defaults to CPU while AppLauncher/rendering stays on cuda:0. "
        "This avoids Isaac Sim 5.0 / Isaac Lab 2.2 Walker S2 articulation startup spam: "
        "getVelocities expected device 0, received device -1."
    ),
)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device=os.environ.get("UBT_SIM_WALKER_S2_DEVICE", "cuda:0"))
args_cli = parser.parse_args()

import sys

sys.argv.append("--/log/level=error")
sys.argv.append("--/log/fileLogLevel=error")
sys.argv.append("--/log/outputStreamLevel=error")
# Force PhysX tensor readback compatibility. Isaac Sim 5.0 / Isaac Lab 2.2 can
# create the articulation velocity buffer on CPU when GPU dynamics suppresses
# readback, which triggers getVelocities device mismatch spam on startup.
sys.argv.append("--/physics/suppressReadback=false")

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.utils.head_material import fix_walker_s2_head_material
from ubt_sim.utils.loop_utils import KeyboardResetController, PerfMonitor, RateLimiter


def _detect_robot(task_name: str | None) -> str:
    """Infer robot type from task name."""
    if task_name and "WalkerC1" in task_name:
        return "walker_c1"
    if task_name and "WalkerS2" in task_name:
        return "walker_s2"
    return "tienkung_pro"


ROBOT = _detect_robot(args_cli.task)


# --- Walker S2 part randomization (no-op for other robots) ---

def _apply_part_randomization_if_requested(env, teleop_interface) -> None:
    if ROBOT != "walker_s2":
        return
    request = teleop_interface.pop_part_randomization_request()
    if request is None:
        return

    randomizer = getattr(env.cfg, "randomize_part_positions", None)
    if randomizer is None:
        print("[WARN] Current task does not support part randomization.")
        return

    try:
        result = randomizer(env, request)
        print(f"[INFO] Randomized Walker S2 part positions: {result}")
    except Exception as exc:
        print(f"[WARN] Failed to randomize Walker S2 part positions: {exc}")


def main():
    # Resolve physics device: Walker S2 uses a dedicated flag, other robots use the
    # AppLauncher device (which may also come from env).
    physics_device = args_cli.physics_device if ROBOT == "walker_s2" else args_cli.device
    env_cfg = parse_env_cfg(args_cli.task, device=physics_device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device(ROBOT)
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
    env_cfg.recorders = None

    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # GUI mode: open viewports bound to the configured C1 camera set. The
    # mentor-sensor preview is opt-in because five RTX viewports are expensive.
    if ROBOT == "walker_c1" and not getattr(args_cli, "headless", False):
        try:
            from omni.kit.viewport.utility import create_viewport_window

            if os.environ.get("UBT_SIM_C1_SENSOR_VIEWPORTS"):
                _sensor_cameras = [
                    (
                        "Stereo Left",
                        "/World/envs/env_0/Robot/head_pitch_link/head_stereo_left/"
                        "head_stereo_left_Camera",
                    ),
                    (
                        "Stereo Right",
                        "/World/envs/env_0/Robot/head_pitch_link/head_stereo_right/"
                        "head_stereo_right_Camera",
                    ),
                    (
                        "Fisheye Left",
                        "/World/envs/env_0/Robot/head_pitch_link/head_fisheye_left/"
                        "head_fisheye_left_Camera",
                    ),
                    (
                        "Fisheye Right",
                        "/World/envs/env_0/Robot/head_pitch_link/head_fisheye_right/"
                        "head_fisheye_right_Camera",
                    ),
                    (
                        "Jaw Camera",
                        "/World/envs/env_0/Robot/head_pitch_link/head_jaw/head_jaw_Camera",
                    ),
                ]
                _sensor_viewports = []
                for index, (_title, _cam_path) in enumerate(_sensor_cameras):
                    if not env.sim.stage.GetPrimAtPath(_cam_path):
                        print(f"[WARN] Sensor camera prim is missing: {_cam_path}")
                        continue
                    _vp = create_viewport_window(
                        _title,
                        width=384,
                        height=300,
                        position_x=30 + (index % 3) * 400,
                        position_y=40 + (index // 3) * 330,
                    )
                    if hasattr(_vp.viewport_api, "set_active_camera"):
                        _vp.viewport_api.set_active_camera(_cam_path)
                    else:
                        _vp.viewport_api.camera_path = _cam_path
                    _sensor_viewports.append(_vp)
                    print(f"[INFO] Sensor viewport '{_title}' opened at {_cam_path}")
            else:
                _cam_path = "/World/envs/env_0/Robot/head_pitch_link/Camera_RGB"
                _vp = create_viewport_window(
                    "HeadCam (dataset view)", width=480, height=360, position_x=60, position_y=60
                )
                if hasattr(_vp.viewport_api, "set_active_camera"):
                    _vp.viewport_api.set_active_camera(_cam_path)
                else:
                    _vp.viewport_api.camera_path = _cam_path
                print(f"[INFO] Head-camera viewport opened at {_cam_path}")
        except Exception as exc:
            print(f"[WARN] Could not open the head-camera viewport: {exc}")

    if ROBOT == "walker_s2":
        print(f"[INFO] Walker S2 render/app device args_cli.device={args_cli.device}")
        print(f"[INFO] Walker S2 physics device args_cli.physics_device={args_cli.physics_device}")
        print(f"[INFO] Walker S2 physics env.device={env.device}")
        print(f"[INFO] Walker S2 physics env.cfg.sim.device={env.cfg.sim.device}")

    keyboard_reset = KeyboardResetController()
    rate_limiter = RateLimiter(args_cli.step_hz)
    perf_monitor = None if args_cli.load_only else (PerfMonitor() if args_cli.perf_stats else None)

    if args_cli.load_only:
        role = {"walker_c1": "Walker C1", "walker_s2": "Walker S2"}.get(ROBOT, "Tienkung Pro")
        print(f"[INFO] {role} load-only mode: ROS control and action preprocessing are disabled.")
        teleop_interface = None
    elif ROBOT == "walker_c1":
        from ubt_sim.devices.walker_c1 import WalkerC1Controller

        teleop_interface = WalkerC1Controller(
            env,
            cmd_port=args_cli.zmq_cmd_port,
            status_port=args_cli.zmq_status_port,
            image_port=args_cli.zmq_image_port,
            jpeg_image_port=args_cli.zmq_jpeg_image_port,
        )
        teleop_interface.display_controls()
    elif ROBOT == "walker_s2":
        from ubt_sim.devices.walker_s2 import WalkerS2Controller

        teleop_interface = WalkerS2Controller(
            env,
            cmd_port=args_cli.zmq_cmd_port,
            status_port=args_cli.zmq_status_port,
            image_port=args_cli.zmq_image_port,
            jpeg_image_port=args_cli.zmq_jpeg_image_port,
        )
        teleop_interface.display_controls()
    else:
        from ubt_sim.devices import TienkungProController

        teleop_interface = TienkungProController(env)
        teleop_interface.display_controls()

    env.reset()
    if ROBOT == "walker_s2":
        fix_walker_s2_head_material(env.sim.stage)
    if teleop_interface is not None:
        teleop_interface.reset()
    if args_cli.load_only:
        print("[INFO] Load-only app update enabled: physics/action/observation stepping is disabled.")
    rate_limiter.update_from_env(env)
    print(f"[INFO] RateLimiter sleep_duration={rate_limiter.sleep_duration:.6f}s")

    # --- Main loop ---
    interrupted = False

    def signal_handler(signum, frame):
        nonlocal interrupted
        interrupted = True
        print("\n[INFO] Ctrl+C detected. Cleaning up...")

    original_sigint_handler = signal.signal(signal.SIGINT, signal_handler)

    try:
        while simulation_app.is_running() and not interrupted:
            with torch.inference_mode():
                if args_cli.load_only:
                    if keyboard_reset.reset_requested:
                        print("[INFO] Resetting environment...")
                        env.sim.reset()
                        env.reset()
                        if ROBOT == "walker_s2":
                            fix_walker_s2_head_material(env.sim.stage)
                        keyboard_reset.reset_requested = False
                    simulation_app.update()
                    rate_limiter.sleep(env)
                    continue

                if keyboard_reset.reset_requested or teleop_interface.reset_requested:
                    print("[INFO] Resetting environment...")
                    env.sim.reset()
                    env.reset()
                    if ROBOT == "walker_s2":
                        fix_walker_s2_head_material(env.sim.stage)
                    teleop_interface.reset()
                    keyboard_reset.reset_requested = False

                if keyboard_reset.episode_complete_requested:
                    if ROBOT == "walker_c1":
                        teleop_interface.request_episode_complete()
                        print("[INFO] Space pressed: requesting episode save and reset...")
                    keyboard_reset.episode_complete_requested = False

                if perf_monitor is not None:
                    t_0 = time.perf_counter()
                    actions = teleop_interface.advance()
                    t_1 = time.perf_counter()
                    _apply_part_randomization_if_requested(env, teleop_interface)
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)
                    t_2 = time.perf_counter()
                else:
                    actions = teleop_interface.advance()
                    _apply_part_randomization_if_requested(env, teleop_interface)
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)

                if actions is None:
                    env.render()
                else:
                    env.step(actions)

                if perf_monitor is not None:
                    t_3 = time.perf_counter()
                    perf_monitor.record(
                        (t_1 - t_0) * 1000,
                        (t_2 - t_1) * 1000,
                        (t_3 - t_2) * 1000,
                    )
                    perf_monitor.maybe_print()

                rate_limiter.sleep(env)

            if interrupted:
                break
    except Exception as e:
        import traceback

        print(f"\n[ERROR] {e}\n")
        traceback.print_exc()
    finally:
        signal.signal(signal.SIGINT, original_sigint_handler)
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
