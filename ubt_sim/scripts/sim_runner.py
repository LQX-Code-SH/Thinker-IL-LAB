# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run ubt_sim simulation environments (Tienkung Pro / Walker S2)."""

import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import signal
import time

import carb
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="UBT Sim simulation environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="UBTSim-TienkungPro-Parlor-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument("--perf_stats", action="store_true", help="Print performance statistics.")
parser.add_argument("--load_only", action="store_true", help="Load and render the environment without ROS control.")
# Walker S2 specific (ignored for Tienkung Pro)
parser.add_argument("--zmq_cmd_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_CMD_PORT", 5655)))
parser.add_argument("--zmq_status_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_STATUS_PORT", 5656)))
parser.add_argument("--zmq_image_port", type=int, default=int(os.environ.get("UBT_SIM_WALKER_S2_IMAGE_PORT", 5657)))
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

app_launcher = AppLauncher(
    vars(args_cli),
    width=int(os.environ.get("UBT_SIM_VIEWPORT_WIDTH", "640")),
    height=int(os.environ.get("UBT_SIM_VIEWPORT_HEIGHT", "360")),
)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

from ubt_sim.utils.head_material import fix_walker_s2_head_material
from ubt_sim.utils.loop_utils import KeyboardResetController, PerfMonitor, RateLimiter


def patch_step_for_profiling(env):
    """Monkey-patch env to collect internal step() timing breakdown.

    Wraps three key bottleneck methods with torch.cuda.synchronize() timers:
      - sim.step()      → physics simulation (GPU)
      - sim.render()    → GPU rendering submission
      - observation_manager.compute() → camera data GPU→CPU readback

    Returns (timings_dict, reset_fn) tuple.
    """
    timings: dict[str, float] = {}

    # -- sim.step(render=False) : physics
    _orig_sim_step = env.sim.step

    def _timed_sim_step(render=False):
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        ret = _orig_sim_step(render=render)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        timings["phys"] = timings.get("phys", 0.0) + (t1 - t0) * 1000
        return ret

    env.sim.step = _timed_sim_step

    # -- sim.render() : GPU rendering
    _orig_sim_render = env.sim.render

    def _timed_sim_render():
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        ret = _orig_sim_render()
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        timings["render"] = timings.get("render", 0.0) + (t1 - t0) * 1000
        return ret

    env.sim.render = _timed_sim_render

    # -- observation_manager.compute() : camera GPU→CPU readback
    _orig_obs_compute = env.observation_manager.compute

    def _timed_obs_compute(*args, **kwargs):
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        ret = _orig_obs_compute(*args, **kwargs)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        timings["obs"] = timings.get("obs", 0.0) + (t1 - t0) * 1000
        return ret

    env.observation_manager.compute = _timed_obs_compute

    def reset_timings():
        for k in list(timings.keys()):
            timings[k] = 0.0

    return timings, reset_timings


def _detect_robot(task_name: str | None) -> str:
    """Infer robot type from task name."""
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


def _ensure_rendering_defaults():
    """Apply rendering settings on every startup, sourced from the user's saved
    USD file if available, otherwise falling back to a full performance preset.

    The Isaac Sim UI "Render Settings" panel saves to a USD file.  This function
    reads that file on every startup so the user's saved preferences survive
    restarts automatically.
    """
    carb_settings = carb.settings.get_settings()
    usd_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "assets", "render_settings", "default.settings.usd",
    )

    # --- Try loading user-saved render settings from USD ---
    loaded_from_usd = False
    try:
        # pxr.Usd lives inside Isaac Sim's ext cache; probe common locations.
        try:
            from pxr import Usd  # type: ignore[import-untyped]
        except ImportError:
            import glob as _glob

            _ext_base = os.path.join(
                os.environ.get("ISAAC_SIM_PATH", "/isaac-sim"), "extscache",
            )
            _candidates = _glob.glob(
                os.path.join(_ext_base, "omni.usd.libs-*", "pxr", "Usd")
            )
            for _c in sorted(_candidates, reverse=True):
                _pkg_root = os.path.dirname(os.path.dirname(_c))
                if _pkg_root not in sys.path:
                    sys.path.insert(0, _pkg_root)
                break
            from pxr import Usd  # type: ignore[import-untyped]

        stage = Usd.Stage.Open(usd_path)
        prim = stage.GetPrimAtPath("/renderSettings")
        if prim:
            custom_data = prim.GetAttribute("customLayerData")
            if custom_data and custom_data.Get():
                data = custom_data.Get()
                if isinstance(data, dict):
                    for key, val in data.items():
                        carb_key = "/" + key.replace(".", "/")
                        carb_settings.set(carb_key, val)
                    print(
                        "[INFO] Loaded %d render settings from %s"
                        % (len(data), usd_path)
                    )
                    loaded_from_usd = True
    except ImportError:
        print("[INFO] pxr.Usd not available, using performance fallback.")
    except Exception:
        print("[INFO] Could not load USD render settings, using performance fallback.")

    # --- Fallback: apply EVERY key from performance.kit ---
    if not loaded_from_usd:
        _apply_performance_preset(carb_settings)

    # Safety: always enforce RaytracedLighting (PathTracing is 5-10x slower)
    if carb_settings.get("/rtx/rendermode") != "RaytracedLighting":
        carb_settings.set("/rtx/rendermode", "RaytracedLighting")
        print("[INFO] Enforced /rtx/rendermode = RaytracedLighting.")


def _apply_performance_preset(carb_settings):
    """Apply the complete set of rtx performance preset keys.

    Mirrors Isaac Lab's ``performance.kit`` rendering preset to eliminate the
    overhead of denoisers, sampled-lighting, reflections, translucency, and
    ambient occlusion.
    """
    # fmt: off
    perf_keys = {
        "/rtx/rendermode":                                         "RaytracedLighting",
        "/rtx/translucency/enabled":                               False,
        "/rtx/reflections/enabled":                                False,
        "/rtx/reflections/denoiser/enabled":                       False,
        "/rtx/directLighting/sampledLighting/denoisingTechnique":  0,
        "/rtx/directLighting/sampledLighting/enabled":             False,
        "/rtx/sceneDb/ambientLightIntensity":                      1.0,
        "/rtx/shadows/enabled":                                    True,
        "/rtx/indirectDiffuse/enabled":                            True,
        "/rtx/indirectDiffuse/denoiser/enabled":                   False,
        "/rtx/domeLight/upperLowerStrategy":                       3,
        "/rtx/ambientOcclusion/enabled":                           False,
        "/rtx/ambientOcclusion/denoiserMode":                      1,
        "/rtx/raytracing/subpixel/mode":                           0,
        "/rtx/raytracing/cached/enabled":                          False,
        "/rtx-transient/dlssg/enabled":                            False,
        "/rtx-transient/dldenoiser/enabled":                       False,
        "/rtx/post/dlss/execMode":                                 0,
        "/rtx/pathtracing/maxSamplesPerLaunch":                    1000000,
        "/rtx/viewTile/limit":                                     1000000,
    }
    # fmt: on
    for key, val in perf_keys.items():
        carb_settings.set(key, val)
    print("[INFO] Applied full performance preset (%d keys)." % len(perf_keys))


def main():
    # Resolve physics device: Walker S2 uses a dedicated flag, Tienkung Pro uses the
    # AppLauncher device (which may also come from env).
    physics_device = args_cli.physics_device if ROBOT == "walker_s2" else args_cli.device
    # 初始化段（gym.make/env.reset/控制器构造）若抛异常，原本不会进入下方 try，
    # 导致 Isaac Sim app + 相机子进程残留占 GPU。把初始化也纳入 try/finally。
    env = None
    teleop_interface = None
    original_sigint_handler = None
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=physics_device, num_envs=args_cli.num_envs)
        env_cfg.use_teleop_device(ROBOT)
        env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
        env_cfg.recorders = None

        env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

        # Pin rendering carb settings so the UI defaults survive restarts.
        _ensure_rendering_defaults()

        if ROBOT == "walker_s2":
            print(f"[INFO] Walker S2 render/app device args_cli.device={args_cli.device}")
            print(f"[INFO] Walker S2 physics device args_cli.physics_device={args_cli.physics_device}")
            print(f"[INFO] Walker S2 physics env.device={env.device}")
            print(f"[INFO] Walker S2 physics env.cfg.sim.device={env.cfg.sim.device}")

        keyboard_reset = KeyboardResetController()
        rate_limiter = RateLimiter(args_cli.step_hz)
        perf_monitor = None if args_cli.load_only else (PerfMonitor() if args_cli.perf_stats else None)

        # Profiling: monkey-patch env.step internals if perf_stats enabled
        step_timings = None
        reset_step_timings = None
        if perf_monitor is not None:
            step_timings, reset_step_timings = patch_step_for_profiling(env)

        if args_cli.load_only:
            role = "Walker S2" if ROBOT == "walker_s2" else "Tienkung Pro"
            print(f"[INFO] {role} load-only mode: ROS control and action preprocessing are disabled.")
            teleop_interface = None
        elif ROBOT == "walker_s2":
            from ubt_sim.devices.walker_s2 import WalkerS2Controller

            teleop_interface = WalkerS2Controller(
                env,
                cmd_port=args_cli.zmq_cmd_port,
                status_port=args_cli.zmq_status_port,
                image_port=args_cli.zmq_image_port,
                camera_names=env.cfg.camera_names,
                render_interval=env.cfg.sim.render_interval,
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
        print("[INFO] Viewport resolution reduced via UBT_SIM_VIEWPORT_WIDTH/HEIGHT env vars (default 640x360)")

        # --- Main loop ---
        interrupted = False

        def signal_handler(signum, frame):
            nonlocal interrupted
            interrupted = True
            print("\n[INFO] Ctrl+C detected. Cleaning up...")

        original_sigint_handler = signal.signal(signal.SIGINT, signal_handler)

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

                if perf_monitor is not None:
                    t_0 = time.perf_counter()
                    actions = teleop_interface.advance()
                    advance_detail = getattr(teleop_interface, "advance_timings", None)
                    t_1 = time.perf_counter()
                    _apply_part_randomization_if_requested(env, teleop_interface)
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)
                    t_2 = time.perf_counter()
                else:
                    actions = teleop_interface.advance()
                    _apply_part_randomization_if_requested(env, teleop_interface)
                    actions = env.cfg.preprocess_device_action(actions, teleop_interface)

                env_step_detail = None
                if actions is None:
                    env.render()
                else:
                    if reset_step_timings is not None:
                        reset_step_timings()
                    env.step(actions)
                    env_step_detail = dict(step_timings) if step_timings is not None else None

                if perf_monitor is not None:
                    t_3 = time.perf_counter()
                    perf_monitor.record(
                        (t_1 - t_0) * 1000,
                        (t_2 - t_1) * 1000,
                        (t_3 - t_2) * 1000,
                        advance_detail=advance_detail,
                        env_step_detail=env_step_detail,
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
        if original_sigint_handler is not None:
            signal.signal(signal.SIGINT, original_sigint_handler)
        if teleop_interface is not None:
            try:
                teleop_interface.close()
            except Exception:
                pass
        if env is not None:
            env.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
