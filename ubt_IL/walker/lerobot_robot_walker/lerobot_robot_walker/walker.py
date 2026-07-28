"""WalkerRobot — LeRobot Robot implementation for Walker S2 humanoid.

Communication via ZMQ to ros2_walker_bridge.py (Bridge2), which talks to
Walker S2 hardware via ROS2 DDS using mc_task_msgs / ecat_task_msgs.
"""

from __future__ import annotations

import json
import logging
import os
import shlex
import signal
import subprocess
import threading
import time

import zmq

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .action_recorder import ActionRecorder
from .config_walker import WalkerRobotConfig
from .hand_utils import clip_hand_value

logger = logging.getLogger(__name__)

# Path where _start_bridge() writes the config JSON for external scripts to read.
_BRIDGE_CONFIG_PATH = "/tmp/walker_bridge_config.json"


def _kill_orphan_bridges() -> None:
    """Terminate any already-running ros2_walker_bridge.py processes.

    Matches only processes whose argv[1] is ros2_walker_bridge.py (the script
    being executed), NOT the lerobot-rollout main process, which carries the
    bridge path as the value of --robot.bridge_script=... but whose argv[1] is
    the lerobot entrypoint. Using pkill -f / pgrep -f here would match the
    lerobot main process's cmdline too and SIGTERM our own parent on startup.
    """
    own_pid = os.getpid()
    parent_pid = os.getppid()
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (own_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        if parts[1].decode("utf-8", "replace").endswith("ros2_walker_bridge.py"):
            pids.append(pid)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if pids:
        time.sleep(0.5)


def _kill_orphan_camera_relays() -> None:
    """Terminate any already-running walker_camera_relay.py processes."""
    own_pid = os.getpid()
    parent_pid = os.getppid()
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (own_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        if parts[1].decode("utf-8", "replace").endswith("walker_camera_relay.py"):
            pids.append(pid)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if pids:
        time.sleep(0.5)


class WalkerRobot(Robot):
    config_class = WalkerRobotConfig
    name = "walker"

    def __init__(self, config: WalkerRobotConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        # Joint definitions from config. Feature names are LeRobot-facing .pos keys.
        self._left_arm_joints = config.left_arm_joints
        self._right_arm_joints = config.right_arm_joints
        self._head_joints = config.head_joints
        self._waist_joints = config.waist_joints
        self._left_hand_joints = config.left_hand_joints
        self._right_hand_joints = config.right_hand_joints
        self._all_joints = config.all_joints
        # 非激活关节(不在 all_joints 中的硬件关节)的静态填充值,部署时用于
        # 把 policy 的子集 action 散射回完整 6 组 bridge 命令。键已带 .pos 后缀。
        self._inactive_fill: dict[str, float] = getattr(config, "_inactive_fill", {})

        # Real joint/actuator names used for hardware-side clipping/mapping.
        self._body_groups = config.body_groups
        self._left_hand_joint_names = config.left_hand_joint_names
        self._right_hand_joint_names = config.right_hand_joint_names
        self._lock_joints = set(config.lock_joints)

        self._group_features = {
            "left_arm": self._left_arm_joints,
            "right_arm": self._right_arm_joints,
            "head": self._head_joints,
            "waist": self._waist_joints,
            "left_hand": self._left_hand_joints,
            "right_hand": self._right_hand_joints,
        }
        self._body_group_names = {
            "left_arm": list(self._body_groups.get("left_arm", [])),
            "right_arm": list(self._body_groups.get("right_arm", [])),
            "head": list(self._body_groups.get("head", [])),
            "waist": list(self._body_groups.get("waist", [])),
        }
        self._hand_group_names = {
            "left_hand": self._left_hand_joint_names,
            "right_hand": self._right_hand_joint_names,
        }

        # ZMQ state (populated in connect)
        self._zmq_context: zmq.Context | None = None
        self._cmd_socket: zmq.Socket | None = None
        self._status_socket: zmq.Socket | None = None
        self._bridge_process: subprocess.Popen | None = None
        self._camera_relay_process: subprocess.Popen | None = None

        # Thread-safe state caches (6 groups)
        self._state_lock = threading.Lock()
        self._group_state: dict[str, list[float]] = {
            group: [0.0] * len(features) for group, features in self._group_features.items()
        }
        self._state_ready = threading.Event()

        # Status receive thread
        self._recv_thread: threading.Thread | None = None
        self._running = False

        self._connected = False

        # Action recorder (env-var gated, lightweight CSV + plot)
        self._recorder: ActionRecorder | None = None
        if os.environ.get("RECORD_ACTIONS", "0") == "1":
            output_dir = os.environ.get("RECORD_OUTPUT_DIR", "/tmp/walker_rollout")
            self._recorder = ActionRecorder(
                output_dir,
                group_features=self._group_features,
                body_group_names=self._body_group_names,
                hand_group_names=self._hand_group_names,
            )

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        motors_ft = {name: float for name in self._all_joints}
        c2i = self.config._camera_to_image_key
        camera_ft = {
            c2i.get(cam, cam): (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }
        return {**motors_ft, **camera_ft}

    @property
    def action_features(self) -> dict[str, type]:
        return {name: float for name in self._all_joints}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        # Start Bridge2 subprocess if enabled
        if self.config.bridge_enabled:
            self._start_bridge()
            if self.config.camera_topics:
                self._start_camera_relay()

        # Create ZMQ context and sockets
        self._zmq_context = zmq.Context()
        host = self.config.zmq_host

        # PUB: send actions to Bridge2
        self._cmd_socket = self._zmq_context.socket(zmq.PUB)
        self._cmd_socket.connect(f"tcp://{host}:{self.config.zmq_cmd_port}")
        self._cmd_socket.setsockopt(zmq.SNDHWM, 1)

        # SUB: receive status from Bridge2
        self._status_socket = self._zmq_context.socket(zmq.SUB)
        self._status_socket.connect(f"tcp://{host}:{self.config.zmq_status_port}")
        self._status_socket.setsockopt(zmq.RCVHWM, 1)
        self._status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Connect cameras (they create their own ZMQ SUB sockets for images)
        for cam in self.cameras.values():
            cam.connect()

        # Start status receive thread
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._status_recv_loop, daemon=True, name="walker_status_recv"
        )
        self._recv_thread.start()

        # Wait for first status message
        logger.info("Waiting for Walker Bridge2 status messages...")
        warmup_start = time.time()
        warmup_timeout = 10.0
        while time.time() - warmup_start < warmup_timeout:
            if self._state_ready.is_set():
                break
            time.sleep(0.1)

        if not self._state_ready.is_set():
            logger.warning("Timed out waiting for Walker Bridge2 status messages.")

        # Start live plot if recorder is active
        if self._recorder is not None:
            self._recorder.start_live_plot()

        self._connected = True
        logger.info("WalkerRobot connected.")

    def _start_bridge(self) -> None:
        # Stop any existing Bridge2 process first (avoid conflicts from auto-start)
        _kill_orphan_bridges()

        config_json = json.dumps(self.config.to_bridge_config())
        cmd = [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
            "source /ubt_IL/walker/walker_sdk_ros2/install/setup.bash 2>/dev/null || true; "
            f"exec /usr/bin/python3 {shlex.quote(self.config.bridge_script)} --config {shlex.quote(config_json)}",
        ]

        logger.info("Starting Walker Bridge2: %s --config <json>", self.config.bridge_script)
        self._bridge_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Write config to a known location for external scripts
        try:
            with open(_BRIDGE_CONFIG_PATH, "w") as f:
                f.write(config_json)
        except OSError:
            logger.warning("Failed to write bridge config to %s", _BRIDGE_CONFIG_PATH)

        # Give bridge time to bind ZMQ ports
        time.sleep(1.0)

    def _start_camera_relay(self) -> None:
        """Start standalone camera relay subprocess (ROS2 shm_msgs → ZMQ JPEG)."""
        from pathlib import Path

        _kill_orphan_camera_relays()

        camera_config = json.dumps({
            "zmq_image_port": self.config.zmq_image_port,
            "camera_topics": self.config.camera_topics,
            "ros_namespace": self.config.ros_namespace,
        })
        script = Path(__file__).resolve().parent.parent.parent / "walker_camera_relay.py"
        cmd = [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
            "source /ubt_IL/walker/walker_sdk_ros2/install/setup.bash 2>/dev/null || true; "
            f"exec /usr/bin/python3 {shlex.quote(str(script))} --config {shlex.quote(camera_config)}",
        ]

        logger.info("Starting Walker Camera Relay: %s", script)
        self._camera_relay_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _status_recv_loop(self) -> None:
        while self._running:
            try:
                msg = self._status_socket.recv_json(flags=zmq.NOBLOCK)
                self._process_status(msg)
            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                logger.error("Status receive error (non-fatal): %s", e)
                time.sleep(0.01)

    def _process_status(self, data: dict) -> None:
        with self._state_lock:
            for group, features in self._group_features.items():
                values = data.get(group, [])
                if len(values) >= len(features):
                    self._group_state[group][:] = values[:len(features)]
        self._state_ready.set()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        with self._state_lock:
            values_by_feature = {}
            for group, features in self._group_features.items():
                state = self._group_state[group]
                for i, name in enumerate(features):
                    values_by_feature[name] = state[i]

        obs: RobotObservation = {name: values_by_feature[name] for name in self._all_joints}

        # Capture images from cameras, applying camera_to_image_key mapping
        # so that observation.images.<key> matches model's input_features.
        c2i = self.config._camera_to_image_key
        for cam_key, cam in self.cameras.items():
            obs_key = c2i.get(cam_key, cam_key)
            obs[obs_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # action 仅含策略关节(self._all_joints);非激活关节用 _inactive_fill 填充,
        # 以组装完整 6 组 bridge 命令(物理序)。按名查找,与 policy 顺序无关。
        inactive = self._inactive_fill

        def get_val(name: str) -> float:
            if name in action:
                return float(action[name])
            return float(inactive.get(name, 0.0))

        grouped = {
            group: [get_val(name) for name in features]
            for group, features in self._group_features.items()
        }

        # Apply safety clipping for body joints if configured.
        if self.config.max_relative_target is not None:
            with self._state_lock:
                current = {group: list(self._group_state[group]) for group in self._body_group_names}
            for group in self._body_group_names:
                grouped[group] = self._clip_relative(
                    grouped[group], current[group], self.config.max_relative_target
                )

        # Apply body joint limit clipping by real ROS joint names.
        for group, joint_names in self._body_group_names.items():
            grouped[group] = self._clamp_body(grouped[group], joint_names)

        # Apply end-effector clipping.
        for group, joint_names in self._hand_group_names.items():
            grouped[group] = [
                clip_hand_value(
                    value,
                    joint_name,
                    self.config.hand_type,
                    self.config.gripper_position_limits,
                )
                for value, joint_name in zip(grouped[group], joint_names)
            ]

        # Snapshot current state for recording (under lock)
        current_state: dict[str, list[float]] = {}
        if self._recorder is not None:
            with self._state_lock:
                current_state = {group: list(self._group_state[group]) for group in self._group_features}

        action_msg = {
            "left_arm": grouped["left_arm"],
            "right_arm": grouped["right_arm"],
            "head": grouped["head"],
            "waist": grouped["waist"],
            "left_hand": grouped["left_hand"],
            "right_hand": grouped["right_hand"],
            "ts": time.time(),
        }

        if self._recorder is not None:
            self._recorder.record(
                action_input=action,
                action_msg=action_msg,
                current_state=current_state,
                timestamp=action_msg["ts"],
            )

        try:
            self._cmd_socket.send_json(action_msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("Action send dropped: ZMQ send buffer full (SNDHWM=1)")

        sent_by_feature = {}
        for group, features in self._group_features.items():
            for i, name in enumerate(features):
                sent_by_feature[name] = grouped[group][i]
        return {name: sent_by_feature[name] for name in self._all_joints}

    @staticmethod
    def _clip_relative(
        goal: list[float], current: list[float], max_diff: float
    ) -> list[float]:
        import numpy as np

        result = []
        for g, c in zip(goal, current):
            diff = np.clip(g - c, -max_diff, max_diff)
            result.append(c + diff)
        return result

    def _clamp_body(self, values: list[float], joint_names: list[str]) -> list[float]:
        """Clamp body joint values to their limits."""
        result = []
        for val, name in zip(values, joint_names):
            if name in self.config.body_joint_limits:
                lo, hi = self.config.body_joint_limits[name]
                val = max(lo, min(hi, float(val)))
            result.append(val)
        return result

    def _home_action(self) -> dict:
        body_home = {}
        offset = 0
        for group in ("left_arm", "right_arm", "head", "waist"):
            n = len(self._group_features[group])
            body_home[group] = self.config.home_position[offset:offset + n]
            offset += n
        return {
            **body_home,
            "left_hand": list(self.config.left_hand_open_position or []),
            "right_hand": list(self.config.right_hand_open_position or []),
            "ts": time.time(),
        }

    @check_if_not_connected
    def disconnect(self) -> None:
        # Optionally return to home position
        if self.config.disable_torque_on_disconnect and self._state_ready.is_set():
            logger.info("Returning to home position...")
            try:
                self._cmd_socket.send_json(self._home_action(), flags=zmq.NOBLOCK)
            except zmq.Again:
                logger.warning("Home action send dropped: ZMQ send buffer full")
            time.sleep(1.0)

        # Finalize action recorder (stop live plot → save CSV → plot PNG)
        if self._recorder is not None:
            try:
                self._recorder.stop_live_plot()
                self._recorder.save()
                self._recorder.plot()
            except Exception as e:
                logger.error("ActionRecorder finalization failed: %s", e)

        # Stop receive thread
        self._running = False
        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=3.0)
            self._recv_thread = None

        # Close ZMQ sockets
        if self._cmd_socket is not None:
            self._cmd_socket.close()
            self._cmd_socket = None
        if self._status_socket is not None:
            self._status_socket.close()
            self._status_socket = None
        if self._zmq_context is not None:
            self._zmq_context.term()
            self._zmq_context = None

        # Terminate camera relay subprocess (before Bridge)
        if self._camera_relay_process is not None:
            logger.info("Stopping Walker Camera Relay...")
            self._camera_relay_process.terminate()
            try:
                self._camera_relay_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._camera_relay_process.kill()
                self._camera_relay_process.wait(timeout=2.0)
            self._camera_relay_process = None

        # Terminate Bridge2 subprocess
        if self._bridge_process is not None:
            logger.info("Stopping Walker Bridge2 subprocess...")
            self._bridge_process.terminate()
            try:
                self._bridge_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bridge_process.kill()
                self._bridge_process.wait(timeout=2.0)
            self._bridge_process = None

        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        self._connected = False
        logger.info("WalkerRobot disconnected.")
