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
        # Relay subprocess stdout/stderr -> 文件（原 DEVNULL 会吞掉所有诊断信息）
        self._camera_relay_log = None

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
        self._camera_relay_log = open("/tmp/walker_camera_relay.log", "ab", buffering=0)
        self._camera_relay_process = subprocess.Popen(
            cmd,
            stdout=self._camera_relay_log,
            stderr=subprocess.STDOUT,
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
        # 路由实际位置到 recorder（60Hz 降采样落盘 actual.csv，供三轨迹对比绘图）
        if self._recorder is not None:
            self._recorder.record_actual(data)

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

    @check_if_not_connected
    def send_action_chunk(
        self,
        action_chunk: dict[str, list[float]],
        *,
        inference_time_sec: float = 0.0,
        obs_time_sec: float = 0.0,
        chunk_id: int = 0,
        fps: float | None = None,
        record: bool = True,
    ) -> None:
        """把整块 action chunk 推给桥接（chunk-to-bridge 模式）。

        与 ``send_action`` 的区别：下发的是一整段 chunk（C 个时刻），而非单步动作。
        桥接按消息含 ``n_points`` 字段分流到 chunk 消费路径（融合/插值/滤波/300Hz
        下发）；不含 ``n_points`` 的旧消息仍走单动作 PD 路径（向后兼容）。

        Args:
            action_chunk: 键为 feature 名（.pos），值为该 feature 的 C 长度列。
                非激活关节（不在 action_chunk 中的硬件关节）用 ``_inactive_fill``
                填充为常量列，与单动作语义一致。
            inference_time_sec: 本次推理耗时（桥接据此做延迟补偿，跳过过期前缀）。
            obs_time_sec: obs 消费时刻（wall clock）；桥接用 L = ts - obs_time_sec 算精确
                延迟（含后处理+发送，比 inference_time_sec 更全），并做 temporal ensemble
                跨块时刻对齐。0 表示未知（退回用 inference_time_sec）。
            chunk_id: chunk 序号（诊断用）。
            fps: chunk 相邻点间距 = 1/fps（桥接据此加密到 300Hz）。
            record: 是否落盘 chunks.csv（策略诊断）。关机回原点等非策略移动传 False，
                避免污染 chunks.png predicted 段。

        本方法只做分组 + 传输；安全 clamp（body 限位 / hand clip）由桥接在
        chunk 消费时统一执行（与单动作路径的桥接 clamp 一致）。
        """
        import numpy as np

        inactive = self._inactive_fill

        # 确定 chunk 长度 C（从任意一个非空列取）
        C = 0
        for col in action_chunk.values():
            if col is not None:
                C = len(col)
                break
        if C == 0:
            return None

        def col_for(name: str) -> list[float]:
            col = action_chunk.get(name)
            if col is not None:
                return [float(v) for v in col]
            fill = float(inactive.get(name, 0.0))
            return [fill] * C

        # 每组构建 [C, len(group)]：每行是一个时刻该组所有关节的值
        grouped: dict[str, list[list[float]]] = {}
        for group, features in self._group_features.items():
            cols = [col_for(name) for name in features]  # [len(group), C]
            grouped[group] = np.array(cols, dtype=float).T.tolist()  # [C, len(group)]

        action_msg = {
            "left_arm": grouped["left_arm"],
            "right_arm": grouped["right_arm"],
            "head": grouped["head"],
            "waist": grouped["waist"],
            "left_hand": grouped["left_hand"],
            "right_hand": grouped["right_hand"],
            "n_points": C,
            "fps": float(fps) if fps is not None else 0.0,
            "inference_time_sec": float(inference_time_sec),
            "obs_time_sec": float(obs_time_sec),
            "chunk_id": int(chunk_id),
            "ts": time.time(),
        }

        # 录制 chunk（act_async 诊断）：与单动作 record 互补，落盘 chunks.csv。
        # act_async 模式下 rollout 期间只走本路径，单动作 record 不会触发，
        # 故必须在此录制才能看到策略实际产出的 chunk（hold 还是真实轨迹）。
        # record=False 用于关机回原点等非策略移动，避免污染 chunks.png predicted 段。
        if record and self._recorder is not None:
            with self._state_lock:
                current_state = {group: list(self._group_state[group]) for group in self._group_features}
            self._recorder.record_chunk(
                action_msg=action_msg,
                current_state=current_state,
                timestamp=action_msg["ts"],
            )

        try:
            self._cmd_socket.send_json(action_msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("Chunk send dropped: ZMQ send buffer full (SNDHWM=1)")
        return None

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

    def _send_home_chunk(self, duration_s: float = 2.0, fps: float = 50.0) -> None:
        """平滑回 home：构造 current->home 插值 chunk 推桥接，300Hz densify 执行后 hold home。

        关机回原点走 chunk 路径（而非单动作），与 act_async 的 300Hz 轨迹发布线程保持
        单一指令源，避免双源拉锯抖动。复用 ``_home_action()`` 的分组目标：10d 等子集模型
        下 left_arm/waist 为空组，桥接 ``_chunk_groups_to_body_array`` 跳过 -> SDK hold，
        与单动作 home 语义一致。直连 ``_cmd_socket`` 发送（不经 send_action_chunk）-> 不污染
        chunks.csv（predicted）；桥接 ``_handle_chunk`` 仍写 fused.csv（home 轨迹可诊断）。
        ``chunk_id=-1`` 哨兵；``obs_time_sec=now`` -> 延迟补偿 L≈0、skip=0。
        """
        home = self._home_action()
        C = max(int(duration_s * fps), 1)
        with self._state_lock:
            current = {g: list(self._group_state[g]) for g in self._group_features}
        grouped: dict[str, list[list[float]]] = {}
        for group, feats in self._group_features.items():
            cur = current[group]
            tgt = home.get(group, cur)
            # [C, len(group)]：每行一个时刻该组所有关节的插值
            grouped[group] = [
                [cur[i] * (1 - t) + float(tgt[i]) * t for i in range(len(feats))]
                for t in (s / C for s in range(1, C + 1))
            ]
        msg = {
            **grouped,
            "n_points": C,
            "fps": float(fps),
            "inference_time_sec": 0.0,
            "obs_time_sec": time.time(),
            "chunk_id": -1,
            "ts": time.time(),
        }
        try:
            self._cmd_socket.send_json(msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("Home chunk send dropped: ZMQ send buffer full")

    @check_if_not_connected
    def disconnect(self) -> None:
        # 平滑回 home：走 chunk 路径（300Hz densify 单源），避免与桥接 300Hz hold 线程
        # 双源冲突抖动。recv 线程仍在运行 -> actual.csv 捕获 home 平滑段。
        if self.config.disable_torque_on_disconnect and self._state_ready.is_set():
            logger.info("Returning to home position (smooth chunk)...")
            try:
                self._send_home_chunk(duration_s=2.0, fps=50.0)
                time.sleep(2.0)  # 等 300Hz densify 执行完
            except Exception as e:
                logger.warning("Home chunk failed: %s", e)

        # 落盘 CSV（快）：在 kill 桥接前 save，actual.csv 含 home 平滑段。PNG 渲染
        # （plot/plot_chunks，~22s）延后到桥接终止后，避免桥接 300Hz 线程存活期间
        # 与 hold 指令长期冲突（旧实现在此处直接画图，拖 ~27s 抖动）。
        if self._recorder is not None:
            try:
                self._recorder.stop_live_plot()
                self._recorder.save()
            except Exception as e:
                logger.error("ActionRecorder save failed: %s", e)

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
        if self._camera_relay_log is not None:
            try:
                self._camera_relay_log.close()
            except Exception:
                pass
            self._camera_relay_log = None

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

        # PNG 渲染（慢，~22s）：桥接已终止、300Hz 线程死亡，机器人 SDK hold 在 home，
        # 无双源冲突。读内存缓冲（save 后缓冲仍在），与桥接存活无关。
        if self._recorder is not None:
            try:
                self._recorder.plot()
                self._recorder.plot_chunks()
            except Exception as e:
                logger.error("ActionRecorder plot failed: %s", e)

        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        self._connected = False
        logger.info("WalkerRobot disconnected.")
