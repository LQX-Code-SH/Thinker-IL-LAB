"""TienKung Dex — 右臂 + 右手灵巧手（因时 / 强脑）。

Communicates with the robot backend via ZMQ through ros2_deploy_bridge.py,
which bridges to the robot through ROS2 DDS.

本轮只驱动**右臂 7 + 右手 13（因时）/ 6（强脑）**：数据里左臂/腰/头全程恒定，
左手未采；这些部位由 ``scripts/deploy/tienkung_dex/robot_ready.sh`` 摆到预备位后
保持不动（bridge 不碰 /waist/cmd 与 /leg/cmd，腿由运控独占）。

Joint layout is configured via TienKungDexRobotConfig.all_joints.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time

import zmq

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_tienkung_dex import TienKungDexRobotConfig
from .hand_utils import clip_hand_position

logger = logging.getLogger(__name__)

# Path where _start_bridge() writes the config JSON for external scripts to read.
_BRIDGE_CONFIG_PATH = "/tmp/tienkung_dex_bridge_config.json"

# 只认本机型的桥接脚本名。tienkung_pro 的桥接也叫 ros2_deploy_bridge.py，
# 按文件名后缀杀进程会把它一起带走，所以额外要求路径里带 tienkung_dex。
_BRIDGE_BASENAME = "ros2_deploy_bridge.py"
_BRIDGE_DIR_TAG = "tienkung_dex"


def _kill_orphan_bridges(expected_script: str = "") -> None:
    """Terminate any already-running tienkung_dex bridge processes.

    Matches only processes whose argv[1] is this robot's bridge script, NOT the
    lerobot-rollout main process, which carries the bridge path as the value of
    --robot.bridge_script=... but whose argv[1] is the lerobot entrypoint. Using
    pkill -f / pgrep -f here would match the lerobot main process's cmdline too
    and SIGTERM our own parent on startup.
    """
    own_pid = os.getpid()
    parent_pid = os.getppid()
    expected_real = os.path.realpath(expected_script) if expected_script else ""
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
        script = parts[1].decode("utf-8", "replace")
        if not script.endswith(_BRIDGE_BASENAME):
            continue
        if expected_real and os.path.realpath(script) != expected_real:
            continue
        if _BRIDGE_DIR_TAG not in script:
            continue
        pids.append(pid)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if pids:
        time.sleep(0.5)


class TienKungDexRobot(Robot):
    config_class = TienKungDexRobotConfig
    name = "tienkung_dex"

    def __init__(self, config: TienKungDexRobotConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        # Joint definitions from config (replace module-level constants)
        self._right_arm_joints = config.right_arm_joints
        self._right_hand_joints = config.right_hand_joints
        self._all_joints = config.all_joints
        # 非激活关节(不在 all_joints 中的硬件关节)的静态填充值,部署时用于
        # 把 policy 的子集 action 散射回完整硬件维度。键已带 .pos 后缀。
        self._inactive_fill: dict[str, float] = getattr(config, "_inactive_fill", {})

        # ZMQ state (populated in connect)
        self._zmq_context: zmq.Context | None = None
        self._cmd_socket: zmq.Socket | None = None
        self._status_socket: zmq.Socket | None = None
        self._bridge_process: subprocess.Popen | None = None

        # Thread-safe state caches
        self._state_lock = threading.Lock()
        self._right_arm_jpos: list[float] = [0.0] * len(config.right_arm_joints)
        self._right_hand_pos: list[float] = [0.0] * len(config.right_hand_joints)
        self._estop_engaged = False
        self._state_ready = threading.Event()

        # Status receive thread
        self._recv_thread: threading.Thread | None = None
        self._running = False

        self._connected = False

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        motors_ft = {name: float for name in self._all_joints}
        camera_ft = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
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
            target=self._status_recv_loop, daemon=True, name="tienkung_dex_status_recv"
        )
        self._recv_thread.start()

        # Wait for first status message
        logger.info("Waiting for Bridge2 status messages...")
        warmup_start = time.time()
        warmup_timeout = 10.0
        while time.time() - warmup_start < warmup_timeout:
            if self._state_ready.is_set():
                break
            time.sleep(0.1)

        if not self._state_ready.is_set():
            logger.warning("Timed out waiting for Bridge2 status messages.")

        self._connected = True
        logger.info("TienKungDexRobot connected.")

    def _start_bridge(self) -> None:
        # Stop any existing Bridge2 process first (avoid conflicts from auto-start)
        _kill_orphan_bridges(self.config.bridge_script)

        config_json = json.dumps(self.config.to_bridge_config())
        cmd = [
            "/usr/bin/python3", self.config.bridge_script,
            "--config", config_json,
        ]

        logger.info("Starting Bridge2: %s --config <json>", self.config.bridge_script)
        self._bridge_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Write config to a known location for external scripts (reset.py, robot_ready.py)
        try:
            with open(_BRIDGE_CONFIG_PATH, "w") as f:
                f.write(config_json)
        except OSError:
            logger.warning("Failed to write bridge config to %s", _BRIDGE_CONFIG_PATH)

        # Give bridge time to bind ZMQ ports
        time.sleep(1.0)

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
        right_arm = data.get("right_arm", [])
        right_hand = data.get("right_hand", [])

        if len(right_arm) >= len(self._right_arm_joints):
            with self._state_lock:
                self._right_arm_jpos[:] = right_arm[:len(self._right_arm_joints)]
        if len(right_hand) >= len(self._right_hand_joints):
            with self._state_lock:
                self._right_hand_pos[:] = right_hand[:len(self._right_hand_joints)]

        estop = bool(data.get("estop", False))
        if estop != self._estop_engaged:
            if estop:
                logger.error("急停已触发（is_estop=true）：Bridge2 已停止下发指令。")
            else:
                logger.info("急停已解除。")
            self._estop_engaged = estop

        self._state_ready.set()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        # 先构建完整硬件状态(2 组,物理通道序),再只返回 policy 关节(self._all_joints)。
        full: dict[str, float] = {}
        with self._state_lock:
            for i, name in enumerate(self._right_arm_joints):
                full[name] = self._right_arm_jpos[i]
            for i, name in enumerate(self._right_hand_joints):
                full[name] = self._right_hand_pos[i]

        obs: RobotObservation = {name: full[name] for name in self._all_joints}

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # action 仅含 policy 关节(self._all_joints);非激活关节用 _inactive_fill 填充,
        # 以组装完整硬件命令(2 组物理通道序)。按名查找,与 policy 顺序无关。
        inactive = self._inactive_fill

        def get_val(name: str) -> float:
            if name in action:
                return action[name]
            return inactive.get(name, 0.0)

        right_arm = [get_val(name) for name in self._right_arm_joints]
        right_hand = [get_val(name) for name in self._right_hand_joints]

        # Apply safety clipping if configured
        if self.config.max_relative_target is not None:
            with self._state_lock:
                current_right = list(self._right_arm_jpos)

            right_arm = self._clip_relative(right_arm, current_right, self.config.max_relative_target)

        # Publish action via ZMQ
        action_msg = {
            "right_arm": right_arm,
            "right_hand": right_hand,
            "ts": time.time(),
        }
        try:
            self._cmd_socket.send_json(action_msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("Action send dropped: ZMQ send buffer full (SNDHWM=1)")

        # Return the actual action sent (after clipping), only for policy joints.
        # 手部按真机通道分辨率量化，使返回值 == 硬件真正执行的值。
        right_hand_sent = clip_hand_position(
            right_hand, self.config.hand_variant, self._right_hand_joints
        )
        sent_map: dict[str, float] = {}
        for i, name in enumerate(self._right_arm_joints):
            sent_map[name] = right_arm[i]
        for i, name in enumerate(self._right_hand_joints):
            sent_map[name] = right_hand_sent[i]
        sent_action: RobotAction = {name: sent_map[name] for name in self._all_joints}
        return sent_action

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

    def hold_trajectory(self) -> None:
        """把当前实测位姿再下发一次，显式 hold 在这里。

        inference_server 的 ``stop`` 会调它。act_async 机型（walker）靠它截断桥接里
        已经排队的 chunk 轨迹；Dex 桥接没有 chunk 队列，``stop`` 之后本就不再有新目标，
        所以这里只是把「停在哪」变成一条显式指令，并消掉服务端那条 AttributeError 告警。
        """
        if not self._state_ready.is_set():
            return
        with self._state_lock:
            arm = list(self._right_arm_jpos)
            hand = list(self._right_hand_pos)
        try:
            self._cmd_socket.send_json(
                {"right_arm": arm, "right_hand": hand, "ts": time.time()},
                flags=zmq.NOBLOCK,
            )
        except zmq.Again:
            logger.warning("Hold action send dropped: ZMQ send buffer full")

    def _send_home_chunk(self, duration_s: float = 2.0, fps: float = 50.0) -> None:
        """平滑回 home：当前读数 -> ``config.home_position`` 的逐帧插值。

        inference_server 的 ``home`` 命令与 ``shutdown`` 都调这个方法，所以必须由机器人
        自己实现（上游只调 ``robot._send_home_chunk``，缺了就是 AttributeError）。

        与 walker 的差别：walker 推整块 chunk（``n_points``/``fps``/``chunk_id``），而 Dex
        桥接只认单步 ``{"right_arm":[7],"right_hand":[N]}``，整块会被当成维度不符丢掉
        （机器人不动）。所以这里按 ``fps`` 定时逐帧下发，桥接 ``_action_loop`` 是
        无 sleep 的 NOBLOCK 轮询，能逐帧消费。

        手指保持当前读数（不张开），与 walker「空组 hold」语义一致；直连 ``_cmd_socket``
        不经 ``send_action``，也就不套 ``max_relative_target``（与 walker home chunk 同），
        单步幅度由 ``1/(duration_s*fps)`` 的插值自然限制，落限位仍由桥接裁剪兜底。
        """
        # 没有真实状态时起点是 zeros，插值会先奔向零位（不是 home），手臂会甩向体侧。
        # 宁可报错让操作员看到，也不要按一个假的起点动。
        if not self._state_ready.is_set():
            raise RuntimeError(
                "尚未收到 Bridge2 状态，无法确定当前位姿，拒绝回 home"
            )

        with self._state_lock:
            start_arm = list(self._right_arm_jpos)
            hand = list(self._right_hand_pos)
        target = list(self.config.home_position)
        n = max(int(duration_s * fps), 1)
        period = 1.0 / fps

        for s in range(1, n + 1):
            deadline = time.monotonic() + period
            t = s / n
            arm = [start_arm[i] * (1 - t) + target[i] * t for i in range(len(target))]
            msg = {"right_arm": arm, "right_hand": hand, "ts": time.time()}
            try:
                self._cmd_socket.send_json(msg, flags=zmq.NOBLOCK)
            except zmq.Again:
                logger.warning("Home action send dropped: ZMQ send buffer full")
            time.sleep(max(0.0, deadline - time.monotonic()))

        logger.info("已回到 home_position（%.1fs @ %.0f Hz）", duration_s, fps)

    @check_if_not_connected
    def disconnect(self) -> None:
        # Optionally return to home position
        if self.config.return_home_on_disconnect and self._state_ready.is_set():
            logger.info("Returning to home position...")
            home_action = {
                "right_arm": list(self.config.home_position),
                "right_hand": list(self.config.hand_open_position),
                "ts": time.time(),
            }
            try:
                self._cmd_socket.send_json(home_action, flags=zmq.NOBLOCK)
            except zmq.Again:
                logger.warning("Home action send dropped: ZMQ send buffer full")
            time.sleep(1.0)

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

        # Terminate Bridge2 subprocess
        if self._bridge_process is not None:
            logger.info("Stopping Bridge2 subprocess...")
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
        logger.info("TienKungDexRobot disconnected.")
