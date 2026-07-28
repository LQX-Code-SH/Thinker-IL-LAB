from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig

from .camera.config_walker_camera import WalkerCameraConfig
from .constants import (
    BODY_JOINT_LIMITS,
    BODY_JOINT_NAMES,
    DEFAULT_CAMERA_FPS,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_MSG_TYPE,
    DEFAULT_CAMERA_TIMEOUT_MS,
    DEFAULT_CAMERA_WARMUP_S,
    DEFAULT_CAMERA_WIDTH,
    DEFAULT_LOCK_JOINTS,
    HAND_OPEN_POSITION,
    HEAD_JOINTS,
    HOME_POSITION,
    LEFT_ARM_JOINTS,
    READY_POSE,
    RIGHT_ARM_JOINTS,
    ROBOT_MODELS,
    TOPIC_BODY_CMD,
    TOPIC_BODY_STATE,
    TOPIC_CAMERA_STEREO,
    TOPIC_LEFT_HAND_CMD,
    TOPIC_LEFT_HAND_STATE,
    TOPIC_RIGHT_HAND_CMD,
    TOPIC_RIGHT_HAND_STATE,
    TOPIC_LEFT_GRIP_CMD,
    TOPIC_LEFT_GRIP_STATE,
    TOPIC_RIGHT_GRIP_CMD,
    TOPIC_RIGHT_GRIP_STATE,
    V4_HAND_JOINT_LIMITS,
    V4_HAND_LEFT_JOINTS,
    V4_HAND_RIGHT_JOINTS,
    WAIST_JOINTS,
    inactive_fill_for,
    joint_names_with_pos,
)

_BODY_GROUPS = ("left_arm", "right_arm", "head", "waist")
_END_EFFECTOR_GROUPS = ("left_hand", "right_hand")
_PLACEHOLDER_RE = re.compile(r"(?:^|_)(?:j|joint)\d+$")


def _feature_name(name: str) -> str:
    return name if name.endswith(".pos") else f"{name}.pos"


def _make_joints(prefix: str, n: int) -> list[str]:
    """Generate legacy feature names: prefix_j1.pos, prefix_j2.pos, ..."""
    return [f"{prefix}_j{i}.pos" for i in range(1, n + 1)]


def _has_placeholder_name(name: str) -> bool:
    return bool(_PLACEHOLDER_RE.search(name.removesuffix(".pos")))


def _as_float_tuple(value: Any, *, name: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{name} must be a two-item list/tuple")
    return (float(value[0]), float(value[1]))


def _validate_unique(names: list[str], *, label: str) -> None:
    if len(names) != len(set(names)):
        duplicates = sorted({name for name in names if names.count(name) > 1})
        raise ValueError(f"{label} has duplicate entries: {duplicates}")


@RobotConfig.register_subclass("walker")
@dataclass
class WalkerRobotConfig(RobotConfig):
    # Optional high-level robot model config. When provided, it is normalized into
    # the legacy fields below so LeRobot still sees ordinary .pos action keys.
    robot_config_path: str | None = None
    robot_model: str = "walker_s2_v4_hand_31d"
    description: str = ""

    # DOF 配置名（取自 JOINT_INDEX_ENUMS）。当非空且 robot_config_path 为 None 时，
    # __post_init__ 会从对应 DOF 枚举派生 all_joints + inactive_fill。
    # 默认 "walker_s2_31d"（全 31，物理序）。
    # 部署子集策略时设为 "walker_s2_10d" 等；新增 DOF 请在 constants.py 注册。
    joint_config: str = "walker_s2_31d"

    # ZMQ configuration (LeRobot ↔ Bridge2 internal communication)
    zmq_host: str = "127.0.0.1"
    zmq_cmd_port: int = 5561       # LeRobot PUB → Bridge2 SUB (actions)
    zmq_status_port: int = 5562    # Bridge2 PUB → LeRobot SUB (status)
    zmq_image_port: int = 5563     # Bridge2 PUB → LeRobot SUB (camera images)
    bridge_enabled: bool = True     # Auto-start Bridge2 subprocess
    bridge_script: str = "/ubt_IL/walker/ros2_walker_bridge.py"

    # ROS2 topics (real robot defaults)
    ros_namespace: str = ""
    cmd_namespace: str = ""

    # End-effector type. Legacy configs use hand_type="v4".
    hand_type: str = "v4"
    end_effector_type: str = "v4_hand_7dof"

    # Joint group definitions (used for ZMQ message grouping)
    left_arm_joints: list[str] = field(default_factory=lambda: _make_joints("left_arm", 7))
    right_arm_joints: list[str] = field(default_factory=lambda: _make_joints("right_arm", 7))
    head_joints: list[str] = field(default_factory=lambda: _make_joints("head", 2))
    waist_joints: list[str] = field(default_factory=lambda: _make_joints("waist", 1))
    left_hand_joints: list[str] = field(default_factory=lambda: _make_joints("left_hand", 7))
    right_hand_joints: list[str] = field(default_factory=lambda: _make_joints("right_hand", 7))

    # Full joint ordering (determines model action/observation tensor dimension mapping).
    all_joints: list[str] = field(default_factory=lambda: (
        _make_joints("left_arm", 7) + _make_joints("right_arm", 7)
        + _make_joints("head", 2) + _make_joints("waist", 1)
        + _make_joints("left_hand", 7) + _make_joints("right_hand", 7)
    ))

    # Real-name groups used by bridge and runtime logs.
    body_groups: dict[str, list[str]] = field(default_factory=dict)
    end_effector_groups: dict[str, list[str]] = field(default_factory=dict)

    # Locked joints (not sent in RobotCommand, keep current position)
    lock_joints: list[str] = field(default_factory=lambda: list(DEFAULT_LOCK_JOINTS))

    # Body joint names (for mapping ZMQ groups to ROS2 topic joint names)
    body_joint_names: list[str] = field(default_factory=lambda: list(BODY_JOINT_NAMES))

    # Hand/gripper names (for mapping ZMQ groups to ROS2 topic/message fields)
    left_hand_joint_names: list[str] = field(default_factory=lambda: list(V4_HAND_LEFT_JOINTS))
    right_hand_joint_names: list[str] = field(default_factory=lambda: list(V4_HAND_RIGHT_JOINTS))

    # Joint limits (for safety clipping)
    body_joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(BODY_JOINT_LIMITS)
    )
    hand_joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(V4_HAND_JOINT_LIMITS)
    )
    gripper_position_limits: tuple[float, float] = (0.0, 0.05)
    gripper_force_limits: tuple[float, float] = (41.0, 100.0)
    gripper_velocity_limits: tuple[float, float] = (0.0, 0.01)
    gripper_acceleration_limits: tuple[float, float] = (0.0, 3.0)
    gripper_force: float = 41.0
    gripper_velocity: float = 0.005
    gripper_acceleration: float = 0.0
    gripper_mode: int = 0

    # Home position (body, by body_joint_names order)
    home_position: list[float] = field(default_factory=lambda: list(HOME_POSITION))

    # End-effector open positions. Legacy hand_open_position is retained for CLI/backward compatibility.
    hand_open_position: list[float] = field(default_factory=lambda: list(HAND_OPEN_POSITION))
    left_hand_open_position: list[float] | None = None
    right_hand_open_position: list[float] | None = None

    # ROS2 topic names (command side / publish)
    topic_body_cmd: str = TOPIC_BODY_CMD
    topic_left_hand_cmd: str = TOPIC_LEFT_HAND_CMD
    topic_right_hand_cmd: str = TOPIC_RIGHT_HAND_CMD

    # ROS2 topic names (status side / subscribe)
    topic_body_state: str = TOPIC_BODY_STATE
    topic_left_hand_state: str = TOPIC_LEFT_HAND_STATE
    topic_right_hand_state: str = TOPIC_RIGHT_HAND_STATE

    # Control
    control_fps: float = 15.0
    body_control_mode: str = "position"     # "velocity" | "pvt" | "position"
    body_velocity_timeout: float = 0.3       # 速度模式超时（秒）
    body_pvt_kp: float | dict | None = None  # PVT Kp 覆盖
    body_pvt_kd: float | dict | None = None  # PVT Kd 覆盖
    max_safe_velocity: float = 1.0           # 速度/PVT 最大关节速度 (rad/s)

    # Safety
    max_relative_target: float | None = None
    disable_torque_on_disconnect: bool = True

    # Cameras (keyed by name matching policy's expected image key)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    camera_topics: dict[str, dict[str, str]] = field(default_factory=dict)

    # Populated by __post_init__ when joint_config resolves to a valid DOF enum.
    # Keys already carry .pos suffix; values are floats.
    _inactive_fill: dict[str, float] = field(default_factory=dict, init=False, repr=False)

    # Populated by __post_init__：相机源 key → 模型 observation.images key 映射。
    # 默认恒等映射（{k: k}），仅当模型使用不同 key 名时才需要非恒等映射。
    _camera_to_image_key: dict[str, str] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self):
        if self.robot_config_path:
            self._load_robot_config(Path(self.robot_config_path))
            # 即使从 JSON 加载了完整硬件配置，仍允许 joint_config 覆盖
            # all_joints + 生成 inactive_fill，以支持子集策略部署。
            if self.joint_config in ROBOT_MODELS:
                joint_order = ROBOT_MODELS[self.joint_config]["joint_order"]
                self.all_joints = joint_names_with_pos(joint_order)
                self._inactive_fill = inactive_fill_for(self.joint_config, joint_order)
        else:
            self._normalize_legacy_groups()
            # 当无 robot_config_path 时，从 ROBOT_MODELS 注册表完整派生所有字段
            # （关节 DOF + 相机配置）。
            if self.joint_config in ROBOT_MODELS:
                spec = ROBOT_MODELS[self.joint_config]
                joint_order = spec["joint_order"]
                self.all_joints = joint_names_with_pos(joint_order)
                self._inactive_fill = inactive_fill_for(self.joint_config, joint_order)
                self._rebuild_groups_from_joint_order(joint_order)
                # 从 spec 构建相机配置
                self._build_cameras_from_spec(spec)
            else:
                raise ValueError(
                    f"joint_config {self.joint_config!r} not registered in ROBOT_MODELS. "
                    f"Available: {list(ROBOT_MODELS)}. "
                    f"新增型号请在 walker constants.py 的 ROBOT_MODELS 注册表添加。"
                )
        self._validate()

    def _rebuild_groups_from_joint_order(self, joint_order: list[str]) -> None:
        """从 joint_order 列表重建 6 组关节 feature 列表（真实关节名 + .pos 后缀）。

        同时推导 end_effector_type：若包含 V4 手关节，则为 v4_hand_7dof；
        若包含 PGC 夹爪执行器名（left_grip/right_grip），则为 pgc_gripper_1dof。
        """
        member_names = set(joint_order)

        # 车身分组（按固定硬件组归类）
        self.left_arm_joints = [_feature_name(n)
                                for n in LEFT_ARM_JOINTS if n in member_names]
        self.right_arm_joints = [_feature_name(n)
                                 for n in RIGHT_ARM_JOINTS if n in member_names]
        self.head_joints = [_feature_name(n)
                            for n in HEAD_JOINTS if n in member_names]
        self.waist_joints = [_feature_name(n)
                             for n in WAIST_JOINTS if n in member_names]
        self.body_joint_names = [n for n in BODY_JOINT_NAMES if n in member_names]
        self.body_groups = {
            "left_arm": [n for n in LEFT_ARM_JOINTS if n in member_names],
            "right_arm": [n for n in RIGHT_ARM_JOINTS if n in member_names],
            "head": [n for n in HEAD_JOINTS if n in member_names],
            "waist": [n for n in WAIST_JOINTS if n in member_names],
        }

        # 末端执行器分组
        has_v4 = bool(member_names & {*V4_HAND_LEFT_JOINTS, *V4_HAND_RIGHT_JOINTS})
        has_grip = "left_grip" in member_names or "right_grip" in member_names
        if has_v4:
            self.end_effector_type = "v4_hand_7dof"
            self.hand_type = "v4"
            self.left_hand_joint_names = [n for n in V4_HAND_LEFT_JOINTS if n in member_names]
            self.right_hand_joint_names = [n for n in V4_HAND_RIGHT_JOINTS if n in member_names]
        elif has_grip:
            self.end_effector_type = "pgc_gripper_1dof"
            self.hand_type = "pgc_gripper_1dof"
            self.left_hand_joint_names = ["left_grip"] if "left_grip" in member_names else []
            self.right_hand_joint_names = ["right_grip"] if "right_grip" in member_names else []
            # PGC 夹爪使用独立 topic，与 V4 手不同
            self.topic_left_hand_cmd = TOPIC_LEFT_GRIP_CMD
            self.topic_right_hand_cmd = TOPIC_RIGHT_GRIP_CMD
            self.topic_left_hand_state = TOPIC_LEFT_GRIP_STATE
            self.topic_right_hand_state = TOPIC_RIGHT_GRIP_STATE
        else:
            # 仅车身关节，无末端执行器
            self.end_effector_type = "v4_hand_7dof"  # 占位，避免 _validate 报错
            self.hand_type = "v4"
            self.left_hand_joint_names = []
            self.right_hand_joint_names = []
        self.left_hand_joints = [_feature_name(n) for n in self.left_hand_joint_names]
        self.right_hand_joints = [_feature_name(n) for n in self.right_hand_joint_names]
        self.end_effector_groups = {
            "left_hand": list(self.left_hand_joint_names),
            "right_hand": list(self.right_hand_joint_names),
        }

        # 解锁 active joint_order 中的关节，仅锁住非活跃关节。
        # lock_joints 中不在 member_names（即不在策略输出中的）的条目保留，
        # 用于禁止 bridge 对非策略控制关节发送命令。
        self.lock_joints = [j for j in self.lock_joints if j not in member_names]

        # 从 READY_POSE 重建 home_position（仅包含 body_joint_names 中的关节）
        self.home_position = [READY_POSE[n] for n in self.body_joint_names]

        # 末端执行器 open position
        if has_v4:
            self.left_hand_open_position = [0.0] * len(self.left_hand_joint_names)
            self.right_hand_open_position = [0.0] * len(self.right_hand_joint_names)
        elif has_grip:
            self.left_hand_open_position = [0.0] * len(self.left_hand_joint_names)
            self.right_hand_open_position = [0.0] * len(self.right_hand_joint_names)
        self.hand_open_position = list(self.left_hand_open_position or [])

        # 环境变量覆盖控制模式（部署时无需完整 ROBOT_CONFIG JSON）
        _env_mode = os.environ.get("BODY_CONTROL_MODE", "")
        if _env_mode:
            self.body_control_mode = _env_mode
            import logging
            _log = logging.getLogger(__name__)
            _log.info("BODY_CONTROL_MODE=%s from environment", _env_mode)
        _env_mvs = os.environ.get("BODY_MAX_SPEED", "")
        if _env_mvs:
            self.max_safe_velocity = float(_env_mvs)
            import logging
            _log = logging.getLogger(__name__)
            _log.info("BODY_MAX_SPEED=%s from environment", _env_mvs)

    def _load_robot_config(self, path: Path) -> None:
        with path.open("r", encoding="utf-8") as f:
            cfg = json.load(f)

        schema_version = cfg.get("schema_version")
        if schema_version != 1:
            raise ValueError(f"Unsupported Walker robot config schema_version: {schema_version!r}")

        self.robot_model = cfg.get("robot_model", self.robot_model)
        self.description = cfg.get("description", "")

        action_order = cfg.get("action_order")
        if not isinstance(action_order, list) or not action_order:
            raise ValueError("robot config requires a non-empty action_order list")
        if any(not isinstance(name, str) or not name for name in action_order):
            raise ValueError("action_order entries must be non-empty strings")
        if any(name.endswith(".pos") for name in action_order):
            raise ValueError("action_order should use real names without '.pos'; the loader adds '.pos'")
        if any(_has_placeholder_name(name) for name in action_order):
            raise ValueError("action_order must use real joint/actuator names, not j1/j2 placeholders")
        _validate_unique(action_order, label="action_order")
        self.all_joints = [_feature_name(name) for name in action_order]

        body_cfg = cfg.get("body", {})
        body_groups = body_cfg.get("groups", {})
        if not isinstance(body_groups, dict):
            raise ValueError("body.groups must be an object")
        self.body_groups = {
            group: self._require_name_list(body_groups, group, section="body.groups")
            for group in _BODY_GROUPS
        }
        for group_names in self.body_groups.values():
            if any(_has_placeholder_name(name) for name in group_names):
                raise ValueError("body.groups must use real ROS joint names, not j1/j2 placeholders")

        self.left_arm_joints = [_feature_name(name) for name in self.body_groups["left_arm"]]
        self.right_arm_joints = [_feature_name(name) for name in self.body_groups["right_arm"]]
        self.head_joints = [_feature_name(name) for name in self.body_groups["head"]]
        self.waist_joints = [_feature_name(name) for name in self.body_groups["waist"]]
        self.body_joint_names = sum((self.body_groups[group] for group in _BODY_GROUPS), [])

        self.lock_joints = list(body_cfg.get("lock_joints", self.lock_joints))
        self.body_control_mode = body_cfg.get("control_mode", self.body_control_mode)
        self.body_velocity_timeout = float(body_cfg.get("velocity_timeout", self.body_velocity_timeout))
        self.body_pvt_kp = body_cfg.get("pvt_kp", self.body_pvt_kp)
        self.body_pvt_kd = body_cfg.get("pvt_kd", self.body_pvt_kd)
        body_home = body_cfg.get("home", {})
        if not isinstance(body_home, dict):
            raise ValueError("body.home must be an object keyed by real joint name")
        missing_home = [name for name in self.body_joint_names if name not in body_home]
        if missing_home:
            raise ValueError(f"body.home missing joints: {missing_home}")
        self.home_position = [float(body_home[name]) for name in self.body_joint_names]
        self.body_joint_limits = {
            name: _as_float_tuple(limits, name=f"body.limits.{name}")
            for name, limits in body_cfg.get("limits", {}).items()
        } or dict(BODY_JOINT_LIMITS)

        ee_cfg = cfg.get("end_effectors", {})
        self.end_effector_type = ee_cfg.get("type", self.end_effector_type)
        self.hand_type = "v4" if self.end_effector_type == "v4_hand_7dof" else self.end_effector_type
        ee_groups = ee_cfg.get("groups", {})
        if not isinstance(ee_groups, dict):
            raise ValueError("end_effectors.groups must be an object")
        self.end_effector_groups = {
            group: self._require_name_list(ee_groups, group, section="end_effectors.groups")
            for group in _END_EFFECTOR_GROUPS
        }
        for group_names in self.end_effector_groups.values():
            if any(_has_placeholder_name(name) for name in group_names):
                raise ValueError("end_effectors.groups must use real actuator/joint names, not j1/j2 placeholders")

        self.left_hand_joints = [_feature_name(name) for name in self.end_effector_groups["left_hand"]]
        self.right_hand_joints = [_feature_name(name) for name in self.end_effector_groups["right_hand"]]
        self.left_hand_joint_names = list(self.end_effector_groups["left_hand"])
        self.right_hand_joint_names = list(self.end_effector_groups["right_hand"])

        ee_open = ee_cfg.get("open", {})
        if not isinstance(ee_open, dict):
            raise ValueError("end_effectors.open must be an object keyed by real joint/actuator name")
        self.left_hand_open_position = [float(ee_open[name]) for name in self.left_hand_joint_names]
        self.right_hand_open_position = [float(ee_open[name]) for name in self.right_hand_joint_names]
        self.hand_open_position = list(self.left_hand_open_position)

        ee_limits = ee_cfg.get("limits", {})
        if self.end_effector_type == "v4_hand_7dof":
            self.hand_joint_limits = dict(V4_HAND_JOINT_LIMITS)
        elif self.end_effector_type == "pgc_gripper_1dof":
            self.gripper_position_limits = _as_float_tuple(
                ee_limits.get("position", self.gripper_position_limits), name="end_effectors.limits.position"
            )
            self.gripper_force_limits = _as_float_tuple(
                ee_limits.get("force", self.gripper_force_limits), name="end_effectors.limits.force"
            )
            self.gripper_velocity_limits = _as_float_tuple(
                ee_limits.get("velocity", self.gripper_velocity_limits), name="end_effectors.limits.velocity"
            )
            self.gripper_acceleration_limits = _as_float_tuple(
                ee_limits.get("acceleration", self.gripper_acceleration_limits),
                name="end_effectors.limits.acceleration",
            )
            defaults = ee_cfg.get("command_defaults", {})
            self.gripper_force = float(defaults.get("force", self.gripper_force))
            self.gripper_velocity = float(defaults.get("velocity", self.gripper_velocity))
            self.gripper_acceleration = float(defaults.get("acceleration", self.gripper_acceleration))
            self.gripper_mode = int(defaults.get("mode", self.gripper_mode))
        else:
            raise ValueError(f"Unsupported end_effector type: {self.end_effector_type!r}")

        ros_cfg = cfg.get("ros", {})
        self.ros_namespace = ros_cfg.get("namespace", self.ros_namespace)
        self.cmd_namespace = ros_cfg.get("cmd_namespace", self.cmd_namespace)
        self.topic_body_cmd = ros_cfg.get("body_cmd_topic", self.topic_body_cmd)
        self.topic_body_state = ros_cfg.get("body_state_topic", self.topic_body_state)
        ee_topics = ee_cfg.get("topics", {})
        self.topic_left_hand_cmd = ee_topics.get("left_cmd", self.topic_left_hand_cmd)
        self.topic_right_hand_cmd = ee_topics.get("right_cmd", self.topic_right_hand_cmd)
        self.topic_left_hand_state = ee_topics.get("left_state", self.topic_left_hand_state)
        self.topic_right_hand_state = ee_topics.get("right_state", self.topic_right_hand_state)

        zmq_cfg = cfg.get("zmq", {})
        self.zmq_host = zmq_cfg.get("host", self.zmq_host)
        self.zmq_cmd_port = int(zmq_cfg.get("cmd_port", self.zmq_cmd_port))
        self.zmq_status_port = int(zmq_cfg.get("status_port", self.zmq_status_port))
        self.zmq_image_port = int(zmq_cfg.get("image_port", self.zmq_image_port))

        bridge_cfg = cfg.get("bridge", {})
        self.bridge_enabled = bool(bridge_cfg.get("enabled", self.bridge_enabled))
        self.bridge_script = bridge_cfg.get("script", self.bridge_script)

        safety_cfg = cfg.get("safety", {})
        self.max_relative_target = safety_cfg.get("max_relative_target", self.max_relative_target)
        if self.max_relative_target is not None:
            self.max_relative_target = float(self.max_relative_target)
        self.disable_torque_on_disconnect = bool(
            safety_cfg.get("disable_torque_on_disconnect", self.disable_torque_on_disconnect)
        )

        self.cameras = self._load_cameras(cfg.get("cameras", {}))
        # JSON 配置不含 camera_to_image_key，默认恒等映射
        self._camera_to_image_key = {k: k for k in self.cameras}

    @staticmethod
    def _require_name_list(data: dict, key: str, *, section: str) -> list[str]:
        value = data.get(key)
        if not isinstance(value, list):
            raise ValueError(f"{section}.{key} must be a list")
        if any(not isinstance(name, str) or not name for name in value):
            raise ValueError(f"{section}.{key} entries must be non-empty strings")
        _validate_unique(value, label=f"{section}.{key}")
        return list(value)

    def _build_cameras_from_spec(self, spec: dict) -> None:
        """从 ROBOT_MODELS spec 构建 self.cameras 和 self.camera_topics。

        在 __post_init__ Path B（无 robot_config_path）中调用，
        使 joint_config 即可独立驱动完整机器人配置。

        spec["camera_topics"] 是 {源 key: ROS2 topic}。
        spec["camera_to_image_key"] 是 {源 key: 模型 obs key}。
        """
        warmup = spec.get("camera_warmup_s", DEFAULT_CAMERA_WARMUP_S)
        topics: dict[str, str] = spec["camera_topics"]
        per_camera_types: dict[str, str] = spec.get("camera_msg_types", {})
        self.camera_topics = {
            k: {
                "topic": v,
                "msg_type": per_camera_types.get(k, DEFAULT_CAMERA_MSG_TYPE),
                "width": 640,
                "height": 360,
            }
            for k, v in topics.items()
        }
        self.cameras = {
            k: WalkerCameraConfig(
                width=DEFAULT_CAMERA_WIDTH,
                height=DEFAULT_CAMERA_HEIGHT,
                fps=DEFAULT_CAMERA_FPS,
                warmup_s=warmup,
                timeout_ms=DEFAULT_CAMERA_TIMEOUT_MS,
                camera_name=k,
                server_address=self.zmq_host,
                port=self.zmq_image_port,
            )
            for k in topics
        }
        self._camera_to_image_key = spec["camera_to_image_key"]

    def _load_cameras(self, cameras_cfg: dict[str, Any]) -> dict[str, CameraConfig]:
        cameras: dict[str, CameraConfig] = {}
        self.camera_topics = {}
        for name, cam_cfg in cameras_cfg.items():
            if not isinstance(cam_cfg, dict):
                raise ValueError(f"cameras.{name} must be an object")
            cam_type = cam_cfg.get("type", "walker_camera")
            if cam_type != "walker_camera":
                raise ValueError(f"Unsupported Walker camera type for {name}: {cam_type!r}")
            ros_topic = cam_cfg.get("ros_topic", TOPIC_CAMERA_STEREO)
            camera_topic_cfg = {
                "topic": ros_topic,
                "msg_type": cam_cfg.get("msg_type", "shm_msgs/Image2m"),
                "width": int(cam_cfg.get("width", 640)),
                "height": int(cam_cfg.get("height", 360)),
            }
            self.camera_topics[name] = camera_topic_cfg
            cameras[name] = WalkerCameraConfig(
                fps=int(cam_cfg["fps"]) if cam_cfg.get("fps") is not None else None,
                width=int(cam_cfg["width"]) if cam_cfg.get("width") is not None else None,
                height=int(cam_cfg["height"]) if cam_cfg.get("height") is not None else None,
                server_address=cam_cfg.get("server_address", self.zmq_host),
                port=int(cam_cfg.get("port", self.zmq_image_port)),
                camera_name=cam_cfg.get("camera_name", name),
                timeout_ms=int(cam_cfg.get("timeout_ms", 5000)),
                warmup_s=int(cam_cfg.get("warmup_s", 3)),
            )
        return cameras

    def _normalize_legacy_groups(self) -> None:
        if not self.body_groups:
            left_n = len(self.left_arm_joints)
            right_n = len(self.right_arm_joints)
            head_n = len(self.head_joints)
            self.body_groups = {
                "left_arm": list(self.body_joint_names[:left_n]),
                "right_arm": list(self.body_joint_names[left_n:left_n + right_n]),
                "head": list(self.body_joint_names[left_n + right_n:left_n + right_n + head_n]),
                "waist": list(self.body_joint_names[left_n + right_n + head_n:]),
            }
        if not self.end_effector_groups:
            self.end_effector_groups = {
                "left_hand": list(self.left_hand_joint_names),
                "right_hand": list(self.right_hand_joint_names),
            }
        if self.left_hand_open_position is None:
            self.left_hand_open_position = list(self.hand_open_position)
        if self.right_hand_open_position is None:
            self.right_hand_open_position = list(self.hand_open_position)

    def _validate(self) -> None:
        group_joints = (
            self.left_arm_joints + self.right_arm_joints
            + self.head_joints + self.waist_joints
            + self.left_hand_joints + self.right_hand_joints
        )
        group_set = set(group_joints)
        all_set = set(self.all_joints)
        # 子集模式：all_joints 只需是 6 组并集的子集（允许用 inactive_fill 补全硬件关节）
        if not all_set.issubset(group_set):
            raise ValueError(
                f"all_joints contains joints not in the 6 hardware groups. "
                f"Unknown: {all_set - group_set}"
            )
        _validate_unique(self.all_joints, label="all_joints")
        for name in self.all_joints:
            if not name.endswith(".pos"):
                raise ValueError(f"Walker action feature must end with .pos: {name}")

        if self.end_effector_type not in ("v4_hand_7dof", "pgc_gripper_1dof"):
            raise ValueError(f"Unsupported end_effector_type: {self.end_effector_type!r}")
        if self.hand_type not in ("v4", "pgc_gripper_1dof"):
            raise ValueError(f"Unsupported hand_type: {self.hand_type!r}")

        expected_home_dim = len(self.body_joint_names)
        if len(self.home_position) != expected_home_dim:
            raise ValueError(
                f"home_position count ({len(self.home_position)}) must match "
                f"body_joint_names count ({expected_home_dim})"
            )

        if len(self.left_hand_open_position or []) != len(self.left_hand_joints):
            raise ValueError("left hand/gripper open position count must match left hand group")
        if len(self.right_hand_open_position or []) != len(self.right_hand_joints):
            raise ValueError("right hand/gripper open position count must match right hand group")

        # 过滤掉不在活跃 body 集合中的 lock_joints（非活跃关节无需 lock，bridge 本来就不发命令）。
        body_set = set(self.body_joint_names)
        stale = [j for j in self.lock_joints if j not in body_set]
        if stale:
            self.lock_joints = [j for j in self.lock_joints if j in body_set]
            import logging
            _log = logging.getLogger(__name__)
            _log.info("Dropped lock_joints not in active body set: %s", stale)

        if self.end_effector_type == "v4_hand_7dof":
            # 允许 0 关节（仅 body 的 DOF 枚举），否则每侧必须恰好 7 关节
            if len(self.left_hand_joints) not in (0, 7) or len(self.right_hand_joints) not in (0, 7):
                raise ValueError("v4_hand_7dof requires 0 or 7 joints per hand")
        if self.end_effector_type == "pgc_gripper_1dof":
            if len(self.left_hand_joints) > 1 or len(self.right_hand_joints) > 1:
                raise ValueError("pgc_gripper_1dof requires at most 1 actuator per side")
            if len(self.left_hand_joints) == 0 and len(self.right_hand_joints) == 0:
                raise ValueError("pgc_gripper_1dof requires at least one actuator")

    def to_bridge_config(self) -> dict:
        """Serialize config fields needed by ros2_walker_bridge.py (system Python 3.10).

        The bridge runs outside the LeRobot venv and cannot import this class.
        This method produces a JSON-serializable dict passed via --config CLI arg.
        """
        return {
            "robot_model": self.robot_model,
            "description": self.description,
            "zmq_cmd_port": self.zmq_cmd_port,
            "zmq_status_port": self.zmq_status_port,
            "ros_namespace": self.ros_namespace,
            "cmd_namespace": self.cmd_namespace,
            "body_groups": self.body_groups,
            "body_joint_names": self.body_joint_names,
            "left_hand_joint_names": self.left_hand_joint_names,
            "right_hand_joint_names": self.right_hand_joint_names,
            "body_joint_limits": self.body_joint_limits,
            "hand_joint_limits": self.hand_joint_limits,
            "hand_type": self.hand_type,
            "end_effector_type": self.end_effector_type,
            "hand_open_position": self.hand_open_position,
            "left_hand_open_position": self.left_hand_open_position,
            "right_hand_open_position": self.right_hand_open_position,
            "gripper_position_limits": self.gripper_position_limits,
            "gripper_force_limits": self.gripper_force_limits,
            "gripper_velocity_limits": self.gripper_velocity_limits,
            "gripper_acceleration_limits": self.gripper_acceleration_limits,
            "gripper_force": self.gripper_force,
            "gripper_velocity": self.gripper_velocity,
            "gripper_acceleration": self.gripper_acceleration,
            "gripper_mode": self.gripper_mode,
            "lock_joints": self.lock_joints,
            "home_position": self.home_position,
            "topic_body_cmd": self.topic_body_cmd,
            "topic_left_hand_cmd": self.topic_left_hand_cmd,
            "topic_right_hand_cmd": self.topic_right_hand_cmd,
            "topic_body_state": self.topic_body_state,
            "topic_left_hand_state": self.topic_left_hand_state,
            "topic_right_hand_state": self.topic_right_hand_state,
        }
