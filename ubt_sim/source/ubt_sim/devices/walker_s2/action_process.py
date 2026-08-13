from __future__ import annotations

import os
import math
from typing import Any

import torch

from ubt_sim.devices.walker_s2.config import (
    WALKER_S2_GRIPPER_HOME_OPENING_M,
    WALKER_S2_GRIPPER_JOINT_CLOSING_M,
    WALKER_S2_GRIPPER_JOINT_SIGNS,
    WALKER_S2_GRIPPER_OPENING_MAX_M,
    WALKER_S2_GRIPPER_OPENING_MIN_M,
    WALKER_S2_HOME_POSE,
    WALKER_S2_LEFT_HAND_JOINTS,
    WALKER_S2_RIGHT_HAND_JOINTS,
)

WALKER_S2_ARM_JOINT_ORDER = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]

WALKER_S2_SDK_BODY_JOINT_ORDER = [
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
]

_ALIAS_TO_SIM_JOINT = {
    name.removesuffix("_joint"): name for name in WALKER_S2_HOME_POSE
}
_ALIAS_TO_SIM_JOINT.update({name: name for name in WALKER_S2_HOME_POSE})

class HoldTargetManager:
    """Manages joint hold targets with proper session lifecycle.

    Encapsulates the previously-module-level state so that hold targets are
    re-initialised from the current simulation joint positions whenever
    ``reset()`` is called (e.g. on environment reset or when a new control
    session starts).  This prevents stale targets from a previous script
    invocation from leaking into a new session and fighting the fresh commands.
    """

    def __init__(self):
        self.action_joint_names: list[str] | None = None
        self.mapping_logged: bool = False
        self.gripper_mapping_logged: set[tuple] = set()
        self.joint_targets: dict[str, float] | None = None

    def reset(self) -> None:
        """Clear hold targets so the next update re-seeds from current sim state."""
        self.joint_targets = None

    def ensure_initialized(self, env) -> None:
        """Lazy-init: seed hold targets from the current simulation joint positions."""
        if self.joint_targets is not None:
            return
        current_pos = _current_joint_map(env)
        if self.action_joint_names is None:
            self.action_joint_names = get_action_joint_names(env)
        self.joint_targets = {
            name: float(current_pos.get(name, WALKER_S2_HOME_POSE.get(name, 0.0)))
            for name in self.action_joint_names
        }
        print("[INFO] Walker S2 hold targets captured from current joint positions.")

    def update(self, body_cmd: dict[str, float], left_grip: dict | None,
               right_grip: dict | None, env) -> None:
        """Apply an incremental command update, initialising on first call."""
        self.ensure_initialized(env)
        self.joint_targets.update(body_cmd)
        self.joint_targets.update(
            _grip_cmd_to_joint_targets("left", left_grip, env))
        self.joint_targets.update(
            _grip_cmd_to_joint_targets("right", right_grip, env))


_hold_manager = HoldTargetManager()

# 重力补偿开关：可通过环境变量 WALKER_S2_GRAVITY_COMP=0 禁用，方便 A/B 对比测试
_enable_gravity_comp = os.environ.get("WALKER_S2_GRAVITY_COMP", "1") == "1"

if _enable_gravity_comp:
    from ubt_sim.devices.walker_s2.gravity_compensation import compute_gravity_offsets
else:
    compute_gravity_offsets = None  # type: ignore[assignment]

# 仿真 PGC 手指关节的 0 位在中间附近；用 [-closing, +closing] 覆盖完整张合行程。
# 外部 GripCmd 开口仍保持 [0, 0.05] m，只在这里改变开口到仿真关节的映射。
WALKER_S2_GRIPPER_SIM_OPEN_JOINT_M = -WALKER_S2_GRIPPER_JOINT_CLOSING_M
WALKER_S2_GRIPPER_SIM_CLOSE_JOINT_M = WALKER_S2_GRIPPER_JOINT_CLOSING_M


def reset_hold_targets() -> None:
    _hold_manager.reset()


def normalize_joint_name(name: str) -> str | None:
    return _ALIAS_TO_SIM_JOINT.get(name)


def get_action_joint_names(env) -> list[str]:
    robot = env.unwrapped.scene["robot"]
    action_manager = env.unwrapped.action_manager
    indices = []
    terms_dict = {}

    for attr_name in ["_terms", "_action_terms", "terms"]:
        if hasattr(action_manager, attr_name):
            val = getattr(action_manager, attr_name)
            if isinstance(val, dict):
                terms_dict = val
                break
            if isinstance(val, list):
                terms_dict = {f"term_{i}": term for i, term in enumerate(val)}
                break

    for _, term in terms_dict.items():
        for attr in ["joint_indices", "_joint_indices", "joint_ids", "_joint_ids"]:
            if hasattr(term, attr):
                indices += list(getattr(term, attr))
                break

    return [robot.joint_names[idx] for idx in indices]


def _current_joint_map(env) -> dict[str, float]:
    robot = env.scene["robot"]
    joint_names = robot.data.joint_names
    joint_pos = robot.data.joint_pos[0].detach().cpu().tolist()
    return dict(zip(joint_names, joint_pos))


def _extract_body_command(command: dict[str, Any]) -> dict[str, float]:
    body = command.get("body", command)
    if body is None:
        return {}

    mapped = {}
    if isinstance(body, dict):
        for name, value in body.items():
            joint_name = normalize_joint_name(str(name))
            if joint_name is not None:
                mapped[joint_name] = float(value)
        return mapped

    if isinstance(body, (list, tuple)):
        for name, value in zip(WALKER_S2_SDK_BODY_JOINT_ORDER, body):
            mapped[name] = float(value)
        return mapped

    return mapped


def _clamp_grip_opening(value: Any) -> float:
    opening = float(value)
    return max(WALKER_S2_GRIPPER_OPENING_MIN_M, min(WALKER_S2_GRIPPER_OPENING_MAX_M, opening))


def _configured_gripper_joints(side: str, joint_names: list[str]) -> list[str]:
    configured = WALKER_S2_LEFT_HAND_JOINTS if side == "left" else WALKER_S2_RIGHT_HAND_JOINTS
    return [name for name in configured if name in joint_names]


def _discover_gripper_joints(env, side: str) -> list[str]:
    robot_joint_names = list(env.scene["robot"].data.joint_names)
    joints = _configured_gripper_joints(side, robot_joint_names)
    if not joints:
        side_tokens = ("l_", "left") if side == "left" else ("r_", "right")
        grip_tokens = ("finger", "grip", "pgc")
        joints = sorted(
            name for name in robot_joint_names
            if any(token in name.lower() for token in side_tokens)
            and any(token in name.lower() for token in grip_tokens)
        )

    log_key = (side, tuple(joints))
    if log_key not in _hold_manager.gripper_mapping_logged:
        if joints:
            missing = [name for name in joints
                       if _hold_manager.action_joint_names is not None
                       and name not in _hold_manager.action_joint_names]
            print(f"[INFO] Walker S2 {side} gripper joints: {joints}")
            if missing:
                print(
                    f"[WARN] Walker S2 {side} gripper joints not in action order: {missing}. "
                    "Add them to the task ActionsCfg to actuate them."
                )
        else:
            candidates = [name for name in robot_joint_names if any(token in name.lower() for token in ("finger", "grip", "pgc"))]
            print(f"[WARN] Walker S2 {side} gripper joints not found. Gripper candidates: {candidates}")
        _hold_manager.gripper_mapping_logged.add(log_key)

    return joints


def _grip_cmd_to_joint_targets(side: str, grip_cmd: Any, env) -> dict[str, float]:
    if not isinstance(grip_cmd, dict):
        return {}
    if int(grip_cmd.get("stop", 0)) != 0:
        return {}

    target_opening = WALKER_S2_GRIPPER_HOME_OPENING_M
    if int(grip_cmd.get("reset", 0)) == 0 and int(grip_cmd.get("homing", 0)) == 0:
        target_opening = grip_cmd.get("pos", target_opening)
    opening = _clamp_grip_opening(target_opening)

    joints = _discover_gripper_joints(env, side)
    active_joints = [name for name in joints
                     if _hold_manager.action_joint_names is None
                     or name in _hold_manager.action_joint_names]
    if not active_joints:
        return {}

    open_ratio = (opening - WALKER_S2_GRIPPER_OPENING_MIN_M) / (
        WALKER_S2_GRIPPER_OPENING_MAX_M - WALKER_S2_GRIPPER_OPENING_MIN_M
    )
    joint_target = (
        WALKER_S2_GRIPPER_SIM_CLOSE_JOINT_M
        + open_ratio * (WALKER_S2_GRIPPER_SIM_OPEN_JOINT_M - WALKER_S2_GRIPPER_SIM_CLOSE_JOINT_M)
    )
    return {
        name: float(WALKER_S2_GRIPPER_JOINT_SIGNS.get(name, 1.0)) * joint_target
        for name in active_joints
    }


def _grip_status_from_joints(side: str, pos_map: dict[str, float], cached_command: dict[str, Any] | None) -> dict[str, float] | None:
    joints = _configured_gripper_joints(side, list(pos_map.keys()))
    if not joints:
        return None

    joint_targets = [
        float(WALKER_S2_GRIPPER_JOINT_SIGNS.get(name, 1.0)) * float(pos_map[name])
        for name in joints
    ]
    joint_target = sum(joint_targets) / float(len(joint_targets))
    open_ratio = (joint_target - WALKER_S2_GRIPPER_SIM_CLOSE_JOINT_M) / (
        WALKER_S2_GRIPPER_SIM_OPEN_JOINT_M - WALKER_S2_GRIPPER_SIM_CLOSE_JOINT_M
    )
    opening = WALKER_S2_GRIPPER_OPENING_MIN_M + open_ratio * (
        WALKER_S2_GRIPPER_OPENING_MAX_M - WALKER_S2_GRIPPER_OPENING_MIN_M
    )
    opening = max(WALKER_S2_GRIPPER_OPENING_MIN_M, min(WALKER_S2_GRIPPER_OPENING_MAX_M, opening))
    grip_cmd = (cached_command or {}).get(f"{side}_grip", {})
    cur = float(grip_cmd.get("cur", 0.0)) if isinstance(grip_cmd, dict) else 0.0
    return {"pos": opening, "vel": 0.0, "cur": cur}


def _to_float_list(value: Any) -> list[float]:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().tolist()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return [float(v) for v in value]


def _get_finger_link_states(env) -> dict[str, Any]:
    robot = env.scene["robot"]
    body_names = getattr(robot, "body_names", None) or getattr(robot.data, "body_names", None) or []
    link_indices = [idx for idx, name in enumerate(body_names) if "finger_link" in str(name).lower()]
    if not link_indices:
        link_indices = [
            idx for idx, name in enumerate(body_names)
            if "finger" in str(name).lower() and "link" in str(name).lower()
        ]

    required_links = {"R_sixforce_link", "R_finger1_link"}
    seen = set(link_indices)
    for idx, name in enumerate(body_names):
        if str(name) in required_links and idx not in seen:
            link_indices.append(idx)
            seen.add(idx)

    links = {}
    for idx in link_indices:
        name = str(body_names[idx])
        links[name] = {
            "pos": _to_float_list(robot.data.body_pos_w[0, idx]),
            "rot": _to_float_list(robot.data.body_quat_w[0, idx]),
        }
    return {"frame": "world", "links": links}


def to_controller_data(command: dict[str, Any], env) -> torch.Tensor:
    if _hold_manager.action_joint_names is None:
        _hold_manager.action_joint_names = get_action_joint_names(env)

    if not _hold_manager.mapping_logged:
        print("[INFO] Walker S2 action joint order:")
        for idx, name in enumerate(_hold_manager.action_joint_names):
            print(f"  [{idx:02d}] {name}")
        _hold_manager.mapping_logged = True

    body_cmd = _extract_body_command(command)
    _hold_manager.update(
        body_cmd,
        command.get("left_grip"),
        command.get("right_grip"),
        env,
    )

    # 重力补偿：从当前关节位置计算重力矩，转换为位置偏移叠加到目标上。
    # 偏移不存储到 _hold_manager.joint_targets（否则逐帧累加），每帧从实际位姿重新计算。
    gravity_offsets: dict[str, float] = {}
    if _enable_gravity_comp and compute_gravity_offsets is not None:
        current_pos = _current_joint_map(env)
        gravity_offsets = compute_gravity_offsets(current_pos)

    values = [
        float(_hold_manager.joint_targets.get(name, WALKER_S2_HOME_POSE.get(name, 0.0)))
        + gravity_offsets.get(name, 0.0)
        for name in _hold_manager.action_joint_names
    ]

    # NaN/inf 守卫：ZMQ body 命令可能带 NaN（Python json.loads 接受 NaN，float(nan) 静默通过）。
    # 非有限值进 PhysX ImplicitActuator 会使整个 articulation 爆 NaN 并永久污染仿真。
    # 检测到则用当前仿真位姿兜底（hold current）并告警，绝不放行 NaN。
    if not all(math.isfinite(v) for v in values):
        cur_map = _current_joint_map(env)
        values = [
            float(cur_map.get(name, 0.0)) if not math.isfinite(values[i])
            else values[i]
            for i, name in enumerate(_hold_manager.action_joint_names)
        ]
        print("[WARN] Walker S2 action_process: non-finite target detected; "
              "holding current pose for affected joints")

    return torch.tensor(values, device=env.device, dtype=torch.float32).unsqueeze(0).repeat(env.num_envs, 1)


def to_ros_data(env, cached_command: dict[str, Any] | None = None) -> dict[str, Any]:
    robot = env.scene["robot"]
    joint_names = list(robot.data.joint_names)
    joint_pos = robot.data.joint_pos[0].detach().cpu().tolist()
    # Avoid reading robot.data.joint_vel here. On the Walker S2 task it can trigger
    # PhysX getVelocities device-mismatch spam when the sim is running on CUDA and
    # a CPU velocity tensor is requested. The bridge only needs position feedback
    # for the current control loop, so publish zero velocities until we add a safe
    # velocity read path.
    joint_vel = [0.0] * len(joint_names)
    pos_map = dict(zip(joint_names, joint_pos))

    sdk_body_pos = [float(pos_map.get(name, WALKER_S2_HOME_POSE.get(name, 0.0))) for name in WALKER_S2_SDK_BODY_JOINT_ORDER]
    sdk_body_vel = [0.0] * len(WALKER_S2_SDK_BODY_JOINT_ORDER)

    status = {
        "joint_names": joint_names,
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "sdk_body_joint_names": WALKER_S2_SDK_BODY_JOINT_ORDER,
        "sdk_body_pos": sdk_body_pos,
        "sdk_body_vel": sdk_body_vel,
    }
    get_part_states = getattr(env.cfg, "get_part_states", None)
    if get_part_states is not None:
        try:
            status["part_states"] = get_part_states(env)
        except Exception as exc:
            status["part_states_error"] = str(exc)
    try:
        status["finger_link_states"] = _get_finger_link_states(env)
    except Exception as exc:
        status["finger_link_states_error"] = str(exc)

    if cached_command:
        for key in ["left_hand", "right_hand"]:
            if key in cached_command:
                status[key] = cached_command[key]

    left_grip = _grip_status_from_joints("left", pos_map, cached_command)
    right_grip = _grip_status_from_joints("right", pos_map, cached_command)
    if left_grip is not None:
        status["left_grip"] = left_grip
    elif cached_command and "left_grip" in cached_command:
        status["left_grip"] = cached_command["left_grip"]
    if right_grip is not None:
        status["right_grip"] = right_grip
    elif cached_command and "right_grip" in cached_command:
        status["right_grip"] = cached_command["right_grip"]
    return status
