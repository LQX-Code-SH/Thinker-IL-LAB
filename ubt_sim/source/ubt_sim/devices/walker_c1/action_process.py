"""Walker C1 指令 → 仿真 action / 状态回传。

外部指令按真机 SDK 语义组织：26 维 = 左臂 7 + 左手 SDK 6 + 右臂 7 + 右手 SDK 6。
手部 SDK 6 维压缩映射到仿真 11 个手部关节（拇指 swing→cmp，其余 *_mcp→(*_mpp,*_ip)），
状态回传按被映射的两个关节取均值反算，保证"下发—回读"闭环不漂移。
也接受命名 dict（``{joint: rad}`` / ``{left,right}_hand`` / 分组列表等），便于脚本逐步下发。
"""

from __future__ import annotations

from typing import Any

import torch

from .config import (
    WALKER_C1_HEAD_JOINTS,
    WALKER_C1_HOME_POSE,
    WALKER_C1_LEFT_ARM_JOINTS,
    WALKER_C1_LEFT_HAND_JOINTS,
    WALKER_C1_LEFT_LEG_JOINTS,
    WALKER_C1_RIGHT_ARM_JOINTS,
    WALKER_C1_RIGHT_HAND_JOINTS,
    WALKER_C1_RIGHT_LEG_JOINTS,
    WALKER_C1_WAIST_JOINTS,
)

WALKER_C1_LEFT_HAND_SDK_JOINTS = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

WALKER_C1_RIGHT_HAND_SDK_JOINTS = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

# 26 维外部指令顺序（与真机 SDK / 采集数据一致）
WALKER_C1_EXTERNAL_ACTION_ORDER = (
    WALKER_C1_LEFT_ARM_JOINTS
    + WALKER_C1_LEFT_HAND_SDK_JOINTS
    + WALKER_C1_RIGHT_ARM_JOINTS
    + WALKER_C1_RIGHT_HAND_SDK_JOINTS
)

_GROUP_JOINTS = {
    "left_arm": WALKER_C1_LEFT_ARM_JOINTS,
    "right_arm": WALKER_C1_RIGHT_ARM_JOINTS,
    "head": WALKER_C1_HEAD_JOINTS,
    "waist": WALKER_C1_WAIST_JOINTS,
    "left_leg": WALKER_C1_LEFT_LEG_JOINTS,
    "right_leg": WALKER_C1_RIGHT_LEG_JOINTS,
}

_ALL_SIM_JOINTS = set(WALKER_C1_HOME_POSE)
_ALIAS_TO_SIM_JOINT = {name: name for name in _ALL_SIM_JOINTS}
_ALIAS_TO_SIM_JOINT.update({name.removesuffix("_joint"): name for name in _ALL_SIM_JOINTS})

_LEFT_HAND_6_TO_11 = {
    "left_thumb_swing": ["L_thumb_cmp_joint"],
    "left_thumb_mcp": ["L_thumb_mpp_joint", "L_thumb_ip_joint"],
    "left_index_mcp": ["L_index_mpp_joint", "L_index_ip_joint"],
    "left_middle_mcp": ["L_middle_mpp_joint", "L_middle_ip_joint"],
    "left_ring_mcp": ["L_ring_mpp_joint", "L_ring_ip_joint"],
    "left_little_mcp": ["L_little_mpp_joint", "L_little_ip_joint"],
}

_RIGHT_HAND_6_TO_11 = {
    "right_thumb_swing": ["R_thumb_cmp_joint"],
    "right_thumb_mcp": ["R_thumb_mpp_joint", "R_thumb_ip_joint"],
    "right_index_mcp": ["R_index_mpp_joint", "R_index_ip_joint"],
    "right_middle_mcp": ["R_middle_mpp_joint", "R_middle_ip_joint"],
    "right_ring_mcp": ["R_ring_mpp_joint", "R_ring_ip_joint"],
    "right_little_mcp": ["R_little_mpp_joint", "R_little_ip_joint"],
}

action_joint_names: list[str] | None = None
_mapping_logged = False
_hold_joint_targets: dict[str, float] | None = None


def reset_hold_targets() -> None:
    global _hold_joint_targets
    _hold_joint_targets = None


def normalize_joint_name(name: str) -> str | None:
    return _ALIAS_TO_SIM_JOINT.get(name)


def get_action_joint_names(env) -> list[str]:
    robot = env.unwrapped.scene["robot"]
    action_manager = env.unwrapped.action_manager
    indices: list[int] = []
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


def _to_float_list(value: Any) -> list[float]:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().tolist()
    if hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, (list, tuple)):
        return []
    return [float(v) for v in value]


def _map_sequence(names: list[str], values: Any) -> dict[str, float]:
    return {name: value for name, value in zip(names, _to_float_list(values))}


def _extract_named_joints(value: Any) -> dict[str, float]:
    if not isinstance(value, dict):
        return {}

    mapped = {}
    for name, raw_value in value.items():
        joint_name = normalize_joint_name(str(name))
        if joint_name is not None:
            mapped[joint_name] = float(raw_value)
    return mapped


def _expand_hand_command(side: str, value: Any) -> dict[str, float]:
    if value is None:
        return {}

    sim_joints = WALKER_C1_LEFT_HAND_JOINTS if side == "left" else WALKER_C1_RIGHT_HAND_JOINTS
    sdk_joints = WALKER_C1_LEFT_HAND_SDK_JOINTS if side == "left" else WALKER_C1_RIGHT_HAND_SDK_JOINTS
    expand_map = _LEFT_HAND_6_TO_11 if side == "left" else _RIGHT_HAND_6_TO_11

    if isinstance(value, dict):
        mapped: dict[str, float] = {}
        for name, raw_value in value.items():
            name = str(name)
            if name in expand_map:
                for sim_name in expand_map[name]:
                    mapped[sim_name] = float(raw_value)
                continue

            joint_name = normalize_joint_name(name)
            if joint_name in sim_joints:
                mapped[joint_name] = float(raw_value)
        return mapped

    values = _to_float_list(value)
    if len(values) == len(sim_joints):
        return dict(zip(sim_joints, values))
    if len(values) == len(sdk_joints):
        mapped = {}
        for name, raw_value in zip(sdk_joints, values):
            for sim_name in expand_map[name]:
                mapped[sim_name] = raw_value
        return mapped

    return {}


def _extract_command_targets(command: dict[str, Any] | list[float] | tuple[float, ...]) -> dict[str, float]:
    if isinstance(command, (list, tuple, torch.Tensor)):
        values = _to_float_list(command)
        if len(values) != len(WALKER_C1_EXTERNAL_ACTION_ORDER):
            return {}
        left_arm_end = len(WALKER_C1_LEFT_ARM_JOINTS)
        left_hand_end = left_arm_end + len(WALKER_C1_LEFT_HAND_SDK_JOINTS)
        right_arm_end = left_hand_end + len(WALKER_C1_RIGHT_ARM_JOINTS)
        targets = _map_sequence(WALKER_C1_LEFT_ARM_JOINTS, values[:left_arm_end])
        targets.update(_expand_hand_command("left", values[left_arm_end:left_hand_end]))
        targets.update(_map_sequence(WALKER_C1_RIGHT_ARM_JOINTS, values[left_hand_end:right_arm_end]))
        targets.update(_expand_hand_command("right", values[right_arm_end:]))
        return targets

    if not isinstance(command, dict):
        return {}

    if "walker_c1" in command:
        return _extract_command_targets(command["walker_c1"])

    targets = _extract_named_joints(command.get("body", command))

    for group, names in _GROUP_JOINTS.items():
        if group in command:
            if isinstance(command[group], dict):
                targets.update(_extract_named_joints(command[group]))
            else:
                targets.update(_map_sequence(names, command[group]))

    targets.update(_expand_hand_command("left", command.get("left_hand")))
    targets.update(_expand_hand_command("right", command.get("right_hand")))
    return targets


def to_controller_data(command: dict[str, Any] | list[float] | tuple[float, ...], env) -> torch.Tensor:
    global action_joint_names, _mapping_logged, _hold_joint_targets

    if action_joint_names is None:
        action_joint_names = get_action_joint_names(env)

    if not _mapping_logged:
        print("[INFO] Walker C1 action joint order:")
        for idx, name in enumerate(action_joint_names):
            print(f"  [{idx:02d}] {name}")
        _mapping_logged = True

    if _hold_joint_targets is None:
        # 启动保持目标锚定为 HOME_POSE，而不是当前关节角：env.reset() 之后
        # 关节可能仍处于未收敛的混乱状态（例如 L_hip_roll ~2.9 rad），
        # 以其为保持目标会让增大的增益把腿驱动到垃圾角度。
        _hold_joint_targets = {
            name: float(WALKER_C1_HOME_POSE.get(name, 0.0))
            for name in action_joint_names
        }
        print("[INFO] Walker C1 startup hold targets set to HOME_POSE.")

    targets = {
        name: value
        for name, value in _extract_command_targets(command).items()
        if name in _hold_joint_targets
    }
    _hold_joint_targets.update(targets)

    values = [
        float(_hold_joint_targets.get(name, WALKER_C1_HOME_POSE.get(name, 0.0)))
        for name in action_joint_names
    ]
    return (
        torch.tensor(values, device=env.device, dtype=torch.float32)
        .unsqueeze(0)
        .repeat(env.num_envs, 1)
    )


def _sdk_hand_status(side: str, pos_map: dict[str, float]) -> dict[str, float]:
    sdk_joints = WALKER_C1_LEFT_HAND_SDK_JOINTS if side == "left" else WALKER_C1_RIGHT_HAND_SDK_JOINTS
    expand_map = _LEFT_HAND_6_TO_11 if side == "left" else _RIGHT_HAND_6_TO_11

    status = {}
    for name in sdk_joints:
        sim_names = expand_map[name]
        vals = [
            float(pos_map.get(sim_name, WALKER_C1_HOME_POSE.get(sim_name, 0.0)))
            for sim_name in sim_names
        ]
        status[name] = sum(vals) / float(len(vals))
    return status


def to_ros_data(env, cached_command: dict[str, Any] | None = None) -> dict[str, Any]:
    robot = env.scene["robot"]
    joint_names = list(robot.data.joint_names)
    joint_pos = robot.data.joint_pos[0].detach().cpu().tolist()
    # 不读 joint_vel：Isaac Sim 5.0 / Lab 2.2 在 CUDA 下可能把速度缓冲建在别的
    # device 上；状态回传只需要位置反馈。真实速度另由 joint_vel_probe 提供。
    joint_vel = [0.0] * len(joint_names)
    pos_map = dict(zip(joint_names, joint_pos))
    left_hand_sdk_status = _sdk_hand_status("left", pos_map)
    right_hand_sdk_status = _sdk_hand_status("right", pos_map)

    status = {
        "joint_names": joint_names,
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "external_action_order": WALKER_C1_EXTERNAL_ACTION_ORDER,
        "left_hand_sdk_joint_names": WALKER_C1_LEFT_HAND_SDK_JOINTS,
        "right_hand_sdk_joint_names": WALKER_C1_RIGHT_HAND_SDK_JOINTS,
        "left_hand_sdk_pos": [float(left_hand_sdk_status[name]) for name in WALKER_C1_LEFT_HAND_SDK_JOINTS],
        "right_hand_sdk_pos": [float(right_hand_sdk_status[name]) for name in WALKER_C1_RIGHT_HAND_SDK_JOINTS],
    }
    if _hold_joint_targets is not None:
        status["target_joint_names"] = list(_hold_joint_targets.keys())
        status["target_joint_pos"] = [float(_hold_joint_targets[name]) for name in _hold_joint_targets]
    if cached_command:
        status["cached_command"] = cached_command
    # 仅供 ROS 侧任务脚本使用的仿真真值（真机无法提供，脚本必须当作可选字段）
    try:
        if "object" in env.scene.keys():
            status["object_pos_w"] = env.scene["object"].data.root_pos_w[0].detach().cpu().tolist()
        status["robot_root_pose_w"] = robot.data.root_state_w[0, :7].detach().cpu().tolist()
        # 诊断探针（不用于控制）：真实关节速度，用于确认帧间残余动态状态。
        status["joint_vel_probe"] = robot.data.joint_vel[0].detach().cpu().tolist()
        body_names = list(robot.data.body_names)
        hand_links = {}
        for i, name in enumerate(body_names):
            if name == "R_palm_link" or (
                name.startswith("R_") and name.endswith(("_ip_link", "_mpp_link", "_cmp_link"))
            ):
                hand_links[name] = robot.data.body_pos_w[0, i].detach().cpu().tolist()
        status["right_hand_links_w"] = hand_links
    except Exception:
        pass
    return status
