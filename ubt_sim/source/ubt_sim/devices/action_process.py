from dataclasses import MISSING, fields
from typing import Any

import isaaclab.envs.mdp as mdp
import torch
from ubt_sim.devices.tienkung_pro.config import (
    TIENKUNG_PRO_JOINT_LIMITS,
    TIENKUNG_PRO_RIGHT_HAND_JOINTS,
    TIENKUNG_PRO_LEFT_HAND_JOINTS,
    TIENKUNG_PRO_MIMIC_JOINTS
)


def init_action_cfg(action_cfg, device):
    """Tienkung Pro action configuration."""

    """Check if all the action configurations are set"""
    for field in fields(action_cfg):
        value = getattr(action_cfg, field.name, None)
        if value is None or value is MISSING:
            raise ValueError(f"Action configuration '{field.name}' for {device} is not set")

    return action_cfg


def preprocess_device_action(action: dict[str, Any], teleop_device) -> torch.Tensor:
    if action.get("tienkung_dex") is not None:
        processed_action = action["tienkung_dex"]
    elif action.get("walker_c1") is not None:
        processed_action = action["walker_c1"]
    elif action.get("tienkung_pro") is not None:
        # Expected input format from teleop se3 agent or other source
        processed_action = action["tienkung_pro"]
    else:
        device_type = getattr(teleop_device, "device_type", type(teleop_device).__name__)
        raise NotImplementedError(f"Not implemented for this device now: {device_type}")
    return processed_action
