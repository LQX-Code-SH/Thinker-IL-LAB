import math
import random
from typing import Any

import torch
from pxr import Gf, Usd, UsdGeom

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.assets import AssetBaseCfg, ArticulationCfg, RigidObjectCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_s2.config import (
    WALKER_S2_CFG,
    WALKER_S2_HOME_POSE,
    WALKER_S2_LEFT_ARM_JOINTS,
    WALKER_S2_RIGHT_ARM_JOINTS,
    WALKER_S2_LEFT_HAND_JOINTS,
    WALKER_S2_RIGHT_HAND_JOINTS,
    WALKER_S2_HEAD_JOINTS,
    WALKER_S2_WAIST_JOINTS,
    WALKER_S2_LEFT_LEG_JOINTS,
    WALKER_S2_RIGHT_LEG_JOINTS,
)
from ubt_sim.env.digital_twin_env_cfg import ManagerBasedRLDigitalTwinEnvCfg
from ubt_sim.utils.config_loader import load_config, resolve_asset_path

# Load scene config from YAML
_TASK_CFG = load_config("walker_s2/pick-part.yaml")
_SCENE_USD_PATH = resolve_asset_path(_TASK_CFG["scene"]["usd_path"])
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_OBJECTS_CFG = _TASK_CFG["objects"]
_CAMERAS_CFG = _TASK_CFG["cameras"]
_CAMERA_NAMES = list(_CAMERAS_CFG.keys())


def _make_tiled_camera(name: str) -> TiledCameraCfg:
    """Construct a TiledCameraCfg from the YAML cameras.<name> entry."""
    cfg = _CAMERAS_CFG[name]
    res = cfg.get("resolution", [640, 480])
    return TiledCameraCfg(
        prim_path=cfg["prim_path"],
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="ros",
        ),
        update_period=0.0333,
        height=int(res[1]),
        width=int(res[0]),
        data_types=cfg.get("data_types", ["rgb"]),
        spawn=None,
    )


def _usd_asset_cfg(asset_cfg: dict) -> AssetBaseCfg:
    return AssetBaseCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=resolve_asset_path(asset_cfg["usd_path"]),
            scale=tuple(asset_cfg.get("scale", (1.0, 1.0, 1.0))),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(asset_cfg.get("pos", (0.0, 0.0, 0.0))),
            rot=tuple(asset_cfg.get("rot", (1.0, 0.0, 0.0, 0.0))),
        ),
    )


def _part_rigid_object_cfg(asset_cfg: dict) -> RigidObjectCfg:
    """Create dynamic graspable part assets; keep table/box on _usd_asset_cfg."""
    return RigidObjectCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=resolve_asset_path(asset_cfg["usd_path"]),
            scale=tuple(asset_cfg.get("scale", (1.0, 1.0, 1.0))),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=bool(asset_cfg.get("disable_gravity", False)),
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=float(asset_cfg.get("mass", 0.05))),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=tuple(asset_cfg.get("pos", (0.0, 0.0, 0.0))),
            rot=tuple(asset_cfg.get("rot", (1.0, 0.0, 0.0, 0.0))),
        ),
    )


_PART_CFGS = {part_cfg["name"]: part_cfg for part_cfg in _OBJECTS_CFG["parts"]}
PICK_PART_KEYS = ("part_a_red",)
_PART_PRIM_NAMES = {
    "part_a_red": "PartA_Red",
}
_DEFAULT_PART_RANDOMIZATION_CFG = {
    "parts": list(PICK_PART_KEYS),
    "relative_to": "initial",
    "range": {
        "x": [-0.05, 0.05],
        "y": [-0.05, 0.05],
        "z": [0.0, 0.0],
        "roll": [0.0, 0.0],
        "pitch": [0.0, 0.0],
        "yaw": [-0.785398, 0.785398],
    },
}


def _to_float_list(value: Any) -> list[float]:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().tolist()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return [float(v) for v in value]


def _quat_normalize(rot: list[float]) -> list[float]:
    norm = math.sqrt(sum(float(v) * float(v) for v in rot))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [float(v) / norm for v in rot]


def _quat_multiply(lhs: list[float], rhs: list[float]) -> list[float]:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return _quat_normalize([
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ])


def _euler_xyz_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return _quat_normalize([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _quat_to_euler_xyz(rot: list[float]) -> tuple[float, float, float]:
    w, x, y, z = _quat_normalize(rot)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _range_pair(value: Any, axis: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"part randomization range.{axis} must be a two-number list")
    low, high = float(value[0]), float(value[1])
    if low > high:
        low, high = high, low
    return [low, high]


def _normalize_part_randomization_request(request: dict[str, Any] | None) -> dict[str, Any]:
    cfg = _OBJECTS_CFG.get("part_randomization", _DEFAULT_PART_RANDOMIZATION_CFG)
    request = request or {}
    if not isinstance(request, dict):
        raise ValueError("part randomization request must be a JSON object")

    part_names = request.get("parts", cfg.get("parts", PICK_PART_KEYS))
    if isinstance(part_names, str):
        part_names = [part_names]
    part_names = [str(name) for name in part_names]
    unknown = [name for name in part_names if name not in PICK_PART_KEYS]
    if unknown:
        raise ValueError(f"unknown part names for randomization: {unknown}")

    base_range = dict(cfg.get("range", {}))
    base_range.update(request.get("range", {}) or {})
    pose_range = {axis: _range_pair(base_range.get(axis, [0.0, 0.0]), axis) for axis in ("x", "y", "z", "roll", "pitch", "yaw")}

    return {
        "parts": part_names,
        "range": pose_range,
        "seed": request.get("seed", cfg.get("seed")),
        "relative_to": str(request.get("relative_to", cfg.get("relative_to", "initial"))),
    }


def _part_initial_pose(part_name: str) -> tuple[list[float], list[float]]:
    cfg = _PART_CFGS[part_name]
    return _to_float_list(cfg.get("pos", (0.0, 0.0, 0.0))), _quat_normalize(_to_float_list(cfg.get("rot", (1.0, 0.0, 0.0, 0.0))))


def _asset_from_scene(env, part_name: str):
    try:
        return env.scene[part_name]
    except Exception:
        return None


def _prim_from_asset_or_stage(env, part_name: str):
    asset = _asset_from_scene(env, part_name)
    for attr in ("prim", "_prim"):
        prim = getattr(asset, attr, None) if asset is not None else None
        if prim is not None and prim.IsValid():
            return prim
    prims = getattr(asset, "prims", None) if asset is not None else None
    if prims:
        prim = prims[0]
        if prim is not None and prim.IsValid():
            return prim

    stage = env.sim.stage
    prim_name = _PART_PRIM_NAMES[part_name]
    for path in (f"/World/envs/env_0/{prim_name}", f"/World/{prim_name}"):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            return prim
    for prim in stage.Traverse():
        if prim.GetName() == prim_name:
            return prim
    return None


def _usd_prim_pose(prim) -> tuple[list[float], list[float]]:
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = transform.ExtractTranslation()
    rotation = transform.ExtractRotation().GetQuat()
    imag = rotation.GetImaginary()
    return [float(translation[0]), float(translation[1]), float(translation[2])], _quat_normalize([
        float(rotation.GetReal()),
        float(imag[0]),
        float(imag[1]),
        float(imag[2]),
    ])


def _prim_path_or_none(prim) -> str | None:
    if prim is None:
        return None
    try:
        if prim.IsValid():
            return str(prim.GetPath())
    except Exception:
        return None
    return None


def _find_first_mesh_prim(root_prim):
    if root_prim is None:
        return None
    try:
        if root_prim.GetTypeName() == "Mesh":
            return root_prim
        for prim in Usd.PrimRange(root_prim):
            if prim.GetTypeName() == "Mesh":
                return prim
    except Exception:
        return None
    return None


def _get_scene_asset_pose_with_source(env, part_name: str) -> dict[str, Any]:
    asset = _asset_from_scene(env, part_name)
    data = getattr(asset, "data", None)
    if data is not None and hasattr(data, "root_pos_w") and hasattr(data, "root_quat_w"):
        return {
            "pos": _to_float_list(data.root_pos_w[0]),
            "rot": _quat_normalize(_to_float_list(data.root_quat_w[0])),
            "source": "asset_data.root_pos_w",
            "prim_path": _prim_path_or_none(_prim_from_asset_or_stage(env, part_name)),
        }
    if hasattr(asset, "get_world_pose"):
        pos, rot = asset.get_world_pose()
        return {
            "pos": _to_float_list(pos),
            "rot": _quat_normalize(_to_float_list(rot)),
            "source": "asset.get_world_pose",
            "prim_path": _prim_path_or_none(_prim_from_asset_or_stage(env, part_name)),
        }
    if hasattr(asset, "get_world_poses"):
        positions, orientations = asset.get_world_poses()
        return {
            "pos": _to_float_list(positions[0]),
            "rot": _quat_normalize(_to_float_list(orientations[0])),
            "source": "asset.get_world_poses",
            "prim_path": _prim_path_or_none(_prim_from_asset_or_stage(env, part_name)),
        }

    prim = _prim_from_asset_or_stage(env, part_name)
    if prim is None:
        pos, rot = _part_initial_pose(part_name)
        return {
            "pos": pos,
            "rot": rot,
            "source": "initial_fallback",
            "prim_path": None,
        }

    mesh_prim = _find_first_mesh_prim(prim)
    if mesh_prim is not None and mesh_prim != prim:
        pos, rot = _usd_prim_pose(mesh_prim)
        return {
            "pos": pos,
            "rot": rot,
            "source": "usd.mesh_child_prim",
            "prim_path": _prim_path_or_none(mesh_prim),
        }

    pos, rot = _usd_prim_pose(prim)
    return {
        "pos": pos,
        "rot": rot,
        "source": "usd.asset_root_prim",
        "prim_path": _prim_path_or_none(prim),
    }


def _get_scene_asset_pose(env, part_name: str) -> tuple[list[float], list[float]]:
    pose = _get_scene_asset_pose_with_source(env, part_name)
    return pose["pos"], pose["rot"]


def _set_usd_prim_pose(prim, pos: list[float], rot: list[float]) -> None:
    xform = UsdGeom.XformCommonAPI(prim)
    roll, pitch, yaw = _quat_to_euler_xyz(rot)
    xform.SetTranslate(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    xform.SetRotate(
        Gf.Vec3f(math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def _set_scene_asset_pose(env, part_name: str, pos: list[float], rot: list[float]) -> None:
    asset = _asset_from_scene(env, part_name)
    pos_tensor = torch.tensor([pos], dtype=torch.float32, device=env.device)
    rot_tensor = torch.tensor([rot], dtype=torch.float32, device=env.device)
    root_pose = torch.cat((pos_tensor, rot_tensor), dim=-1)

    if hasattr(asset, "write_root_pose_to_sim"):
        asset.write_root_pose_to_sim(root_pose)
        if hasattr(asset, "write_root_velocity_to_sim"):
            asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=env.device))
        return
    if hasattr(asset, "set_world_pose"):
        asset.set_world_pose(position=pos, orientation=rot)
        if hasattr(asset, "set_velocities"):
            asset.set_velocities([0.0] * 6)
        return
    if hasattr(asset, "set_world_poses"):
        asset.set_world_poses(positions=pos_tensor, orientations=rot_tensor)
        if hasattr(asset, "set_velocities"):
            asset.set_velocities(torch.zeros((1, 6), dtype=torch.float32, device=env.device))
        return

    prim = _prim_from_asset_or_stage(env, part_name)
    if prim is None:
        raise RuntimeError(f"Could not find prim for part '{part_name}'")
    _set_usd_prim_pose(prim, pos, rot)


def get_pick_part_states(env) -> dict[str, Any]:
    parts = {}
    for part_name in PICK_PART_KEYS:
        pose = _get_scene_asset_pose_with_source(env, part_name)
        part_state = {"pos": pose["pos"], "rot": pose["rot"], "source": pose["source"]}
        if pose.get("prim_path") is not None:
            part_state["prim_path"] = pose["prim_path"]
        parts[part_name] = part_state
    return {"frame": "world", "source_schema": "part_pose_v2", "parts": parts}


def randomize_pick_part_pieces(env, request: dict[str, Any] | None = None) -> dict[str, Any]:
    cfg = _normalize_part_randomization_request(request)
    rng = random.Random(cfg["seed"])
    pose_range = cfg["range"]
    result = {"frame": "world", "seed": cfg["seed"], "parts": {}}

    for part_name in cfg["parts"]:
        if cfg["relative_to"] == "current":
            base_pos, base_rot = _get_scene_asset_pose(env, part_name)
        else:
            base_pos, base_rot = _part_initial_pose(part_name)

        pos = [
            base_pos[0] + rng.uniform(*pose_range["x"]),
            base_pos[1] + rng.uniform(*pose_range["y"]),
            base_pos[2] + rng.uniform(*pose_range["z"]),
        ]
        delta_rot = _euler_xyz_to_quat(
            rng.uniform(*pose_range["roll"]),
            rng.uniform(*pose_range["pitch"]),
            rng.uniform(*pose_range["yaw"]),
        )
        rot = _quat_multiply(base_rot, delta_rot)
        _set_scene_asset_pose(env, part_name, pos, rot)
        result["parts"][part_name] = {"pos": pos, "rot": rot}

    return result


@configclass
class WalkerS2PickPartSceneCfg(InteractiveSceneCfg):
    scene = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Scene",
        spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
    )

    table = _usd_asset_cfg(_OBJECTS_CFG["table"]).replace(prim_path="{ENV_REGEX_NS}/Table")
    box = _usd_asset_cfg(_OBJECTS_CFG["box"]).replace(prim_path="{ENV_REGEX_NS}/Box")
    part_a_red = _part_rigid_object_cfg(_PART_CFGS["part_a_red"]).replace(prim_path="{ENV_REGEX_NS}/PartA_Red")

    robot = WALKER_S2_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(_ROBOT_INIT_STATE["pos"]),
            rot=tuple(_ROBOT_INIT_STATE["rot"]),
            joint_pos=WALKER_S2_HOME_POSE,
        ),
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

    # Multi-camera setup: 4 individual camera views
    stereo_left = _make_tiled_camera("stereo_left")
    stereo_right = _make_tiled_camera("stereo_right")
    wrist_left = _make_tiled_camera("wrist_left")
    wrist_right = _make_tiled_camera("wrist_right")


@configclass
class ActionsCfg:
    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_ARM_JOINTS,
        scale=1.0,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_ARM_JOINTS,
        scale=1.0,
    )
    left_gripper_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_gripper_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_HEAD_JOINTS,
        scale=1.0,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_WAIST_JOINTS,
        scale=1.0,
    )
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_LEFT_LEG_JOINTS,
        scale=1.0,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_S2_RIGHT_LEG_JOINTS,
        scale=1.0,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        joint_pos = mdp.ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    pass


@configclass
class RewardsCfg:
    pass


@configclass
class TerminationsCfg:
    pass


@configclass
class WalkerS2PickPartEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    scene: WalkerS2PickPartSceneCfg = WalkerS2PickPartSceneCfg(num_envs=1, env_spacing=5.0)
    viewer = ViewerCfg(eye=(2.8, 2.0, 2.0), lookat=(0.8, 0.25, 1.0))

    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    episode_length_s = float(_TASK_CFG["simulation"].get("episode_length_s", 1000.0))
    dynamic_reset_gripper_effort_limit = False

    def __post_init__(self):
        super().__post_init__()
        self.sim.use_fabric = True
        self.sim.enable_scene_query_support = False
        self.sim.disable_contact_processing = True
        self.sim.dt = float(_TASK_CFG["simulation"].get("dt", 0.01))
        self.decimation = int(_TASK_CFG["simulation"].get("decimation", 1))
        self.sim.render_interval = int(_TASK_CFG["simulation"].get("render_interval", 3))

    def use_teleop_device(self, teleop_device: str) -> None:
        self.task_type = teleop_device

    @property
    def camera_names(self) -> list[str]:
        """Return ordered camera names from the YAML config."""
        return list(_CAMERA_NAMES)

    def preprocess_device_action(self, action, teleop_device):
        if isinstance(action, dict) and "walker_s2" in action:
            return action["walker_s2"]
        return action

    def randomize_part_positions(self, env, request: dict[str, Any] | None = None) -> dict[str, Any]:
        return randomize_pick_part_pieces(env, request)

    def get_part_states(self, env) -> dict[str, Any]:
        return get_pick_part_states(env)
