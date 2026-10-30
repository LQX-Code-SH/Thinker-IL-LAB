"""Walker C1 / Astron 客厅抓放任务（UBTSim-WalkerC1-Parlor-v0）。

场景家具（parlor scene_v2）纯视觉、无碰撞，物理由三组不可见碰撞体/刚体提供：
  - 桌面下的不可见立方体（0.60 × 1.20 × 0.06），对齐叠加层移动后的桌子；
  - 盘子位置上的不可见圆柱（r 0.085 / h 0.05）作为放置目标面；
  - 可抓取苹果沿用原 parlor 苹果外观，但保留已验证的隐球碰撞
    （scene 里的装饰苹果已由叠加层置为 inactive）。
"""

from __future__ import annotations

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import EventTermCfg, ObservationGroupCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from ubt_sim.devices.walker_c1.config import (
    WALKER_C1_CFG,
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
from ubt_sim.env.digital_twin_env_cfg import ManagerBasedRLDigitalTwinEnvCfg
from ubt_sim.utils.config_loader import load_config, resolve_asset_path

_TASK_CFG = load_config("walker_c1/parlor.yaml")
_SCENE_CFG = _TASK_CFG["scene"]
_SCENE_USD_PATH = resolve_asset_path(_SCENE_CFG["usd_path"])
_APPLE_USD_PATH = resolve_asset_path("robots/walker_c1/c1_task_apple.usda")
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_CAMERAS_CFG = _TASK_CFG["cameras"]
# 依赖 dict 保序：camera_names 顺序 = YAML 中定义顺序（采集数据列顺序依赖此顺序）
_CAMERA_NAMES = list(_CAMERAS_CFG.keys())

PARLOR_SCENE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
)

# ---------------------------------------------------------------- 抓放任务道具
_TABLE_TOP_Z = 0.902
_TABLE_X_SHIFT = -0.16
_TASK_X_SHIFT = -0.04  # 保持已验证的苹果 x 位置独立于桌子平移
_TABLE_POS = (8.444 + _TABLE_X_SHIFT, 6.083, _TABLE_TOP_Z - 0.03)
_TABLE_SIZE = (0.60, 1.20, 0.06)
_PLATE_POS = (8.19, 6.083, 0.905)
_PLATE_RADIUS = 0.085
_PLATE_HEIGHT = 0.05
_GRASP_OBJECT_RADIUS = 0.040
_GRASP_OBJECT_MASS = 0.10
# 苹果起点：桌面靠前边缘、右手前方（预备位掌心约在世界系 (8.04, 5.89, 0.85+)）
_GRASP_OBJECT_INIT_POS = (
    8.21 + _TASK_X_SHIFT,
    5.86,
    _TABLE_TOP_Z + _GRASP_OBJECT_RADIUS + 0.002,
)


def _make_tiled_camera(name: str) -> TiledCameraCfg:
    cfg = _CAMERAS_CFG[name]
    res = cfg.get("resolution", [640, 480])
    return TiledCameraCfg(
        prim_path=cfg["prim_path"],
        offset=TiledCameraCfg.OffsetCfg(
            pos=tuple(cfg.get("pos", (0.0, 0.0, 0.0))),
            rot=tuple(cfg.get("rot", (1.0, 0.0, 0.0, 0.0))),
            convention="ros",
        ),
        # 每步 readback：内容刷新频率与发送频率同步，避免采集到重复帧
        update_period=0.0,
        height=int(res[1]),
        width=int(res[0]),
        data_types=cfg.get("data_types", ["rgb"]),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=float(cfg.get("focal_length", 24.0)),
            focus_distance=float(cfg.get("focus_distance", 400.0)),
            horizontal_aperture=float(cfg.get("horizontal_aperture", 44.76)),
            vertical_aperture=float(cfg.get("vertical_aperture", 24.99)),
            clipping_range=tuple(cfg.get("clipping_range", (0.1, 1.0e5))),
        ),
    )


@configclass
class WalkerC1ParlorSceneCfg(InteractiveSceneCfg):
    scene = PARLOR_SCENE_CFG.replace(
        prim_path=_SCENE_CFG.get("prim_path", "{ENV_REGEX_NS}/Scene")
    )

    robot = WALKER_C1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(_ROBOT_INIT_STATE["pos"]),
            rot=tuple(_ROBOT_INIT_STATE["rot"]),
            joint_pos=WALKER_C1_HOME_POSE,
        ),
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TableTopCollider",
        spawn=sim_utils.CuboidCfg(
            size=_TABLE_SIZE,
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_POS),
    )

    plate = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlateCollider",
        spawn=sim_utils.CylinderCfg(
            radius=_PLATE_RADIUS,
            height=_PLATE_HEIGHT,
            axis="Z",
            visible=False,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2, dynamic_friction=1.2
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=_PLATE_POS),
    )

    object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        spawn=sim_utils.UsdFileCfg(
            usd_path=_APPLE_USD_PATH,
            # 不要为了阻止释放后滚动而加大 angular_damping：不能滚的球会被
            # 闭合的手指挤飞（实测 damping=2.0 时每次下探都把苹果弹下桌）。
            # 滚动改由"在盘面上方约 1 cm 释放"控制。
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=_GRASP_OBJECT_MASS),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=_GRASP_OBJECT_INIT_POS),
    )

    head_rgb = _make_tiled_camera("head_rgb")


@configclass
class ActionsCfg:
    """动作分组与 devices/walker_c1/config.py 的关节分组一一对应。

    手部通道 ``use_default_offset=False``：手部 HOME 为 0，且真机 SDK 下发的是
    绝对角度；臂/头/腰/腿 HOME 也基本为 0（仅膝 0.08），保留默认 offset 语义
    与参考实现一致。
    """

    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_ARM_JOINTS,
        scale=1.0,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_ARM_JOINTS,
        scale=1.0,
    )
    left_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_HEAD_JOINTS,
        scale=1.0,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_WAIST_JOINTS,
        scale=1.0,
    )
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_LEFT_LEG_JOINTS,
        scale=1.0,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=WALKER_C1_RIGHT_LEG_JOINTS,
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
        joint_vel = mdp.ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # 没有该事件时 env.reset() 不会恢复场景实体状态：苹果会停在上一条轨迹的
    # 位置，机器人也保留上一集的关节状态。
    reset_scene = EventTermCfg(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class RewardsCfg:
    pass


@configclass
class TerminationsCfg:
    pass


@configclass
class WalkerC1ParlorEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    scene: WalkerC1ParlorSceneCfg = WalkerC1ParlorSceneCfg(num_envs=1, env_spacing=5.0)
    viewer = ViewerCfg(eye=(5.5, 6.0, 2.4), lookat=(7.9, 6.08, 1.2))

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

    # ---------------------------------------------------------- 设备面接口
    @property
    def camera_names(self) -> list[str]:
        return list(_CAMERA_NAMES)

    def use_teleop_device(self, teleop_device: str) -> None:
        self.task_type = teleop_device

    def preprocess_device_action(self, action, teleop_device):
        if isinstance(action, dict) and "walker_c1" in action:
            return action["walker_c1"]
        return action
