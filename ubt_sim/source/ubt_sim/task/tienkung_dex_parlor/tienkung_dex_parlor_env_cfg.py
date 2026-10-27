"""TienKung Dex parlor 场景配置。

场景复用 ``assets/scenes/parlor/scene_v2.usd``，机器人初始位姿与相机参数
全部来自 ``config/tienkung_dex/parlor.yaml``，改场景/相机不需要改代码。
"""

from functools import partial

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.envs import ViewerCfg
from isaaclab.managers import ObservationGroupCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass

from ubt_sim.devices.tienkung_dex.config import (
    TIENKUNG_DEX_CFG,
    TIENKUNG_DEX_HEAD_JOINTS,
    TIENKUNG_DEX_HOME_POSE,
    TIENKUNG_DEX_LEFT_ARM_JOINTS,
    TIENKUNG_DEX_LEFT_HAND_JOINTS,
    TIENKUNG_DEX_LEFT_LEG_JOINTS,
    TIENKUNG_DEX_RIGHT_ARM_JOINTS,
    TIENKUNG_DEX_RIGHT_HAND_JOINTS,
    TIENKUNG_DEX_RIGHT_LEG_JOINTS,
    TIENKUNG_DEX_WAIST_JOINTS,
)
from ubt_sim.env.digital_twin_env_cfg import ManagerBasedRLDigitalTwinEnvCfg
from ubt_sim.utils.config_loader import load_config, resolve_asset_path
from ubt_sim.utils.scene_overrides import spawn_scene_with_table_lift

_TASK_CFG = load_config("tienkung_dex/parlor.yaml")
_SCENE_CFG = _TASK_CFG["scene"]
_SCENE_USD_PATH = resolve_asset_path(_SCENE_CFG["usd_path"])
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_CAMERAS_CFG = _TASK_CFG["cameras"]
# 依赖 dict 保序：camera_names 顺序 = YAML 中定义顺序（采集数据列顺序依赖此顺序）
_CAMERA_NAMES = list(_CAMERAS_CFG.keys())

PARLOR_SCENE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_SCENE_USD_PATH,
        # 桌面高度覆盖（YAML 的 scene.table_z_scale）：加载后拉伸 table 并抬高
        # 台上物体，不影响其他机型共用的 scene_v2.usd
        func=partial(
            spawn_scene_with_table_lift,
            table_z_scale=float(_SCENE_CFG.get("table_z_scale", 1.0)),
            lifted_prims=tuple(_SCENE_CFG.get("table_top_prims", ())),
        ),
    ),
)


def _make_tiled_camera(name: str) -> TiledCameraCfg:
    cfg = _CAMERAS_CFG[name]
    res = cfg.get("resolution", [640, 360])
    horizontal_aperture = float(cfg.get("horizontal_aperture", 44.76))
    vertical_aperture = float(cfg.get("vertical_aperture", 24.99))
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
            horizontal_aperture=horizontal_aperture,
            vertical_aperture=vertical_aperture,
            clipping_range=tuple(cfg.get("clipping_range", (0.1, 1.0e5))),
        ),
    )


@configclass
class TienkungDexParlorSceneCfg(InteractiveSceneCfg):
    scene = PARLOR_SCENE_CFG.replace(prim_path=_TASK_CFG["scene"].get("prim_path", "{ENV_REGEX_NS}/Scene"))

    robot = TIENKUNG_DEX_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(_ROBOT_INIT_STATE["pos"]),
            rot=tuple(_ROBOT_INIT_STATE["rot"]),
            joint_pos=TIENKUNG_DEX_HOME_POSE,
        ),
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

    head_rgb = _make_tiled_camera("head_rgb")
    head_depth = _make_tiled_camera("head_depth")


@configclass
class ActionsCfg:
    """Action 分组：与 devices/tienkung_dex/config.py 的关节分组一一对应。

    所有通道统一 ``use_default_offset=False``：action 就是关节目标绝对角度。
    Isaac Lab 默认 ``use_default_offset=True``，会把 action 解释成
    ``default_joint_pos + action`` 的偏移量——本机型站立姿态非零
    （``TIENKUNG_DEX_HOME_POSE``），带偏移的话真机下发的绝对角度到仿真里就
    会整体平移一个站姿量，同一份控制脚本无法同时驱真机和仿真。
    """

    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_LEFT_ARM_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_RIGHT_ARM_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    left_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_LEFT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_RIGHT_HAND_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_HEAD_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_WAIST_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_LEFT_LEG_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_DEX_RIGHT_LEG_JOINTS,
        scale=1.0,
        use_default_offset=False,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

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
    pass


@configclass
class RewardsCfg:
    pass


@configclass
class TerminationsCfg:
    pass


@configclass
class TienkungDexParlorEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    """TienKung Dex parlor 环境配置。"""

    scene: TienkungDexParlorSceneCfg = TienkungDexParlorSceneCfg(num_envs=1, env_spacing=5.0)

    viewer = ViewerCfg(eye=(5.5, 6.0, 2.0), lookat=(7.9, 6.08, 1.0))

    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    episode_length_s = float(_TASK_CFG["simulation"].get("episode_length_s", 1000.0))
    dynamic_reset_gripper_effort_limit = False

    # 手部变体：inspire（RH5DG2，18 关节/手）| brainco（Revo2，11 关节/手）
    task_variant: str = "inspire"

    def __post_init__(self):
        super().__post_init__()
        self.sim.use_fabric = True
        self.sim.enable_scene_query_support = False
        # 遥操作采集不需要接触力处理，关闭可显著提速
        self.sim.disable_contact_processing = True
        self.sim.dt = float(_TASK_CFG["simulation"].get("dt", 0.01))
        self.decimation = int(_TASK_CFG["simulation"].get("decimation", 1))
        self.sim.render_interval = int(_TASK_CFG["simulation"].get("render_interval", 1))

    @property
    def camera_names(self) -> list[str]:
        return list(_CAMERA_NAMES)

    def use_teleop_device(self, teleop_device: str) -> None:
        self.task_type = teleop_device

    def preprocess_device_action(self, action, teleop_device):
        from ubt_sim.devices.action_process import preprocess_device_action

        return preprocess_device_action(action, teleop_device)
