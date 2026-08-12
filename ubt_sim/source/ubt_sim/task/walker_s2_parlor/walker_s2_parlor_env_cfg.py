import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
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
_TASK_CFG = load_config("walker_s2/parlor.yaml")
_SCENE_USD_PATH = resolve_asset_path(_TASK_CFG["scene"]["usd_path"])
_ROBOT_INIT_STATE = _TASK_CFG["robot"]["init_state"]
_CAMERAS_CFG = _TASK_CFG["cameras"]  # dict[str, dict]
# 依赖 Python 3.7+ dict 保序特性，camera_names 顺序 = YAML 中定义顺序
_CAMERA_NAMES = list(_CAMERAS_CFG.keys())

PARLOR_SCENE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
)


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
        # update_period=0.0: 每步 readback，内容更新频率与发送频率同步。
        # 原 0.0333 按仿真 dt(0.01) 累积，每 4 仿真步才 readback 一次，
        # 导致主循环墙钟 ~20Hz 时内容仅 5Hz 刷新，3/4 帧为重复画面。
        update_period=0.0,
        height=int(res[1]),
        width=int(res[0]),
        data_types=cfg.get("data_types", ["rgb"]),
        spawn=None,
    )


@configclass
class WalkerS2ParlorSceneCfg(InteractiveSceneCfg):
    scene = PARLOR_SCENE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

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
class WalkerS2ParlorEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    scene: WalkerS2ParlorSceneCfg = WalkerS2ParlorSceneCfg(num_envs=1, env_spacing=5.0)
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
