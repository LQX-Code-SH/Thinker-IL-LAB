from pxr import Usd, UsdGeom, Gf, Sdf
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ViewerCfg
from isaaclab.actuators import ImplicitActuatorCfg

from isaaclab.scene import InteractiveSceneCfg
from ubt_sim.devices.tienkung_pro.config import (
    TIENKUNG_PRO_CFG,
    TIENKUNG_PRO_HOME_POSE,
    TIENKUNG_PRO_LEFT_ARM_JOINTS,
    TIENKUNG_PRO_RIGHT_ARM_JOINTS,
    TIENKUNG_PRO_LEFT_HAND_JOINTS,
    TIENKUNG_PRO_RIGHT_HAND_JOINTS,
    TIENKUNG_PRO_HEAD_JOINTS,
    TIENKUNG_PRO_WAIST_JOINTS,
    TIENKUNG_PRO_LEFT_LEG_JOINTS,
    TIENKUNG_PRO_RIGHT_LEG_JOINTS,
    TIENKUNG_PRO_USD_PATH,
)
from ubt_sim.utils.config_loader import load_config, resolve_asset_path
from ubt_sim.env.digital_twin_env_cfg import (
    ManagerBasedRLDigitalTwinEnvCfg,
)

# Load scene config from YAML
_TASK_CFG = load_config("tienkung_pro/parlor.yaml")
_SCENE_USD_PATH = resolve_asset_path(_TASK_CFG["scene"]["usd_path"])

PARLOR_SCENE_CFG = AssetBaseCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=_SCENE_USD_PATH),
)


@configclass
class TienkungProParlorSceneCfg(InteractiveSceneCfg):
    # Scene
    scene = PARLOR_SCENE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")
    # Robot
    robot = TIENKUNG_PRO_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(7.80, 6.08257, 0.02),
            rot=(0.99939, 0.0, 0.0349, 0.0), # 向前倾斜 4 度 (绕 Y 轴旋转)
            # rot=(0.99813, 0.0, 0.06105, 0.0), # 向前倾斜 7 度 (绕 Y 轴旋转) --- IGNORE ---
            joint_pos=TIENKUNG_PRO_HOME_POSE
        )
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )

        # Camera
    camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/humanoid/head_roll_link/Camera_RGB",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.075191, 0.010935, 0.074436),
            rot=(0.430457, -0.560985, 0.560987, -0.430459),
            convention="ros",
        ),
        # update_period=0.0: 每步 readback，内容更新频率与发送频率同步。
        # 原 0.0333 按仿真 dt(0.01) 累积，每 4 仿真步才 readback 一次，
        # 导致主循环墙钟 ~20Hz 时内容仅 5Hz 刷新，3/4 帧为重复画面。
        update_period=0.0,
        height=360,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=44.76,
            vertical_aperture=24.99,
            clipping_range=(0.1, 1.0e5)
        ),
    )

    # 要开启深度相机请关闭注释
    camera_depth = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/humanoid/head_roll_link/Camera_Depth",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.075191, 0.010935, 0.074436),
            rot=(0.430457, -0.560985, 0.560987, -0.430459),
            convention="ros",
        ),
        # update_period=0.0: 每步 readback，内容更新频率与发送频率同步。
        # 原 0.0333 按仿真 dt(0.01) 累积，每 4 仿真步才 readback 一次，
        # 导致主循环墙钟 ~20Hz 时内容仅 5Hz 刷新，3/4 帧为重复画面。
        update_period=0.0,
        height=360,
        width=640,
        data_types=["depth"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=48.0,
            vertical_aperture=30.58,
            clipping_range=(0.1, 1.0e5)
        ),
    )

@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    # Arms (Joint Position Control)
    left_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_LEFT_ARM_JOINTS,
        scale=1.0,
    )
    right_arm_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_RIGHT_ARM_JOINTS,
        scale=1.0,
    )

    # Hands (Joint Position Control)
    left_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_LEFT_HAND_JOINTS,
        scale=1.0,
    )
    right_hand_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_RIGHT_HAND_JOINTS,
        scale=1.0,
    )

    # Head and Waist
    head_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_HEAD_JOINTS,
        scale=1.0,
    )
    waist_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_WAIST_JOINTS,
        scale=1.0,
    )

    # Legs
    left_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_LEFT_LEG_JOINTS,
        scale=1.0,
    )
    right_leg_action = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=TIENKUNG_PRO_RIGHT_LEG_JOINTS,
        scale=1.0,
    )



@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""
    @configclass
    class PolicyCfg(ObservationGroupCfg):
        # Basic joint positions and velocities
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
    """Configuration for events."""
    pass


@configclass
class RewardsCfg:
    """Reward terms for the environment."""
    pass


@configclass
class TerminationsCfg:
    """Termination terms for the environment."""
    pass


@configclass
class TienkungProParlorEnvCfg(ManagerBasedRLDigitalTwinEnvCfg):
    """Configuration for the Tienkung Pro Parlor environment."""
    # Scene
    scene: TienkungProParlorSceneCfg = TienkungProParlorSceneCfg(num_envs=1, env_spacing=5.0)

    # Viewer
    # Update viewer to focus on the robot at (7.9, 6.08, 0.02)
    viewer = ViewerCfg(eye=(5.5, 6.0, 2.0), lookat=(7.9, 6.08, 1.0))

    # Components
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    episode_length_s = 1000.0
    dynamic_reset_gripper_effort_limit = False

    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        self.sim.use_fabric = True
        self.sim.enable_scene_query_support = False

        # 物理求解器优化

        # 如果不需要机器人和环境发生物理碰撞，开启这个会极大地提速
        self.sim.disable_contact_processing = True

        # 尝试将物理步长稍微调大一点点（如果稳定性允许）
        # 100Hz 物理已经很低了，再低可能会让手部控制变飘，暂不建议再降
        self.sim.dt = 0.01
        self.decimation = 1

        # 渲染优化：每 10 步物理才渲染一次画面 (约 30Hz)
        self.sim.render_interval = 3


    def use_teleop_device(self, teleop_device: str) -> None:
        """
        Configure the environment for a specific teleoperation device.
        This updates the action configuration based on the device.
        """
        self.task_type = teleop_device
        from ubt_sim.devices.action_process import init_action_cfg
        self.actions = init_action_cfg(self.actions, device=teleop_device)

    def preprocess_device_action(self, action, teleop_device):
        """
        Preprocess the action from the teleoperation device before feeding it to the environment.
        This handles device-specific mapping (e.g., 0-1 range to joint limits).
        """
        from ubt_sim.devices.action_process import preprocess_device_action
        return preprocess_device_action(action, teleop_device)
