#!/usr/bin/env python3
"""ROS2 Deploy Bridge for LeRobot + TienKung Dex.

Bridges between LeRobot (Python 3.12, ZMQ) and the robot backend via ROS2 DDS.

Runs on Python 3.10 (system) with rclpy for ROS2 communication.

用法：

  # --config（推荐，由 TienKungDexRobot._start_bridge() 传入）
  python3 ros2_deploy_bridge.py --config '{"zmq_cmd_port":5559, ...}'

  # 独立调试（已弃用的旧参数）
  python3 ros2_deploy_bridge.py --zmq_cmd_port 5559 --zmq_status_port 5560

ZMQ 内部端口（LeRobot ↔ Bridge2）：

  5559: LeRobot PUB → Bridge2 SUB（动作）
  5560: Bridge2 PUB → LeRobot SUB（状态）
  5558: Bridge2 PUB → DexCamera SUB（JPEG；仅 image_enabled 时由 /ob_camera_head 转发）

只驱动**右臂 7 + 右手 13（因时）/ 6（强脑）**。左臂/腰/头/腿由
``scripts/deploy/tienkung_dex/robot_ready.py`` 摆到预备位后保持不动——
本桥接**默认不碰 /waist/cmd 与 /leg/cmd**，腿永远不发（真机由行走控制器独占。
注意 /waist/cmd 与 /leg/cmd 与 /arm/cmd 同用 ArmCtrl 消息，只是话题不同）。
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import threading
import time
from typing import Any

import zmq

logger = logging.getLogger("ros2_deploy_bridge")


# ── 运行期常量副本 ──────────────────────────────────────────────────────────
# 本文件跑在系统 Python 3.10（ROS2 Humble），**不能** import 3.12 venv 里的
# lerobot_robot_tienkung_dex，也看不到 ubt_sim。因此关节表 / 限位 / 手部通道表
# 在这里留一份运行期副本：权威定义在 ubt_sim/teleoperation/control/tienkung_dex/
# constants.py，插件包 tienkung_dex/lerobot_robot_tienkung_dex/.../constants.py
# 是第二份，本文件是第三份。改动必须三处手工同步。
#
# 启动断言（_assert_hand_table）会拿插件经 --config 传下来的右手关节序与本文件
# 的通道表逐字比对，不一致直接拒启——那道断言就是防这份副本悄悄过期。

DEFAULT_VARIANT = "inspire"

# 右臂 7 关节，顺序 = motor id 21..27 的顺序 = 插件 right_arm_joints 的顺序
RIGHT_ARM_JOINTS = [
    "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint",
    "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint",
    "elbow_yaw_r_joint",  # 真机命名 right_wrist_yaw
    "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
]
RIGHT_ARM_MOTOR_IDS = [21, 22, 23, 24, 25, 26, 27]

# 右臂 URDF 限位 (lower, upper)，单位 rad。Bridge2 是最后一道机械保护。
BODY_JOINT_LIMITS = {
    "shoulder_pitch_r_joint": (-2.844887, 2.844887),
    "shoulder_roll_r_joint": (-3.333579, 0.191986),
    "shoulder_yaw_r_joint": (-2.844887, 2.844887),
    "elbow_pitch_r_joint": (-2.548181, 0.191986),
    "elbow_yaw_r_joint": (-2.844887, 2.844887),
    "wrist_pitch_r_joint": (-1.32645, 1.32645),
    "wrist_roll_r_joint": (-1.32645, 1.32645),
}

# 真机各组位置指令的 spd/cur（沿用 SDK demo 实测值：手臂 cur=40 才够扛重力）。
# 不填会让真机走 SDK 默认值，行为不可控。
GROUP_CMD_LIMITS = {"arm": {"spd": 0.5, "cur": 40.0}}

# 真机 ControlMode（HWI §7.1）：0=位置 1=阻抗 2=速度 ...
CONTROL_MODE_POSITION = 0
# 强脑 SetMotorMulti 的 SDK 默认控制模式
HAND_CMD_MODE = 1
# 因时 SetAngle.hand_id：1=左手 2=右手（预留字段，真机按话题分左右）
INSPIRE_RIGHT_HAND_ID = 2

BRAINCO_CODE_MIN, BRAINCO_CODE_MAX = 1, 1000

# 强脑 Revo2：6 电机顺序 thumb_flex / thumb_rot / index / middle / ring / pinky
BRAINCO_MOTOR_JOINTS: tuple[tuple[str, ...], ...] = (
    ("thumb_proximal",),
    ("thumb_metacarpal",),
    ("index_proximal",),
    ("middle_proximal",),
    ("ring_proximal",),
    ("pinky_proximal",),
)

# 因时 RH56：13 通道，(仿真关节短名, 码下限, 码上限, 0 rad 对应码, 方向)。
# 屈曲通道码值越大越伸直，方向 d=-1；侧摆通道零位 0、方向 d=+1。
INSPIRE_CHANNELS: tuple[tuple[str, int, int, int, int], ...] = (
    ("pinky_mcp", 900, 1650, 1650, -1),    # 0  小拇指指根
    ("ring_mcp", 900, 1650, 1650, -1),     # 1  无名指指根
    ("middle_mcp", 900, 1650, 1650, -1),   # 2  中指指根
    ("middle_yaw", -150, 150, 0, +1),      # 3  中指侧摆
    ("index_mcp", 900, 1650, 1650, -1),    # 4  食指指根
    ("index_yaw", -150, 150, 0, +1),       # 5  食指侧摆
    ("pinky_pip", 1150, 1900, 1900, -1),   # 6  小拇指指中
    ("ring_pip", 1150, 1900, 1900, -1),    # 7  无名指指中
    ("middle_pip", 1150, 1900, 1900, -1),  # 8  中指指中
    ("index_pip", 1150, 1900, 1900, -1),   # 9  食指指中
    ("thumb_yaw", 650, 1750, 1750, -1),    # 10 大拇指侧摆
    ("thumb_mcp", 1250, 1600, 1600, -1),   # 11 大拇指一关节弯曲
    ("thumb_pip", 1500, 2040, 2040, -1),   # 12 大拇指二关节弯曲
)

INSPIRE_CHANNEL_JOINTS: tuple[tuple[str, ...], ...] = tuple(
    (short,) for short, *_ in INSPIRE_CHANNELS
)

HAND_CHANNEL_COUNT = {
    "brainco": len(BRAINCO_MOTOR_JOINTS),
    "inspire": len(INSPIRE_CHANNEL_JOINTS),
}

# 因时通道码的物理单位：0.1° 一个码
INSPIRE_CODE_DEG = 0.1

# 强脑关节限位（rad）。hand_rad_to_code 按 action_limit 归一化成闭合度。
HAND_JOINT_LIMITS = {
    "brainco": {
        "thumb_metacarpal": (0.0, 1.55334),
        "thumb_proximal": (0.0, 1.0472),
        "thumb_distal": (0.0, 1.15192),
        "index_proximal": (0.0, 1.41),
        "index_distal": (0.0, 1.69297),
        "middle_proximal": (0.0, 1.41),
        "middle_distal": (0.0, 1.69297),
        "ring_proximal": (0.0, 1.41),
        "ring_distal": (0.0, 1.69297),
        "pinky_proximal": (0.0, 1.41),
        "pinky_distal": (0.0, 1.69297),
    },
}


def _short_name(joint_name: str) -> str:
    """``right_index_mcp_joint`` -> ``index_mcp``。"""
    short = joint_name
    for prefix in ("left_", "right_"):
        if short.startswith(prefix):
            short = short[len(prefix):]
            break
    return short.removesuffix("_joint")


def action_limit(joint_name: str, variant: str = DEFAULT_VARIANT) -> float:
    """关节"闭合/动作"方向的极限（绝对值较大的一侧）。"""
    short = _short_name(joint_name)
    low, high = HAND_JOINT_LIMITS[variant][short]
    return low if abs(low) > abs(high) else high


def hand_channel_joints(variant: str, side: str) -> tuple[tuple[str, ...], ...]:
    """返回该变体/左右手每个真机通道对应的仿真关节全名（可能为空元组）。"""
    prefix = f"{side}_"
    if variant == "brainco":
        table = BRAINCO_MOTOR_JOINTS
    elif variant == "inspire":
        table = INSPIRE_CHANNEL_JOINTS
    else:
        raise ValueError(f"未知的灵巧手变体: {variant}")
    return tuple(tuple(f"{prefix}{joint}_joint" for joint in entry) for entry in table)


def inspire_code_to_rad(channel: int, code: int) -> float:
    """因时通道码（0.1°，带零位偏置与方向）-> 仿真关节角（rad）。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    clipped = min(max(int(code), low), high)
    return (clipped - zero) * direction * INSPIRE_CODE_DEG * math.pi / 180.0


def inspire_rad_to_code(channel: int, rad: float) -> int:
    """仿真关节角（rad）-> 因时通道码；超出真机行程时夹到通道上下限。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    code = zero + float(rad) * 180.0 / math.pi / (direction * INSPIRE_CODE_DEG)
    return int(round(min(max(code, low), high)))


def hand_percent_to_code(variant: str, percent: float) -> int:
    """闭合度 [0, 1] -> 强脑通道值（取整并夹在值域内）。"""
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def hand_code_to_rad(variant: str, channel: int, code: int, joint_name: str) -> float:
    """真机通道值 -> 仿真关节角（rad），两个变体的统一入口。"""
    if variant == "brainco":
        clipped = min(max(int(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX)
        percent = (clipped - BRAINCO_CODE_MIN) / float(BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)
        return percent * action_limit(joint_name, variant)
    return inspire_code_to_rad(channel, code)


def hand_rad_to_code(variant: str, channel: int, rad: float, joint_name: str) -> int:
    """仿真关节角（rad）-> 真机通道值，两个变体的统一入口。"""
    if variant == "brainco":
        limit = action_limit(joint_name, variant)
        ratio = float(rad) / limit if abs(limit) > 1e-6 else 0.0
        return hand_percent_to_code(variant, ratio)
    return inspire_rad_to_code(channel, rad)


def _assert_hand_table(variant: str, joints_from_plugin: list[str] | None) -> list[str]:
    """校验手部通道表完整，并与插件传下来的硬件关节序逐字比对。

    "13 个手部关节名全部命中通道表、无重复、无遗漏"是这套映射唯一能自证的地方：
    数据集手序（URDF: thumb→pinky）与真机通道序（pinky→thumb）不是同一个排列，
    只要有一处错位，策略下发的手指就会整体错位，而画面看不出来。
    """
    table = hand_channel_joints(variant, "right")
    names = [entry[0] if entry else "" for entry in table]
    expected = HAND_CHANNEL_COUNT[variant]

    if len(table) != expected:
        raise RuntimeError(
            f"{variant} 通道表长度 {len(table)} != HAND_CHANNEL_COUNT {expected}"
        )
    empty = [i for i, name in enumerate(names) if not name]
    if empty:
        raise RuntimeError(f"{variant} 通道表有空通道: {empty}")
    if len(set(names)) != len(names):
        raise RuntimeError(f"{variant} 通道表有重复关节: {names}")

    if joints_from_plugin:
        stripped = [name.removesuffix(".pos") for name in joints_from_plugin]
        if stripped != names:
            raise RuntimeError(
                "插件传入的右手关节序与本文件通道表不一致（插件与桥接的常量副本已漂移）：\n"
                f"  插件: {stripped}\n  桥接: {names}"
            )
    return names


# ── Default config values (used when --config is not provided) ────────────────
_DEFAULT_CFG = {
    "zmq_cmd_port": 5559,
    "zmq_status_port": 5560,
    "ros_namespace": "",
    "cmd_namespace": "",
    "hand_variant": DEFAULT_VARIANT,
    "right_arm_motor_ids": list(RIGHT_ARM_MOTOR_IDS),
    "arm_speed": GROUP_CMD_LIMITS["arm"]["spd"],
    "arm_current": GROUP_CMD_LIMITS["arm"]["cur"],
    "topic_arm_cmd": "/arm/cmd",
    "topic_right_hand_cmd": "",  # 留空 = 按 hand_variant 选
    "topic_robot_state": "/robot_state",
    "topic_estop": "/power/board/key_status",
    "topic_reset": "/sim/cmd_reset",
    "image_enabled": False,
    "image_port": 5558,
    "topic_image_head": "/ob_camera_head/color/image_raw",
}

# 手型 -> (指令话题, 状态话题)，与真机话题表一致
_HAND_TOPICS = {
    "inspire": ("/right_hand/angle_set", "/right_hand/angle_actual"),
    "brainco": ("/right_hand/set_motor_multi", "/right_hand/motor_status"),
}


def _status_qos(depth: int = 10):
    """状态订阅统一用 BEST_EFFORT。

    QoS 兼容性是单向的：BEST_EFFORT 订阅能收 RELIABLE 与 BEST_EFFORT 两种发布端，
    而 RELIABLE 订阅只收 RELIABLE 发布端。真机各节点的 QoS 不由我们决定，一旦
    ``/robot_state`` 是 BEST_EFFORT，默认（RELIABLE）订阅会表现为"话题列表可见、
    ``ros2 topic echo`` 有数，但桥接一行收不到"——最难查的一类故障。
    状态本来就是缓存语义，丢一帧无影响，所以这里取兼容性最大的那一侧。
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ZMQInternalBridge:
    """ZMQ sockets for communication with LeRobot process.

    Bridge2 binds (server), LeRobot connects (client).
    """

    def __init__(self, cmd_port: int, status_port: int):
        self.context = zmq.Context()
        # SUB: receive actions from LeRobot
        self.cmd_socket = self.context.socket(zmq.SUB)
        self.cmd_socket.bind(f"tcp://*:{cmd_port}")
        self.cmd_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_socket.setsockopt(zmq.RCVHWM, 1)

        # PUB: forward status to LeRobot
        self.status_socket = self.context.socket(zmq.PUB)
        self.status_socket.bind(f"tcp://*:{status_port}")
        self.status_socket.setsockopt(zmq.SNDHWM, 1)

        # PUB: forward JPEG frames to DexCamera（仅 image_enabled 时创建）
        self.image_socket: zmq.Socket | None = None

        logger.info("ZMQ internal bridge: cmd=%d, status=%d", cmd_port, status_port)

    def bind_image(self, port: int) -> None:
        self.image_socket = self.context.socket(zmq.PUB)
        self.image_socket.bind(f"tcp://*:{port}")
        # 相机端 RCVHWM=1，这里同样只留最新一帧，积压无意义
        self.image_socket.setsockopt(zmq.SNDHWM, 1)
        logger.info("ZMQ image bridge: %d", port)

    def recv_action(self, timeout_ms: int = 100) -> dict | None:
        try:
            msg = self.cmd_socket.recv_json(flags=zmq.NOBLOCK)
            return msg
        except zmq.Again:
            return None

    def send_status(self, status: dict) -> None:
        try:
            self.status_socket.send_json(status, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Status send dropped: ZMQ send buffer full (SNDHWM=1)")

    def send_image(self, jpg_bytes: bytes) -> None:
        if self.image_socket is None:
            return
        try:
            self.image_socket.send(jpg_bytes, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Image send dropped: ZMQ send buffer full (SNDHWM=1)")

    def close(self) -> None:
        if self.image_socket is not None:
            self.image_socket.close(linger=0)
            self.image_socket = None
        self.cmd_socket.close(linger=0)
        self.status_socket.close(linger=0)
        self.context.term()


class RealRobotBridge:
    """ROS2 DDS ↔ TienKung Dex hardware.

    Subscribes /robot_state + 右手状态 + 急停（+ 头相机），发布 /arm/cmd + 手指令。
    所有硬件常量（motor id、spd/cur、话题名、手型）都从 cfg 读。
    """

    def __init__(self, zmq_bridge: ZMQInternalBridge, cfg: dict):
        self.zmq_bridge = zmq_bridge
        self._cfg = cfg
        self._variant = cfg["hand_variant"]
        if self._variant not in HAND_CHANNEL_COUNT:
            raise RuntimeError(f"未知 hand_variant: {self._variant!r}")

        self._right_arm_motor_ids = [int(m) for m in cfg["right_arm_motor_ids"]]
        if len(self._right_arm_motor_ids) != len(RIGHT_ARM_JOINTS):
            raise RuntimeError(
                f"right_arm_motor_ids 需要 {len(RIGHT_ARM_JOINTS)} 个，"
                f"收到 {len(self._right_arm_motor_ids)} 个"
            )
        self._arm_speed = float(cfg.get("arm_speed", GROUP_CMD_LIMITS["arm"]["spd"]))
        self._arm_current = float(cfg.get("arm_current", GROUP_CMD_LIMITS["arm"]["cur"]))

        hand_cmd_topic, hand_state_topic = _HAND_TOPICS[self._variant]
        self._topic_right_hand_cmd = cfg.get("topic_right_hand_cmd") or hand_cmd_topic
        self._topic_right_hand_state = hand_state_topic

        # 启动断言：通道表自洽 + 与插件传下来的关节序一致（不一致拒启）
        self._right_hand_joints = _assert_hand_table(
            self._variant, cfg.get("right_hand_joints")
        )
        self._hand_channel_count = HAND_CHANNEL_COUNT[self._variant]

        ros_namespace = (cfg.get("ros_namespace") or "").rstrip("/")
        cmd_namespace = (cfg.get("cmd_namespace") or "").rstrip("/")

        self._msgs = self._load_msgs()

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("ros2_deploy_bridge")

        # State caches
        self._right_arm_jpos = [0.0] * len(self._right_arm_motor_ids)
        self._right_hand_pos = [0.0] * self._hand_channel_count
        self._estop = False
        self._state_lock = threading.Lock()

        # Publishers — command side (cmd_namespace)
        arm_topic = f"{cmd_namespace}{cfg['topic_arm_cmd']}" if cmd_namespace else cfg["topic_arm_cmd"]
        self._arm_cmd_pub = self._node.create_publisher(
            self._msgs["ArmCtrl"], arm_topic, 10
        )
        hand_cmd_full = (
            f"{cmd_namespace}{self._topic_right_hand_cmd}"
            if cmd_namespace else self._topic_right_hand_cmd
        )
        self._right_hand_pub = self._node.create_publisher(
            self._msgs["hand_command"], hand_cmd_full, 10
        )

        # Subscribers — status side (ros_namespace)
        status_qos = _status_qos()
        self._node.create_subscription(
            self._msgs["RobotState"],
            f"{ros_namespace}{cfg['topic_robot_state']}",
            self._robot_state_callback, status_qos,
        )
        self._node.create_subscription(
            self._msgs["hand_state"],
            f"{ros_namespace}{self._topic_right_hand_state}",
            self._hand_state_callback, status_qos,
        )
        self._create_estop_subscription(cfg, ros_namespace)

        # 头相机：只有真机才需要转发（仿真里 AsyncJpegCameraSender 直发 5758）
        self._cv_bridge = None
        self._cv2 = None
        self._last_image_ts = 0.0
        self._image_min_interval = 1.0 / 30.0
        if cfg.get("image_enabled", False):
            self._setup_image_forwarding(cfg, ros_namespace)

        # Start executor
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2_executor"
        )
        self._executor_thread.start()

        # Action forwarding thread
        self._running = True
        self._action_thread = threading.Thread(
            target=self._action_loop, daemon=True, name="action_forward"
        )
        self._action_thread.start()

        logger.info(
            "Real robot bridge started (ns=%s, cmd_ns=%s, hand=%s, motors=%s, spd=%.2f, cur=%.0f)",
            ros_namespace, cmd_namespace, self._variant,
            self._right_arm_motor_ids, self._arm_speed, self._arm_current,
        )
        logger.info("右手通道 -> 关节: %s", dict(enumerate(self._right_hand_joints)))

    # ------------------------------------------------------------------ 消息包
    def _load_msgs(self) -> dict[str, Any]:
        """导入真机消息包；缺包时给出可执行的修复提示。"""
        try:
            from ros2_bridge_msgs.msg import ArmCtrl, MotorCtrl, RobotState
        except ImportError as exc:
            raise RuntimeError(
                f"{exc}：需要 ros2_bridge_msgs。容器内先 bash run.sh check，"
                "或 source /opt/tienkung_dex_msgs_ws/install/setup.bash"
            ) from exc

        msgs: dict[str, Any] = {
            "ArmCtrl": ArmCtrl,
            "MotorCtrl": MotorCtrl,
            "RobotState": RobotState,
        }
        if self._variant == "brainco":
            try:
                from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti
            except ImportError as exc:
                raise RuntimeError(
                    f"{exc}：brainco 变体需要 brainco_hand_msgs，"
                    "容器内先 bash run.sh check"
                ) from exc
            msgs.update({"hand_command": SetMotorMulti, "hand_state": MotorStatus})
        else:
            try:
                from inspire_hand_msgs.msg import GetAngleAct, SetAngle
            except ImportError as exc:
                raise RuntimeError(
                    f"{exc}：inspire 变体需要 inspire_hand_msgs，"
                    "容器内先 bash run.sh check"
                ) from exc
            msgs.update({"hand_command": SetAngle, "hand_state": GetAngleAct})
        return msgs

    def _create_estop_subscription(self, cfg: dict, ros_namespace: str) -> None:
        """急停订阅，消息包不可用时降级为"无急停"并告警（仿真里通常没有这个包）。"""
        try:
            from bodyctrl_msgs.msg import PowerBoardKeyStatus
        except ImportError:
            logger.warning(
                "bodyctrl_msgs 不可用：急停监听已关闭。真机部署前必须补上"
                "（/power/board/key_status 是唯一能在紧急时停发的软件通道）。"
            )
            return
        self._node.create_subscription(
            PowerBoardKeyStatus,
            f"{ros_namespace}{cfg['topic_estop']}",
            self._estop_callback, _status_qos(),
        )

    def _setup_image_forwarding(self, cfg: dict, ros_namespace: str) -> None:
        """真机头相机：sensor_msgs/Image → JPEG → ZMQ image_port。"""
        import cv2
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self._cv2 = cv2
        self._cv_bridge = CvBridge()
        self._jpeg_quality = int(cfg.get("image_quality", 80))

        self.zmq_bridge.bind_image(int(cfg["image_port"]))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._node.create_subscription(
            Image, f"{ros_namespace}{cfg['topic_image_head']}", self._image_callback, qos
        )
        logger.info(
            "图像转发已开启：%s -> ZMQ %d",
            cfg["topic_image_head"], cfg["image_port"],
        )

    # ---------------------------------------------------------------- 回调
    def _robot_state_callback(self, msg: Any) -> None:
        """/robot_state：按关节 ID 取右臂 7 维（不是按位取）。"""
        pos_by_id = {int(motor.name): float(motor.pos) for motor in msg.arm.status}
        missing = [mid for mid in self._right_arm_motor_ids if mid not in pos_by_id]
        if missing:
            if not getattr(self, "_warned_missing_ids", None):
                self._warned_missing_ids = set()
            new = [mid for mid in missing if mid not in self._warned_missing_ids]
            if new:
                self._warned_missing_ids.update(new)
                logger.warning("/robot_state 缺少右臂关节 ID %s，本帧状态不更新", new)
            return

        with self._state_lock:
            for i, mid in enumerate(self._right_arm_motor_ids):
                self._right_arm_jpos[i] = pos_by_id[mid]
        self._publish_status()

    def _hand_state_callback(self, msg: Any) -> None:
        """右手反馈：因时 GetAngleAct(13 码) / 强脑 MotorStatus(6 闭合度) → rad。"""
        values = list(msg.positions) if self._variant == "brainco" else list(msg.joint_values)
        if len(values) < self._hand_channel_count:
            return
        rad = [
            hand_code_to_rad(self._variant, ch, values[ch], self._right_hand_joints[ch])
            for ch in range(self._hand_channel_count)
        ]
        with self._state_lock:
            self._right_hand_pos[:] = rad
        self._publish_status()

    def _estop_callback(self, msg: Any) -> None:
        engaged = bool(getattr(msg, "is_estop").data)
        if engaged == self._estop:
            return
        self._estop = engaged
        if engaged:
            logger.error("急停触发（is_estop=true）：已停止下发手臂与手指令。")
        else:
            logger.warning("急停解除（is_estop=false）：恢复下发。")
        self._publish_status()

    def _image_callback(self, msg: Any) -> None:
        now = time.monotonic()
        if now - self._last_image_ts < self._image_min_interval:
            return
        self._last_image_ts = now
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = self._cv2.imencode(
                ".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
        except Exception as exc:  # noqa: BLE001 - 单帧失败不应打死执行器
            logger.warning("图像转换失败: %s", exc)
            return
        if ok:
            self.zmq_bridge.send_image(buf.tobytes())

    # ---------------------------------------------------------------- 状态
    def _publish_status(self) -> None:
        with self._state_lock:
            status = {
                "right_arm": list(self._right_arm_jpos),
                "right_hand": list(self._right_hand_pos),
                "estop": self._estop,
                "ts": time.time(),
            }
        self.zmq_bridge.send_status(status)

    # ---------------------------------------------------------------- 指令
    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is None:
                continue
            self._publish_arm_command(action.get("right_arm", []))
            self._publish_hand_command(action.get("right_hand", []))

    def _publish_arm_command(self, right_arm: list) -> None:
        if not right_arm:
            return
        if self._estop:
            return
        if len(right_arm) != len(RIGHT_ARM_JOINTS):
            logger.warning(
                "Arm command dimension mismatch: right_arm=%d (expect %d). Skipping.",
                len(right_arm), len(RIGHT_ARM_JOINTS),
            )
            return

        msg = self._msgs["ArmCtrl"]()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "arm"
        msg.mode = CONTROL_MODE_POSITION

        for joint_name, motor_id, value in zip(
            RIGHT_ARM_JOINTS, self._right_arm_motor_ids, right_arm
        ):
            low, high = BODY_JOINT_LIMITS[joint_name]
            pos = min(max(float(value), low), high)
            if pos != float(value):
                logger.debug("%s 目标 %.4f 超限，夹到 [%.4f, %.4f]", joint_name, value, low, high)
            cmd = self._msgs["MotorCtrl"]()
            cmd.name = int(motor_id)
            cmd.pos = pos
            cmd.spd = self._arm_speed
            cmd.cur = self._arm_current
            msg.ctrl.append(cmd)

        self._arm_cmd_pub.publish(msg)

    def _publish_hand_command(self, position: list) -> None:
        if not position:
            return
        if self._estop:
            return
        if len(position) != self._hand_channel_count:
            logger.warning(
                "Hand command dimension mismatch: right_hand=%d (expect %d). Skipping.",
                len(position), self._hand_channel_count,
            )
            return

        # rad -> 真机通道码（因时按通道上下限夹取；强脑按关节行程折算闭合度）
        codes = [
            hand_rad_to_code(self._variant, ch, position[ch], self._right_hand_joints[ch])
            for ch in range(self._hand_channel_count)
        ]

        msg = self._msgs["hand_command"]()
        if self._variant == "brainco":
            count = HAND_CHANNEL_COUNT["brainco"]
            msg.mode = HAND_CMD_MODE
            msg.positions = codes[:count]
            msg.speeds = [0] * count
            msg.currents = [0] * count
            msg.pwms = [0] * count
            msg.durations = [0] * count
        else:
            msg.hand_id = INSPIRE_RIGHT_HAND_ID
            msg.joint_values = codes
        self._right_hand_pub.publish(msg)

    def stop(self) -> None:
        self._running = False
        if self._action_thread.is_alive():
            self._action_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


def _is_alive(pid: int) -> bool:
    """Check whether a process is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # process exists but we lack permission to signal it


def kill_existing_bridge(tag: str = "tienkung_dex") -> None:
    """Find and kill any already-running bridge processes of *this* robot.

    Sends SIGTERM first for graceful shutdown, then SIGKILL after 3 s if
    the process is still alive.  Waits an extra 0.5 s for ZMQ ports to be
    released before returning.

    与 tienkung_pro 的同名函数有两处不同，都是必须的：
    1. 额外要求路径里带 *tag*——三个机型的桥接脚本都叫 ros2_deploy_bridge.py，
       只按文件名匹配会把同时运行的 tienkung_pro 桥接一起杀掉；
    2. 只匹配 argv[1]（被执行的脚本），不匹配整条 cmdline。lerobot-rollout
       主进程把桥接路径作为 --robot.bridge_script= 的值带在身上，
       pgrep -f / pkill -f 会命中它，等于桥接启动时杀掉自己的父进程。
    """
    current_pid = os.getpid()
    parent_pid = os.getppid()

    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in (current_pid, parent_pid):
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                parts = f.read().split(b"\x00")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if len(parts) < 2:
            continue
        script = parts[1].decode("utf-8", "replace")
        if script.endswith("ros2_deploy_bridge.py") and tag in script:
            pids.append(pid)

    if not pids:
        return

    logger.info("Found existing bridge processes (PIDs: %s), terminating ...", pids)

    # --- graceful SIGTERM ---
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    # wait up to 3 s for them to exit
    alive = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [p for p in pids if _is_alive(p)]
        if not alive:
            break
        time.sleep(0.1)
    else:
        # --- force SIGKILL for stragglers ---
        for pid in alive:
            logger.warning("Force killing bridge process %d", pid)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.1)

    # give the OS a moment to release ZMQ sockets
    time.sleep(0.5)
    logger.info("Previous bridge instances terminated.")


def _reset_once(topic: str, repeat: int = 3, interval: float = 0.1) -> int:
    """发 ``/sim/cmd_reset``（仿真专用，真机没有订阅者）。

    连发 *repeat* 次是为了躲 SUB 连接建立期的竞态：仿真侧的订阅者可能还没挂上，
    单发一次会静默丢失。
    """
    import rclpy
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("tienkung_dex_reset")
    pub = node.create_publisher(Bool, topic, 1)
    try:
        for _ in range(max(1, repeat)):
            pub.publish(Bool(data=True))
            time.sleep(interval)
        logger.info("已发送 %s x%d", topic, max(1, repeat))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ROS2 Deploy Bridge for LeRobot + TienKung Dex"
    )
    parser.add_argument("--config", type=str, default=None,
                        help="JSON config string from TienKungDexRobotConfig.to_bridge_config() "
                             "(recommended). If omitted, legacy CLI args or defaults are used.")
    parser.add_argument("--variant", type=str, default=None, choices=["inspire", "brainco"],
                        help="灵巧手变体（默认取 config 里的 hand_variant）")
    parser.add_argument("--domain-id", type=int, default=None,
                        help="覆盖 ROS_DOMAIN_ID（146=仿真，0=真机）；必须在 rclpy.init 前设置")
    parser.add_argument("--enable-image", action="store_true",
                        help="把 /ob_camera_head 的 ROS 图像转成 JPEG 发到 image_port（真机用）")
    parser.add_argument("--reset", action="store_true",
                        help="只发一次 /sim/cmd_reset 然后退出（仿真复位）")
    # Legacy args (deprecated, kept for standalone testing)
    parser.add_argument("--zmq_cmd_port", type=int, default=None,
                        help="(deprecated) ZMQ port for receiving actions from LeRobot (bind SUB)")
    parser.add_argument("--zmq_status_port", type=int, default=None,
                        help="(deprecated) ZMQ port for sending status to LeRobot (bind PUB)")
    parser.add_argument("--ros_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for status topics (subscribe)")
    parser.add_argument("--cmd_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for command topics (publish)")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    # Build cfg dict: start from defaults, overlay --config JSON, then CLI overrides
    cfg = dict(_DEFAULT_CFG)

    if args.config:
        try:
            cfg.update(json.loads(args.config))
        except json.JSONDecodeError as e:
            logger.error("Failed to parse --config JSON: %s", e)
            return 1

    if args.variant is not None:
        cfg["hand_variant"] = args.variant
    if args.enable_image:
        cfg["image_enabled"] = True
    if args.zmq_cmd_port is not None:
        cfg["zmq_cmd_port"] = args.zmq_cmd_port
    if args.zmq_status_port is not None:
        cfg["zmq_status_port"] = args.zmq_status_port
    if args.ros_namespace is not None:
        cfg["ros_namespace"] = args.ros_namespace
    if args.cmd_namespace is not None:
        cfg["cmd_namespace"] = args.cmd_namespace

    # Configure logging early so kill_existing_bridge() messages are visible
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    # 必须在任何 rclpy 调用之前设置
    if args.domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    if args.reset:
        return _reset_once(cfg["topic_reset"])

    # --- kill any existing bridge before starting ---
    kill_existing_bridge()

    zmq_bridge = ZMQInternalBridge(cfg["zmq_cmd_port"], cfg["zmq_status_port"])
    try:
        robot_bridge = RealRobotBridge(zmq_bridge, cfg)
    except RuntimeError as exc:
        logger.error("桥接启动失败：%s", exc)
        zmq_bridge.close()
        return 1

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Bridge running (ros_ns=%s, cmd_ns=%s). Press Ctrl+C to stop.",
                cfg.get("ros_namespace", ""), cfg.get("cmd_namespace", ""))
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down...")
    robot_bridge.stop()
    zmq_bridge.close()
    logger.info("Bridge stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
