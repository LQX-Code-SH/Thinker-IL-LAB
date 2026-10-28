#!/usr/bin/env python3
"""TienKung Dex ROS2 <-> ZMQ 桥接。

数据流：

* ROS2 -> 仿真：关节指令转成扁平 JSON ``{关节名: 目标}``，通过 ZMQ PUB 发到
  ``zmq.cmd_port``（仿真侧 SUB 连接）；
* 仿真 -> ROS2：从 ``zmq.status_port`` 收状态，打包成 ROS 消息发布；
* 图像：调用共享的 C++ 图像桥（``../zmq_image_bridge``）把 ZMQ raw 帧转成
  ``sensor_msgs/Image``（RGB + 深度）。

话题与消息类型与具身天工 3.0 真机完全一致（``ros2_bridge_msgs`` 的
``/arm|head|waist|leg/cmd``、``/robot_state``，灵巧手的
``/left_hand/set_motor_multi`` / ``/left_hand/angle_set`` 等），按关节 ID 编址。
仿真与真机只有 ``ROS_DOMAIN_ID`` 不同（仿真 146 / 真机 0），控制脚本不用改。

用法（容器内）：

    source /opt/ros/humble/setup.bash
    /usr/bin/python3 teleoperation/bridges/tienkung_dex/tienkung_dex_ros2_zmq_bridge.py \
        --config teleoperation/bridges/tienkung_dex/bridge_config.yaml
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading

import rclpy
import yaml
import zmq
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

# 复用 control/tienkung_dex/ 下的纯数据模块（不依赖 isaaclab）
sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "control", "tienkung_dex"),
)
from constants import (
    CONTROL_MODE_POSITION,
    DEFAULT_VARIANT,
    GROUP_JOINT_IDS,
    JOINT_ID_TO_SIM,
    hand_channel_joints,
    hand_code_to_rad,
    hand_joints,
    hand_rad_to_code,
)

MSG_TYPES = {
    "Bool": Bool,
    "Image": Image,
    "Float32": Float32,
    "Point": Point,
}


def load_bridge_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def load_real_msgs(variant: str) -> dict:
    """导入真机消息包；缺包时给出可执行的修复提示。

    ``brainco_hand_msgs`` / ``inspire_hand_msgs`` 优先用厂商包（真机自带）；
    仿真机上装的是 ``teleoperation/msgs/tienkung_dex/dex_hand_msgs_src`` 下按
    SDK 用法重建的同名包，字段布局未经厂商核对——这里检测到用的是重建版就告警。
    """
    try:
        from ros2_bridge_msgs.msg import ArmCtrl, HeadCtrl, MotorStatus, RobotState
    except ImportError as exc:
        raise RuntimeError(
            f"{exc}：真机话题档需要 ros2_bridge_msgs。"
            "容器内先 bash run.sh init（会编译 teleoperation/msgs/tienkung_dex），"
            "或 source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash"
        ) from exc

    msgs = {
        "ArmCtrl": ArmCtrl,
        "HeadCtrl": HeadCtrl,
        "MotorStatus": MotorStatus,
        "RobotState": RobotState,
    }
    if variant == "brainco":
        try:
            from brainco_hand_msgs.msg import MotorStatus as HandMotorStatus
            from brainco_hand_msgs.msg import SetMotorMulti
        except ImportError as exc:
            raise RuntimeError(
                f"{exc}：brainco 变体的真机话题档需要 brainco_hand_msgs，"
                "容器内先 bash run.sh init"
            ) from exc
        msgs.update({"hand_command": SetMotorMulti, "hand_state": HandMotorStatus})
        _warn_if_local_stub("brainco_hand_msgs", "brainco 灵巧手")
    else:
        try:
            from inspire_hand_msgs.msg import GetAngleAct, SetAngle
        except ImportError as exc:
            raise RuntimeError(
                f"{exc}：inspire 变体的真机话题档需要 inspire_hand_msgs，"
                "容器内先 bash run.sh init"
            ) from exc
        msgs.update({"hand_command": SetAngle, "hand_state": GetAngleAct})
        _warn_if_local_stub("inspire_hand_msgs", "因时灵巧手")
    return msgs


def _warn_if_local_stub(package: str, label: str) -> None:
    """检测是否在用仿真端重建的消息包（而非厂商包），用一次告警提示。"""
    module = sys.modules.get(package)
    path = getattr(module, "__file__", "") or ""
    if "/opt/ubt_sim/tienkung_dex_msgs/" in path:
        print(
            f"[WARN] {label}消息使用仿真端重建的 {package}（{path}）："
            "字段布局未与厂商包核对；上真机请 source 厂商包",
            file=sys.stderr,
        )


class TienkungDexRosBridge(Node):
    def __init__(self, config_path: str, variant: str = DEFAULT_VARIANT):
        super().__init__("tienkung_dex_ros_bridge")
        self.cfg = load_bridge_config(config_path)
        self.variant = variant
        self.left_hand_joints, self.right_hand_joints = hand_joints(variant)
        zmq_cfg = self.cfg["zmq"]

        self.zmq_context = zmq.Context()
        self.cmd_socket = self.zmq_context.socket(zmq.PUB)
        # HWM=10：一帧内可能连发 body + 双手 3 条消息，HWM=1 会丢后续消息
        self.cmd_socket.setsockopt(zmq.SNDHWM, 10)
        self.cmd_socket.bind(f"tcp://*:{zmq_cfg['cmd_port']}")

        self.status_socket = self.zmq_context.socket(zmq.SUB)
        self.status_socket.connect(f"tcp://127.0.0.1:{zmq_cfg['status_port']}")
        self.status_socket.setsockopt(zmq.RCVHWM, 1)
        self.status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.pubs: dict[str, any] = {}
        self._warned: set[str] = set()
        self._running = True
        self.cpp_bridge_process = None

        self._setup_io()

        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True, name="zmq-status-poll")
        self._poll_thread.start()

        self.get_logger().info(
            f"TienKung Dex ROS2-ZMQ Bridge started (variant={self.variant}, "
            f"domain={os.environ.get('ROS_DOMAIN_ID', '0')})"
        )
        if os.environ.get("DISABLE_CPP_IMAGE_BRIDGE", "0") != "1":
            self.start_cpp_bridge()

    # ------------------------------------------------------------------ 话题
    def _setup_io(self) -> None:
        """按关节 ID 编址，话题与消息类型对齐具身天工 3.0（仿真/真机同一套）。"""
        self.real_msgs = load_real_msgs(self.variant)
        pub_cfg = self.cfg["topics"]["pub"]
        self.pubs["robot_state"] = self.create_publisher(
            self.real_msgs["RobotState"], pub_cfg["robot_state"]["topic"], pub_cfg["robot_state"].get("qos", 10)
        )

        # 任务真值（苹果↔盘心距离）：配置里没有就跳过，老配置仍能跑
        task_cfg = pub_cfg.get("task_dist")
        if task_cfg:
            self.pubs["task_dist"] = self.create_publisher(
                MSG_TYPES[task_cfg["type"]], task_cfg["topic"], task_cfg.get("qos", 10)
            )

        for key, spec in self.cfg["topics"]["sub"].items():
            if key.endswith("_command") and key != "reset":
                group = key[: -len("_command")]
                self.create_subscription(
                    self.real_msgs[spec["type"]],
                    spec["topic"],
                    lambda msg, g=group: self.group_command_cb(g, msg),
                    spec.get("qos", 10),
                )
            elif key == "reset":
                self.create_subscription(MSG_TYPES[spec["type"]], spec["topic"], self.reset_cb, spec.get("qos", 10))
            elif key == "apple_offset":
                self.create_subscription(
                    MSG_TYPES[spec["type"]], spec["topic"], self.apple_offset_cb, spec.get("qos", 10)
                )

        hand_cfg = self.cfg["hand_topics"][self.variant]
        for side in ("left", "right"):
            self.pubs[f"{side}_hand_state"] = self.create_publisher(
                self.real_msgs["hand_state"], hand_cfg[side]["state"], 10
            )
            self.create_subscription(
                self.real_msgs["hand_command"],
                hand_cfg[side]["command"],
                lambda msg, s=side: self.hand_command_cb(s, msg),
                10,
            )
        self._publish_status = self._publish_real_status

    # ---------------------------------------------------------------- ROS -> ZMQ
    def group_command_cb(self, group: str, msg) -> None:
        """真机 ArmCtrl / HeadCtrl：``ctrl[].name`` 是关节 ID，``pos`` 是绝对角度。"""
        mode = int(getattr(msg, "mode", CONTROL_MODE_POSITION))
        if mode != CONTROL_MODE_POSITION and f"mode:{group}" not in self._warned:
            self._warned.add(f"mode:{group}")
            self.get_logger().warn(
                f"/{group}/cmd 下发了 mode={mode}（非位置模式）；仿真只有位置控制，按 pos 执行"
            )
        action: dict[str, float] = {}
        for ctrl in msg.ctrl:
            joint_id = int(ctrl.name)
            sim_name = JOINT_ID_TO_SIM.get(joint_id)
            if sim_name is None:
                if f"id:{joint_id}" not in self._warned:
                    self._warned.add(f"id:{joint_id}")
                    self.get_logger().warn(f"/{group}/cmd 收到未知关节 ID {joint_id}，已忽略")
                continue
            action[sim_name] = float(ctrl.pos)
        if action:
            self.cmd_socket.send_json(action)

    def hand_command_cb(self, side: str, msg) -> None:
        """真机灵巧手指令 -> 仿真关节弧度。

        强脑 6 电机：1~1000 按闭合度乘关节行程；
        因时 13 通道：单位 0.1° 的绝对角码，(code - 零位码) × 方向 × 0.1° 即关节角
        （屈曲通道码越大越伸直，通道定义见 ``constants.INSPIRE_CHANNELS``）。
        """
        if self.variant == "brainco":
            codes = list(msg.positions)
        else:
            codes = list(msg.joint_values)
        action: dict[str, float] = {}
        table = hand_channel_joints(self.variant, side)
        for channel, joints in enumerate(table):
            if not joints or channel >= len(codes):
                continue
            joint = joints[0]
            action[joint] = hand_code_to_rad(self.variant, channel, codes[channel], joint)
        if action:
            self.cmd_socket.send_json(action)

    def reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.cmd_socket.send_json({"reset": True})

    def apple_offset_cb(self, msg: Point) -> None:
        """苹果位置偏移（世界系 x/y，米）。仿真侧按「复位位姿 + 偏移」摆苹果。"""
        self.cmd_socket.send_json({"apple_offset": [float(msg.x), float(msg.y)]})

    # ---------------------------------------------------------------- ZMQ -> ROS
    def _poll_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self.status_socket, zmq.POLLIN)
        while self._running:
            socks = dict(poller.poll(timeout=100))
            if self.status_socket not in socks:
                continue
            try:
                status = self.status_socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            except Exception as exc:  # 序列化异常不应打死线程
                self.get_logger().warn(f"status recv failed: {exc}")
                continue
            self._publish_status(status)

    def _publish_real_status(self, status: dict) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_robot_state(status, stamp)
        self._publish_hand_state(status, stamp)
        self._publish_task_dist(status)

    def _publish_robot_state(self, status: dict, stamp) -> None:
        pos_map = dict(zip(status.get("joint_names") or [], status.get("joint_pos") or []))
        vel_map = dict(zip(status.get("joint_names") or [], status.get("joint_vel") or []))
        if not pos_map:
            return

        state = self.real_msgs["RobotState"]()
        state.header.stamp = stamp
        for group in ("head", "waist", "leg", "arm"):
            container = getattr(state, group)
            container.header.stamp = stamp
            for joint_id in GROUP_JOINT_IDS[group]:
                sim_name = JOINT_ID_TO_SIM[joint_id]
                if sim_name not in pos_map:
                    continue
                motor = self.real_msgs["MotorStatus"]()
                motor.name = joint_id
                motor.pos = float(pos_map[sim_name])
                motor.speed = float(vel_map.get(sim_name, 0.0))
                container.status.append(motor)
        self.pubs["robot_state"].publish(state)

    def _publish_task_dist(self, status: dict) -> None:
        """苹果↔盘心距离（仿真真值）转成 ``/sim/task_completed``。

        仿真算不出距离时发的是 -1.0，照原样透传——消费端（采集脚本）自己判
        "负数 = 没有真值"，桥接不替它做决定。配置里没有这个发布器就跳过。
        """
        if "task_dist" not in self.pubs or "task_dist" not in status:
            return
        msg = Float32()
        msg.data = float(status["task_dist"])
        self.pubs["task_dist"].publish(msg)

    def _publish_hand_state(self, status: dict, stamp) -> None:
        pos_map = dict(zip(status.get("joint_names") or [], status.get("joint_pos") or []))
        if not pos_map:
            return

        for side in ("left", "right"):
            table = hand_channel_joints(self.variant, side)
            codes = []
            for channel, joints in enumerate(table):
                if not joints:
                    codes.append(0)
                    continue
                joint = joints[0]
                codes.append(
                    hand_rad_to_code(self.variant, channel, pos_map.get(joint, 0.0), joint)
                )
            msg = self.real_msgs["hand_state"]()
            if self.variant == "brainco":
                msg.positions = codes
            else:
                msg.joint_names = [joints[0] if joints else f"{side}_unused_{i}"
                                   for i, joints in enumerate(table)]
                msg.joint_values = codes
            self.pubs[f"{side}_hand_state"].publish(msg)

    # ------------------------------------------------------------- C++ 图像桥
    def start_cpp_bridge(self) -> None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        bridge_dir = os.path.dirname(script_dir)
        build_script = os.path.join(bridge_dir, "build_cpp_bridge.sh")
        executable = os.path.join(bridge_dir, "zmq_image_bridge")

        subprocess.run(["chmod", "+x", build_script], cwd=bridge_dir)
        build = subprocess.run([build_script], cwd=bridge_dir, capture_output=True, text=True)
        if build.returncode != 0 or not os.path.isfile(executable):
            self.get_logger().error(f"C++ 图像桥编译失败，图像发布不可用:\n{build.stderr}")
            return

        pub_cfg = self.cfg["topics"]["pub"]
        zmq_cfg = self.cfg["zmq"]
        args = [
            executable,
            "--zmq-port", str(zmq_cfg["image_port"]),
            "--rgb-topic", pub_cfg["image_rgb"]["topic"],
            "--depth-topic", pub_cfg["image_depth"]["topic"],
            "--msg-type", pub_cfg["image_rgb"].get("type", "Image"),
        ]
        self.get_logger().info(f"Starting C++ image bridge: {' '.join(args)}")
        try:
            self.cpp_bridge_process = subprocess.Popen(args, cwd=bridge_dir)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"C++ 图像桥启动失败: {exc}")
            self.cpp_bridge_process = None

    def stop(self) -> None:
        self._running = False
        if self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        if self.cpp_bridge_process is not None:
            self.cpp_bridge_process.terminate()
            try:
                self.cpp_bridge_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.cpp_bridge_process.kill()
        for sock in (self.cmd_socket, self.status_socket):
            try:
                sock.close(linger=0)
            except Exception:
                pass
        try:
            self.zmq_context.term()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="TienKung Dex ROS2 <-> ZMQ bridge")
    default_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_config.yaml")
    parser.add_argument("--config", default=default_cfg, help="桥接配置文件路径")
    parser.add_argument("--variant", default=DEFAULT_VARIANT, choices=["inspire", "brainco"], help="手部变体")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    try:
        node = TienkungDexRosBridge(args.config, args.variant)
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
