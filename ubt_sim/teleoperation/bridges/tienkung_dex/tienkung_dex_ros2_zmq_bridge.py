#!/usr/bin/env python3
"""TienKung Dex ROS2 <-> ZMQ 桥接。

数据流：

* ROS2 -> 仿真：``JointState`` 指令（body / 双手）转成扁平 JSON ``{关节名: 目标}``，
  通过 ZMQ PUB 发到 ``zmq.cmd_port``（仿真侧 SUB 连接）；
* 仿真 -> ROS2：从 ``zmq.status_port`` 收状态，按 body / 左右手拆成 ``JointState`` 发布；
* 图像：调用共享的 C++ 图像桥（``../zmq_image_bridge``）把 ZMQ raw 帧转成
  ``sensor_msgs/Image``（RGB + 深度）。

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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

# 复用 control/tienkung_dex/constants.py（纯数据，不依赖 isaaclab）
sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "control", "tienkung_dex"),
)
from constants import BODY_JOINTS, DEFAULT_VARIANT, hand_joints, normalize_joint_name, split_status

MSG_TYPES = {
    "JointState": JointState,
    "Bool": Bool,
    "Image": Image,
}

# 指令节流：物理步 100Hz，转发更快只会丢弃
_BODY_PUB_INTERVAL = 1.0 / 100.0


def load_bridge_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


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
        for key, spec in self.cfg["topics"]["pub"].items():
            if key in ("image_rgb", "image_depth"):  # 由 C++ 桥负责
                continue
            self.pubs[key] = self.create_publisher(MSG_TYPES[spec["type"]], spec["topic"], spec.get("qos", 10))

        self._last_body_pub = 0.0
        self._running = True
        self.cpp_bridge_process = None
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True, name="zmq-status-poll")
        self._poll_thread.start()

        sub_callbacks = {
            "body_command": self.command_cb,
            "left_hand_command": self.command_cb,
            "right_hand_command": self.command_cb,
            "reset": self.reset_cb,
        }
        for key, spec in self.cfg["topics"]["sub"].items():
            callback = sub_callbacks.get(key)
            if callback is None:
                continue
            self.create_subscription(MSG_TYPES[spec["type"]], spec["topic"], callback, spec.get("qos", 10))

        self.get_logger().info(f"TienKung Dex ROS2-ZMQ Bridge started (variant={self.variant})")
        if os.environ.get("DISABLE_CPP_IMAGE_BRIDGE", "0") != "1":
            self.start_cpp_bridge()

    # ---------------------------------------------------------------- ROS -> ZMQ
    def command_cb(self, msg: JointState) -> None:
        action: dict[str, float] = {}
        for name, position in zip(msg.name, msg.position):
            sim_name = normalize_joint_name(name)
            if sim_name is not None:
                action[sim_name] = float(position)
        if action:
            self.cmd_socket.send_json(action)

    def reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.cmd_socket.send_json({"reset": True})

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
            self.publish_status(status)

    def publish_status(self, status: dict) -> None:
        joint_names = status.get("joint_names") or []
        joint_pos = status.get("joint_pos") or []
        joint_vel = status.get("joint_vel") or []
        groups = split_status(joint_names, joint_pos, self.variant)
        vel_map = dict(zip(joint_names, joint_vel))
        stamp = self.get_clock().now().to_msg()

        for key, names in (
            ("body_state", BODY_JOINTS),
            ("left_hand_state", self.left_hand_joints),
            ("right_hand_state", self.right_hand_joints),
        ):
            group = "body" if key == "body_state" else ("left_hand" if key == "left_hand_state" else "right_hand")
            values = groups[group]
            if not values or key not in self.pubs:
                continue
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [name for name in names if name in values]
            msg.position = [float(values[name]) for name in msg.name]
            msg.velocity = [float(vel_map.get(name, 0.0)) for name in msg.name]
            msg.effort = [0.0] * len(msg.name)
            self.pubs[key].publish(msg)

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
    node = TienkungDexRosBridge(args.config, args.variant)
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
