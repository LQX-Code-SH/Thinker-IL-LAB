#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool, String
import yaml
import zmq

try:
    from shm_msgs.msg import Image2m
except ImportError:
    Image2m = None  # shm_msgs may not be available in some environments

try:
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
    from mc_state_msgs.msg import RobotState
    from ecat_task_msgs.msg import GripCmd, GripStatus
except ImportError as exc:
    raise ImportError(
        "Walker S2 ROS2 SDK messages not found. Build and source vendored messages first, e.g.\n"
        "  cd /ubt_sim/docker && bash run.sh init\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash\n"
        "Then verify /usr/bin/python3 can import mc_task_msgs, mc_state_msgs, and ecat_task_msgs."
    ) from exc


WALKER_S2_SDK_BODY_JOINT_ORDER = [
    "L_elbow_roll_joint",
    "L_elbow_yaw_joint",
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
    "R_elbow_roll_joint",
    "R_elbow_yaw_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "waist_yaw_joint",
]

WALKER_S2_LEFT_HAND_JOINT_ORDER = [
    "left_thumb_swing",
    "left_thumb_mcp",
    "left_thumb_pip",
    "left_index_mcp",
    "left_middle_mcp",
    "left_ring_mcp",
    "left_little_mcp",
]

WALKER_S2_RIGHT_HAND_JOINT_ORDER = [
    "right_thumb_swing",
    "right_thumb_mcp",
    "right_thumb_pip",
    "right_index_mcp",
    "right_middle_mcp",
    "right_ring_mcp",
    "right_little_mcp",
]

_ALIAS_TO_SIM_JOINT = {name.removesuffix("_joint"): name for name in WALKER_S2_SDK_BODY_JOINT_ORDER}
_ALIAS_TO_SIM_JOINT.update({name: name for name in WALKER_S2_SDK_BODY_JOINT_ORDER})

MSG_TYPES = {
    "RobotCommand": RobotCommand,
    "JointCommand": JointCommand,
    "GripCmd": GripCmd,
    "Bool": Bool,
    "String": String,
    "RobotState": RobotState,
    "JointState": JointState,
    "GripStatus": GripStatus,
    "Image": Image,
    "Image2m": Image2m,
}


def load_bridge_config(config_path: str) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def _normalize_body_name(name: str) -> str | None:
    return _ALIAS_TO_SIM_JOINT.get(name)


class WalkerS2RosBridge(Node):
    def __init__(self, config_path: str):
        super().__init__("walker_s2_ros_bridge")
        self.cfg = load_bridge_config(config_path)
        zmq_cfg = self.cfg["zmq"]

        self.zmq_context = zmq.Context()
        self.cmd_socket = self.zmq_context.socket(zmq.PUB)
        # HWM=10：回放/控制脚本一帧内会连发 body+双手+双夹爪共 5 条消息，HWM=1
        # 会把第 2-5 条全部丢弃（仿真按物理步 ~10ms 批量 drain），夹爪开合的
        # 台阶指令偶发丢失。10 足够容纳单帧突发，又不积压过期指令。
        self.cmd_socket.setsockopt(zmq.SNDHWM, 10)
        self.cmd_socket.bind(f"tcp://*:{zmq_cfg['cmd_port']}")

        self.status_socket = self.zmq_context.socket(zmq.SUB)
        self.status_socket.connect(f"tcp://127.0.0.1:{zmq_cfg['status_port']}")
        self.status_socket.setsockopt(zmq.RCVHWM, 1)
        self.status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.pubs = {}
        self.latest_left_hand = {}
        self.latest_right_hand = {}
        self.latest_left_grip = None
        self.latest_right_grip = None

        # Body command rate limiter: prevent 500 Hz hold/idle traffic from
        # flooding the ZMQ pipe.  The sim physics runs at ~100 Hz (dt=0.01);
        # forwarding more often than that wastes CPU on serialisation /
        # deserialisation of messages that will be overwritten before the next
        # physics step anyway.
        self._last_body_pub_time = 0.0
        self._body_pub_interval = 1.0 / 100.0  # 100 Hz
        self._pvt_warned = False

        sub_callbacks = {
            "command": self.command_cb,
            "left_hand_command": lambda msg: self.hand_cb(msg, "left"),
            "right_hand_command": lambda msg: self.hand_cb(msg, "right"),
            "left_grip_command": lambda msg: self.grip_cb(msg, "left"),
            "right_grip_command": lambda msg: self.grip_cb(msg, "right"),
            "reset": self.reset_cb,
            "randomize_parts": self.randomize_parts_cb,
        }
        for key, spec in self.cfg["topics"]["sub"].items():
            msg_type = MSG_TYPES[spec["type"]]
            callback = sub_callbacks.get(key)
            if callback is not None:
                self.create_subscription(msg_type, spec["topic"], callback, spec.get("qos", 1))

        for key, spec in self.cfg["topics"]["pub"].items():
            if key in ("image_rgb", "image_depth"):
                continue
            msg_type = MSG_TYPES[spec["type"]]
            self.pubs[key] = self.create_publisher(msg_type, spec["topic"], spec.get("qos", 10))

        self._running = True
        self._poll_thread = threading.Thread(target=self._poll_loop)
        self._poll_thread.start()
        self.get_logger().info("Walker S2 ROS2-ZMQ bridge started")

        self.cpp_bridge_process = None
        if os.environ.get("DISABLE_CPP_IMAGE_BRIDGE", "0") != "1":
            self.start_cpp_bridge()
        else:
            self.get_logger().info("C++ Image Bridge disabled by environment variable.")

    def start_cpp_bridge(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cpp_dir = os.path.dirname(script_dir)  # 共享 C++ bridge 在上级 bridges/ 目录
        build_script = os.path.join(cpp_dir, "build_cpp_bridge.sh")
        executable = os.path.join(cpp_dir, "zmq_image_bridge")

        # Always rebuild: the C++ file is small (427 lines) and compiles in seconds.
        # Timestamp-based skipping has caused hard-to-diagnose "Unknown camera"
        # warnings when a stale binary ignored new CLI flags like --camera-topics.
        self.get_logger().info(f"Building C++ Bridge: {build_script} ...")
        subprocess.run(["chmod", "+x", build_script], cwd=cpp_dir)
        res = subprocess.run([build_script], cwd=cpp_dir, capture_output=True, text=True)
        if res.returncode != 0:
            self.get_logger().error(
                f"C++ Bridge build failed:\n{res.stderr}\n"
                "Image publishing will be unavailable. Set DISABLE_CPP_IMAGE_BRIDGE=1 to suppress."
            )
            return

        if not os.path.isfile(executable):
            self.get_logger().error("C++ Bridge executable not found. Image publishing unavailable.")
            return

        pub_cfg = self.cfg["topics"]["pub"]
        zmq_cfg = self.cfg["zmq"]
        image_msg_type = pub_cfg["image_rgb"].get("type", "Image2m")
        args = [
            executable,
            "--zmq-port",
            str(zmq_cfg["image_port"]),
            "--rgb-topic",
            pub_cfg["image_rgb"]["topic"],
            "--depth-topic",
            pub_cfg["image_depth"]["topic"],
            "--msg-type",
            image_msg_type,
        ]
        camera_topics_raw = self.cfg.get("camera_topics", {})
        if camera_topics_raw:
            # Normalize: support both old {"name":"topic"} and new {"name":{...}} formats
            normalized = {}
            for name, value in camera_topics_raw.items():
                if isinstance(value, str):
                    normalized[name] = {"topic": value, "msg_type": image_msg_type}
                else:
                    normalized[name] = value
            camera_topics_json = json.dumps(normalized, separators=(',', ':'))
            args.extend(["--camera-topics", camera_topics_json])
        self.get_logger().info(f"Starting C++ Bridge: {' '.join(args)}")
        try:
            self.cpp_bridge_process = subprocess.Popen(args, cwd=cpp_dir)
        except Exception as exc:
            self.get_logger().error(f"Failed to start C++ Bridge: {exc}")
            self.cpp_bridge_process = None

    def command_cb(self, msg: RobotCommand):
        body = {}
        for cmd in msg.joint_cmd:
            control_mode = int(cmd.control_mode)
            if control_mode != int(JointCmd.MODE_POSITION):
                if not self._pvt_warned:
                    self.get_logger().warning(
                        f"Non-MODE_POSITION command (mode={control_mode}) discarded "
                        f"in simulation. Only MODE_POSITION={int(JointCmd.MODE_POSITION)} "
                        f"is supported. PVT / force-control modes are ignored."
                    )
                    self._pvt_warned = True
                continue
            joint_name = _normalize_body_name(cmd.name)
            if joint_name is not None:
                body[joint_name] = float(cmd.position)
        if body:
            now = time.time()
            if now - self._last_body_pub_time >= self._body_pub_interval:
                self.cmd_socket.send_json({"body": body, "source": "walker_sdk_robot_command"})
                self._last_body_pub_time = now

    def hand_cb(self, msg: JointCommand, side: str):
        names = list(msg.names)
        if not names:
            names = WALKER_S2_LEFT_HAND_JOINT_ORDER if side == "left" else WALKER_S2_RIGHT_HAND_JOINT_ORDER

        hand = {}
        for name, position in zip(names, msg.position):
            hand[str(name)] = float(position)

        if side == "left":
            self.latest_left_hand = hand
            self.cmd_socket.send_json({"left_hand": hand, "source": "walker_sdk_left_hand_command"})
        else:
            self.latest_right_hand = hand
            self.cmd_socket.send_json({"right_hand": hand, "source": "walker_sdk_right_hand_command"})

    def grip_cb(self, msg: GripCmd, side: str):
        grip = {
            "pos": float(msg.pos),
            "vel": float(msg.vel),
            "force": float(msg.force),
            "cur": float(msg.cur),
            "mode": int(msg.mode),
            "stop": int(msg.stop),
            "reset": int(msg.reset),
            "homing": int(msg.homing),
        }
        if side == "left":
            self.latest_left_grip = grip
            self.cmd_socket.send_json({"left_grip": grip, "source": "walker_sdk_left_grip_command"})
        else:
            self.latest_right_grip = grip
            self.cmd_socket.send_json({"right_grip": grip, "source": "walker_sdk_right_grip_command"})

    def reset_cb(self, msg: Bool):
        if msg.data:
            self.cmd_socket.send_json({"reset": True})

    def randomize_parts_cb(self, msg: String):
        data = msg.data.strip()
        try:
            payload = {} if not data else json.loads(data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid randomize_parts JSON: {exc}")
            return

        if payload is True or payload is None:
            payload = {}
        if not isinstance(payload, dict):
            self.get_logger().error("randomize_parts payload must be a JSON object or true.")
            return

        self.cmd_socket.send_json(
            {
                "randomize_part_sorting_pieces": payload,
                "source": "walker_s2_ros_bridge_randomize_parts",
            }
        )

    def _poll_loop(self):
        poller = zmq.Poller()
        poller.register(self.status_socket, zmq.POLLIN)
        while self._running:
            socks = dict(poller.poll(timeout=1))
            if self.status_socket in socks:
                try:
                    msg = self.status_socket.recv_json(flags=zmq.NOBLOCK)
                    self.publish_status(msg)
                except zmq.Again:
                    pass  # poll 竞态：消息已被消费，无害
                except Exception as e:
                    self.get_logger().error(f"status poll: publish failed: {e}")

    def publish_status(self, data: dict):
        header = self.get_clock().now().to_msg()
        state = RobotState()
        state.header.stamp = header
        state.joint_states.header.stamp = header
        state.joint_states.name = list(data.get("sdk_body_joint_names", WALKER_S2_SDK_BODY_JOINT_ORDER))
        state.joint_states.position = [float(v) for v in data.get("sdk_body_pos", [])]
        state.joint_states.velocity = [float(v) for v in data.get("sdk_body_vel", [])]
        state.joint_states.effort = [0.0] * len(state.joint_states.name)
        self.pubs["state"].publish(state)

        self.pubs["left_hand_state"].publish(self._make_hand_state("left", data.get("left_hand", self.latest_left_hand)))
        self.pubs["right_hand_state"].publish(self._make_hand_state("right", data.get("right_hand", self.latest_right_hand)))
        self.pubs["left_grip_state"].publish(self._make_grip_state(data.get("left_grip", self.latest_left_grip)))
        self.pubs["right_grip_state"].publish(self._make_grip_state(data.get("right_grip", self.latest_right_grip)))

        if "part_states" in data and "part_states" in self.pubs:
            msg = String()
            msg.data = json.dumps(data["part_states"], ensure_ascii=False)
            self.pubs["part_states"].publish(msg)
        if "part_states_error" in data:
            self.get_logger().warning(f"Failed to read part states: {data['part_states_error']}")
        if "finger_link_states" in data and "finger_link_states" in self.pubs:
            msg = String()
            msg.data = json.dumps(data["finger_link_states"], ensure_ascii=False)
            self.pubs["finger_link_states"].publish(msg)
        if "finger_link_states_error" in data:
            self.get_logger().warning(f"Failed to read finger_link states: {data['finger_link_states_error']}")

    def _make_hand_state(self, side: str, hand_data: dict | None) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = WALKER_S2_LEFT_HAND_JOINT_ORDER if side == "left" else WALKER_S2_RIGHT_HAND_JOINT_ORDER
        hand_data = hand_data or {}
        msg.name = names
        msg.position = [float(hand_data.get(name, 0.0)) for name in names]
        msg.velocity = [0.0] * len(names)
        msg.effort = [0.0] * len(names)
        return msg

    def _make_grip_state(self, grip_data: dict | None) -> GripStatus:
        msg = GripStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.init_state = 1
        msg.grip_state = 1
        msg.error_code = 0
        msg.homed = 1
        if grip_data:
            msg.pos = float(grip_data.get("pos", 0.0))
            msg.vel = float(grip_data.get("vel", 0.0))
            msg.cur = float(grip_data.get("cur", 0.0))
        else:
            msg.pos = 0.0
            msg.vel = 0.0
            msg.cur = 0.0
        return msg

    def stop(self):
        self._running = False
        self._poll_thread.join()
        if self.cpp_bridge_process:
            self.get_logger().info("Stopping C++ Bridge...")
            self.cpp_bridge_process.terminate()
            try:
                self.cpp_bridge_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.cpp_bridge_process.kill()


def main():
    parser = argparse.ArgumentParser(description="Walker S2 ROS2-ZMQ bridge")
    default_config = os.path.join(os.path.dirname(os.path.abspath(__file__)), "walker_s2_bridge_config.yaml")
    parser.add_argument("--config", default=default_config, help="Path to Walker S2 bridge YAML config.")
    args = parser.parse_args()

    rclpy.init()
    node = WalkerS2RosBridge(args.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
