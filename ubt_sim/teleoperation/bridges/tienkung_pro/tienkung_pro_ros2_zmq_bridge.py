#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
try:
    from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus, CmdSetMotorPosition, CmdMotorCtrl
except ImportError:
    raise ImportError(
        "bodyctrl_msgs not found. "
        "source /opt/ros/humble/setup.bash && "
        "colcon build --packages-select bodyctrl_msgs"
    )

import zmq
import yaml
import os
import sys
import subprocess
import threading
import time

# 从 control/tienkung_pro/constants.py 导入共享常量（零依赖，纯数据）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "control", "tienkung_pro"))
from constants import ID_TO_NAME as _ID_TO_NAME, NAME_TO_ID as _NAME_TO_ID
from constants import HAND_L_MAP as _HAND_L_MAP, HAND_R_MAP as _HAND_R_MAP
from constants import ID_HEAD, ID_ARM_L, ID_ARM_R, ID_WAIST, ID_LEG_L, ID_LEG_R


def load_bridge_config():
    """Load bridge configuration from bridge_config.yaml next to this script."""
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_config.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


# Map config type strings to ROS2 message classes
MSG_TYPES = {
    "CmdSetMotorPosition": CmdSetMotorPosition,
    "CmdMotorCtrl": CmdMotorCtrl,
    "JointState": JointState,
    "Point": Point,
    "Bool": Bool,
    "MotorStatusMsg": MotorStatusMsg,
    "Float32": Float32,
    "Image": Image,
}


class TienkungProRosBridge(Node):
    def __init__(self):
        super().__init__('tienkung_pro_ros_bridge')

        # Load config
        self.cfg = load_bridge_config()
        zmq_cfg = self.cfg["zmq"]

        # ZMQ setup
        self.zmq_context = zmq.Context()

        # Command PUB
        self.cmd_socket = self.zmq_context.socket(zmq.PUB)
        self.cmd_socket.bind(f"tcp://*:{zmq_cfg['cmd_port']}")

        # Status SUB
        self.status_socket = self.zmq_context.socket(zmq.SUB)
        self.status_socket.connect(f"tcp://127.0.0.1:{zmq_cfg['status_port']}")
        self.status_socket.setsockopt(zmq.RCVHWM, 1)
        self.status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # ID Definition — 从 constants.py 共享
        self.ID_HEAD = ID_HEAD
        self.ID_ARM_L = ID_ARM_L
        self.ID_ARM_R = ID_ARM_R
        self.ID_WAIST = ID_WAIST
        self.ID_LEG_L = ID_LEG_L
        self.ID_LEG_R = ID_LEG_R

        # Reverse Mapping for Commands (ID to Name) — 从 constants.py 共享
        self.ID_TO_NAME = _ID_TO_NAME
        self.NAME_TO_ID = _NAME_TO_ID

        self.HAND_L_MAP = _HAND_L_MAP
        self.HAND_R_MAP = _HAND_R_MAP

        # --- Dynamic subscriptions from config ---
        SUB_CALLBACKS = {
            "arm_cmd_pos":   self.cmd_pos_cb,
            "arm_cmd_ctrl":  self.cmd_ctrl_cb,
            "head_cmd_pos":  self.cmd_pos_cb,
            "head_cmd_ctrl": self.cmd_ctrl_cb,
            "leg_cmd_pos":   self.cmd_pos_cb,
            "leg_cmd_ctrl":  self.cmd_ctrl_cb,
            "waist_cmd_pos": self.cmd_pos_cb,
            "waist_cmd_ctrl":self.cmd_ctrl_cb,
            "hand_l_ctrl":   lambda m: self.hand_cb(m, "left"),
            "hand_r_ctrl":   lambda m: self.hand_cb(m, "right"),
            "apple_offset":  self.apple_cb,
            "cmd_reset":     self.cmd_reset_cb,
        }
        for key, spec in self.cfg["topics"]["sub"].items():
            msg_type = MSG_TYPES[spec["type"]]
            cb = SUB_CALLBACKS.get(key)
            if cb:
                self.create_subscription(msg_type, spec["topic"], cb, spec.get("qos", 1))

        # --- Dynamic publishers from config ---
        self.pubs = {}
        for key, spec in self.cfg["topics"]["pub"].items():
            # image_rgb and image_depth are handled by C++ bridge, skip here
            if key in ("image_rgb", "image_depth"):
                continue
            msg_type = MSG_TYPES[spec["type"]]
            self.pubs[key] = self.create_publisher(msg_type, spec["topic"], spec.get("qos", 10))

        # Threaded polling
        self._running = True
        self._poll_thread = threading.Thread(target=self._poll_loop)
        self._poll_thread.start()
        self.get_logger().info("Tienkung Pro ROS 2 Bridge (Control Only) Started")

        # Start C++ Image Bridge (skip with env var DISABLE_CPP_IMAGE_BRIDGE=1)
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
        res = subprocess.run([build_script], cwd=cpp_dir,
                             capture_output=True, text=True)
        if res.returncode != 0:
            self.get_logger().error(
                f"C++ Bridge build failed:\n{res.stderr}\n"
                "Image publishing will be unavailable. "
                "Set DISABLE_CPP_IMAGE_BRIDGE=1 to suppress."
            )
            return

        if not os.path.isfile(executable):
            self.get_logger().error("C++ Bridge executable not found. Image publishing unavailable.")
            return

        # Pass config values to C++ bridge via CLI args
        pub_cfg = self.cfg["topics"]["pub"]
        zmq_cfg = self.cfg["zmq"]
        image_msg_type = pub_cfg["image_rgb"].get("type", "Image2m")
        args = [
            executable,
            "--zmq-port",    str(zmq_cfg["image_port"]),
            "--rgb-topic",   pub_cfg["image_rgb"]["topic"],
            "--depth-topic", pub_cfg["image_depth"]["topic"],
            "--msg-type",    image_msg_type,
        ]
        self.get_logger().info(f"Starting C++ Bridge: {' '.join(args)}")
        try:
            self.cpp_bridge_process = subprocess.Popen(args, cwd=cpp_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to start C++ Bridge: {e}")
            self.cpp_bridge_process = None

    def cmd_pos_cb(self, msg):
        current_action = {}
        for cmd in msg.cmds:
            joint_name = self.ID_TO_NAME.get(cmd.name)
            if joint_name:
                current_action[joint_name] = float(cmd.pos)
        self.cmd_socket.send_json(current_action)

    def cmd_ctrl_cb(self, msg):
        current_action = {}
        for cmd in msg.cmds:
            joint_name = self.ID_TO_NAME.get(cmd.name)
            if joint_name:
                current_action[joint_name] = float(cmd.pos)
        self.cmd_socket.send_json(current_action)

    def hand_cb(self, msg, side):
        current_action = {}
        map_target = self.HAND_L_MAP if side == "left" else self.HAND_R_MAP
        for i, name in enumerate(msg.name):
            try:
                id_val = int(name)
                joint_name = map_target.get(id_val)
                if joint_name:
                    current_action[joint_name] = float(msg.position[i])
            except: pass
        self.cmd_socket.send_json(current_action)

    def apple_cb(self, msg):
        self.cmd_socket.send_json({"apple_offset": [msg.x, msg.y]})

    def cmd_reset_cb(self, msg):
        if msg.data:
             self.cmd_socket.send_json({"reset": True})

    def _poll_loop(self):
        poller = zmq.Poller()
        poller.register(self.status_socket, zmq.POLLIN)

        while self._running:
            socks = dict(poller.poll(timeout=1))
            if self.status_socket in socks:
                try:
                    msg = self.status_socket.recv_json(flags=zmq.NOBLOCK)
                    self.publish_status(msg)
                    if "task_dist" in msg:
                        f = Float32()
                        f.data = float(msg["task_dist"])
                        self.pubs["task_dist"].publish(f)
                except Exception as e:
                    pass

    def publish_status(self, data):
        # Support both old list format and new dict format
        if isinstance(data, dict) and "joint_names" in data:
            pos_map = dict(zip(data["joint_names"], data["joint_pos"]))
            finger_percentages = data.get("finger_percentages", {})
            vel_map = dict(zip(data["joint_names"], data["joint_vel"])) if "joint_vel" in data else {}
        else:
            pos_map = data
            finger_percentages = {}
            vel_map = {}

        header = self.get_clock().now().to_msg()

        def create_motor_msg(id_range):
            m_msg = MotorStatusMsg()
            m_msg.header.stamp = header
            for name, id_val in self.NAME_TO_ID.items():
                if id_val in id_range and name in pos_map:
                    s = MotorStatus()
                    s.name = id_val
                    s.pos = float(pos_map[name])
                    m_msg.status.append(s)
            return m_msg

        def create_hand_msg(hand_map):
            h_msg = JointState()
            h_msg.header.stamp = header
            h_msg.name = []
            h_msg.position = []
            h_msg.velocity = []

            for i in range(1, 7):
                sim_name = hand_map.get(i)
                h_msg.name.append(str(i))

                pos_val = 0.0
                vel_val = 0.0

                if sim_name and sim_name in finger_percentages:
                    pos_val = float(finger_percentages[sim_name])
                    if sim_name in vel_map:
                        vel_val = float(vel_map[sim_name])

                h_msg.position.append(pos_val)
                h_msg.velocity.append(vel_val)
                h_msg.effort.append(0.0)

            return h_msg

        self.pubs["arm_status"].publish(create_motor_msg(self.ID_ARM_L + self.ID_ARM_R))
        self.pubs["head_status"].publish(create_motor_msg(self.ID_HEAD))
        self.pubs["leg_status"].publish(create_motor_msg(self.ID_LEG_L + self.ID_LEG_R))
        self.pubs["waist_status"].publish(create_motor_msg(self.ID_WAIST))

        self.pubs["hand_l_state"].publish(create_hand_msg(self.HAND_L_MAP))
        self.pubs["hand_r_state"].publish(create_hand_msg(self.HAND_R_MAP))

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

if __name__ == '__main__':
    rclpy.init()
    node = TienkungProRosBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
