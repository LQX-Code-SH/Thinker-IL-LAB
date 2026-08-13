#!/usr/bin/env python3
"""Walker C1 / Astron ROS2 <-> ZMQ bridge.

Exposes the real-robot SDK topic surface on top of the in-sim
WalkerC1Controller ZMQ interface, so control code written against the SDK
topics runs unchanged against the simulator (ROS_DOMAIN_ID=146) and the real
robot (ROS_DOMAIN_ID=0):

  /mc/sdk/robot_command   (mc_task_msgs/RobotCommand)  -> ZMQ {"body": {...}}
  /mc/left_hand/command   (mc_task_msgs/JointCommand)  -> ZMQ {"left_hand": {...}}
  /mc/right_hand/command  (mc_task_msgs/JointCommand)  -> ZMQ {"right_hand": {...}}
  /sim/cmd_reset          (std_msgs/Bool, sim only)    -> ZMQ {"reset": true}

  ZMQ status -> /mc/sdk/robot_state (mc_state_msgs/RobotState, body joints)
             -> /mc/{left,right}_hand/joint_states (sensor_msgs/JointState,
                SDK 6-joint order)
  ZMQ images -> shared C++ image bridge (same as S2/Tienkung).

Run (inside the container, Py3.10 ROS side):
  source /opt/ros/humble/setup.bash
  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
  ROS_DOMAIN_ID=146 /usr/bin/python3 \
    /ubt_sim/teleoperation/bridges/walker_c1/walker_c1_ros2_zmq_bridge.py
"""
from __future__ import annotations

import argparse
import os
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
import yaml
import zmq

try:
    from shm_msgs.msg import Image2m
except ImportError:
    Image2m = None

try:
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
    from mc_state_msgs.msg import RobotState
except ImportError as exc:
    raise ImportError(
        "Walker SDK ROS2 messages not found. Build and source them first:\n"
        "  cd /ubt_sim/docker && bash run.sh init\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash"
    ) from exc


# Body joints exposed on /mc/sdk/robot_state (sim URDF names; everything
# except the 22 finger joints, which go out via the hand JointState topics).
WALKER_C1_SDK_BODY_JOINT_ORDER = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint", "L_elbow_yaw_joint", "L_wrist_pitch_joint", "L_wrist_roll_joint",
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint", "R_elbow_yaw_joint", "R_wrist_pitch_joint", "R_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "L_hip_pitch_joint", "L_hip_roll_joint", "L_hip_yaw_joint",
    "L_knee_pitch_joint", "L_ankle_pitch_joint", "L_ankle_roll_joint",
    "R_hip_pitch_joint", "R_hip_roll_joint", "R_hip_yaw_joint",
    "R_knee_pitch_joint", "R_ankle_pitch_joint", "R_ankle_roll_joint",
]

# SDK 6-joint hand command/state order (from the Astron SDK doc, verified in
# C1_joint_map.md).
WALKER_C1_LEFT_HAND_SDK_ORDER = [
    "left_thumb_swing", "left_thumb_mcp", "left_index_mcp",
    "left_middle_mcp", "left_ring_mcp", "left_little_mcp",
]
WALKER_C1_RIGHT_HAND_SDK_ORDER = [
    "right_thumb_swing", "right_thumb_mcp", "right_index_mcp",
    "right_middle_mcp", "right_ring_mcp", "right_little_mcp",
]

# Accept incoming body joint names with or without the "_joint" suffix, and
# tolerate the elbow naming ambiguity: the SDK kinematics doc calls the 4th
# arm joint "elbow_roll" while the URDF/sim calls it "elbow_pitch"
# (C1_joint_map.md, unresolved until verified on the real robot). Both names
# map to the sim joint either way, so control code works regardless.
_ALIAS_TO_SIM_JOINT: dict[str, str] = {}
for _name in WALKER_C1_SDK_BODY_JOINT_ORDER:
    _ALIAS_TO_SIM_JOINT[_name] = _name
    _ALIAS_TO_SIM_JOINT[_name.removesuffix("_joint")] = _name
for _side in ("L", "R"):
    _ALIAS_TO_SIM_JOINT[f"{_side}_elbow_roll_joint"] = f"{_side}_elbow_pitch_joint"
    _ALIAS_TO_SIM_JOINT[f"{_side}_elbow_roll"] = f"{_side}_elbow_pitch_joint"

MSG_TYPES = {
    "RobotCommand": RobotCommand,
    "JointCommand": JointCommand,
    "Bool": Bool,
    "String": String,
    "Point": Point,
    "RobotState": RobotState,
    "JointState": JointState,
    "Image": Image,
    "Image2m": Image2m,
}


class WalkerC1RosBridge(Node):
    def __init__(self, config_path: str):
        super().__init__("walker_c1_ros_bridge")
        with open(config_path) as f:
            self.cfg = yaml.safe_load(f)
        zmq_cfg = self.cfg["zmq"]

        self.zmq_context = zmq.Context()
        self.cmd_socket = self.zmq_context.socket(zmq.PUB)
        # NOT SNDHWM=1 (the S2 value): body and hand commands arrive as
        # separate back-to-back messages, and a 1-deep send queue drops one of
        # them systematically. The sim controller drains and MERGES all
        # pending messages every step, so a deeper queue is correct here.
        self.cmd_socket.setsockopt(zmq.SNDHWM, 16)
        self.cmd_socket.bind(f"tcp://*:{zmq_cfg['cmd_port']}")

        self.status_socket = self.zmq_context.socket(zmq.SUB)
        self.status_socket.connect(f"tcp://127.0.0.1:{zmq_cfg['status_port']}")
        self.status_socket.setsockopt(zmq.RCVHWM, 1)
        self.status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.pubs = {}
        sub_callbacks = {
            "command": self.command_cb,
            "left_hand_command": lambda msg: self.hand_cb(msg, "left"),
            "right_hand_command": lambda msg: self.hand_cb(msg, "right"),
            "reset": self.reset_cb,
            "set_object_pose": self.set_object_pose_cb,
        }
        for key, spec in self.cfg["topics"]["sub"].items():
            callback = sub_callbacks.get(key)
            if callback is not None:
                self.create_subscription(MSG_TYPES[spec["type"]], spec["topic"], callback, spec.get("qos", 1))

        for key, spec in self.cfg["topics"]["pub"].items():
            if key in ("image_rgb", "image_depth"):
                continue
            self.pubs[key] = self.create_publisher(MSG_TYPES[spec["type"]], spec["topic"], spec.get("qos", 10))

        self._running = True
        self._last_episode_complete_id = None
        self._poll_thread = threading.Thread(target=self._poll_loop)
        self._poll_thread.start()
        self.get_logger().info("Walker C1 ROS2-ZMQ bridge started")

        self.cpp_bridge_process = None
        if os.environ.get("DISABLE_CPP_IMAGE_BRIDGE", "0") != "1":
            self.start_cpp_bridge()
        else:
            self.get_logger().info("C++ Image Bridge disabled by environment variable.")

    # ── ROS -> ZMQ ──
    def command_cb(self, msg: RobotCommand):
        body = {}
        for cmd in msg.joint_cmd:
            if int(cmd.control_mode) != int(JointCmd.MODE_POSITION):
                continue
            joint_name = _ALIAS_TO_SIM_JOINT.get(str(cmd.name))
            if joint_name is not None:
                body[joint_name] = float(cmd.position)
        if body:
            self.cmd_socket.send_json({"body": body, "source": "walker_sdk_robot_command"})

    def hand_cb(self, msg: JointCommand, side: str):
        names = list(msg.names)
        if not names:
            names = WALKER_C1_LEFT_HAND_SDK_ORDER if side == "left" else WALKER_C1_RIGHT_HAND_SDK_ORDER
        hand = {str(name): float(pos) for name, pos in zip(names, msg.position)}
        self.cmd_socket.send_json({f"{side}_hand": hand, "source": f"walker_sdk_{side}_hand_command"})

    def reset_cb(self, msg: Bool):
        if msg.data:
            self.cmd_socket.send_json({"reset": True})

    def set_object_pose_cb(self, msg: Point):
        # Sim-only: teleport the graspable object to a world position.
        self.cmd_socket.send_json(
            {"set_object_pose": [float(msg.x), float(msg.y), float(msg.z)],
             "source": "walker_c1_set_object_pose"}
        )

    # ── ZMQ -> ROS ──
    def _poll_loop(self):
        poller = zmq.Poller()
        poller.register(self.status_socket, zmq.POLLIN)
        while self._running:
            socks = dict(poller.poll(timeout=1))
            if self.status_socket in socks:
                try:
                    msg = self.status_socket.recv_json(flags=zmq.NOBLOCK)
                    self.publish_status(msg)
                except Exception:
                    pass

    def publish_status(self, data: dict):
        stamp = self.get_clock().now().to_msg()
        episode_complete_id = int(data.get("episode_complete_id", 0))
        if self._last_episode_complete_id is None:
            # The first status packet establishes a baseline.  This prevents
            # a bridge restart from replaying an old Space event.
            self._last_episode_complete_id = episode_complete_id
        elif episode_complete_id > self._last_episode_complete_id:
            self._last_episode_complete_id = episode_complete_id
            if "episode_complete" in self.pubs:
                episode_msg = Bool()
                episode_msg.data = True
                self.pubs["episode_complete"].publish(episode_msg)
                self.get_logger().info(
                    f"episode completion requested from simulator: {episode_complete_id}"
                )
        pos_map = dict(zip(data.get("joint_names", []), data.get("joint_pos", [])))
        vel_map = dict(zip(data.get("joint_names", []), data.get("joint_vel", [])))

        state = RobotState()
        state.header.stamp = stamp
        state.joint_states.header.stamp = stamp
        state.joint_states.name = list(WALKER_C1_SDK_BODY_JOINT_ORDER)
        state.joint_states.position = [float(pos_map.get(n, 0.0)) for n in WALKER_C1_SDK_BODY_JOINT_ORDER]
        state.joint_states.velocity = [float(vel_map.get(n, 0.0)) for n in WALKER_C1_SDK_BODY_JOINT_ORDER]
        state.joint_states.effort = [0.0] * len(WALKER_C1_SDK_BODY_JOINT_ORDER)
        self.pubs["state"].publish(state)

        self.pubs["left_hand_state"].publish(
            self._make_hand_state(
                data.get("left_hand_sdk_joint_names", WALKER_C1_LEFT_HAND_SDK_ORDER),
                data.get("left_hand_sdk_pos", []),
            )
        )
        self.pubs["right_hand_state"].publish(
            self._make_hand_state(
                data.get("right_hand_sdk_joint_names", WALKER_C1_RIGHT_HAND_SDK_ORDER),
                data.get("right_hand_sdk_pos", []),
            )
        )

        if "object_state" in self.pubs and "object_pos_w" in data:
            import json as _json

            msg = String()
            msg.data = _json.dumps(
                {
                    "object_pos_w": data.get("object_pos_w"),
                    "robot_root_pose_w": data.get("robot_root_pose_w"),
                    "right_hand_links_w": data.get("right_hand_links_w"),
                    "sim_step": data.get("sim_step"),
                    "joint_vel_probe": data.get("joint_vel_probe"),
                    "joint_names": data.get("joint_names"),
                }
            )
            self.pubs["object_state"].publish(msg)

    def _make_hand_state(self, names: list, positions: list) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [str(n) for n in names]
        msg.position = [float(p) for p in positions] or [0.0] * len(msg.name)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        return msg

    # ── shared C++ image bridge (same binary as S2/Tienkung) ──
    def start_cpp_bridge(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cpp_dir = os.path.dirname(script_dir)
        build_script = os.path.join(cpp_dir, "build_cpp_bridge.sh")
        executable = os.path.join(cpp_dir, "zmq_image_bridge")
        cpp_source = os.path.join(cpp_dir, "zmq_image_bridge.cpp")
        cmake_source = os.path.join(cpp_dir, "CMakeLists.txt")

        need_build = True
        if os.path.isfile(executable) and os.access(executable, os.X_OK):
            exe_mtime = os.path.getmtime(executable)
            src_mtime = max(os.path.getmtime(cpp_source), os.path.getmtime(cmake_source))
            if exe_mtime > src_mtime:
                need_build = False

        if need_build:
            self.get_logger().info(f"Building C++ Bridge: {build_script} ...")
            subprocess.run(["chmod", "+x", build_script], cwd=cpp_dir)
            res = subprocess.run([build_script], cwd=cpp_dir, capture_output=True, text=True)
            if res.returncode != 0:
                self.get_logger().error(
                    f"C++ Bridge build failed:\n{res.stderr}\nImage topics unavailable."
                )
                return
        if not os.path.isfile(executable):
            self.get_logger().error("C++ Bridge executable not found. Image topics unavailable.")
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
        self.get_logger().info(f"Starting C++ Bridge: {' '.join(args)}")
        try:
            self.cpp_bridge_process = subprocess.Popen(args, cwd=cpp_dir)
        except Exception as exc:
            self.get_logger().error(f"Failed to start C++ Bridge: {exc}")
            self.cpp_bridge_process = None

    def stop(self):
        self._running = False
        self._poll_thread.join()
        if self.cpp_bridge_process:
            self.cpp_bridge_process.terminate()
            try:
                self.cpp_bridge_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.cpp_bridge_process.kill()


def main():
    parser = argparse.ArgumentParser(description="Walker C1 ROS2-ZMQ bridge")
    default_config = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "walker_c1_bridge_config.yaml"
    )
    parser.add_argument("--config", default=default_config)
    args = parser.parse_args()

    rclpy.init()
    node = WalkerC1RosBridge(args.config)
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
