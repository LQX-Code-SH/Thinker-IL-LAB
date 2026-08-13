from __future__ import annotations

from typing import Any
import json
import struct
import time

import cv2

from ..device_base import DeviceBase
from .action_process import reset_hold_targets, to_controller_data, to_ros_data

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


class WalkerC1Controller(DeviceBase):
    """Minimal ZMQ controller for Walker C1 / Astron simulation."""

    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device
        self.reset_requested = False
        self._action: dict[str, Any] | list[float] = {}
        self._jpeg_frame_count = 0
        self._episode_complete_id = 0
        self.jpeg_unit_test = kwargs.get("jpeg_unit_test", True)

        self.cmd_port = int(kwargs.get("cmd_port", 5655))
        self.status_port = int(kwargs.get("status_port", 5656))
        self.image_port = int(kwargs.get("image_port", 5657))
        self.jpeg_image_port = int(kwargs.get("jpeg_image_port", 5658))

        if HAS_ZMQ:
            self.context = zmq.Context()

            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.setsockopt(zmq.RCVHWM, 1)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            self.img_socket = self.context.socket(zmq.PUB)
            self.img_socket.setsockopt(zmq.SNDHWM, 1)
            self.img_socket.bind(f"tcp://*:{self.image_port}")

            self.jpeg_socket = self.context.socket(zmq.PUB)
            self.jpeg_socket.setsockopt(zmq.SNDHWM, 1)
            self.jpeg_socket.bind(f"tcp://*:{self.jpeg_image_port}")

            print(f"[INFO] Walker C1 ZMQ command sub connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] Walker C1 ZMQ status pub bound to tcp://*:{self.status_port}")
            print(f"[INFO] Walker C1 ZMQ image pub bound to tcp://*:{self.image_port}")
            print(f"[INFO] Walker C1 ZMQ JPEG pub bound to tcp://*:{self.jpeg_image_port}")
        else:
            print("[WARNING] zmq not found. Walker C1 ZMQ control will not be available.")

    def __str__(self) -> str:
        return "Walker C1 ZMQ Controller"

    def reset(self):
        self._action = {}
        self.reset_requested = False
        reset_hold_targets()

    def request_episode_complete(self) -> None:
        """Send a monotonic event to the ROS recorder through status ZMQ."""
        self._episode_complete_id += 1

    def _apply_pending_object_pose(self) -> None:
        pos = getattr(self, "_pending_object_pos", None)
        if pos is None or "object" not in self.env.scene.keys():
            self._pending_object_pos = None
            return
        try:
            import torch

            obj = self.env.scene["object"]
            pose = obj.data.root_state_w[:, :7].clone()
            pose[0, 0], pose[0, 1], pose[0, 2] = pos[0], pos[1], pos[2]
            pose[0, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=pose.device)
            obj.write_root_pose_to_sim(pose)
            obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=pose.device))
            print(f"[INFO] Walker C1 object teleported to {pos}")
        except Exception as exc:
            print(f"[WARN] set_object_pose failed: {exc}")
        finally:
            self._pending_object_pos = None

    def add_callback(self, key, func):
        pass

    def _merge_command(self, msg: dict[str, Any]) -> None:
        if msg.get("reset"):
            self.reset_requested = True
            return

        if "set_object_pose" in msg:
            # Sim-only helper (Tienkung-style): the task script COMMANDS the
            # graspable object's world position instead of sensing it.
            pose = msg["set_object_pose"]
            if isinstance(pose, (list, tuple)) and len(pose) >= 3:
                self._pending_object_pos = [float(v) for v in pose[:3]]
            return

        if "walker_c1" in msg:
            payload = msg["walker_c1"]
            if isinstance(payload, (dict, list)):
                self._action = payload
            return

        if isinstance(self._action, list):
            self._action = {}

        if "body" in msg:
            body = msg.get("body") or {}
            if isinstance(body, dict) and isinstance(self._action, dict):
                self._action.setdefault("body", {}).update(body)
            else:
                self._action["body"] = body

        for key in [
            "left_arm",
            "right_arm",
            "left_hand",
            "right_hand",
            "head",
            "waist",
            "left_leg",
            "right_leg",
        ]:
            if key in msg and isinstance(self._action, dict):
                self._action[key] = msg[key]

    def _send_status(self) -> None:
        status = to_ros_data(self.env, self._action if isinstance(self._action, dict) else {"walker_c1": self._action})
        status["episode_complete_id"] = self._episode_complete_id
        self._sim_step = getattr(self, "_sim_step", 0) + 1
        status["sim_step"] = self._sim_step
        self.pub_socket.send_json(status, flags=zmq.NOBLOCK)

    def _send_camera_data(self) -> None:
        if "camera" not in self.env.scene.keys():
            return

        camera = self.env.scene["camera"]
        try:
            rgb_tensor = camera.data.output.get("rgb") if camera.data.output is not None else None
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return

            rgb = rgb_tensor[0].cpu().numpy()
            metadata = {
                "width": int(rgb.shape[1]),
                "height": int(rgb.shape[0]),
                "format": "raw",
            }
            self.img_socket.send_json(metadata, flags=zmq.SNDMORE | zmq.NOBLOCK)
            self.img_socket.send(rgb.tobytes(), flags=zmq.SNDMORE | zmq.NOBLOCK)
            self.img_socket.send(b"", flags=zmq.NOBLOCK)
        except Exception:
            return

    def _send_jpeg_camera_data(self) -> None:
        if "camera" not in self.env.scene.keys():
            return

        camera = self.env.scene["camera"]
        try:
            rgb_tensor = camera.data.output.get("rgb") if camera.data.output is not None else None
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return

            rgb = rgb_tensor[0].cpu().numpy()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ret, buf = cv2.imencode(".jpg", bgr)
            if not ret:
                return

            msg = buf.tobytes()
            if self.jpeg_unit_test:
                msg = struct.pack("dI", time.time(), self._jpeg_frame_count) + msg
                self._jpeg_frame_count += 1
            self.jpeg_socket.send(msg, flags=zmq.NOBLOCK)
        except Exception:
            return

    def advance(self) -> dict[str, Any]:
        if HAS_ZMQ:
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                    self._merge_command(msg)
                except zmq.Again:
                    break
                except json.JSONDecodeError:
                    break
            self._apply_pending_object_pose()

            try:
                self._send_status()
            except Exception:
                pass
            try:
                self._send_camera_data()
            except Exception:
                pass
            try:
                self._send_jpeg_camera_data()
            except Exception:
                pass

        return {"walker_c1": to_controller_data(self._action, self.env)}

    def display_controls(self):
        if HAS_ZMQ:
            print("Walker C1 Controller: simulation ZMQ interface enabled")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://127.0.0.1:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
            print(f"  - JPEG Pub:    tcp://*:{self.jpeg_image_port} (image_client compatible)")
