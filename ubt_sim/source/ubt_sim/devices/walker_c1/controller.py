"""Walker C1 / Astron 仿真侧 ZMQ 设备。

四端口：5655 指令（SUB connect）/ 5656 状态（PUB）/ 5657 原始 RGB / 5658 JPEG。
图像发送复用 tienkung_pro 的异步发送器（GPU 异步 D2H + 后台线程编码/发送），
主循环（advance）只做非阻塞入队，不阻塞物理步进。
"""

from __future__ import annotations

import json
import time
from typing import Any

from ..device_base import DeviceBase
from ..tienkung_pro.controller import AsyncJpegCameraSender, AsyncRawCameraSender
from .action_process import reset_hold_targets, to_controller_data, to_ros_data

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


class WalkerC1Controller(DeviceBase):
    """Walker C1 仿真控制器：ZMQ 指令/状态 + 头部相机推流。"""

    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device
        self.reset_requested = False
        self._action: dict[str, Any] | list[float] = {}
        self._jpeg_frame_count = 0
        self._episode_complete_id = 0
        self._sim_step = 0
        self._pending_object_pos: list[float] | None = None
        self._step_count = 0
        self.jpeg_unit_test = kwargs.get("jpeg_unit_test", True)

        self._render_interval = int(kwargs.get("render_interval", 1))
        # 相机 key 由 task 的 camera_names 传入（YAML 中定义顺序）
        self.camera_names: list[str] = list(kwargs.get("camera_names") or [])
        self._rgb_camera = self._pick_camera()

        self.cmd_port = int(kwargs.get("cmd_port", 5655))
        self.status_port = int(kwargs.get("status_port", 5656))
        self.image_port = int(kwargs.get("image_port", 5657))
        self.jpeg_image_port = int(kwargs.get("jpeg_image_port", 5658))

        self._raw_sender = None
        self._jpeg_sender = None

        if HAS_ZMQ:
            self.context = zmq.Context()

            self.sub_socket = self.context.socket(zmq.SUB)
            # RCVHWM 与桥接侧 SNDHWM=16 配套：body 与 hand 是两条背靠背消息，
            # 1 深度队列会系统性丢掉其中一条；advance() 每个物理步全量 drain
            # 后合并，因此更深的队列不会积压过期指令。
            self.sub_socket.setsockopt(zmq.RCVHWM, 16)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            self._raw_sender = AsyncRawCameraSender(self.image_port)
            self._jpeg_sender = AsyncJpegCameraSender(
                self.jpeg_image_port, unit_test=self.jpeg_unit_test
            )

            print(f"[INFO] Walker C1 ZMQ command sub connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] Walker C1 ZMQ status pub bound to tcp://*:{self.status_port}")
            print(f"[INFO] Walker C1 ZMQ image pub bound to tcp://*:{self.image_port} (async thread)")
            print(f"[INFO] Walker C1 ZMQ JPEG pub bound to tcp://*:{self.jpeg_image_port} (async thread)")
        else:
            print("[WARNING] zmq not found. Walker C1 ZMQ control will not be available.")

    def __str__(self) -> str:
        return "Walker C1 ZMQ Controller"

    # ------------------------------------------------------------------ utils
    def _pick_camera(self) -> str | None:
        """从 camera_names 中挑选第一路在场景里存在的相机。"""
        scene_keys = self.env.scene.keys() if hasattr(self.env.scene, "keys") else []
        for name in self.camera_names:
            if name in scene_keys:
                return name
        return self.camera_names[0] if self.camera_names else None

    def _should_send_camera(self) -> bool:
        if self._render_interval <= 1:
            return True
        if self._step_count == 0:
            return True
        return self._step_count % self._render_interval == 1

    def _send_camera_data(self) -> None:
        if not self._should_send_camera() or self._raw_sender is None or self._rgb_camera is None:
            return
        try:
            camera = self.env.scene[self._rgb_camera]
            output = camera.data.output
            rgb_tensor = output.get("rgb") if output is not None else None
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return
            rgb_gpu = rgb_tensor[0]
            depth_gpu = None
            depth_tensor = output.get("depth") if output is not None else None
            if depth_tensor is not None and depth_tensor.shape[0] != 0:
                depth_gpu = depth_tensor[0]

            # 不带 "camera" 键：C++ 图像桥见到它就走"按相机名路由"分支，
            # 该分支只发 RGB；无此键时走单相机分支，按 --rgb-topic /
            # --depth-topic 同时发 RGB 与深度。
            metadata = {
                "width": int(rgb_gpu.shape[1]),
                "height": int(rgb_gpu.shape[0]),
                "format": "raw",
                "camera_name": self._rgb_camera,
            }
            self._raw_sender.send(metadata, rgb_gpu, depth_gpu)
        except Exception:
            return

    def _send_jpeg_camera_data(self) -> None:
        if not self._should_send_camera() or self._jpeg_sender is None or self._rgb_camera is None:
            return
        try:
            output = self.env.scene[self._rgb_camera].data.output
            rgb_tensor = output.get("rgb") if output is not None else None
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return
            self._jpeg_sender.send(rgb_tensor[0], time.time(), self._jpeg_frame_count)
            self._jpeg_frame_count += 1
        except Exception:
            return

    # ------------------------------------------------------------ DeviceBase
    def reset(self):
        self._action = {}
        self.reset_requested = False
        self._step_count = 0
        reset_hold_targets()

    def request_episode_complete(self) -> None:
        """通过状态 ZMQ 向 ROS 侧录制器发出一个单调递增的完成事件。"""
        self._episode_complete_id += 1

    def add_callback(self, key, func):
        pass

    def _apply_pending_object_pose(self) -> None:
        pos = self._pending_object_pos
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

    def _merge_command(self, msg: dict[str, Any]) -> None:
        if msg.get("reset"):
            self.reset_requested = True
            return

        if "set_object_pose" in msg:
            # 仅仿真：任务脚本直接指定可抓取物体的世界坐标（真机无此能力）
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
        status = to_ros_data(
            self.env,
            self._action if isinstance(self._action, dict) else {"walker_c1": self._action},
        )
        status["episode_complete_id"] = self._episode_complete_id
        self._sim_step += 1
        status["sim_step"] = self._sim_step
        status["timestamp"] = time.time()
        self.pub_socket.send_json(status, flags=zmq.NOBLOCK)

    def advance(self) -> dict[str, Any]:
        if HAS_ZMQ:
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                except json.JSONDecodeError:
                    break
                if isinstance(msg, dict):
                    self._merge_command(msg)
            self._apply_pending_object_pose()

            try:
                self._send_status()
            except Exception:
                pass
            self._send_camera_data()
            self._send_jpeg_camera_data()

        self._step_count += 1
        return {"walker_c1": to_controller_data(self._action, self.env)}

    def close(self):
        for sender in (self._raw_sender, self._jpeg_sender):
            if sender is not None:
                try:
                    sender.close()
                except Exception:
                    pass
        if not HAS_ZMQ:
            return
        for sock in (getattr(self, "sub_socket", None), getattr(self, "pub_socket", None)):
            try:
                if sock is not None:
                    sock.close(linger=0)
            except Exception:
                pass
        try:
            self.context.term()
        except Exception:
            pass

    def display_controls(self):
        if HAS_ZMQ:
            print("Walker C1 Controller: simulation ZMQ interface enabled")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://127.0.0.1:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
            print(f"  - JPEG Pub:    tcp://*:{self.jpeg_image_port} (image_client compatible)")
            print(f"  - Cameras:     rgb={self._rgb_camera}")
