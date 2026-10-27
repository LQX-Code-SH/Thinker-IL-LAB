"""TienKung Dex 仿真控制器：ZMQ 指令接收 + 状态/图像回传。

与 ``TienkungProController`` 保持同一套协议，便于复用 ROS2-ZMQ 桥接与采集脚本：

* 指令（bridge -> sim，SUB）：扁平 JSON ``{关节名: 目标}``，可选 ``{"reset": true}``；
* 状态（sim -> bridge，PUB）：``joint_names / joint_pos / joint_vel / hand_percentages``；
* 图像（sim -> bridge，PUB）：raw 端口发送 ``metadata + rgb + depth`` 多段消息，
  JPEG 端口发送 ``[ts, seq] + jpg``（兼容 ``image_client.py``）。

异步相机发送器直接从 ``devices/tienkung_pro/controller.py`` 复用，避免复制 GPU 拷贝逻辑。
"""

from __future__ import annotations

import time
from typing import Any

from ..device_base import DeviceBase
from ..tienkung_pro.controller import AsyncJpegCameraSender, AsyncRawCameraSender
from .action_process import to_controller_data, to_ros_data

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


DEFAULT_CMD_PORT = 5755
DEFAULT_STATUS_PORT = 5756
DEFAULT_IMAGE_PORT = 5757
DEFAULT_JPEG_PORT = 5758


class TienkungDexController(DeviceBase):
    """TienKung Dex 的 ZMQ 遥操作控制器。"""

    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device

        self._action: dict[str, Any] = {}
        self.reset_requested = False
        # inspire | brainco：只影响手部关节集合与 mimic 展开
        self.variant = str(kwargs.get("variant", "inspire"))

        self._render_interval = int(kwargs.get("render_interval", 1))
        self._step_count = 0

        # 相机 key：由 task 的 camera_names 传入（yaml 中定义顺序）
        self.camera_names: list[str] = list(kwargs.get("camera_names") or [])
        self._rgb_camera = self._pick_camera(("rgb", "rgba"), exclude="depth")
        self._depth_camera = self._pick_camera(("depth",), exclude=None)

        self._raw_sender = None
        self._jpeg_sender = None
        self._jpeg_frame_count = 0

        if HAS_ZMQ:
            self.cmd_port = int(kwargs.get("cmd_port", DEFAULT_CMD_PORT))
            self.status_port = int(kwargs.get("status_port", DEFAULT_STATUS_PORT))
            self.image_port = int(kwargs.get("image_port", DEFAULT_IMAGE_PORT))
            self.jpeg_port = int(kwargs.get("jpeg_port", DEFAULT_JPEG_PORT))

            self.context = zmq.Context()
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            self._raw_sender = AsyncRawCameraSender(self.image_port)
            self._jpeg_sender = AsyncJpegCameraSender(
                self.jpeg_port, unit_test=kwargs.get("jpeg_unit_test", True)
            )

            print(f"[INFO] ZMQ Subscriber connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] ZMQ Publisher bound to tcp://*:{self.status_port}")
            print(f"[INFO] ZMQ Image Publisher bound to tcp://*:{self.image_port} (async thread)")
            print(f"[INFO] ZMQ JPEG Publisher bound to tcp://*:{self.jpeg_port} (async thread)")
        else:
            print("[WARNING] zmq not found. ZMQ control will not be available.")

    def __str__(self) -> str:
        return "TienKung Dex ZMQ Controller"

    # ------------------------------------------------------------------ utils
    def _pick_camera(self, data_types: tuple[str, ...], exclude: str | None) -> str | None:
        """从 camera_names 中挑选指定数据类型的第一路相机。"""
        for name in self.camera_names:
            if exclude and exclude in name:
                continue
            scene_keys = self.env.scene.keys() if hasattr(self.env.scene, "keys") else []
            if name not in scene_keys:
                continue
            sensor_cfg = getattr(self.env.scene[name], "cfg", None)
            cfg_types = list(getattr(sensor_cfg, "data_types", []) or [])
            if any(t in cfg_types for t in data_types):
                return name
        # 退化路径：按命名约定猜测
        for name in self.camera_names:
            if exclude and exclude in name:
                continue
            return name
        return None

    def _should_send_camera(self) -> bool:
        if self._render_interval == 1:
            return True
        if self._step_count == 0:
            return True
        return self._step_count % self._render_interval == 1

    def _send_camera_data(self) -> None:
        if not self._should_send_camera() or self._raw_sender is None or self._rgb_camera is None:
            return
        try:
            rgb_tensor = self.env.scene[self._rgb_camera].data.output.get("rgb")
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return
            rgb_gpu = rgb_tensor[0]

            depth_gpu = None
            if self._depth_camera is not None:
                depth_output = self.env.scene[self._depth_camera].data.output
                depth_tensor = depth_output.get("depth") if depth_output is not None else None
                if depth_tensor is not None and depth_tensor.shape[0] != 0:
                    depth_gpu = depth_tensor[0]

            # 不要用 "camera" 键：C++ 图像桥看到它就走"按相机名路由"分支，而该
            # 分支只发 RGB（没有深度路径，也没有 sensor_msgs/Image 的按相机发布
            # 器），单相机对的 dex 会因此丢帧。不带该键时桥接走单相机分支，
            # 按配置的 --rgb-topic / --depth-topic 同时发 RGB 和深度。
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
            rgb_tensor = self.env.scene[self._rgb_camera].data.output.get("rgb")
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

    def add_callback(self, key, func):
        pass

    def _apply_command(self, msg: dict[str, Any]) -> None:
        """合并一条指令，并清掉同一关节的另一种编址方式。

        手部关节有两种下发方式：绝对弧度 ``{joint: rad}``（legacy 档）与
        归一化闭合度 ``{joint}_percent``（真机档）。``_action`` 是累积字典，
        ``action_process._resolve_value`` 又让 ``_percent`` 优先——两种方式都
        用过时，后到的绝对弧度会被先前残留的 percent 永久压住（表现为手不动）。
        这里按"最后到达者生效"清掉对方的键。
        """
        suffix = "_percent"
        for key in msg:
            if not isinstance(key, str) or key == "reset":
                continue
            if key.endswith(suffix):
                self._action.pop(key[: -len(suffix)], None)
            else:
                self._action.pop(f"{key}{suffix}", None)
        self._action.update(msg)

    def advance(self) -> dict[str, Any]:
        """接收 ZMQ 指令、回传状态与相机，返回 Isaac Lab action。"""
        if HAS_ZMQ:
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                except Exception:
                    break
                if isinstance(msg, dict):
                    self._apply_command(msg)

            if self._action.pop("reset", False):
                self.reset_requested = True

            try:
                status = to_ros_data(self.env, self.variant)
                status["timestamp"] = time.time()
                self.pub_socket.send_json(status, flags=zmq.NOBLOCK)
            except Exception:
                pass

            self._send_camera_data()
            self._send_jpeg_camera_data()

        self._step_count += 1
        return {"tienkung_dex": to_controller_data(self._action, self.env, self.variant)}

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
            print("TienKung Dex Controller: ROS Interface enabled via ZMQ Bridge")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://*:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
            print(f"  - JPEG Pub:    tcp://*:{self.jpeg_port} (image_client compatible)")
            print(f"  - Cameras:     rgb={self._rgb_camera} depth={self._depth_camera}")
            print(f"  - Hand variant: {self.variant}")
