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

import numpy as np
import omni.usd
from ..device_base import DeviceBase
from ..tienkung_pro.controller import AsyncJpegCameraSender, AsyncRawCameraSender
from .action_process import to_controller_data, to_ros_data

try:  # 苹果是刚体：位姿要经 RigidPrim 读，raw USD 在物理步进后不刷新
    from isaacsim.core.prims import RigidPrim
except ImportError:  # pragma: no cover
    try:
        from omni.isaac.core.prims import RigidPrim
    except ImportError:
        RigidPrim = None

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


DEFAULT_CMD_PORT = 5755
DEFAULT_STATUS_PORT = 5756
DEFAULT_IMAGE_PORT = 5757
DEFAULT_JPEG_PORT = 5758

# 任务成功判定用的场景 prim：与 tienkung_pro 共用 scenes/parlor/scene_v2.usd，路径相同。
# 苹果中心到盘心的距离随状态一起发出去（status["task_dist"]），桥接转成
# /sim/task_completed (Float32)，采集脚本据此决定这一集存不存。
APPLE_PRIM_PATH = "/World/envs/env_0/Scene/apple"
PLATE_PRIM_PATH = "/World/envs/env_0/Scene/plate"


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

        # 苹果/盘子的世界位姿（任务成功判定）。apple 是刚体，用 RigidPrim 才拿得到
        # 物理步进后的位姿；plate 是静态的，raw Prim 够用且避开 RigidBody 层级的
        # CUDA 问题——与 devices/tienkung_pro/controller.py 的取舍一致。
        self.apple_prim = self._init_prim(APPLE_PRIM_PATH, rigid=True)
        self.plate_prim = self._init_prim(PLATE_PRIM_PATH, rigid=False)
        # 苹果随机化（--randomize-apple）的基准位姿：复位后第一次收到偏移时记下来，
        # 之后每集的绝对位置都是「它 + 偏移」。reset() 会清掉，逼着重新取一次。
        self.apple_initial_pos: np.ndarray | None = None

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

    # ------------------------------------------------------- 任务判定（apple↔plate）
    def _init_prim(self, prim_path: str, rigid: bool = True):
        """取场景 prim；``rigid=True`` 时包成 RigidPrim 以便读物理更新后的位姿。

        读不到就返回 None，调用方按"拿不到真值"降级处理（``task_dist = -1``）。
        实现与 ``devices/tienkung_pro/controller.py`` 的同名方法一致。
        """
        if rigid and RigidPrim is None:
            print("[WARNING] RigidPrim 不可用，退回 raw Prim（位姿可能不随物理步进更新）")
            rigid = False
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path) if stage else None
            if prim is None or not prim.IsValid():
                print(f"[WARNING] Prim not found: {prim_path}")
                return None
            if not rigid:
                return prim
            try:
                try:
                    rigid_prim = RigidPrim(prim_path)
                except TypeError:  # 老版本是关键字参数
                    rigid_prim = RigidPrim(prim_path=prim_path)
                rigid_prim.initialize()
                return rigid_prim
            except Exception as exc:  # 刚体初始化失败也要能跑
                print(f"[WARNING] RigidPrim init failed for {prim_path}: {exc}；退回 raw Prim")
                return prim
        except Exception as exc:
            print(f"[WARNING] Could not init prim {prim_path}: {exc}")
            return None

    @staticmethod
    def _get_prim_pose(prim_obj):
        """返回 (position, orientation)；拿不到就 (None, None)。"""
        try:
            if hasattr(prim_obj, "get_world_pose"):        # RigidPrim
                pos, rot = prim_obj.get_world_pose()
            elif hasattr(prim_obj, "get_world_poses"):     # 向量化 PrimView
                positions, orientations = prim_obj.get_world_poses()
                if len(positions) == 0:
                    return None, None
                pos, rot = positions[0], orientations[0]
            else:                                          # raw USD Prim
                from pxr import Usd, UsdGeom

                xform = UsdGeom.Xformable(prim_obj).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                return tuple(xform.ExtractTranslation()), None
            pos = pos.cpu().numpy() if hasattr(pos, "cpu") else pos
            rot = rot.cpu().numpy() if hasattr(rot, "cpu") else rot
            return pos, rot
        except Exception:
            return None, None

    def _get_apple_plate_dist(self) -> float:
        """苹果中心到盘心的距离（米）；拿不到位姿返回 -1.0，调用方据此跳过判定。"""
        if not self.apple_prim or not self.plate_prim:
            return -1.0
        apple_pos, _ = self._get_prim_pose(self.apple_prim)
        plate_pos, _ = self._get_prim_pose(self.plate_prim)
        if apple_pos is None or plate_pos is None:
            return -1.0
        try:
            return float(
                sum((float(a) - float(b)) ** 2 for a, b in zip(apple_pos, plate_pos)) ** 0.5
            )
        except Exception:
            return -1.0

    # ------------------------------------------------------- 苹果随机化
    def _set_prim_pose(self, prim_obj, position, orientation=None) -> None:
        """设置刚体世界位姿，并把速度清零（不清会带着旧速度飞出去）。"""
        if hasattr(prim_obj, "set_world_pose"):        # RigidPrim
            prim_obj.set_world_pose(position=position, orientation=orientation)
            if hasattr(prim_obj, "set_velocities"):
                prim_obj.set_velocities(np.zeros(6))
        elif hasattr(prim_obj, "set_world_poses"):     # 向量化 PrimView
            import torch

            device = getattr(prim_obj, "_device", "cuda:0")
            positions = torch.tensor([position], dtype=torch.float32, device=device)
            orientations = (
                torch.tensor([orientation], dtype=torch.float32, device=device)
                if orientation is not None
                else None
            )
            prim_obj.set_world_poses(positions=positions, orientations=orientations)
            if hasattr(prim_obj, "set_velocities"):
                prim_obj.set_velocities(torch.zeros((1, 6), dtype=torch.float32, device=device))
        else:
            print("[WARNING] 苹果 prim 不能设置位姿（RigidPrim 初始化失败？），随机化不生效")

    def _apply_apple_offset(self) -> None:
        """按 ``apple_offset``（世界系 x/y 偏移，米）把苹果挪到「基准位姿 + 偏移」。

        与 ``tienkung_pro`` 同一套做法：复位后第一次收到偏移时把苹果当时的位姿
        记成基准，之后每集都用「基准 + 偏移」的绝对位置——即使上一集的苹果还
        躺在盘子里，也不会一集集越挪越偏。z 取基准高度，不取当前高度。
        """
        # 先摘掉，别让它跟着 _action 流进关节映射
        offset = self._action.pop("apple_offset", None)
        if offset is None or self.apple_prim is None:
            return
        try:
            dx, dy = float(offset[0]), float(offset[1])
        except (TypeError, ValueError, IndexError):
            print(f"[WARNING] apple_offset 格式不对，忽略: {offset}")
            return

        pos, rot = self._get_prim_pose(self.apple_prim)
        if pos is None:
            print("[WARNING] 读不到苹果位姿，本次随机化跳过")
            return
        if self.apple_initial_pos is None:
            self.apple_initial_pos = np.asarray(pos, dtype=float)
            print(f"[INFO] 苹果基准位姿: {np.round(self.apple_initial_pos, 4).tolist()}")

        target = self.apple_initial_pos.copy()
        target[0] += dx
        target[1] += dy
        self._set_prim_pose(self.apple_prim, target, rot)
        print(
            f"[INFO] 苹果偏移 ({dx:+.3f}, {dy:+.3f}) m -> 世界系 "
            f"{np.round(target, 4).tolist()}"
        )

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
        # 场景复位后苹果回到初始位姿，基准要重新取（与 tienkung_pro 相同）
        self.apple_initial_pos = None

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

            # 苹果随机化（--randomize-apple）：先挪苹果再出状态，同一步里
            # task_dist 就已经反映新位置
            self._apply_apple_offset()

            try:
                status = to_ros_data(self.env, self.variant)
                status["timestamp"] = time.time()
                # 苹果↔盘心距离（真值），供采集脚本判定这一集是否真的抓放成功
                status["task_dist"] = self._get_apple_plate_dist()
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
