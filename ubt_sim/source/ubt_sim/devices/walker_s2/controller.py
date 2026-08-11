from __future__ import annotations

from typing import Any
import json
import queue
import threading
import time

import numpy as np

from ..device_base import DeviceBase
from .action_process import reset_hold_targets, to_controller_data, to_ros_data

try:
    import zmq

    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


class AsyncCameraSender:
    """Background thread that sends camera frames over ZMQ without blocking the main loop.

    The main thread pushes (metadata_list, rgb_list) tuples to an internal queue.
    The background thread consumes the queue, converts numpy arrays to bytes, and
    sends them as ZMQ multipart messages.

    Port is kept identical (5657 default) so existing consumers need no changes.
    """

    def __init__(self, image_port: int, camera_names: list[str]):
        self._camera_names = camera_names
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(f"tcp://*:{image_port}")
        self._queue: queue.Queue = queue.Queue(maxsize=2)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="zmq-camera-sender")
        self._thread.start()

    def _run_loop(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:  # sentinel for shutdown
                break
            try:
                metadata_list, rgb_list = item
                for meta, rgb in zip(metadata_list, rgb_list):
                    self._sock.send_json(meta, flags=zmq.SNDMORE | zmq.NOBLOCK)
                    self._sock.send(rgb.tobytes(), flags=zmq.SNDMORE | zmq.NOBLOCK)
                    self._sock.send(b"", flags=zmq.NOBLOCK)
            except Exception:
                continue

    def send(self, metadata_list: list[dict], rgb_list: list[np.ndarray]) -> None:
        """Non-blocking: push a frame to the queue. Drops silently when queue is full."""
        if self._queue.full():
            return
        try:
            self._queue.put_nowait((metadata_list, rgb_list))
        except queue.Full:
            pass

    def close(self) -> None:
        """Gracefully stop the background thread and clean up ZMQ resources."""
        self._running = False
        try:
            self._queue.put_nowait(None)  # sentinel
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)
        self._sock.close(linger=0)
        self._ctx.term()


class WalkerS2Controller(DeviceBase):
    """Controller for Walker S2 that receives ROS2 commands through ZMQ."""

    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device
        self.reset_requested = False
        self.part_randomization_request: dict[str, Any] | None = None
        self._action: dict[str, Any] = {"body": {}}

        self._camera_names: list[str] = kwargs.get("camera_names", [])
        self._render_interval: int = int(kwargs.get("render_interval", 1))
        self._step_count: int = 0

        self.cmd_port = int(kwargs.get("cmd_port", 5655))
        self.status_port = int(kwargs.get("status_port", 5656))
        self.image_port = int(kwargs.get("image_port", 5657))

        if HAS_ZMQ:
            self.context = zmq.Context()

            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.setsockopt(zmq.RCVHWM, 1)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            # Camera frames sent via background thread — same port, no consumer changes needed.
            self._camera_sender = AsyncCameraSender(self.image_port, self._camera_names)

            print(f"[INFO] Walker S2 ZMQ command sub connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] Walker S2 ZMQ status pub bound to tcp://*:{self.status_port}")
            print(f"[INFO] Walker S2 ZMQ image pub bound to tcp://*:{self.image_port} (async thread)")
        else:
            print("[WARNING] zmq not found. Walker S2 ZMQ control will not be available.")

    def __str__(self) -> str:
        return "Walker S2 ZMQ Controller"

    def reset(self):
        self._action = {"body": {}}
        self.reset_requested = False
        self.part_randomization_request = None
        self._step_count = 0
        reset_hold_targets()

    def add_callback(self, key, func):
        pass

    def _merge_command(self, msg: dict[str, Any]) -> None:
        if msg.get("reset"):
            self.reset_requested = True
            return

        if "randomize_part_sorting_pieces" in msg:
            payload = msg.get("randomize_part_sorting_pieces")
            if payload is True or payload is None:
                payload = {}
            if isinstance(payload, dict):
                self.part_randomization_request = payload
            else:
                print("[WARN] Ignoring invalid part randomization payload; expected object or true.")
            return

        if "body" in msg:
            body = msg.get("body") or {}
            if isinstance(body, dict):
                # Replace rather than update to avoid stale joints from
                # previous messages accumulating when the controller uses
                # publish_changed_only.  HoldTargetManager (action_process)
                # already persists the full set of joint targets; _action
                # only needs to represent the current frame's command.
                self._action["body"] = dict(body)
            else:
                self._action["body"] = body

        for key in ["left_hand", "right_hand", "left_grip", "right_grip"]:
            if key in msg:
                self._action[key] = msg[key]

    def pop_part_randomization_request(self) -> dict[str, Any] | None:
        request = self.part_randomization_request
        self.part_randomization_request = None
        return request

    def _send_status(self) -> None:
        status = to_ros_data(self.env, self._action)
        self.pub_socket.send_json(status, flags=zmq.NOBLOCK)

    def _should_send_camera(self) -> bool:
        """Return True only on steps that follow a render step.
        Camera data is produced by env.step() every render_interval steps,
        and advance() reads the data produced by the *previous* env.step().
        Therefore we send at step 0 (initial frame) and at steps where
        step_count % render_interval == 1 (new frame rendered in previous step).
        Special case: render_interval=1 means every step renders, always send.
        """
        if self._render_interval == 1:
            return True
        if self._step_count == 0:
            return True
        return self._step_count % self._render_interval == 1

    def _send_camera_data(self) -> None:
        """Copy camera frames and push them to the async sender queue (non-blocking).

        Data from env.scene is read in the main thread, numpy-copied, and handed
        off to the background ZMQ sender thread.  The main loop is never blocked
        by ZMQ socket I/O.
        """
        if not self._should_send_camera():
            return
        metadata_list: list[dict] = []
        rgb_list: list[np.ndarray] = []
        for cam_name in self._camera_names:
            if cam_name not in self.env.scene.keys():
                continue
            camera = self.env.scene[cam_name]
            try:
                rgb_tensor = camera.data.output.get("rgb") if camera.data.output is not None else None
                if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                    continue
                rgb = np.copy(rgb_tensor[0].cpu().numpy())
                metadata = {
                    "width": int(rgb.shape[1]),
                    "height": int(rgb.shape[0]),
                    "format": "raw",
                    "camera": cam_name,
                }
                metadata_list.append(metadata)
                rgb_list.append(rgb)
            except Exception:
                continue
        if metadata_list:
            self._camera_sender.send(metadata_list, rgb_list)

    def advance(self) -> dict[str, Any]:
        t_zmq_recv = 0.0
        t_send_status = 0.0
        t_send_camera = 0.0

        if HAS_ZMQ:
            t0 = time.perf_counter()
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                    self._merge_command(msg)
                except zmq.Again:
                    break
                except json.JSONDecodeError:
                    break
            t1 = time.perf_counter()
            t_zmq_recv = (t1 - t0) * 1000

            try:
                self._send_status()
            except Exception:
                pass
            t2 = time.perf_counter()
            t_send_status = (t2 - t1) * 1000

            try:
                self._send_camera_data()
            except Exception:
                pass
            t3 = time.perf_counter()
            t_send_camera = (t3 - t2) * 1000

        self.advance_timings = {
            "zmq_recv": t_zmq_recv,
            "send_status": t_send_status,
            "send_camera": t_send_camera,
        }

        action = {"walker_s2": to_controller_data(self._action, self.env)}
        self._step_count += 1
        return action

    def close(self):
        """Cleanly shut down the async camera sender and ZMQ resources."""
        if HAS_ZMQ:
            try:
                self._camera_sender.close()
            except Exception:
                pass
            try:
                self.sub_socket.close(linger=0)
            except Exception:
                pass
            try:
                self.pub_socket.close(linger=0)
            except Exception:
                pass
            try:
                self.context.term()
            except Exception:
                pass

    def display_controls(self):
        if HAS_ZMQ:
            print("Walker S2 Controller: ROS2 SDK interface enabled via ZMQ bridge")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://127.0.0.1:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
