from typing import Any
import time
import struct
import queue
import threading
import torch
import json
import numpy as np
import cv2
import omni.usd
from pxr import Gf, UsdGeom, Usd
try:
    from isaacsim.core.prims import RigidPrim
except ImportError:
    try:
        from omni.isaac.core.prims import RigidPrim
    except ImportError:
        RigidPrim = None

from ..device_base import DeviceBase
from .action_process import to_controller_data, to_ros_data

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False


class AsyncRawCameraSender:
    """Background thread that sends rgb+depth multipart frames over ZMQ (image port).

    The main thread launches asynchronous GPU→CPU copies (non_blocking D2H) into
    pinned host buffers and enqueues them; the background thread waits on the
    CUDA event, then encodes and sends.  The sim main loop never blocks on GPU
    sync or socket I/O.

    Buffers are allocated lazily on the first frame (so resolution is taken from
    the actual tensor, not config) and recycled through a free queue.  Depth is
    converted m→mm uint16 on the GPU before the copy, halving the D2H payload.
    """

    def __init__(self, image_port: int, num_buffers: int = 3):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)  # Only keep latest image
        self._sock.bind(f"tcp://*:{image_port}")
        self._cuda = torch.cuda.is_available()
        self._num_buffers = num_buffers
        self._h: int | None = None
        self._w: int | None = None
        self._err_count = 0
        self._free: queue.Queue = queue.Queue()  # buffers recycled by the bg thread
        self._queue: queue.Queue = queue.Queue(maxsize=num_buffers)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="zmq-raw-camera-sender")
        self._thread.start()

    def _log_err(self, stage: str) -> None:
        """Print the first few errors with traceback (throttled) — never fail silently."""
        self._err_count += 1
        if self._err_count <= 5:
            import traceback
            print(f"[WARN] AsyncRawCameraSender {stage} failed (#{self._err_count}):",
                  flush=True)
            traceback.print_exc()

    def _make_buffers(self, h: int, w: int) -> None:
        for _ in range(self._num_buffers):
            self._free.put({
                "rgb": torch.empty((h, w, 3), dtype=torch.uint8, pin_memory=self._cuda),
                # depth 缓冲按实际张量形状惰性分配(见 send),形状与 rgb 无关:
                # Isaac Lab 5.0 里 rgb (640,360,3) 与 depth (360,640) 维度顺序相反,
                # 且 depth 可能晚于首帧才就绪。
                "depth": None,
                "event": torch.cuda.Event() if self._cuda else None,
                "metadata": None,
                "has_depth": False,
            })

    def _run_loop(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:  # sentinel for shutdown
                break
            try:
                if item["event"] is not None:
                    item["event"].synchronize()  # wait for the D2H copies
                rgb_bytes = item["rgb"].numpy().tobytes()
                depth_bytes = item["depth"].numpy().tobytes() if item["has_depth"] else b""
                # Send as multi-part message (raw bytes for maximum quality)
                self._sock.send_json(item["metadata"], flags=zmq.SNDMORE | zmq.NOBLOCK)
                self._sock.send(rgb_bytes, flags=zmq.SNDMORE | zmq.NOBLOCK)
                self._sock.send(depth_bytes, flags=zmq.NOBLOCK)
            except Exception:
                self._log_err("run_loop")
            finally:
                self._free.put(item)  # recycle buffer

    def send(self, metadata: dict, rgb_gpu, depth_gpu) -> None:
        """Non-blocking: copy device tensors into a pinned buffer async and enqueue.

        rgb_gpu: (H, W, 3) uint8; depth_gpu: (H, W) float32 meters or None.
        Drops the frame silently when no buffer is free (consumer falling behind).
        """
        try:
            if self._h is None:  # lazy buffer allocation from the first frame
                self._h, self._w = int(rgb_gpu.shape[0]), int(rgb_gpu.shape[1])
                self._make_buffers(self._h, self._w)
            buf = self._free.get_nowait()
        except queue.Empty:
            return
        try:
            buf["metadata"] = metadata
            async_copy = self._cuda and rgb_gpu.is_cuda
            buf["rgb"].copy_(rgb_gpu, non_blocking=async_copy)
            buf["has_depth"] = False
            if depth_gpu is not None:
                try:
                    # depth 缓冲按实际张量形状惰性分配/重建:depth 与 rgb 维度
                    # 顺序相反(rgb (640,360,3) vs depth (360,640)),且 depth
                    # 可能晚于首帧才就绪,不能依赖一次性初始化。
                    dshape = tuple(depth_gpu.shape)
                    if buf["depth"] is None or tuple(buf["depth"].shape) != dshape:
                        buf["depth"] = torch.empty(dshape, dtype=torch.uint16,
                                                   pin_memory=self._cuda)
                    # Convert to mm (x1000) and uint16 on the GPU — halves D2H bytes.
                    # nan_to_num: 深度张量可能含 NaN/Inf(裁剪范围外像素)。
                    depth_mm = torch.nan_to_num((depth_gpu * 1000.0).clamp(0, 65535),
                                                nan=0.0, posinf=65535.0, neginf=0.0)
                    buf["depth"].copy_(depth_mm.to(torch.uint16), non_blocking=async_copy)
                    buf["has_depth"] = True
                except Exception:
                    # 降级: depth 失败只发 rgb,不丢整帧(并记录真实异常)
                    self._log_err("depth_convert")
            if buf["event"] is not None:
                buf["event"].record()  # markers on the same stream, ordered after copies
            if self._queue.full():
                self._free.put(buf)  # consumer behind — drop this frame
                return
            self._queue.put_nowait(buf)
        except Exception:
            self._log_err("send")
            self._free.put(buf)

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


class AsyncJpegCameraSender:
    """Background thread that JPEG-encodes and sends frames over ZMQ (jpeg port).

    Same pinned-buffer + CUDA-event pattern as AsyncRawCameraSender: the main
    thread only launches the async D2H and captures ts/seq at enqueue time (keeps
    unit-test timing honest); cv2.imencode runs on the background thread.
    """

    def __init__(self, jpeg_port: int, unit_test: bool = True, num_buffers: int = 3):
        self._unit_test = unit_test
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        self._sock.bind(f"tcp://*:{jpeg_port}")
        self._cuda = torch.cuda.is_available()
        self._num_buffers = num_buffers
        self._h: int | None = None
        self._w: int | None = None
        self._err_count = 0
        self._free: queue.Queue = queue.Queue()
        self._queue: queue.Queue = queue.Queue(maxsize=num_buffers)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="zmq-jpeg-camera-sender")
        self._thread.start()

    def _log_err(self, stage: str) -> None:
        """Print the first few errors with traceback (throttled) — never fail silently."""
        self._err_count += 1
        if self._err_count <= 5:
            import traceback
            print(f"[WARN] AsyncJpegCameraSender {stage} failed (#{self._err_count}):",
                  flush=True)
            traceback.print_exc()

    def _make_buffers(self, h: int, w: int) -> None:
        for _ in range(self._num_buffers):
            self._free.put({
                "rgb": torch.empty((h, w, 3), dtype=torch.uint8, pin_memory=self._cuda),
                "event": torch.cuda.Event() if self._cuda else None,
                "ts": 0.0,
                "seq": 0,
            })

    def _run_loop(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:  # sentinel for shutdown
                break
            try:
                if item["event"] is not None:
                    item["event"].synchronize()  # wait for the D2H copy
                # JPEG encode
                bgr = cv2.cvtColor(item["rgb"].numpy(), cv2.COLOR_RGB2BGR)
                ret, buf = cv2.imencode('.jpg', bgr)
                if not ret:
                    continue
                msg = buf.tobytes()
                if self._unit_test:
                    msg = struct.pack('dI', item["ts"], item["seq"]) + msg
                self._sock.send(msg, flags=zmq.NOBLOCK)
            except Exception:
                self._log_err("run_loop")
            finally:
                self._free.put(item)  # recycle buffer

    def send(self, rgb_gpu, ts: float, seq: int) -> None:
        """Non-blocking: copy the device tensor into a pinned buffer async and enqueue.

        rgb_gpu: (H, W, 3) uint8.  Drops the frame silently when no buffer is free.
        """
        try:
            if self._h is None:  # lazy buffer allocation from the first frame
                self._h, self._w = int(rgb_gpu.shape[0]), int(rgb_gpu.shape[1])
                self._make_buffers(self._h, self._w)
            buf = self._free.get_nowait()
        except queue.Empty:
            return
        try:
            buf["rgb"].copy_(rgb_gpu, non_blocking=self._cuda and rgb_gpu.is_cuda)
            if buf["event"] is not None:
                buf["event"].record()
            buf["ts"] = ts
            buf["seq"] = seq
            if self._queue.full():
                self._free.put(buf)  # consumer behind — drop this frame
                return
            self._queue.put_nowait(buf)
        except Exception:
            self._log_err("send")
            self._free.put(buf)

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


class TienkungProController(DeviceBase):
    """Controller for Tienkung Pro that receives actions via ZMQ bridge.
    This allows ROS 2 Humble (Python 3.10) to communicate with this environment (Python 3.11).
    """
    def __init__(self, env, **kwargs):
        super().__init__()
        self.env = env
        self.device = env.device
        self._last_camera_send_time = -1.0
        
        self._last_depth_send_time = -1.0
        # Initialize default action buffer
        self._action = {}
        self.apple_initial_pos = None  # To store original position for relative offset
        self.reset_requested = False
        
        # Camera send gating: only publish on steps that follow a render step
        # (same _should_send_camera logic as walker_s2; render_interval comes
        # from env.cfg.sim.render_interval via sim_runner kwargs).
        self._render_interval = int(kwargs.get('render_interval', 1))
        self._step_count = 0

        # Cache Prims
        self.apple_prim = self._init_prim("/World/envs/env_0/Scene/apple", rigid=True)
        # Use simple Prim for plate to avoid RigidBody hierarchy issues (CUDA crash)
        self.plate_prim = self._init_prim("/World/envs/env_0/Scene/plate", rigid=False)

        if HAS_ZMQ:
            self.cmd_port = int(kwargs.get('cmd_port', 5555))
            self.status_port = int(kwargs.get('status_port', 5556))
            self.image_port = int(kwargs.get('image_port', 5557))
            self.jpeg_port = int(kwargs.get('jpeg_port', 5558))

            self.context = zmq.Context()
            # Subscriber for control commands
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(f"tcp://127.0.0.1:{self.cmd_port}")
            self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

            # Publisher for status feedback
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.setsockopt(zmq.SNDHWM, 1)
            self.pub_socket.bind(f"tcp://*:{self.status_port}")

            # Camera frames sent via background threads — same ports, no consumer changes needed.
            self._raw_sender = AsyncRawCameraSender(self.image_port)
            self._jpeg_sender = AsyncJpegCameraSender(
                self.jpeg_port, unit_test=kwargs.get('jpeg_unit_test', True))
            self._jpeg_frame_count = 0

            print(f"[INFO] ZMQ Subscriber connected to tcp://127.0.0.1:{self.cmd_port}")
            print(f"[INFO] ZMQ Publisher bound to tcp://*:{self.status_port}")
            print(f"[INFO] ZMQ Image Publisher bound to tcp://*:{self.image_port} (async thread)")
            print(f"[INFO] ZMQ JPEG Image Publisher bound to tcp://*:{self.jpeg_port} (async thread)")
        else:
            print("[WARNING] zmq not found. ZMQ control will not be available.")

    def __str__(self) -> str:
        return "Tienkung Pro ZMQ Controller"

    def _init_prim(self, prim_path, rigid=True):
        if not RigidPrim and rigid:
            print("[WARNING] RigidPrim class not available (isaacsim/omni.isaac.core not found)")
            # Fallback to non-rigid if class missing
            rigid = False
            
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim.IsValid():
                    if not rigid:
                        print(f"[INFO] Prim (read-only) initialized: {prim_path}")
                        return prim

                    try:
                        try:
                            rp = RigidPrim(prim_path)
                        except TypeError:
                            rp = RigidPrim(prim_path=prim_path)
                        rp.initialize()
                        print(f"[INFO] RigidPrim initialized: {prim_path}")
                        return rp
                    except Exception as e:
                        print(f"[WARNING] Failed to initialize RigidPrim for {prim_path}: {e}. Returning raw Prim.")
                        return prim
                else:
                    print(f"[WARNING] Prim not found at {prim_path}")
        except Exception as e:
            print(f"[WARNING] Could not initialize RigidPrim {prim_path}: {e}")
        return None

    def reset(self):
        self._action = {}
        self.apple_initial_pos = None  # Reset initial position cache
        self.reset_requested = False
        self._step_count = 0

    def reset_to_pose(self, pose_dict: dict[str, float]):
        """Reset the internal action buffer to a specific pose."""
        self._action = pose_dict.copy()

    def add_callback(self, key, func):
        pass

    def _get_status_from_sim(self) -> dict:
        """Fetch current robot state from simulation."""
        
        return to_ros_data(self.env)

    def _should_send_camera(self) -> bool:
        """Return True only on steps that follow a render step.

        Camera data is produced by env.step() every render_interval steps, and
        advance() reads the data produced by the *previous* env.step().  Therefore
        we send at step 0 (initial frame) and at steps where
        step_count % render_interval == 1 (new frame rendered in previous step).
        Special case: render_interval=1 means every step renders, always send.
        """
        if self._render_interval == 1:
            return True
        if self._step_count == 0:
            return True
        return self._step_count % self._render_interval == 1

    def _send_camera_data(self):
        """Hand the GPU camera tensors to the async sender (non-blocking).

        The sender copies them into pinned buffers asynchronously (non_blocking
        D2H); the main thread only launches the copies and never waits on GPU
        sync.  Depth conversion and ZMQ socket I/O happen off the main thread.
        """
        if not self._should_send_camera():
            return
        if "camera" not in self.env.scene.keys():
            return

        camera = self.env.scene["camera"]
        camera_depth = self.env.scene["camera_depth"]
        try:
            rgb_tensor = camera.data.output["rgb"]
            depth_tensor = camera_depth.data.output.get("depth") if camera_depth.data.output is not None else None
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return

            rgb_gpu = rgb_tensor[0]  # (H, W, 3) uint8 on device
            depth_gpu = depth_tensor[0] if (depth_tensor is not None and depth_tensor.shape[0] != 0) else None

            metadata = {
                "width": int(rgb_gpu.shape[1]),
                "height": int(rgb_gpu.shape[0]),
                "format": "raw", # Raw bytes format
            }
            self._raw_sender.send(metadata, rgb_gpu, depth_gpu)
        except Exception:
            return

    def _send_jpeg_camera_data(self):
        """Hand the GPU camera tensor to the async JPEG sender (image_client.py compatible).

        Gated by _should_send_camera like the raw port; ts/seq are captured on the
        main thread, D2H and cv2.imencode run on the background thread.
        """
        if not self._should_send_camera():
            return
        if "camera" not in self.env.scene.keys():
            return

        camera = self.env.scene["camera"]
        try:
            rgb_tensor = camera.data.output["rgb"]
            if rgb_tensor is None or rgb_tensor.shape[0] == 0:
                return

            rgb_gpu = rgb_tensor[0]  # (H, W, 3), RGB order
            self._jpeg_sender.send(rgb_gpu, time.time(), self._jpeg_frame_count)
            self._jpeg_frame_count += 1
        except Exception:
            return

    def _get_prim_pose(self, prim_obj):
        """Helper to get position and rotation from a RigidPrim (or similar wrapper)."""
        current_pos = None
        current_rot = None
        
        # Case 1: get_world_pose (Standard)
        if hasattr(prim_obj, "get_world_pose"):
            current_pos, current_rot = prim_obj.get_world_pose()
        
        # Case 2: get_world_poses (Vectorized / XFormPrimView)
        elif hasattr(prim_obj, "get_world_poses"):
                positions, orientations = prim_obj.get_world_poses()
                if len(positions) > 0:
                    current_pos = positions[0] 
                    current_rot = orientations[0]
                    if isinstance(current_pos, torch.Tensor):
                        current_pos = current_pos.cpu().numpy()
                    if isinstance(current_rot, torch.Tensor):
                        current_rot = current_rot.cpu().numpy()

        else:
            # Fallback: USD
            prim = getattr(prim_obj, "prim", None)
            if prim is None:
                prims = getattr(prim_obj, "prims", [])
                if len(prims) > 0:
                    prim = prims[0]
            
            # Check if prim_obj itself is a Usd.Prim
            if prim is None and isinstance(prim_obj, Usd.Prim):
                prim = prim_obj
            
            if prim:
                xform = UsdGeom.Xformable(prim)
                world_transform = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = world_transform.ExtractTranslation()
                rotation = world_transform.ExtractRotation().GetQuat()
                current_pos = [translation[0], translation[1], translation[2]]
                current_rot = np.array([rotation.GetReal(), rotation.GetImaginary()[0], rotation.GetImaginary()[1], rotation.GetImaginary()[2]])
        
        return current_pos, current_rot

    def _set_prim_pose(self, prim_obj, pos, rot=None):
        """Helper to set position and rotation for a RigidPrim."""
        # Set pose
        if hasattr(prim_obj, "set_world_pose"):
            prim_obj.set_world_pose(position=pos, orientation=rot)
            if hasattr(prim_obj, "set_velocities"):
                zero_vel = np.zeros(6)
                prim_obj.set_velocities(zero_vel)
        elif hasattr(prim_obj, "set_world_poses"):
            device = prim_obj._device if hasattr(prim_obj, "_device") else "cuda:0"
            pos_tensor = torch.tensor([pos], dtype=torch.float32, device=device)
            rot_tensor = torch.tensor([rot], dtype=torch.float32, device=device) if rot is not None else None
            prim_obj.set_world_poses(positions=pos_tensor, orientations=rot_tensor)
            if hasattr(prim_obj, "set_velocities"):
                vel_tensor = torch.zeros((1, 6), dtype=torch.float32, device=device)
                prim_obj.set_velocities(vel_tensor)

    def _apply_apple_offset(self):
        if not self.apple_prim or "apple_offset" not in self._action:
            return

        try:
            offset = self._action["apple_offset"]
            self._action.pop("apple_offset")
            
            if not (offset[0]==0 and offset[1]==0): 
                current_pos, current_rot = self._get_prim_pose(self.apple_prim)
                
                # Use hardcoded Z default if getting pose fails
                if current_pos is None:
                    current_pos = [0, 0, 0.8]
                    current_rot = [1, 0, 0, 0]

                try:
                    z_val = current_pos[2]
                except:
                    z_val = 0.8
                
                if z_val < 0.1:
                    print(f"[WARNING] Apple Z is {z_val:.3f}, resetting to default table height 0.8")
                    z_val = 0.8

                if self.apple_initial_pos is None:
                        if current_pos is not None:
                            self.apple_initial_pos = np.array(current_pos) if not isinstance(current_pos, np.ndarray) else current_pos
                            print(f"[INFO] Apple Initial Position Captured: {self.apple_initial_pos}")

                if self.apple_initial_pos is not None:
                        base_x = self.apple_initial_pos[0]
                        base_y = self.apple_initial_pos[1]
                else:
                        base_x = 0.0
                        base_y = 0.0

                new_pos = np.array([base_x + float(offset[0]), base_y + float(offset[1]), z_val])
                
                self._set_prim_pose(self.apple_prim, new_pos, current_rot)
                
                print(f"[INFO] Applied apple offset: {offset} (Pos: {new_pos})")
        except Exception as e:
            print(f"[WARNING] move error apple: {e}")
            import traceback
            traceback.print_exc()

    def _get_apple_plate_dist(self):
        if not self.apple_prim or not self.plate_prim:
            return -1.0
        
        try:
            apple_pos, _ = self._get_prim_pose(self.apple_prim)
            plate_pos, _ = self._get_prim_pose(self.plate_prim)
            
            if apple_pos is not None and plate_pos is not None:
                dist = np.linalg.norm(np.array(apple_pos) - np.array(plate_pos))
                return float(dist)
        except Exception:
            pass
        return -1.0

    def advance(self) -> dict[str, Any]:
        """
        Receive actions from ZMQ and send status back.
        """
        t_send_camera = 0.0
        t_send_jpeg = 0.0

        if HAS_ZMQ:
            # 1. Receive control commands
            while True:
                try:
                    msg = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                    self._action.update(msg)
                except zmq.Again:
                    break

            # 1.0 Handle Reset
            if "reset" in self._action:
                if self._action.pop("reset"):
                    self.reset_requested = True

            # 1.1 Apply Apple Offset
            self._apply_apple_offset()

            # 1.2 Check Task Dist
            dist = self._get_apple_plate_dist()

            try:
                status = self._get_status_from_sim()
                status["task_dist"] = dist
                self.pub_socket.send_json(status, flags=zmq.NOBLOCK)
            except Exception:
                pass
            t0 = time.perf_counter()
            try:
                # send camera data (ignore timing return)
                self._send_camera_data()
            except Exception:
                pass
            t1 = time.perf_counter()
            try:
                self._send_jpeg_camera_data()
            except Exception:
                pass
            t_send_camera = (t1 - t0) * 1000
            t_send_jpeg = (time.perf_counter() - t1) * 1000

        self.advance_timings = {
            "send_camera": t_send_camera,
            "send_jpeg": t_send_jpeg,
        }
        self._step_count += 1
        return {"tienkung_pro": to_controller_data(self._action,self.env)}

    def close(self):
        """Cleanly shut down the async camera senders and ZMQ resources."""
        if HAS_ZMQ:
            for sender in (getattr(self, "_raw_sender", None),
                           getattr(self, "_jpeg_sender", None)):
                if sender is not None:
                    try:
                        sender.close()
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
        """Display the controls."""
        if HAS_ZMQ:
            print("Tienkung Pro Controller: Full ROS Interface enabled via ZMQ Bridge")
            print(f"  - Command Sub: tcp://127.0.0.1:{self.cmd_port}")
            print(f"  - Status Pub:  tcp://*:{self.status_port}")
            print(f"  - Image Pub:   tcp://*:{self.image_port} (raw multipart)")
            print(f"  - JPEG Pub:    tcp://*:{self.jpeg_port} (image_client compatible)")
