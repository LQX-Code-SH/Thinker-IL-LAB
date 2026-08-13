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

    The main thread pushes (metadata, rgb, depth_m) tuples to an internal queue;
    only the GPU→CPU np.copy runs on the main thread.  Depth conversion (m→mm,
    clip, uint16) and all ZMQ socket I/O happen on the background thread, so the
    sim main loop is never blocked by encoding or socket sends.
    """

    def __init__(self, image_port: int):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)  # Only keep latest image
        self._sock.bind(f"tcp://*:{image_port}")
        self._queue: queue.Queue = queue.Queue(maxsize=2)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="zmq-raw-camera-sender")
        self._thread.start()

    def _run_loop(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:  # sentinel for shutdown
                break
            metadata, rgb, depth_m = item
            try:
                # Convert depth to mm (x1000) and uint16 for standard ROS depth
                depth_bytes = b""
                if depth_m is not None:
                    depth_img_mm = np.clip(depth_m * 1000.0, 0, 65535)
                    depth_bytes = depth_img_mm.astype(np.uint16).tobytes()

                # Send as multi-part message (raw bytes for maximum quality)
                self._sock.send_json(metadata, flags=zmq.SNDMORE | zmq.NOBLOCK)
                self._sock.send(rgb.tobytes(), flags=zmq.SNDMORE | zmq.NOBLOCK)
                self._sock.send(depth_bytes, flags=zmq.NOBLOCK)
            except Exception:
                continue

    def send(self, metadata: dict, rgb: np.ndarray, depth_m: np.ndarray | None) -> None:
        """Non-blocking: push a frame to the queue. Drops silently when queue is full."""
        if self._queue.full():
            return
        try:
            self._queue.put_nowait((metadata, rgb, depth_m))
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


class AsyncJpegCameraSender:
    """Background thread that JPEG-encodes and sends frames over ZMQ (jpeg port).

    The main thread pushes (rgb, ts, seq) tuples; timestamp and sequence number
    are captured on the main thread at enqueue time to keep unit-test timing
    honest.  cv2.imencode runs on the background thread.
    """

    def __init__(self, jpeg_port: int, unit_test: bool = True):
        self._unit_test = unit_test
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 1)
        self._sock.bind(f"tcp://*:{jpeg_port}")
        self._queue: queue.Queue = queue.Queue(maxsize=2)
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True,
                                        name="zmq-jpeg-camera-sender")
        self._thread.start()

    def _run_loop(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:  # sentinel for shutdown
                break
            rgb, ts, seq = item
            try:
                # JPEG encode
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ret, buf = cv2.imencode('.jpg', bgr)
                if not ret:
                    continue
                msg = buf.tobytes()
                if self._unit_test:
                    msg = struct.pack('dI', ts, seq) + msg
                self._sock.send(msg, flags=zmq.NOBLOCK)
            except Exception:
                continue

    def send(self, rgb: np.ndarray, ts: float, seq: int) -> None:
        """Non-blocking: push a frame to the queue. Drops silently when queue is full."""
        if self._queue.full():
            return
        try:
            self._queue.put_nowait((rgb, ts, seq))
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
        """Copy camera frames and push them to the async sender queue (non-blocking).

        Only the GPU→CPU numpy copy runs on the main thread; depth conversion and
        ZMQ socket I/O happen on the background sender thread.
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

            # Pull tensors to CPU (copied: the underlying buffer is reused next step)
            rgb = np.copy(rgb_tensor[0].cpu().numpy())
            depth_m = None
            if depth_tensor is not None and depth_tensor.shape[0] != 0:
                depth_m = np.copy(depth_tensor[0].cpu().numpy())

            metadata = {
                "width": rgb.shape[1],
                "height": rgb.shape[0],
                "format": "raw", # Raw bytes format
            }
            self._raw_sender.send(metadata, rgb, depth_m)
        except Exception:
            return

    def _send_jpeg_camera_data(self):
        """Push JPEG-encoded camera frames to the async sender (image_client.py compatible).

        Gated by _should_send_camera like the raw port; ts/seq are captured on the
        main thread, cv2.imencode runs on the background thread.
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

            rgb = np.copy(rgb_tensor[0].cpu().numpy())  # (H, W, 3), RGB order
            self._jpeg_sender.send(rgb, time.time(), self._jpeg_frame_count)
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
