"""ZMQ image receiver base class.

复制自 lerobot_robot_tienkung（不跨包 import：两个插件包各自独立维护）。
提供：
- ZMQ SUB 连接与订阅
- JPEG 帧解码（cv2.imdecode）
- 按 offset 切分拼接帧（一个发送端推多路相机时用）
- 后台接收线程 + 线程安全的最新帧缓冲
- 可选 cv2.imshow 实时显示
"""

from __future__ import annotations

import logging
import struct
import threading
import time

import cv2
import numpy as np
import zmq

# Unit_Test header: double (8 bytes) + uint32 (4 bytes)
_HEADER_SIZE = struct.calcsize('dI')

logger = logging.getLogger(__name__)


class ZMQImageReceiver:
    """Base class for receiving image frames from a ZMQ JPEG sender.

    发送端每帧是一条完整 JPEG 字节串。已知两种发送端：
    ubt_sim 的 AsyncJpegCameraSender（带 12 字节 'dI' 头）与 ImageServer（无头）。
    头部存在与否靠"剥了能解、原样不能解"来判断，两种都可直连。

    Usage:
        receiver = ZMQImageReceiver(
            server_address="127.0.0.1", port=5758,
            offset_x=0, width=640, height=360,
            display=True,
        )
        receiver.start()
        frame, ts = receiver.get_latest_frame()
        receiver.stop()
    """

    def __init__(
        self,
        server_address: str = "127.0.0.1",
        port: int = 5558,
        offset_x: int = 0,
        width: int = 640,
        height: int = 360,
        display: bool = False,
    ):
        self._server_address = server_address
        self._port = port
        self._offset_x = offset_x
        self._width = width
        self._height = height
        self._display = display

        # ZMQ state (initialized in start())
        self._zmq_context: zmq.Context | None = None
        self._zmq_socket: zmq.Socket | None = None

        # Thread-safe frame buffer
        self._frame_lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_timestamp: float | None = None
        self._new_frame_event = threading.Event()

        # Receive thread
        self._recv_thread: threading.Thread | None = None
        self._running = False

    def __str__(self) -> str:
        return f"ZMQImageReceiver(zmq://{self._server_address}:{self._port}, offset={self._offset_x})"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Create ZMQ SUB connection and start the background receive thread."""
        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.SUB)
        self._zmq_socket.connect(f"tcp://{self._server_address}:{self._port}")
        self._zmq_socket.setsockopt(zmq.RCVHWM, 1)
        self._zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name=f"{self}_recv"
        )
        self._recv_thread.start()
        logger.info("%s started.", self)

    def stop(self) -> None:
        """Stop the receive thread and close the ZMQ connection."""
        self._running = False
        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=3.0)
            self._recv_thread = None

        if self._zmq_socket is not None:
            self._zmq_socket.close()
            self._zmq_socket = None
        if self._zmq_context is not None:
            self._zmq_context.term()
            self._zmq_context = None

        if self._display:
            cv2.destroyAllWindows()

        with self._frame_lock:
            self._latest_frame = None
            self._latest_timestamp = None
            self._new_frame_event.clear()

        logger.info("%s stopped.", self)

    # ------------------------------------------------------------------
    # Background receive loop
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        """Background loop: receive JPEG → strip header → decode → split → buffer → display."""
        while self._running:
            try:
                message = self._zmq_socket.recv(flags=zmq.NOBLOCK)

                # 剥头部：能剥开而原样解不开，说明发送端加了 'dI' 头（ubt_sim）。
                jpg_bytes = message
                if len(message) > _HEADER_SIZE:
                    np_stripped = np.frombuffer(message[_HEADER_SIZE:], dtype=np.uint8)
                    frame_stripped = cv2.imdecode(np_stripped, cv2.IMREAD_COLOR)
                    np_raw = np.frombuffer(message, dtype=np.uint8)
                    frame_raw = cv2.imdecode(np_raw, cv2.IMREAD_COLOR)
                    if frame_stripped is not None and frame_raw is None:
                        jpg_bytes = message[_HEADER_SIZE:]

                # JPEG decode
                np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
                full_frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
                if full_frame is None:
                    continue

                # Offset-based frame splitting
                x1 = self._offset_x + self._width
                if x1 > full_frame.shape[1]:
                    logger.warning(
                        "%s: frame too narrow (need %d, got %d)",
                        self, x1, full_frame.shape[1],
                    )
                    continue
                frame = np.ascontiguousarray(full_frame[:, self._offset_x:x1])

                # Store in thread-safe buffer
                capture_time = time.perf_counter()
                with self._frame_lock:
                    self._latest_frame = frame
                    self._latest_timestamp = capture_time
                self._new_frame_event.set()

                # Real-time display
                if self._display:
                    h, w = frame.shape[:2]
                    display_frame = cv2.resize(frame, (w // 2, h // 2))
                    window_name = str(self)
                    cv2.imshow(window_name, display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self._display = False
                        cv2.destroyWindow(window_name)

            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                logger.warning("%s: recv error: %s", self, e)
                time.sleep(0.01)

    # ------------------------------------------------------------------
    # Frame access
    # ------------------------------------------------------------------

    def get_latest_frame(self) -> tuple[np.ndarray | None, float | None]:
        """Return the latest frame and its timestamp (non-blocking peek).

        Returns:
            (frame, timestamp) or (None, None) if no frame available.
        """
        with self._frame_lock:
            return self._latest_frame, self._latest_timestamp

    def wait_for_new_frame(self, timeout_ms: float) -> tuple[np.ndarray | None, float | None]:
        """Wait for a new (unconsumed) frame up to *timeout_ms*.

        Returns:
            (frame, timestamp) or (None, None) on timeout.
        """
        if self._new_frame_event.wait(timeout=timeout_ms / 1000.0):
            with self._frame_lock:
                frame = self._latest_frame
                timestamp = self._latest_timestamp
                self._new_frame_event.clear()
            return frame, timestamp
        return None, None
