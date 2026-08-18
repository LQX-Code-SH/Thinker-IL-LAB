#!/usr/bin/env python3
"""Stream a ROS camera into XRoboToolkit's PICO Remote Vision window.

The protocol mirrors XRoboToolkit-Orin-Video-Sender: the PICO opens a control
connection to port 13579, sends an OPEN_CAMERA request, and then listens for a
second TCP connection carrying length-prefixed H.264 access units.
"""
from __future__ import annotations

import shutil
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Optional

import numpy as np


MAX_CONTROL_FRAME_BYTES = 1024 * 1024
MAX_VIDEO_DIMENSION = 4096


@dataclass(frozen=True)
class CameraRequest:
    width: int
    height: int
    fps: int
    bitrate: int
    enable_mv_hevc: int
    render_mode: int
    port: int
    camera: str
    ip: str


def parse_control_body(body: bytes) -> tuple[str, bytes]:
    """Decode XRoboToolkit's little-endian command/data envelope."""
    if len(body) < 8:
        raise ValueError("control body is shorter than 8 bytes")
    command_length = struct.unpack_from("<i", body, 0)[0]
    if command_length < 0 or 4 + command_length + 4 > len(body):
        raise ValueError(f"invalid command length {command_length}")
    command_end = 4 + command_length
    command = body[4:command_end].split(b"\0", 1)[0].decode("utf-8")
    data_length = struct.unpack_from("<i", body, command_end)[0]
    data_start = command_end + 4
    data_end = data_start + data_length
    if data_length < 0 or data_end > len(body):
        raise ValueError(f"invalid command data length {data_length}")
    return command, body[data_start:data_end]


def parse_camera_request(data: bytes) -> CameraRequest:
    """Decode the version-1 CameraRequestData sent by the Unity client."""
    if len(data) < 33:
        raise ValueError("camera request is too short")
    if data[:2] != b"\xca\xfe":
        raise ValueError("camera request magic is not CA FE")
    if data[2] != 1:
        raise ValueError(f"unsupported camera request version {data[2]}")
    fields = struct.unpack_from("<7i", data, 3)
    offset = 31

    def compact_string() -> str:
        nonlocal offset
        if offset >= len(data):
            raise ValueError("camera request is missing a string length")
        length = data[offset]
        offset += 1
        if offset + length > len(data):
            raise ValueError("camera request contains a truncated string")
        value = data[offset:offset + length].decode("utf-8")
        offset += length
        return value

    camera = compact_string()
    ip = compact_string()
    return CameraRequest(*fields, camera=camera, ip=ip)


def _recv_exact(connection: socket.socket, size: int) -> Optional[bytes]:
    data = bytearray()
    while len(data) < size:
        try:
            chunk = connection.recv(size - len(data))
        except socket.timeout:
            continue
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


class AnnexBAccessUnitParser:
    """Group an Annex-B H.264 stream into access units using AUD NALs."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._current = bytearray()
        self._current_has_vcl = False

    @staticmethod
    def _start_codes(data: bytes) -> list[int]:
        starts: list[int] = []
        index = 0
        while index + 3 <= len(data):
            if data[index:index + 4] == b"\x00\x00\x00\x01":
                starts.append(index)
                index += 4
            elif data[index:index + 3] == b"\x00\x00\x01":
                starts.append(index)
                index += 3
            else:
                index += 1
        return starts

    @staticmethod
    def _nal_type(nal: bytes) -> int:
        if nal.startswith(b"\x00\x00\x00\x01"):
            offset = 4
        elif nal.startswith(b"\x00\x00\x01"):
            offset = 3
        else:
            return -1
        return (nal[offset] & 0x1F) if offset < len(nal) else -1

    def _accept_nal(self, nal: bytes, output: list[bytes]) -> None:
        nal_type = self._nal_type(nal)
        if nal_type == 9 and self._current and self._current_has_vcl:
            output.append(bytes(self._current))
            self._current.clear()
            self._current_has_vcl = False
        self._current.extend(nal)
        if 1 <= nal_type <= 5:
            self._current_has_vcl = True

    def feed(self, chunk: bytes) -> list[bytes]:
        self._buffer.extend(chunk)
        data = bytes(self._buffer)
        starts = self._start_codes(data)
        if not starts:
            # A start code can be split across reads; only the last three
            # zero-ish bytes are useful until the next chunk arrives.
            if len(self._buffer) > 3:
                del self._buffer[:-3]
            return []
        if starts[0] > 0:
            data = data[starts[0]:]
            starts = [value - starts[0] for value in starts]
        output: list[bytes] = []
        for index in range(len(starts) - 1):
            self._accept_nal(data[starts[index]:starts[index + 1]], output)
        self._buffer = bytearray(data[starts[-1]:])
        return output

    def flush(self) -> list[bytes]:
        output: list[bytes] = []
        if self._buffer:
            self._accept_nal(bytes(self._buffer), output)
            self._buffer.clear()
        if self._current:
            output.append(bytes(self._current))
            self._current.clear()
            self._current_has_vcl = False
        return output


def _ros_image_to_rgb(msg: Any) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()
    channel_counts = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    if encoding not in channel_counts:
        raise ValueError(f"unsupported Remote Vision image encoding {msg.encoding!r}")
    channels = channel_counts[encoding]
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError(f"invalid ROS image geometry {width}x{height}, step={step}")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("ROS image data is shorter than height * step")
    rows = raw[:height * step].reshape(height, step)
    image = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == "rgb8":
        rgb = image
    elif encoding == "bgr8":
        rgb = image[..., ::-1]
    elif encoding == "rgba8":
        rgb = image[..., :3]
    elif encoding == "bgra8":
        rgb = image[..., 2::-1]
    else:
        rgb = np.repeat(image, 3, axis=2)
    return np.ascontiguousarray(rgb)


class PicoHeadsetView:
    """ROS camera subscriber plus XRoboToolkit Remote Vision TCP sender."""

    def __init__(
        self,
        node: Any,
        camera_topic: str,
        control_port: int = 13579,
        ffmpeg_path: Optional[str] = None,
    ) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        import cv2

        self.node = node
        self.camera_topic = camera_topic
        self.control_port = int(control_port)
        self.cv2 = cv2
        self.ffmpeg_path = ffmpeg_path or shutil.which("ffmpeg")
        if not self.ffmpeg_path:
            raise RuntimeError("--headset-mode requires ffmpeg in PATH")
        self._check_encoder()
        if not 1 <= self.control_port <= 65535:
            raise ValueError(f"invalid Remote Vision control port {self.control_port}")

        self._closing = threading.Event()
        self._frame_condition = threading.Condition()
        self._latest_image: Optional[Any] = None
        self._latest_sequence = 0
        self._stream_lock = threading.Lock()
        self._stream_stop: Optional[threading.Event] = None
        self._stream_thread: Optional[threading.Thread] = None
        self._control_lock = threading.Lock()
        self._control_connection: Optional[socket.socket] = None

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("0.0.0.0", self.control_port))
        self._server.listen(1)
        self._server.settimeout(0.5)
        node.create_subscription(
            Image, camera_topic, self._image_callback, qos_profile_sensor_data
        )
        self._listener = threading.Thread(
            target=self._listen_loop, daemon=True, name="pico_remote_vision_control"
        )
        self._listener.start()
        node.get_logger().info(
            f"PICO Remote Vision ready on 0.0.0.0:{self.control_port}; "
            f"source={camera_topic}"
        )

    def _check_encoder(self) -> None:
        try:
            result = subprocess.run(
                [self.ffmpeg_path, "-hide_banner", "-encoders"],
                check=False,
                text=True,
                capture_output=True,
                timeout=5.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"cannot inspect ffmpeg: {exc}") from exc
        has_libx264 = any(
            len(parts) >= 2 and parts[1] == "libx264"
            for parts in (line.split() for line in result.stdout.splitlines())
        )
        if result.returncode != 0 or not has_libx264:
            raise RuntimeError("--headset-mode requires an ffmpeg build with libx264")

    def _image_callback(self, msg: Any) -> None:
        with self._frame_condition:
            self._latest_image = msg
            self._latest_sequence += 1
            self._frame_condition.notify_all()

    def _listen_loop(self) -> None:
        while not self._closing.is_set():
            try:
                connection, address = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.node.get_logger().info(
                f"PICO Remote Vision control connected from {address[0]}:{address[1]}"
            )
            try:
                with self._control_lock:
                    self._control_connection = connection
                connection.settimeout(0.5)
                self._handle_control_connection(connection, address[0])
            except Exception as exc:
                if not self._closing.is_set():
                    self.node.get_logger().warn(f"Remote Vision control error: {exc}")
            finally:
                with self._control_lock:
                    if self._control_connection is connection:
                        self._control_connection = None
                try:
                    connection.close()
                except OSError:
                    pass
                self._stop_stream()

    def _handle_control_connection(
        self, connection: socket.socket, peer_ip: str
    ) -> None:
        while not self._closing.is_set():
            header = _recv_exact(connection, 4)
            if header is None:
                return
            body_length = struct.unpack(">I", header)[0]
            if body_length <= 0 or body_length > MAX_CONTROL_FRAME_BYTES:
                raise ValueError(f"invalid Remote Vision frame length {body_length}")
            body = _recv_exact(connection, body_length)
            if body is None:
                return
            command, data = parse_control_body(body)
            if command == "OPEN_CAMERA":
                request = self._validated_request(parse_camera_request(data), peer_ip)
                self._start_stream(request)
            elif command == "CLOSE_CAMERA":
                self._stop_stream()
            elif command == "AUDIO_SESSION":
                # Video is independent.  The current Unity client explicitly
                # falls back to flat/mono when no AUDIO_CONFIG is returned.
                self.node.get_logger().info(
                    "PICO requested audio negotiation; continuing video-only"
                )
            else:
                self.node.get_logger().info(
                    f"ignoring unsupported Remote Vision command {command!r}"
                )

    def _validated_request(self, request: CameraRequest, peer_ip: str) -> CameraRequest:
        if not (2 <= request.width <= MAX_VIDEO_DIMENSION):
            raise ValueError(f"invalid video width {request.width}")
        if not (2 <= request.height <= MAX_VIDEO_DIMENSION):
            raise ValueError(f"invalid video height {request.height}")
        if request.width % 2 or request.height % 2:
            raise ValueError("H.264 video dimensions must be even")
        if not 1 <= request.port <= 65535:
            raise ValueError(f"invalid PICO video port {request.port}")
        if request.enable_mv_hevc:
            raise ValueError("PICO requested HEVC, but this simulator sender supports H.264 only")
        if request.ip and request.ip != peer_ip:
            self.node.get_logger().warn(
                f"PICO advertised video IP {request.ip}, using control peer {peer_ip}"
            )
        return replace(
            request,
            ip=peer_ip,
            fps=max(1, min(int(request.fps), 60)),
            bitrate=max(250_000, min(int(request.bitrate), 20_000_000)),
        )

    def _start_stream(self, request: CameraRequest) -> None:
        self._stop_stream()
        stop_event = threading.Event()
        thread = threading.Thread(
            target=self._stream_loop,
            args=(request, stop_event),
            daemon=True,
            name="pico_remote_vision_video",
        )
        with self._stream_lock:
            self._stream_stop = stop_event
            self._stream_thread = thread
        thread.start()

    def _stop_stream(self) -> None:
        with self._stream_lock:
            stop_event = self._stream_stop
            thread = self._stream_thread
            self._stream_stop = None
            self._stream_thread = None
        if stop_event is not None:
            stop_event.set()
            with self._frame_condition:
                self._frame_condition.notify_all()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=4.0)

    def _ffmpeg_command(self, request: CameraRequest) -> list[str]:
        key_interval = max(1, min(request.fps, 30))
        return [
            self.ffmpeg_path,
            "-hide_banner", "-loglevel", "error", "-nostdin",
            "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", f"{request.width}x{request.height}",
            "-framerate", str(request.fps), "-i", "pipe:0",
            "-an", "-c:v", "libx264", "-preset", "ultrafast",
            "-tune", "zerolatency", "-profile:v", "baseline",
            "-pix_fmt", "yuv420p", "-bf", "0", "-g", str(key_interval),
            "-b:v", str(request.bitrate), "-maxrate", str(request.bitrate),
            "-bufsize", str(max(250_000, request.bitrate // 2)),
            "-x264-params",
            f"aud=1:repeat-headers=1:keyint={key_interval}:min-keyint={key_interval}:scenecut=0",
            "-flush_packets", "1", "-f", "h264", "pipe:1",
        ]

    def _resize_letterbox(self, rgb: np.ndarray, width: int, height: int) -> np.ndarray:
        source_height, source_width = rgb.shape[:2]
        if source_width == width and source_height == height:
            return rgb
        scale = min(width / source_width, height / source_height)
        resized_width = max(2, int(round(source_width * scale)))
        resized_height = max(2, int(round(source_height * scale)))
        interpolation = self.cv2.INTER_AREA if scale < 1.0 else self.cv2.INTER_LINEAR
        resized = self.cv2.resize(rgb, (resized_width, resized_height), interpolation=interpolation)
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        x_offset = (width - resized_width) // 2
        y_offset = (height - resized_height) // 2
        canvas[y_offset:y_offset + resized_height, x_offset:x_offset + resized_width] = resized
        return canvas

    def _send_encoded_stream(
        self,
        process: subprocess.Popen,
        connection: socket.socket,
        stop_event: threading.Event,
        error: list[Exception],
    ) -> None:
        parser = AnnexBAccessUnitParser()
        try:
            while True:
                chunk = process.stdout.read(65536)
                if not chunk:
                    break
                for access_unit in parser.feed(chunk):
                    connection.sendall(struct.pack(">I", len(access_unit)) + access_unit)
            for access_unit in parser.flush():
                connection.sendall(struct.pack(">I", len(access_unit)) + access_unit)
        except (OSError, AttributeError) as exc:
            error.append(exc)
            stop_event.set()
            with self._frame_condition:
                self._frame_condition.notify_all()

    def _stream_loop(self, request: CameraRequest, stop_event: threading.Event) -> None:
        connection: Optional[socket.socket] = None
        process: Optional[subprocess.Popen] = None
        reader: Optional[threading.Thread] = None
        reader_errors: list[Exception] = []
        try:
            self.node.get_logger().info(
                f"opening PICO video {request.ip}:{request.port}, "
                f"{request.width}x{request.height}@{request.fps}, "
                f"H.264 {request.bitrate} bit/s"
            )
            connection = socket.create_connection((request.ip, request.port), timeout=5.0)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(None)
            process = subprocess.Popen(
                self._ffmpeg_command(request),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
            reader = threading.Thread(
                target=self._send_encoded_stream,
                args=(process, connection, stop_event, reader_errors),
                daemon=True,
                name="pico_remote_vision_h264_sender",
            )
            reader.start()
            self.node.get_logger().info("PICO Remote Vision video streaming started")
            with self._frame_condition:
                last_sequence = self._latest_sequence
            last_sent_at = 0.0
            warned_at = time.monotonic() + 3.0
            while not stop_event.is_set() and not self._closing.is_set():
                with self._frame_condition:
                    self._frame_condition.wait_for(
                        lambda: (
                            stop_event.is_set()
                            or self._closing.is_set()
                            or self._latest_sequence != last_sequence
                        ),
                        timeout=0.5,
                    )
                    image_msg = self._latest_image
                    sequence = self._latest_sequence
                if stop_event.is_set() or self._closing.is_set():
                    break
                if image_msg is None or sequence == last_sequence:
                    if time.monotonic() >= warned_at:
                        self.node.get_logger().warn(
                            f"waiting for Remote Vision camera topic {self.camera_topic}"
                        )
                        warned_at = time.monotonic() + 3.0
                    continue
                last_sequence = sequence
                now = time.monotonic()
                if now - last_sent_at < 1.0 / request.fps:
                    continue
                try:
                    rgb = _ros_image_to_rgb(image_msg)
                    frame = self._resize_letterbox(rgb, request.width, request.height)
                    remaining = memoryview(frame.tobytes())
                    while remaining:
                        written = process.stdin.write(remaining)
                        if written is None or written <= 0:
                            raise BrokenPipeError("ffmpeg stdin stopped accepting frames")
                        remaining = remaining[written:]
                    last_sent_at = now
                except (BrokenPipeError, OSError) as exc:
                    reader_errors.append(exc)
                    break
                except Exception as exc:
                    self.node.get_logger().warn(f"skipping Remote Vision frame: {exc}")
            stop_event.set()
        except Exception as exc:
            if not self._closing.is_set() and not stop_event.is_set():
                self.node.get_logger().error(f"PICO Remote Vision stream failed: {exc}")
        finally:
            stop_event.set()
            if process is not None:
                if process.stdin is not None:
                    try:
                        process.stdin.close()
                    except OSError:
                        pass
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
                if reader is not None:
                    reader.join(timeout=2.0)
                stderr = b""
                if process.stderr is not None:
                    stderr = process.stderr.read().strip()
                if process.returncode not in (0, 255) and stderr and not self._closing.is_set():
                    self.node.get_logger().error(
                        "ffmpeg Remote Vision encoder stopped: "
                        + stderr.decode("utf-8", errors="replace")[-1000:]
                    )
            if connection is not None:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
            if reader_errors and not self._closing.is_set():
                self.node.get_logger().warn(
                    f"PICO Remote Vision video connection closed: {reader_errors[-1]}"
                )

    def close(self) -> None:
        if self._closing.is_set():
            return
        self._closing.set()
        try:
            self._server.close()
        except OSError:
            pass
        with self._control_lock:
            control_connection = self._control_connection
            self._control_connection = None
        if control_connection is not None:
            try:
                control_connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                control_connection.close()
            except OSError:
                pass
        self._stop_stream()
        self._listener.join(timeout=2.0)
