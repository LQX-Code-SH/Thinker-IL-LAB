#!/usr/bin/env python3
"""
Walker S2 sim 直接 ZMQ 相机通道客户端（绕过 ROS bridge 的 6MB 丢帧瓶颈）。

背景
----
- sim 侧 AsyncCameraSender 在 5657 端口 PUB，每相机一个 multipart：
      send_json({width, height, format:"raw", camera:name}, SNDMORE)
      send(rgb8 原始字节, SNDMORE)
      send(b"")                        # 空 depth 占位
- 原链路经 C++ bridge 转 shm_msgs/Image6m 走 DDS(BEST_EFFORT + UDP 分片)，stereo_right
  实测重复率 ~44%。本模块作为 5657 的第二个 SUB 订阅者（ZMQ PUB 天然支持多订阅者，
  per-connection 独立队列，bridge 慢不拖累采集），走 TCP loopback 自带重传，物理不丢。
- 约束：不介入仿真渲染线程、sim 侧零改动、不做任何编码——本模块只收 raw 字节 reshape。

接口
----
对齐 Camera / CameraShmClient 的 recorder 侧接口：
    _get_latest_frame() -> _Frame | None     # img 为 raw rgb8 ndarray (H, W, 3)
    wait_for_image(timeout) -> bool
    is_available() -> bool
    cleanup()

用法
----
    from utils.zmq_camera_client import ZMQCameraSource
    source = ZMQCameraSource(camera_names=list(CAMERA_TOPICS.keys()), port=5657)
    cam = source.get_client("stereo_left")
    frame = cam._get_latest_frame()
    ...
    source.cleanup()
"""

import json
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

try:
    import zmq
except ImportError as _e:  # pragma: no cover
    zmq = None
    _ZMQ_IMPORT_ERROR = _e

try:
    from utils.camera import _Frame
except ImportError:  # 直接以脚本方式运行时
    from camera import _Frame

# 与 walker_s2_bridge_config.yaml 的 zmq_image_port 一致
DEFAULT_ZMQ_IMAGE_PORT = 5657
_DEFAULT_HOST = "127.0.0.1"


class ZMQCameraSource:
    """单一 SUB 订阅 5657，按 meta['camera'] 分路维护"最新未消费"队列。

    一个 SUB socket 收一遍全部四路流，避免每相机一个 socket 造成 4 倍收包量。
    队列 + 消费游标（见 _ZMQCameraClient._get_latest_frame）消除拍频重复：
    只要 capture 频率 ≤ 渲染频率（如 16 ≤ 17Hz），每次 poll 都能取到新帧，
    recorder 侧 _fresh_frames 全部命中、duplicated ≈ 0%。
    """

    def __init__(
        self,
        camera_names,
        port: int = DEFAULT_ZMQ_IMAGE_PORT,
        host: str = _DEFAULT_HOST,
    ):
        if zmq is None:
            raise ImportError(f"pyzmq not available: {_ZMQ_IMPORT_ERROR}")
        self.camera_names = list(camera_names)
        self._port = port
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.RCVHWM, 8)  # 与 bridge 一致；丢最旧帧无妨
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")  # 订阅全部相机
        self._sock.connect(f"tcp://{host}:{port}")
        self._lock = threading.Lock()
        # 每相机"最新未消费"队列：poll 取队列最新帧且 ts > 已消费游标 -> 零重复。
        # maxlen=8 缓冲渲染抖动，溢出丢最旧（宁可丢旧不丢新）。
        self._queues = {name: deque(maxlen=8) for name in self.camera_names}
        self._consumed_ts = {name: 0.0 for name in self.camera_names}
        self._running = True
        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="zmq-cam-source",
        )
        self._thread.start()
        self._clients = {
            name: _ZMQCameraClient(self, name) for name in self.camera_names
        }

    # ------------------------------------------------------------------
    # 接收线程
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        """重组 multipart（meta + rgb + 空 depth），按 camera 分路写入缓存。"""
        while self._running:
            try:
                parts = self._sock.recv_multipart()
            except zmq.ZMQError:
                if self._running:
                    time.sleep(0.001)
                continue
            if len(parts) < 2:  # 协议异常，跳过
                continue
            try:
                meta = json.loads(parts[0])
                name = meta.get("camera")
                if name not in self._queues:
                    continue
                height = int(meta["height"])
                width = int(meta["width"])
                rgb = np.frombuffer(parts[1], dtype=np.uint8)
                if rgb.size != height * width * 3:
                    continue
                rgb = rgb.reshape((height, width, 3))
                # ts 用接收时刻：loopback 延迟 <1ms；消费游标只要求 ts 单调递增
                frame = _Frame(
                    img=rgb,
                    ts=time.time(),
                    height=height,
                    width=width,
                    encoding="rgb8",  # sim 发 raw RGB8，recorder 据此做 RGB2BGR
                    step=width * 3,
                    frame_id=name,
                )
                with self._lock:
                    self._queues[name].append(frame)
            except Exception:
                continue

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def get_client(self, name: str):
        """按相机名取 recorder 兼容视图（接口对齐 Camera/CameraShmClient）。"""
        if name not in self._clients:
            raise KeyError(f"unknown camera: {name}")
        return self._clients[name]

    def cleanup(self) -> None:
        """停止接收线程并释放 socket/context。"""
        self._running = False
        try:
            self._sock.close(linger=0)
            self._ctx.term()
        except Exception:
            pass
        self._thread.join(timeout=2.0)


class _ZMQCameraClient:
    """per-camera 视图：共享 source 的缓存，暴露 recorder 需要的接口。"""

    def __init__(self, source: ZMQCameraSource, name: str):
        self._source = source
        self.name = name

    def _get_latest_frame(self) -> Optional[_Frame]:
        """返回最新未消费帧；无新帧返回 None（recorder 复用上一帧）。

        消费游标语义：即使队列深处还有积压，也只推进到当前最新帧，保证
        capture 频率 ≤ 渲染频率时每次 poll 都能拿到新帧（零重复）。
        """
        with self._source._lock:
            q = self._source._queues.get(self.name)
            if not q:
                return None
            frame = q[-1]  # 最新帧
            if frame.ts <= self._source._consumed_ts[self.name]:
                return None  # 最新帧已被消费过
            self._source._consumed_ts[self.name] = frame.ts
            return frame

    def wait_for_image(self, timeout: float = 5.0) -> bool:
        """阻塞等待第一帧到达（覆盖 ZMQ SUB 慢启动：连接建立前 PUB 消息被丢弃）。"""
        start = time.time()
        while time.time() - start < timeout:
            if self.is_available():
                return True
            time.sleep(0.01)
        return False

    def is_available(self) -> bool:
        with self._source._lock:
            return bool(self._source._queues.get(self.name))

    def cleanup(self) -> None:
        pass  # 统一由 ZMQCameraSource.cleanup() 清理
