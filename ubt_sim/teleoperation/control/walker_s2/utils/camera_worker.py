#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 相机多进程隔离模块。

把高开销相机(stereo Image6m, 6MB/帧)的订阅 + 解码 + JPEG 编码移到独立子进程,
避免其 deepcopy/decode 持续占用主进程 GIL 从而饿死 wrist 相机回调(根因见
recorder-camera-undersampling-fix.md:stereo 6MB 双重拷贝把 wrist_left 挤到 3Hz)。

主进程通过 SharedMemory + 序号读取子进程产出的 JPEG bytes,recorder 统一按 bytes 存储。

组件:
  - CameraWorker:    主进程创建/管理,子进程执行(rclpy 订阅 + decode + imencode + 写 shm)
  - CameraShmClient: 主进程读取端,供 recorder 取帧(非 Node,暴露 _get_latest_frame)
  - create_camera_source: 工厂,返回 (worker, client)

子进程用 spawn 启动(规避 fork-after-threads + rclpy 死锁);SharedMemory 以 pid 命名,
finally + atexit 兜底 unlink 防残留。
"""

import os
import struct
import multiprocessing as mp
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .camera import Camera, _Frame

# SharedMemory 布局: 20B header + JPEG payload
# header: uint32 len | float64 ts | uint32 height | uint32 width  (struct '<IdII')
_HEADER_FMT = "<IdII"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)  # 20
_DEFAULT_MAX_PAYLOAD = 1 << 20  # 1MB(JPEG 320x480 q95 ~30-50KB,余量充足)
_STALL_WARN_THRESH = 30         # seq 连续不变 N 次告警(子进程可能崩溃/卡住)


def _resolve_msg_type(msg_type_name: str):
    """字符串 'Image6m' -> shm_msgs.msg.Image6m 类。在子进程内调用。"""
    import shm_msgs.msg
    return getattr(shm_msgs.msg, msg_type_name)


def _to_bgr(img: np.ndarray, encoding: str) -> np.ndarray:
    """把 decode_image 的输出统一成 BGR(uint8, 3 通道),供 cv2.imencode。

    decode_image 对 rgb8 返回 RGB、bgr8/yuv422 返回 BGR、mono8 返回 2D 灰度。
    imencode 期望 BGR,故 rgb8 需 RGB2BGR,灰度需 GRAY2BGR。
    与原 recorder save_data 的 `cvtColor(RGB2BGR)+imencode` 产出一致的 BGR JPEG。
    """
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # bgr8 / yuv422 -> 已是 BGR
    return np.ascontiguousarray(img, dtype=np.uint8)


def _run_subprocess(name, topic, msg_type_name, shm_name, shm_size, seq, lock, jpeg_quality, stop_event):
    """子进程入口(模块级,便于 spawn pickle):独立 rclpy context 订阅 -> decode -> imencode -> 写 shm。

    stop_event 由主进程 set 触发优雅退出(spin_once 轮询),避免 SIGTERM 致
    ExternalShutdownException + 二次 shutdown 噪声。
    """
    from multiprocessing import shared_memory

    shm = shared_memory.SharedMemory(name=shm_name)  # attach,子进程只 close 不 unlink
    try:
        # spawn 子进程是全新解释器,默认 context 干净且与主进程隔离(不同进程);
        # 直接 init 默认 context,Node/Executor 不传 context 即用它。
        rclpy.init()
        node = Node(f"cam_worker_{name}")
        msg_type = _resolve_msg_type(msg_type_name)
        qos = QoSProfile(
            # depth=50: 6MB 消息反序列化偶发超过发布周期(~51ms)时,
            # 大缓冲吸收突发积压, 显著减少过载丢帧(原 10 在 6MB 话题上丢帧~12%)
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cb_group = MutuallyExclusiveCallbackGroup()

        def cb(msg):
            try:
                img = Camera.decode_image(msg)
                src_enc = Camera.resolve_encoding(msg)
                img = _to_bgr(img, src_enc)
                ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
                if not ok:
                    return
                data = enc.tobytes()
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                h, w = img.shape[0], img.shape[1]
                with lock:
                    if _HEADER_SIZE + len(data) > shm_size:
                        return  # 超容量,跳过(不应发生)
                    struct.pack_into(_HEADER_FMT, shm.buf, 0, len(data), ts, h, w)
                    shm.buf[_HEADER_SIZE:_HEADER_SIZE + len(data)] = data
                    seq.value += 1
            except Exception:
                pass

        node.create_subscription(msg_type, topic, cb, qos, callback_group=cb_group)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            # 轮询 stop_event 优雅退出;SIGTERM 仅作 stop() 兜底
            # timeout 0.01: 事件驱动, 不影响消息处理延迟, 只加快 stop 响应
            while not stop_event.is_set():
                executor.spin_once(timeout_sec=0.01)
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            executor.shutdown()
            try:
                rclpy.shutdown()
            except Exception:
                pass  # 可能已被信号处理程序 shutdown
    finally:
        shm.close()


class CameraWorker:
    """管理一个相机子进程:子进程订阅话题 -> decode -> JPEG -> 写 SharedMemory。"""

    def __init__(self, name, topic, msg_type_name, shm_name, shm_size, seq, lock,
                 stop_event, jpeg_quality=95, start_method="spawn"):
        self.name = name
        self.topic = topic
        self.msg_type_name = msg_type_name
        self.shm_name = shm_name
        self.shm_size = shm_size
        self.seq = seq              # mp.Value('Q')
        self.lock = lock            # mp.Lock()
        self.stop_event = stop_event  # mp.Event
        self.jpeg_quality = int(jpeg_quality)
        self._ctx = mp.get_context(start_method)
        self._process: Optional[mp.Process] = None

    def start(self):
        self._process = self._ctx.Process(
            target=_run_subprocess,
            args=(self.name, self.topic, self.msg_type_name, self.shm_name,
                  self.shm_size, self.seq, self.lock, self.jpeg_quality, self.stop_event),
            name=f"cam-{self.name}", daemon=True,
        )
        self._process.start()

    def stop(self):
        if self._process is None:
            return
        # 优先优雅退出:set event 让子进程跳出 spin_once 循环,自行 destroy/shutdown
        if self.stop_event is not None:
            self.stop_event.set()
        if self._process.is_alive():
            self._process.join(timeout=3.0)
            if self._process.is_alive():
                self._process.terminate()  # 兜底 SIGTERM
                self._process.join(timeout=1.0)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(timeout=1.0)
        self._process = None

    def is_alive(self) -> bool:
        return self._process is not None and self._process.is_alive()


class CameraShmClient:
    """主进程读取端:从 SharedMemory 读子进程产出的 JPEG bytes,返回 _Frame(非 Node)。

    暴露 _get_latest_frame / wait_for_image / is_available,供 recorder / pick_part
    像主进程 Camera 一样使用(img 字段为 JPEG bytes 而非 ndarray)。
    """

    def __init__(self, shm, seq, lock, name):
        self.shm = shm              # SharedMemory(create=True) 句柄
        self.seq = seq
        self.lock = lock
        self.name = name
        self._last_seq = -1
        self._cached: Optional[_Frame] = None
        self._stall = 0

    def _get_latest_frame(self) -> Optional[_Frame]:
        cur = self.seq.value
        if cur == self._last_seq:
            self._stall += 1
            if self._stall == _STALL_WARN_THRESH:
                print(f"[CameraShmClient] WARNING: camera '{self.name}' stalled "
                      f"(seq unchanged for {_STALL_WARN_THRESH} polls, subprocess may have crashed)")
            return self._cached
        with self.lock:
            length, ts, h, w = struct.unpack_from(_HEADER_FMT, self.shm.buf, 0)
            if length <= 0 or _HEADER_SIZE + length > self.shm.size:
                return self._cached
            img = bytes(self.shm.buf[_HEADER_SIZE:_HEADER_SIZE + length])
        self._cached = _Frame(img=img, ts=ts, height=int(h), width=int(w),
                              encoding="jpeg", step=0, frame_id="")
        self._last_seq = cur
        self._stall = 0
        return self._cached

    def wait_for_image(self, timeout: float = 5.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            if self.seq.value > 0:
                return True
            time.sleep(0.01)
        return False

    def is_available(self) -> bool:
        return self.seq.value > 0

    def cleanup(self):
        """关闭并释放 SharedMemory（unlink）。主进程退出前调用，防 /dev/shm 残留。"""
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass


def create_camera_source(topic, msg_type_name, name, jpeg_quality=95,
                         max_payload=_DEFAULT_MAX_PAYLOAD, start_method="spawn"):
    """工厂:创建 SharedMemory + 序号 + 锁,返回 (CameraWorker, CameraShmClient)。

    调用方负责 worker.start() 与最终 worker.stop()/cleanup()。
    """
    from multiprocessing import shared_memory

    ctx = mp.get_context(start_method)
    shm_name = f"walker_s2_cam_{name}_{os.getpid()}"
    # 清理同名残留(上次崩溃未 unlink)
    try:
        old = shared_memory.SharedMemory(name=shm_name)
        old.close()
        old.unlink()
    except FileNotFoundError:
        pass
    shm = shared_memory.SharedMemory(create=True, size=_HEADER_SIZE + max_payload, name=shm_name)
    seq = ctx.Value("Q", 0)
    lock = ctx.Lock()
    stop_event = ctx.Event()
    worker = CameraWorker(
        name=name, topic=topic, msg_type_name=msg_type_name,
        shm_name=shm_name, shm_size=shm.size, seq=seq, lock=lock,
        stop_event=stop_event, jpeg_quality=jpeg_quality, start_method=start_method,
    )
    client = CameraShmClient(shm=shm, seq=seq, lock=lock, name=name)
    return worker, client
