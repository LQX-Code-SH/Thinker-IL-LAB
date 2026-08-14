#!/usr/bin/env python3
"""
Walker S2 相机图像订阅与解码模块

从 gr00t_inference.py + image_processor.py 提取，封装 shm_msgs 共享内存图像的
订阅、深拷贝、buffer 管理、encoding 解析、像素解码全流程，与 VLA 推理逻辑解耦。

支持的消息类型：
  - shm_msgs/Image2m（默认，当前生产用）
  - shm_msgs/Image1m、Image4m 等其他尺寸
  - sensor_msgs/Image（标准 ROS2 图像，兼容模式）

【运行前置条件】

1. 确保 ROS2 环境已 source（包含 shm_msgs）：

    source /home/ubt/additional/scripts/setup.sh

2. 机器人相机话题正在发布（如 /sensor/camera/stereo/color/raw）

【使用示例】

    # Python API — 实例方式（订阅 + 自动解码）：
    from camera import Camera
    import rclpy, threading
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    cam = Camera(topic="/sensor/camera/stereo/color/raw")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(cam)
    threading.Thread(target=executor.spin, daemon=True).start()

    cam.wait_for_image(timeout=5.0)
    img = cam.get_latest_image()          # numpy array (H, W, 3)
    info = cam.get_image_info()           # {'height': ..., 'encoding': ..., ...}

    # Python API — 静态方式（直接解码已有消息）：
    img = Camera.decode_image(msg)        # 对任意 shm_msgs/Image* 或 sensor_msgs/Image 消息解码

    # 命令行：
    python3 camera.py                                          # 持续打印帧信息
    python3 camera.py --preview                                # cv2.imshow 本地窗口预览 (按 Q 或 ESC 退出)
    python3 camera.py --save --count 5                         # 保存 5 帧 PNG
    python3 camera.py --topic /sensor/camera/stereo/color/raw  # 指定话题
"""

import argparse
import copy
import os
import threading
import time
from typing import Optional

import yaml
from collections import deque, namedtuple

import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ============================================================================
# 常量
# ============================================================================

DEFAULT_TOPIC = "/sensor/camera/stereo/color/raw"
DEFAULT_BUFFER_SIZE = 2
DEFAULT_BRIDGE_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, os.pardir, "bridges", "walker_s2_bridge_config.yaml")

# shm_msgs/String 的 char[256] 数组长度
_SHM_STRING_MAX_SIZE = 256

# 已知 encoding 前缀列表，用于截断末尾乱码字符
# 当 shm_msgs/String 的 encoding 字段含尾部垃圾数据时，
# 若字符串以列表中的 encoding 名开头，则截取该名还原
# (列表内不存在互为前缀的项，首个 startswith 命中即正确)
_KNOWN_ENCODINGS = [
    "bgr8", "rgb8", "bgra8", "rgba8",
    "mono8", "mono16",
    "yuv422", "yuyv422", "uyvy422",
    "16UC1", "16SC1", "32FC1", "32SC1",
    "8UC1", "8UC3", "8UC4",
    "8SC1", "8SC3", "8SC4",
]

# 已解码帧的统一结构。Camera._get_latest_frame 返回 img=ndarray(源 encoding/dtype);
# camera_worker.CameraShmClient 返回 img=JPEG bytes。recorder 据 isinstance 分支处理。
_Frame = namedtuple(
    "_Frame", ["img", "ts", "height", "width", "encoding", "step", "frame_id"]
)


class Camera(Node):
    """Walker S2 相机图像订阅与解码节点。

    封装 shm_msgs 共享内存图像的订阅、深拷贝、buffer 管理、encoding 解析、
    像素解码全流程。既可作为 ROS2 节点实例化使用（订阅话题 + 自动解码），
    也可直接使用静态方法对已有消息解码。

    Parameters
    ----------
    topic : str
        相机图像话题名。
    msg_type : type
        消息类型，默认 shm_msgs.msg.Image2m。也支持其他 shm_msgs/Image* 或
        sensor_msgs.msg.Image。
    buffer_size : int
        图像 buffer 容量（保留最近 N 帧）。
    node_name : str
        ROS2 节点名。
    """

    def __init__(
        self,
        topic: Optional[str] = None,
        msg_type=None,
        config_path: Optional[str] = None,
        buffer_size: int = DEFAULT_BUFFER_SIZE,
        node_name: str = "camera_node",
    ):
        super().__init__(node_name)

        if topic is None:
            topic = self._default_topic_from_config(config_path)

        # Walker sim and real deployment use shm_msgs/Image2m；sensor_msgs/Image is a legacy fallback.
        if msg_type is None:
            from shm_msgs.msg import Image2m
            msg_type = Image2m

        self._msg_type = msg_type
        self._topic = topic

        # 线程安全的图像 buffer
        self._buffer: deque = deque(maxlen=buffer_size)
        self._buffer_lock = threading.Lock()

        # 传感器话题 QoS：必须 BEST_EFFORT + VOLATILE，否则收不到消息
        qos_sub_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 订阅相机话题
        self._sub = self.create_subscription(
            msg_type=msg_type,
            topic=topic,
            callback=self._callback,
            qos_profile=qos_sub_sensor,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f"Camera initialized: topic='{topic}', "
            f"msg_type={msg_type.__name__}, buffer_size={buffer_size}"
        )


    @staticmethod
    def _default_topic_from_config(config_path: Optional[str] = None) -> str:
        config_path = config_path or DEFAULT_BRIDGE_CONFIG
        try:
            with open(config_path) as f:
                cfg = yaml.safe_load(f) or {}
            return cfg["topics"]["pub"]["image_rgb"]["topic"]
        except (FileNotFoundError, KeyError, TypeError):
            return DEFAULT_TOPIC

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _callback(self, msg):
        """话题回调：深拷贝 → 存入 buffer。"""
        cloned = self._clone_msg(msg)
        if cloned is None:
            self.get_logger().warning("Failed to clone image message, skipping")
            return
        with self._buffer_lock:
            self._buffer.append(cloned)

    # ------------------------------------------------------------------
    # 深拷贝
    # ------------------------------------------------------------------

    @staticmethod
    def _clone_msg(msg):
        """深拷贝图像消息。

        shm_msgs 的图像消息底层是共享内存映射，回调返回后 DDS 可能覆盖底层内存，
        因此必须深拷贝后才能存入 buffer。

        对 shm_msgs 消息（encoding 字段为 char 数组）：逐字段手动复制。
        对标准 sensor_msgs/Image（encoding 为普通 string）：使用 copy.deepcopy。
        """
        # 判断是否为 shm_msgs 消息：encoding 字段有 .data 属性（char 数组）
        is_shm = hasattr(msg, 'encoding') and hasattr(msg.encoding, 'data')

        if is_shm:
            return Camera._clone_shm_msg(msg)
        else:
            try:
                return copy.deepcopy(msg)
            except Exception:
                return None

    @staticmethod
    def _clone_shm_msg(msg):
        """逐字段深拷贝 shm_msgs 图像消息。"""
        try:
            new_msg = type(msg)()  # 用原始类型构造空消息
            new_msg.header.stamp.sec = msg.header.stamp.sec
            new_msg.header.stamp.nanosec = msg.header.stamp.nanosec
            new_msg.header.frame_id = msg.header.frame_id
            new_msg.height = msg.height
            new_msg.width = msg.width
            new_msg.encoding = msg.encoding
            new_msg.is_bigendian = msg.is_bigendian
            new_msg.step = msg.step
            new_msg.data = msg.data
            return new_msg
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Encoding 解析
    # ------------------------------------------------------------------

    @staticmethod
    def resolve_encoding(msg) -> str:
        """将消息的 encoding 字段解析为标准 Python string。

        shm_msgs/String 类型：char[256] 数组 → 拼接非零字符 → 若以已知 encoding 名开头，截断其后乱码。
        标准 string：直接返回。

        Parameters
        ----------
        msg : 图像消息对象

        Returns
        -------
        str
            标准 encoding 字符串，如 'bgr8', 'rgb8', 'yuv422' 等。
        """
        raw = msg.encoding

        # shm_msgs/String：char[256] 数组
        if hasattr(raw, 'data'):
            encoding = ''.join(chr(c) for c in raw.data if c != 0)
        else:
            encoding = str(raw)

        # 截断尾部乱码：匹配已知 encoding 前缀
        for known in _KNOWN_ENCODINGS:
            if encoding.startswith(known):
                encoding = known
                break

        return encoding

    # ------------------------------------------------------------------
    # 解码
    # ------------------------------------------------------------------

    @staticmethod
    def yuv422_to_bgr(yuv_data, width, height, order="UYVY"):
        """将 YUV422 原始字节转换为 BGR numpy 数组。

        Parameters
        ----------
        yuv_data : bytes
            YUV422 编码的原始像素数据。
        width : int
            图像宽度（像素）。
        height : int
            图像高度（像素）。
        order : str
            色度采样顺序，'YUYV' 或 'UYVY'。

        Returns
        -------
        numpy.ndarray
            BGR 格式图像，形状 (height, width, 3)，dtype uint8。
        """
        if isinstance(yuv_data, np.ndarray):
            yuv = yuv_data.reshape((height, width // 2, 4))
        else:
            yuv = np.frombuffer(yuv_data, dtype=np.uint8).reshape((height, width // 2, 4))
        y = np.zeros((height, width), dtype=np.uint8)
        u = np.zeros((height, width // 2), dtype=np.uint8)
        v = np.zeros((height, width // 2), dtype=np.uint8)

        if order == "YUYV":
            y[:, 0::2] = yuv[:, :, 0]
            u[:, :] = yuv[:, :, 1]
            y[:, 1::2] = yuv[:, :, 2]
            v[:, :] = yuv[:, :, 3]
        elif order == "UYVY":
            u[:, :] = yuv[:, :, 0]
            y[:, 0::2] = yuv[:, :, 1]
            v[:, :] = yuv[:, :, 2]
            y[:, 1::2] = yuv[:, :, 3]
        else:
            raise ValueError(f"Unsupported YUV422 order: {order}")

        u_full = np.repeat(u, 2, axis=1)
        v_full = np.repeat(v, 2, axis=1)
        yuv_img = cv2.merge((y, u_full, v_full))
        return cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)

    @staticmethod
    def decode_image(msg, encoding=None):
        """将 shm_msgs/Image* 或 sensor_msgs/Image 消息解码为 numpy 数组。

        支持的 encoding：bgr8, rgb8, mono8, yuv422, 16UC1, 32FC1。

        Parameters
        ----------
        msg : 图像消息
            shm_msgs/Image* 或 sensor_msgs/Image 消息对象。
        encoding : str, optional
            指定输出色彩空间。如传入 'bgr8'，则 rgb8 图像会自动转换为 bgr8。
            默认 None 表示按原始 encoding 输出。

        Returns
        -------
        numpy.ndarray
            解码后的图像数组。

        Raises
        ------
        ValueError
            encoding 不受支持时。
        """
        height = msg.height
        width = msg.width
        step = msg.step
        src_encoding = Camera.resolve_encoding(msg)
        byte_count = height * step

        # uint8 像素路径: 用 numpy 视图替代 bytes() 全量拷贝。
        # shm_msgs 的 data 是固定大小数组(如 uint8[6291456]), 但有效像素只有
        # height*step(320x240x3≈230KB)。rclpy 固定数组字段 .data 是 numpy ndarray,
        # ravel() 对连续数组返回视图(零拷贝); 非 ndarray 时退回 np.frombuffer。
        # 只取前 byte_count 个元素。
        if src_encoding in ("bgr8", "rgb8", "mono8", "yuv422"):
            raw = msg.data
            data = raw.ravel() if isinstance(raw, np.ndarray) else np.frombuffer(raw, dtype=np.uint8)
            view = data[:byte_count]  # 视图, 不拷贝

            if src_encoding == "bgr8":
                img = view.reshape((height, width, 3))
            elif src_encoding == "rgb8":
                img = view.reshape((height, width, 3))
                if encoding == "bgr8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif src_encoding == "mono8":
                img = view.reshape((height, width))
            else:  # yuv422
                img = Camera.yuv422_to_bgr(view, width, height, order="UYVY")
        else:
            img_data = bytes(msg.data)  # 16UC1/32FC1 等罕见路径保持原样
            if src_encoding == "16UC1":
                img = np.frombuffer(img_data, dtype=np.uint16)[:byte_count].reshape((height, width))
            elif src_encoding == "32FC1":
                img = np.frombuffer(img_data, dtype=np.float32)[:byte_count].reshape((height, width))
            else:
                raise ValueError(f"Unsupported encoding: {src_encoding}")

        return img

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    def wait_for_image(self, timeout: float = 5.0) -> bool:
        """阻塞等待第一帧图像到达。

        Parameters
        ----------
        timeout : float
            最大等待时间（秒）。

        Returns
        -------
        bool
            True 表示在超时前收到图像，False 表示超时。
        """
        start = time.time()
        while time.time() - start < timeout:
            with self._buffer_lock:
                if len(self._buffer) > 0:
                    return True
            time.sleep(0.01)
        return False

    def get_latest_image(self, encoding=None):
        """获取最新帧的解码 numpy 数组。

        Parameters
        ----------
        encoding : str, optional
            指定输出色彩空间（如 'bgr8'）。默认 None 按原始 encoding 输出。

        Returns
        -------
        numpy.ndarray or None
            解码后的图像数组，无数据时返回 None。
        """
        msg = self.get_latest_msg()
        if msg is None:
            return None
        try:
            return self.decode_image(msg, encoding=encoding)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return None

    def get_latest_msg(self):
        """获取最新帧的原始 ROS 消息。

        Returns
        -------
        消息对象 or None
            最新帧的图像消息，无数据时返回 None。
        """
        with self._buffer_lock:
            if len(self._buffer) > 0:
                return self._buffer[-1]
        return None

    def _get_latest_frame(self):
        """返回最新帧的 _Frame(img=已解码 ndarray, ts=header.stamp, ...)。

        一次取已解码图像 + 元信息,消除 recorder 原 get_latest_image() + get_image_info()
        的双锁双取,且去重命中时下游不再白 decode 一帧。保持源 dtype(深度 16UC1/32FC1
        不被压成 uint8)。供 recorder / camera_worker 共用 _Frame 接口。
        """
        with self._buffer_lock:
            msg = self._buffer[-1] if len(self._buffer) > 0 else None
        if msg is None:
            return None
        try:
            img = np.ascontiguousarray(self.decode_image(msg))  # 保持源 dtype
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return None
        frame_id = msg.header.frame_id
        if hasattr(frame_id, 'data'):
            frame_id = ''.join(chr(c) for c in frame_id.data if c != 0)
        return _Frame(
            img=img,
            ts=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            height=msg.height,
            width=msg.width,
            encoding=self.resolve_encoding(msg),
            step=msg.step,
            frame_id=frame_id,
        )

    def get_image_info(self):
        """获取最新帧的元信息。

        Returns
        -------
        dict or None
            包含 height, width, encoding, step, timestamp, frame_id 的字典，
            无数据时返回 None。
        """
        msg = self.get_latest_msg()
        if msg is None:
            return None

        encoding = self.resolve_encoding(msg)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # frame_id 可能是 shm_msgs/String 或标准 string
        frame_id = msg.header.frame_id
        if hasattr(frame_id, 'data'):
            frame_id = ''.join(chr(c) for c in frame_id.data if c != 0)

        return {
            "height": msg.height,
            "width": msg.width,
            "encoding": encoding,
            "step": msg.step,
            "timestamp": timestamp,
            "frame_id": frame_id,
        }

    def is_available(self) -> bool:
        """buffer 中是否有图像数据。"""
        with self._buffer_lock:
            return len(self._buffer) > 0


# ============================================================================
# CLI 入口
# ============================================================================

def cmd_print_info(cam, save=False, preview=False, no_print=False, count=0, interval=1.0):
    """持续打印/保存/预览图像信息。

    preview 模式使用 cv2.imshow 实时显示相机画面，按 Q 或 ESC 退出。
    """
    saved = 0
    shown = 0

    if preview:
        cv2.namedWindow("Camera Preview", cv2.WINDOW_NORMAL)
        print("  📷 Preview started — press Q or ESC in the preview window to exit\n")

    try:
        while rclpy.ok():
            if not cam.is_available():
                time.sleep(0.1)
                continue

            info = cam.get_image_info()
            if info is None:
                time.sleep(0.1)
                continue

            # 打印帧信息
            if not no_print:
                print(
                    f"[{time.strftime('%H:%M:%S')}] "
                    f"{info['width']}x{info['height']} "
                    f"encoding={info['encoding']} "
                    f"step={info['step']} "
                    f"ts={info['timestamp']:.3f}"
                )

            # cv2.imshow 实时预览
            if preview:
                img = cam.get_latest_image(encoding="bgr8")
                if img is not None:
                    cv2.imshow("Camera Preview", img)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):  # ESC / Q 退出
                        print("Preview closed.")
                        break
                    shown += 1
                    if count > 0 and shown >= count:
                        print(f"  Previewed {shown} frames, done.")
                        break

            # 保存为 PNG
            if save:
                img = cam.get_latest_image(encoding="bgr8")
                if img is not None:
                    filename = f"camera_frame_{info['timestamp']:.3f}.png"
                    cv2.imwrite(filename, img)
                    print(f"  → Saved: {filename}")
                    saved += 1
                    if count > 0 and saved >= count:
                        print(f"  Saved {saved} frames, done.")
                        break

            # 同时开启 preview 和 save 时，count 对两者各自生效；正常由上方各分支
            # 单独 break，此兜底条件需两者都达到 count 才退出
            if count > 0:
                if (not preview or shown >= count) and (not save or saved >= count):
                    break

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if preview:
            cv2.destroyAllWindows()


def main(args=None):
    parser = argparse.ArgumentParser(description="Walker S2 相机图像订阅与解码工具")
    parser.add_argument(
        "--topic", type=str, default=None,
        help=f"相机图像话题 (default: bridge config or {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--msg-type", type=str, default="Image2m",
        choices=["Image8k", "Image512k", "Image1m", "Image2m", "Image4m", "Image6m", "Image8m",
                 "sensor_msgs/Image"],
        help="图像消息类型 (default: Image2m / shm_msgs.msg.Image2m)",
    )
    parser.add_argument(
        "--preview", action="store_true",
        help="cv2.imshow 实时预览 (按 Q 或 ESC 退出)",
    )
    parser.add_argument(
        "--no-print", action="store_true",
        help="不打印帧信息（所有模式生效）",
    )
    parser.add_argument(
        "--save", action="store_true",
        help="保存图像为 PNG",
    )
    parser.add_argument(
        "--count", type=int, default=0,
        help="保存/预览帧数 (0=无限, default: 0)",
    )
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="打印/保存间隔秒数 (default: 1.0)",
    )

    args = parser.parse_args(args)

    # 解析消息类型
    if args.msg_type == "sensor_msgs/Image":
        from sensor_msgs.msg import Image as MsgType
    else:
        import shm_msgs.msg
        MsgType = getattr(shm_msgs.msg, args.msg_type)

    rclpy.init()
    cam = Camera(topic=args.topic, msg_type=MsgType)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(cam)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"Waiting for image on topic: {args.topic} ...")
    if not cam.wait_for_image(timeout=10.0):
        print(f"Timeout: no image received on '{args.topic}' after 10s")
        print("Check that the camera topic is publishing and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp is set.")
        cam.destroy_node()
        rclpy.shutdown()
        return

    mode = "preview" if args.preview else ("save" if args.save else "info display")
    print(f"Image received! Starting {mode}...\n")
    cmd_print_info(cam, save=args.save, preview=args.preview,
                   no_print=args.no_print, count=args.count, interval=args.interval)

    cam.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
