"""Configuration for DexCamera.

Registers as CameraConfig type "dex_camera" so that
``--robot.cameras.<key>.type=dex_camera`` works on the CLI.

**类型名必须与 tienkung 插件的 ``image_server`` 不同**：两个插件包都会被
``register_third_party_plugins()`` 自动 import，同名注册会撞 ChoiceRegistry。
"""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, ColorMode


@CameraConfig.register_subclass("dex_camera")
@dataclass
class DexCameraConfig(CameraConfig):
    """Configuration for a camera that receives frames over ZMQ as JPEG.

    两个来源共用这个相机，只是端口/地址不同：

    * **仿真**：ubt_sim 的 AsyncJpegCameraSender 直接在 5758 发 JPEG（带
      ``struct.pack('dI', ts, frame_id)`` 的 12 字节头）。零转发、零 cv2 侧依赖。
    * **真机**：没有这条链路，由 Bridge2 把 ``/ob_camera_head/color/image_raw``
      编成 JPEG 发到 5558（``--enable-image``）。

    Attributes:
        server_address: ZMQ 发送端地址。
        port: ZMQ 端口（仿真 5758 / 真机 5558）。
        offset_x: 拼接帧里本相机画面的水平起始像素（单目留 0）。
        display: 是否弹 cv2.imshow 实时预览。
        color_mode: 输出颜色格式（默认 BGR，与 cv2.imdecode 一致）。
        timeout_ms: 异步读取超时（毫秒）。
        warmup_s: 连接后等待首帧的秒数。
    """

    server_address: str = "127.0.0.1"
    port: int = 5558
    offset_x: int = 0
    display: bool = False
    color_mode: ColorMode = ColorMode.BGR
    timeout_ms: int = 5000
    warmup_s: int = 3
    width: int = 640
    height: int = 360
