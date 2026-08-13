import cv2
import zmq
import time
import struct
from collections import deque
import numpy as np
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # pyrealsense2 未安装；RealSenseCamera 实例化时会抛清晰错误
import pyorbbecsdk as ob
import logging

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger_mp = logging.getLogger(__name__)


class RealSenseCamera(object):
    def __init__(self, img_shape, fps, serial_number=None, enable_depth=False) -> None:
        """
        img_shape: [height, width]
        serial_number: serial number
        """
        if rs is None:
            raise RuntimeError(
                "RealSenseCamera 需要 pyrealsense2，但未安装。"
                "请 pip install pyrealsense2，或将 camera_type 改为 'opencv'/'orbbec'。"
            )
        self.img_shape = img_shape
        self.fps = fps
        self.serial_number = serial_number
        self.enable_depth = enable_depth

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.init_realsense()

    def init_realsense(self):

        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_number is not None:
            config.enable_device(self.serial_number)

        config.enable_stream(rs.stream.color, self.img_shape[1], self.img_shape[0], rs.format.bgr8, self.fps)

        if self.enable_depth:
            config.enable_stream(rs.stream.depth, self.img_shape[1], self.img_shape[0], rs.format.z16, self.fps)

        profile = self.pipeline.start(config)
        self._device = profile.get_device()
        if self._device is None:
            logger_mp.error('[Image Server] pipe_profile.get_device() is None .')
        if self.enable_depth:
            assert self._device is not None
            depth_sensor = self._device.first_depth_sensor()
            self.g_depth_scale = depth_sensor.get_depth_scale()

        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if self.enable_depth:
            depth_frame = aligned_frames.get_depth_frame()

        if not color_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        depth_image = np.asanyarray(depth_frame.get_data()) if self.enable_depth else None
        return color_image, depth_image

    def release(self):
        self.pipeline.stop()


class OrbbecCamera(object):
    def __init__(self, img_shape, fps, device_index=None, enable_depth=False) -> None:
        """
        img_shape: [height, width]
        device_index: device index for Orbbec camera
        enable_depth: enable depth stream
        """
        self.img_shape = img_shape
        self.fps = fps
        self.device_index = device_index
        self.enable_depth = enable_depth
        self.init_orbbec()

    def init_orbbec(self):
        self.pipeline = ob.Pipeline()
        config = ob.Config()

        color_profiles = self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)

        color_profile = color_profiles.get_video_stream_profile(
            width=self.img_shape[1],
            height=self.img_shape[0],
            format=ob.OBFormat.BGR,
            fps=self.fps
        )
        config.enable_stream(color_profile)

        if self.enable_depth:
            depth_profiles = self.pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            depth_profile = depth_profiles.get_video_stream_profile(
                width=self.img_shape[1],
                height=self.img_shape[0],
                format=ob.OBFormat.Y16,
                fps=self.fps
            )
            config.enable_stream(depth_profile)

        self.pipeline.start(config)
        logger_mp.info(f"[Orbbec] Camera started: {self.img_shape[1]}x{self.img_shape[0]} @ {self.fps}fps")

    def get_frame(self):
        try:
            frameset = self.pipeline.wait_for_frames(2000)
        except Exception as e:
            logger_mp.warning(f"Orbbec wait_for_frames failed: {e}")
            return None, None

        if frameset is None:
            return None, None

        color_frame = frameset.get_color_frame()
        if color_frame is None:
            return None, None

        h, w = color_frame.get_height(), color_frame.get_width()
        color_bgr = np.asanyarray(color_frame.get_data()).reshape(h, w, 3)

        depth_image = None
        if self.enable_depth:
            depth_frame = frameset.get_depth_frame()
            if depth_frame is not None:
                dh, dw = depth_frame.get_height(), depth_frame.get_width()
                depth_image = np.asanyarray(depth_frame.get_data()).reshape(dh, dw)

        return color_bgr, depth_image

    def release(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()


class OpenCVCamera():
    def __init__(self, device_id, img_shape, fps):
        """
        decive_id: /dev/video* or *
        img_shape: [height, width]
        """
        self.id = device_id
        self.cap = cv2.VideoCapture(self.id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_shape[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Test if the camera can read frames
        if not self._can_read_frame():
            logger_mp.error(f"[Image Server] Camera {self.id} Error: Failed to initialize the camera or read frames. Exiting...")
            self.release()

    def _can_read_frame(self):
        success, _ = self.cap.read()
        return success

    def release(self):
        self.cap.release()

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        return frame, None


class ImageServer:
    def __init__(self, config, port = 5555, Unit_Test = False):
        """
        config example1:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv, realsense, or orbbec
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            'wrist_camera_type': 'realsense',
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': ["218622271789", "241222076627"],      # realsense camera's serial number
        }

        config example2:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'realsense',                                  # opencv, realsense, or orbbec
            'head_camera_image_shape': [480, 640],                            # Head camera resolution  [height, width]
            'head_camera_id_numbers': ["218622271739"],                       # realsense camera's serial number
            'wrist_camera_type': 'orbbec',
            'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            'wrist_camera_id_numbers': [0],                                   # orbbec camera index (0 for first device)
        }

        If you are not using the wrist camera, you can comment out its configuration, like this below:
        config:
        {
            'fps':30                                                          # frame per second
            'head_camera_type': 'opencv',                                     # opencv or realsense
            'head_camera_image_shape': [480, 1280],                           # Head camera resolution  [height, width]
            'head_camera_id_numbers': [0],                                    # '/dev/video0' (opencv)
            #'wrist_camera_type': 'realsense', 
            #'wrist_camera_image_shape': [480, 640],                           # Wrist camera resolution  [height, width]
            #'wrist_camera_id_numbers': ["218622271789", "241222076627"],      # serial number (realsense)
        }
        """
        logger_mp.info(config)
        self.fps = config.get('fps', 30)
        self.port = port
        self.Unit_Test = Unit_Test

        self.head_camera_type = config.get('head_camera_type', 'opencv')
        self.head_image_shape = config.get('head_camera_image_shape', [480, 640])
        self.head_camera_id_numbers = config.get('head_camera_id_numbers', [0])

        self.wrist_camera_type = config.get('wrist_camera_type', None)
        self.wrist_image_shape = config.get('wrist_camera_image_shape', [480, 640])
        self.wrist_camera_id_numbers = config.get('wrist_camera_id_numbers', None)

        self.head_cameras = []
        self.wrist_cameras = []

        self._init_head_cameras()
        self._init_wrist_cameras()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 10)
        self.socket.bind(f"tcp://*:{self.port}")

        if self.Unit_Test:
            self._init_performance_metrics()

        logger_mp.info("[Image Server] Image server has started, waiting for client connections...")


    def _init_head_cameras(self):
        for idx in self.head_camera_id_numbers:
            if self.head_camera_type == 'opencv':
                cam = OpenCVCamera(idx, self.head_image_shape, self.fps)
            elif self.head_camera_type == 'realsense':
                cam = RealSenseCamera(self.head_image_shape, self.fps, idx)
            elif self.head_camera_type == 'orbbec':
                cam = OrbbecCamera(self.head_image_shape, self.fps, idx)
            else:
                raise ValueError(f"Unsupported head_camera_type: {self.head_camera_type}")
            self.head_cameras.append(cam)

    def _init_wrist_cameras(self):
        if not self.wrist_camera_type or not self.wrist_camera_id_numbers:
            return

        for idx in self.wrist_camera_id_numbers:
            if self.wrist_camera_type == 'opencv':
                cam = OpenCVCamera(idx, self.wrist_image_shape, self.fps)
            elif self.wrist_camera_type == 'realsense':
                cam = RealSenseCamera(self.wrist_image_shape, self.fps, idx)
            elif self.wrist_camera_type == 'orbbec':
                cam = OrbbecCamera(self.wrist_image_shape, self.fps, idx)
            else:
                raise ValueError(f"Unsupported wrist_camera_type: {self.wrist_camera_type}")
            self.wrist_cameras.append(cam)

    def _init_performance_metrics(self):
        self.frame_count = 0  # Total frames sent
        self.time_window = 1.0  # Time window for FPS calculation (in seconds)
        self.frame_times = deque()  # Timestamps of frames sent within the time window
        self.start_time = time.time()  # Start time of the streaming

    def _update_performance_metrics(self, current_time):
        # Add current time to frame times deque
        self.frame_times.append(current_time)
        # Remove timestamps outside the time window
        while self.frame_times and self.frame_times[0] < current_time - self.time_window:
            self.frame_times.popleft()
        # Increment frame count
        self.frame_count += 1

    def _print_performance_metrics(self, current_time):
        if self.frame_count % 30 == 0:
            elapsed_time = current_time - self.start_time
            real_time_fps = len(self.frame_times) / self.time_window
            logger_mp.info(f"[Image Server] FPS: {real_time_fps:.2f}, Frames: {self.frame_count}, Time: {elapsed_time:.1f}s")

    def _close(self):
        for cam in self.head_cameras:
            cam.release()
        for cam in self.wrist_cameras:
            cam.release()
        self.socket.close()
        self.context.term()
        logger_mp.info("[Image Server] The server has been closed.")

    def send_process(self):
        try:
            while True:
                head_frames = []
                for cam in self.head_cameras:
                    color, _ = cam.get_frame()
                    if color is None:
                        logger_mp.error("[Image Server] Head camera frame failed.")
                        break
                    head_frames.append(color)

                if len(head_frames) != len(self.head_cameras):
                    time.sleep(0.01)
                    continue

                head_color = cv2.hconcat(head_frames)

                if self.wrist_cameras:
                    wrist_frames = []
                    for cam in self.wrist_cameras:
                        color, _ = cam.get_frame()
                        if color is None:
                            logger_mp.error("[Image Server] Wrist camera frame failed.")
                            break
                        wrist_frames.append(color)

                    if len(wrist_frames) != len(self.wrist_cameras):
                        time.sleep(0.01)
                        continue

                    wrist_color = cv2.hconcat(wrist_frames)

                    if head_color.shape[0] != wrist_color.shape[0]:
                        raise RuntimeError("Head/Wrist height mismatch")

                    full_color = cv2.hconcat([head_color, wrist_color])
                else:
                    full_color = head_color

                ret, buf = cv2.imencode('.jpg', full_color)
                if not ret:
                    continue

                msg = buf.tobytes()
                if self.Unit_Test:
                    msg = struct.pack('dI', time.time(), self.frame_count) + msg
                    self._update_performance_metrics(time.time())
                    self._print_performance_metrics(time.time())

                self.socket.send(msg)

        except KeyboardInterrupt:
            logger_mp.info("[Image Server] Interrupted by user.")
        finally:
            self._close()


if __name__ == "__main__":
    config = {
        'fps': 30,
        'head_camera_type': 'orbbec',
        'head_camera_image_shape': [480, 640],
        'head_camera_id_numbers': [0],  # 相机索引
    }

    server = ImageServer(config, Unit_Test=False)
    server.send_process()