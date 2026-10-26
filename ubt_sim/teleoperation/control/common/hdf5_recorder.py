"""通用 HDF5 单集录制核心（与机型无关）。

机型适配层只需按约定 key 逐帧调用 :meth:`EpisodeRecorder.add_frame`，
由本模块负责缓冲、图像编码与 HDF5 落盘，保证不同机型的数据结构一致：

```
joint_observations/{joint_position, joint_velocity, joint_effort}
hand_observations/{hand_left_position, ..., hand_right_effort}
action_observations/{joint_position, hand_left_position, hand_right_position}
camera_observations/images/<camera_name>          # PNG 编码字节流（vlen uint8）
camera_observations/depth_images/<camera_name>    # uint16 毫米，65535 无效
timestamp
```

约定 key：

* ``joint_position`` / ``joint_velocity`` / ``joint_effort``：1-D 数组
* ``hand_left_position`` / ``hand_right_position`` / ...：1-D 数组
* ``action_*``：1-D 数组
* ``png:<name>``：HxWx3 uint8 RGB
* ``depth:<name>``：HxW float32（米）或 uint16（毫米）
* ``timestamp``：标量
"""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np

DEPTH_SENTINEL = 65535  # uint16 最大值，表示无效深度

_JOINT_KEYS = ("joint_position", "joint_velocity", "joint_effort")
_HAND_KEYS = (
    "hand_left_position",
    "hand_left_velocity",
    "hand_left_effort",
    "hand_right_position",
    "hand_right_velocity",
    "hand_right_effort",
)
_ACTION_KEYS = ("joint_position", "hand_left_position", "hand_right_position")


def encode_rgb_png(rgb: np.ndarray) -> np.ndarray:
    """RGB → PNG 字节流（uint8 1-D）。"""
    ok, buf = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError("PNG 编码失败")
    return np.frombuffer(buf.tobytes(), dtype=np.uint8)


def to_depth_mm(depth: np.ndarray) -> np.ndarray:
    """深度统一为 uint16 毫米；输入为米制浮点时换算并裁剪。"""
    if depth.dtype == np.uint16:
        return depth
    depth_m = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    depth_mm = np.clip(depth_m * 1000.0, 0, DEPTH_SENTINEL).astype(np.uint16)
    depth_mm[depth_m <= 0.0] = DEPTH_SENTINEL
    return depth_mm


class EpisodeRecorder:
    """按帧缓冲，退出时一次性写入单个 HDF5 文件。"""

    def __init__(self) -> None:
        self._frames: dict[str, list[Any]] = OrderedDict()
        self._recording = False
        self._dropped = 0

    @property
    def recording(self) -> bool:
        return self._recording

    @property
    def num_frames(self) -> int:
        return len(self._frames.get("timestamp", [])) if self._frames else 0

    def start(self) -> None:
        self._frames = OrderedDict()
        self._recording = True

    def stop(self) -> None:
        self._recording = False

    def add_frame(self, payload: dict[str, Any]) -> bool:
        """写入一帧；未开始录制或 payload 缺少 timestamp 时丢弃。"""
        if not self._recording:
            return False
        if "timestamp" not in payload:
            self._dropped += 1
            return False
        for key, value in payload.items():
            if value is None:
                continue
            if key.startswith("png:"):
                value = encode_rgb_png(value)
            elif key.startswith("depth:"):
                value = to_depth_mm(value)
            self._frames.setdefault(key, []).append(value)
        return True

    def save(self, path: str | Path, attrs: dict[str, Any] | None = None) -> Path:
        """把缓冲写入 HDF5；返回实际路径。空缓冲会抛 ValueError。"""
        if not self._frames:
            raise ValueError("没有可保存的数据帧")

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        with h5py.File(path, "w") as f:
            for key, values in self._frames.items():
                _write_key(f, key, values)
            for name, value in (attrs or {}).items():
                f.attrs[name] = value
            f.attrs["num_frames"] = self.num_frames
            f.attrs["dropped_frames"] = self._dropped

        return path


def _write_key(handle: h5py.File, key: str, values: list[Any]) -> None:
    """按 key 约定写入对应 HDF5 分组。"""
    if key.startswith("png:"):
        group = handle.require_group("camera_observations/images")
        dataset = group.create_dataset(key[4:], shape=(len(values),), dtype=h5py.vlen_dtype(np.uint8))
        for i, value in enumerate(values):
            dataset[i] = value
        return

    if key.startswith("depth:"):
        group = handle.require_group("camera_observations/depth_images")
        group.create_dataset(key[6:], data=np.stack(values).astype(np.uint16), compression="gzip", compression_opts=4)
        return

    if key == "timestamp":
        handle.create_dataset("timestamp", data=np.asarray(values, dtype=np.float64))
        return

    if key in _JOINT_KEYS:
        group = handle.require_group("joint_observations")
        group.create_dataset(key, data=np.asarray(values, dtype=np.float32))
        return

    if key in _HAND_KEYS:
        group = handle.require_group("hand_observations")
        group.create_dataset(key, data=np.asarray(values, dtype=np.float32))
        return

    if key.startswith("action_"):
        group = handle.require_group("action_observations")
        name = key[len("action_"):]
        if name in _ACTION_KEYS:
            group.create_dataset(name, data=np.asarray(values, dtype=np.float32))
            return
        group.create_dataset(name, data=np.asarray(values, dtype=np.float32))
        return

    # 未知 key 直接放根节点，避免静默丢数据
    handle.create_dataset(key.replace("/", "_"), data=np.asarray(values))
