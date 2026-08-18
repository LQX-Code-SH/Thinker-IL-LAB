"""Small adapter around XRoboToolkit; intentionally independent of GMR."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    try:
        from .pico_hand_tracking import HandTrackingData
    except ImportError:
        from pico_hand_tracking import HandTrackingData


@dataclass(frozen=True)
class PicoFrame:
    headset_pose: Any
    left_controller_pose: Any
    right_controller_pose: Any
    controls: dict
    timestamp_ns: int
    # Optional and default-off so the established controller teleoperation
    # frame contract remains backward compatible.
    left_hand: HandTrackingData | None = None
    right_hand: HandTrackingData | None = None


class PicoSource:
    def __init__(self, hand_tracking_enabled: bool = False, tracking_to_output=None):
        try:
            import xrobotoolkit_sdk as sdk
        except ImportError as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk is not installed for this Python; install the cp310 wheel "
                "from xgmr_tmp/pico/pico_teleop/deps first"
            ) from exc
        self.sdk = sdk
        self.hand_tracking_enabled = bool(hand_tracking_enabled)
        self.tracking_to_output = tracking_to_output
        self._parse_hand_tracking_state = None
        if self.hand_tracking_enabled:
            try:
                from .pico_hand_tracking import parse_hand_tracking_state
            except ImportError:
                from pico_hand_tracking import parse_hand_tracking_state
            self._parse_hand_tracking_state = parse_hand_tracking_state
            required = (
                "get_left_hand_tracking_state",
                "get_right_hand_tracking_state",
                "get_left_hand_is_active",
                "get_right_hand_is_active",
            )
            missing = [name for name in required if not hasattr(sdk, name)]
            if missing:
                raise RuntimeError(
                    "installed xrobotoolkit_sdk does not expose hand tracking: "
                    + ", ".join(missing)
                    + "; install/rebuild XRoboToolkit-PC-Service-Pybind with "
                    "26-joint hand APIs"
                )
        self.sdk.init()

    def close(self) -> None:
        self.sdk.close()

    def _read_hands(
        self, timestamp_ns: int
    ) -> tuple[HandTrackingData, HandTrackingData]:
        if not self.hand_tracking_enabled:
            raise RuntimeError("hand tracking was not enabled for this PicoSource")
        sdk = self.sdk
        parse_hand_tracking_state = self._parse_hand_tracking_state
        left_hand = parse_hand_tracking_state(
            "left",
            sdk.get_left_hand_tracking_state(),
            sdk.get_left_hand_is_active(),
            timestamp_ns,
            self.tracking_to_output,
        )
        right_hand = parse_hand_tracking_state(
            "right",
            sdk.get_right_hand_tracking_state(),
            sdk.get_right_hand_is_active(),
            timestamp_ns,
            self.tracking_to_output,
        )
        return left_hand, right_hand

    def read_hands(self) -> tuple[HandTrackingData, HandTrackingData]:
        """Read only hand APIs, allowing operation with no controllers paired."""
        timestamp_ns = int(self.sdk.get_time_stamp_ns())
        return self._read_hands(timestamp_ns)

    def read(self) -> PicoFrame:
        sdk = self.sdk
        controls = {
            "LeftController": {
                "index_trig": float(sdk.get_left_trigger()),
                "grip": float(sdk.get_left_grip()),
                "key_one": bool(sdk.get_X_button()),
                "key_two": bool(sdk.get_Y_button()),
                "axis": sdk.get_left_axis(),
                "axis_click": bool(sdk.get_left_axis_click()),
            },
            "RightController": {
                "index_trig": float(sdk.get_right_trigger()),
                "grip": float(sdk.get_right_grip()),
                "key_one": bool(sdk.get_A_button()),
                "key_two": bool(sdk.get_B_button()),
                "axis": sdk.get_right_axis(),
                "axis_click": bool(sdk.get_right_axis_click()),
            },
        }
        timestamp_ns = int(sdk.get_time_stamp_ns())
        left_hand = None
        right_hand = None
        if self.hand_tracking_enabled:
            left_hand, right_hand = self._read_hands(timestamp_ns)
        return PicoFrame(
            headset_pose=sdk.get_headset_pose(),
            left_controller_pose=sdk.get_left_controller_pose(),
            right_controller_pose=sdk.get_right_controller_pose(),
            controls=controls,
            timestamp_ns=timestamp_ns,
            left_hand=left_hand,
            right_hand=right_hand,
        )


class MockPicoSource:
    """Deterministic, small PICO motions for cloud/simulator validation.

    Right A is held in every frame, so the regular deadman and anchor path are
    exercised.  This source is intentionally unavailable for ``mode=real``.
    """

    def __init__(self):
        self.started = time.monotonic()

    def close(self) -> None:
        pass

    @staticmethod
    def _pose(position, quaternion=(0.0, 0.0, 0.0, 1.0)):
        return [*position, *quaternion]

    def read(self) -> PicoFrame:
        elapsed = time.monotonic() - self.started
        phase = min(max(elapsed - 1.0, 0.0), 8.0)
        envelope = math.sin(math.pi * phase / 8.0) ** 2
        lateral = 0.035 * envelope * math.sin(0.7 * elapsed)
        vertical = 0.025 * envelope * math.sin(0.9 * elapsed)
        forward = 0.040 * envelope * math.sin(0.5 * elapsed)
        # Source axes follow Unity: +x right, +y up, +z forward.
        left = self._pose((-0.28 - lateral, 1.20 + vertical, 0.42 + forward))
        right = self._pose((0.28 + lateral, 1.20 + vertical, 0.42 + forward))
        yaw = 0.12 * envelope * math.sin(0.4 * elapsed)
        headset = self._pose((0.0, 1.65, 0.0), (0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)))
        trigger = 0.5 * envelope * (1.0 + math.sin(1.2 * elapsed))
        controls = {
            "LeftController": {
                "index_trig": trigger,
                "grip": 0.0,
                "key_one": False,
                "key_two": False,
                "axis": [0.0, 0.0],
                "axis_click": False,
            },
            "RightController": {
                "index_trig": trigger,
                "grip": 0.0,
                "key_one": True,
                "key_two": False,
                "axis": [0.0, 0.0],
                "axis_click": False,
            },
        }
        return PicoFrame(headset, left, right, controls, time.time_ns())
