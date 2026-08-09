"""Shared constants and utilities used by CLI and config tools."""

# Default ROS2 topics for Walker S2 wrist cameras (right first — USB enumeration
# typically discovers right wrist before left on the Jetson).
DEFAULT_WRIST_TOPICS = [
    "/sensor/camera/wrist_right/color/raw",
    "/sensor/camera/wrist_left/color/raw",
]


def resolve_topic(index: int) -> str:
    """Map a 0-based camera index to a wrist topic.

    Indices 0 and 1 map to the standard right/left wrist topics.
    Additional cameras get a generic ``/camera/realsense_{index}`` topic.
    """
    if index < len(DEFAULT_WRIST_TOPICS):
        return DEFAULT_WRIST_TOPICS[index]
    return f"/camera/realsense_{index}"


def topic_to_frame_id(topic: str, index: int = 0) -> str:
    """Extract a reasonable frame_id from a ROS2 topic path.

    /sensor/camera/wrist_left/color/raw -> wrist_left
    /test/camera                        -> camera_0
    """
    parts = topic.strip("/").split("/")
    if len(parts) >= 3 and parts[0] in ("sensor", "camera"):
        return parts[2]
    return f"camera_{index}"
