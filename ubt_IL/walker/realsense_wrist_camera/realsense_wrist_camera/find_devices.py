"""Device discovery CLI tool — also auto-generates config file."""

import argparse
import json
import sys
from pathlib import Path

from ._common import resolve_topic, topic_to_frame_id


def main():
    parser = argparse.ArgumentParser(
        description="Discover Intel RealSense cameras and generate config"
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Output devices as JSON",
    )
    parser.add_argument(
        "--no-save", action="store_true",
        help="Do not auto-generate config file",
    )
    parser.add_argument(
        "--config-dir",
        help="Directory for generated config (default: package dir/configs)",
    )
    parser.add_argument(
        "--config-name", default="wrist_cameras.json",
        help="Filename for generated config (default: wrist_cameras.json)",
    )
    parser.add_argument(
        "--msg-type", default="shm_msgs/Image1m",
        help="ROS2 message type for config (default: shm_msgs/Image1m)",
    )
    parser.add_argument(
        "--width", type=int, default=640,
        help="Frame width (default: 640)",
    )
    parser.add_argument(
        "--height", type=int, default=480,
        help="Frame height (default: 480)",
    )
    parser.add_argument(
        "--fps", type=int, default=60,
        help="Frame rate (default: 60)",
    )
    args = parser.parse_args()

    try:
        from realsense_wrist_camera.driver import RealSenseD405Driver
    except ImportError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        print("Install with: pip install pyrealsense2", file=sys.stderr)
        sys.exit(1)

    devices = RealSenseD405Driver.discover()

    if not devices:
        print("No RealSense devices detected.", file=sys.stderr)
        print("Check: USB connection, udev rules, and kernel modules.",
              file=sys.stderr)
        sys.exit(1)

    # Print discovery results
    if args.json:
        print(json.dumps(devices, indent=2))
    else:
        for i, dev in enumerate(devices):
            topic = resolve_topic(i)
            print(f"[{i}] {dev['name']}  →  {topic}")
            print(f"    Serial:  {dev['serial']}")
            print(f"    USB:     {dev['usb_type']}")
            print(f"    FW:      {dev['firmware']}")
            print()

    # Auto-generate config file
    if args.no_save:
        return

    config_dir = args.config_dir
    if config_dir is None:
        # Default: package's own configs directory
        pkg_dir = Path(__file__).resolve().parent.parent
        config_dir = pkg_dir / "configs"
    else:
        config_dir = Path(config_dir)

    config_dir.mkdir(parents=True, exist_ok=True)
    config_path = config_dir / args.config_name

    cameras_cfg = []
    for i, dev in enumerate(devices):
        topic = DEFAULT_WRIST_TOPICS[i] if i < len(DEFAULT_WRIST_TOPICS) else f"/camera/realsense_{i}"
        frame_id = topic_to_frame_id(topic, i)
        cameras_cfg.append({
            "serial": dev["serial"],
            "topic": topic,
            "msg_type": args.msg_type,
            "frame_id": frame_id,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
        })

    config = {"cameras": cameras_cfg}

    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
        f.write("\n")

    print(f"Config written to: {config_path}")
    print(f"  Start service:   realsense-wrist-camera --config {config_path}")


if __name__ == "__main__":
    main()
