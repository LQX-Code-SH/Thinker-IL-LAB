#!/usr/bin/env python3
"""Reset Walker C1 between rollout episodes.

Two reset concepts (same split as ubt_sim):

* ``--mode sim-scene``: publish ``std_msgs/Bool(True)`` on ``/sim/cmd_reset`` so
  the ubt_sim scene teleports the robot to home and re-places the object.
  No-op on real hardware (the topic does not exist there).
* ``--mode task``: drive the arms/hands to the task-ready pose by delegating to
  ``robot_ready.py`` (three-segment interpolation, hands open at 0 rad).

Usage:
  python3 reset.py --mode sim-scene
  python3 reset.py --mode task
  python3 reset.py --mode task --hold-s 1.0
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RESET_TOPIC = "/sim/cmd_reset"


def reset_sim_scene(topic: str) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init()
    node = Node("walker_c1_reset")
    pub = node.create_publisher(Bool, topic, 10)
    msg = Bool()
    msg.data = True
    # 连发几帧，避免 DDS discovery 尚未完成导致首帧丢失
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print(f"[reset] published Bool(True) x5 on {topic}")
    return 0


def reset_task_pose(extra_args: list[str]) -> int:
    script = SCRIPT_DIR / "robot_ready.py"
    if not script.is_file():
        print(f"[reset] robot_ready.py not found: {script}", file=sys.stderr)
        return 1
    print(f"[reset] delegating task-ready pose to {script}")
    return subprocess.call([sys.executable, str(script), *extra_args])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["sim-scene", "task", "both"], default="sim-scene",
                        help="重置方式：sim-scene（仅仿真场景）/ task（预备位）/ both")
    parser.add_argument("--reset-topic", default=DEFAULT_RESET_TOPIC, help="仿真复位话题")
    parser.add_argument("--hold-s", default="0.5", help="摆位完成后保持时间（传给 robot_ready.py）")
    args, extra = parser.parse_known_args()

    code = 0
    if args.mode in ("sim-scene", "both"):
        code = reset_sim_scene(args.reset_topic)
    if code == 0 and args.mode in ("task", "both"):
        code = reset_task_pose(["--hold-s", str(args.hold_s), *extra])
    return code


if __name__ == "__main__":
    raise SystemExit(main())
