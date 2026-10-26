#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位 tienkung_dex 仿真场景（发布 /sim/cmd_reset）。

用法::

    /usr/bin/python3 teleoperation/control/tienkung_dex/reset.py [--repeat 3] [--interval 0.2]
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

RESET_TOPIC = "/sim/cmd_reset"


def main() -> int:
    parser = argparse.ArgumentParser(description="Reset tienkung_dex simulation")
    parser.add_argument("--repeat", type=int, default=3, help="重复发布次数（ZMQ SUB 连接的启动竞态兜底）")
    parser.add_argument("--interval", type=float, default=0.2, help="发布间隔（秒）")
    args = parser.parse_args()

    rclpy.init()
    node = Node("tienkung_dex_reset")
    pub = node.create_publisher(Bool, RESET_TOPIC, 1)
    msg = Bool()
    msg.data = True

    for _ in range(max(1, args.repeat)):
        pub.publish(msg)
        time.sleep(args.interval)

    node.get_logger().info(f"已发送复位指令到 {RESET_TOPIC}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
