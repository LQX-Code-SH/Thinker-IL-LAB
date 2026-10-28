#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位 tienkung_dex **仿真**场景（发布 ``/sim/cmd_reset``）。

在 lerobot-tienkung 容器内运行：

    source /opt/ros/humble/setup.bash
    ROS_DOMAIN_ID=146 /usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_dex/reset.py

真机没有这个复位话题（场景不存在）。真机的"复位"是 :file:`robot_ready.py --real`。

默认重复发 3 次：ubt_sim 侧的复位订阅端在仿真刚起时可能还没连上，单发一次会丢。
"""

from __future__ import annotations

import argparse
import sys
import time

DEFAULT_TOPIC = "/sim/cmd_reset"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="复位 tienkung_dex 仿真场景")
    p.add_argument("--topic", default=DEFAULT_TOPIC, help=f"复位话题（默认 {DEFAULT_TOPIC}）")
    p.add_argument("--repeat", type=int, default=3, help="重复发布次数（默认 3，兜底连接竞态）")
    p.add_argument("--interval", type=float, default=0.2, help="发布间隔秒（默认 0.2）")
    args = p.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init()
    node = Node("tienkung_dex_reset")
    try:
        pub = node.create_publisher(Bool, args.topic, 1)
        msg = Bool()
        msg.data = True
        # 先给 DDS 一点发现时间，否则前几次 publish 没有订阅者
        time.sleep(0.5)
        for _ in range(max(1, args.repeat)):
            pub.publish(msg)
            time.sleep(max(0.0, args.interval))
        print(f"[reset] 已发送 {max(1, args.repeat)} 次复位指令到 {args.topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
