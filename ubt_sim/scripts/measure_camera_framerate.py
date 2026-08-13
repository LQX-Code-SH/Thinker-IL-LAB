#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测量 sim 端相机真实帧率:消息速率 vs 唯一内容速率。

背景
----
`ros2 topic hz` 只数消息速率,不区分内容是否重复。tienkung_pro 的
`_send_camera_data` 每仿真步重发同一渲染缓冲(`render_interval=3` ⇒ 约 2/3 为
stale),导致 ~18.8Hz 消息里只有 ~6.3Hz 唯一内容。本脚本逐帧比对内容哈希,
直接给出"真实帧率"(唯一内容速率)、重复率,并估算 render_interval。

两种模式
--------
  --mode zmq   直连 sim ZMQ 5557(原始 multipart,绕过 C++ bridge / ROS2 DDS),
               定位 stale 重发是否起源于 sim 侧。【确诊 sim 端用,推荐先跑】
  --mode ros2  订阅 ROS2 /ob_camera_head/color/image_raw(sensor_msgs/Image),
               测量 recorder 实际接收到的内容。【对照 recorder 输入用】

判读
----
  - zmq 与 ros2 两种模式的"唯一速率"都远低于"消息速率" ⇒ stale 起源于 sim 端
    (未做 render_interval 门控),C++ bridge / recorder 无关。
  - est render_interval ≈ msg_rate / unique_rate ≈ 3.0 ⇒ 印证 render_interval=3。
  - 若 zmq 唯一速率高、ros2 唯一速率低 ⇒ 损耗在 C++ bridge / DDS 链路。

用法
----
  python3 scripts/measure_camera_framerate.py --mode zmq
  python3 scripts/measure_camera_framerate.py --mode ros2
  python3 scripts/measure_camera_framerate.py --mode zmq --duration 30 --interval 2
"""
import argparse
import hashlib
import time
from collections import deque


class FrameStats:
    """累计 + 滚动窗口帧统计。msg_times 记每条消息接收时刻,unique_times 记
    “内容与前帧不同”的消息接收时刻。两者按窗口截断即可算出各自速率。"""

    def __init__(self):
        self.msg_times: deque = deque()
        self.unique_times: deque = deque()
        self.prev_hash = None
        self.total_msgs = 0
        self.total_dups = 0

    def update(self, content: bytes) -> None:
        now = time.monotonic()
        self.total_msgs += 1
        self.msg_times.append(now)
        h = hashlib.md5(content).hexdigest()
        if h != self.prev_hash:
            self.unique_times.append(now)
            self.prev_hash = h
        else:
            self.total_dups += 1

    def window(self, secs: float) -> dict:
        cutoff = time.monotonic() - secs
        while self.msg_times and self.msg_times[0] < cutoff:
            self.msg_times.popleft()
        while self.unique_times and self.unique_times[0] < cutoff:
            self.unique_times.popleft()
        n_msg = len(self.msg_times)
        n_uniq = len(self.unique_times)
        msg_rate = n_msg / secs if secs > 0 else 0.0
        uniq_rate = n_uniq / secs if secs > 0 else 0.0
        dup_ratio = (n_msg - n_uniq) / n_msg if n_msg else 0.0
        est_ri = msg_rate / uniq_rate if uniq_rate > 0 else float("nan")
        return dict(n_msg=n_msg, n_uniq=n_uniq, msg_rate=msg_rate,
                    uniq_rate=uniq_rate, dup_ratio=dup_ratio, est_ri=est_ri)

    def summary(self, dur: float) -> dict:
        n_uniq = self.total_msgs - self.total_dups
        msg_rate = self.total_msgs / dur if dur > 0 else 0.0
        uniq_rate = n_uniq / dur if dur > 0 else 0.0
        dup_ratio = self.total_dups / self.total_msgs if self.total_msgs else 0.0
        est_ri = msg_rate / uniq_rate if uniq_rate > 0 else float("nan")
        return dict(total_msgs=self.total_msgs, total_dups=self.total_dups,
                    n_uniq=n_uniq, msg_rate=msg_rate, uniq_rate=uniq_rate,
                    dup_ratio=dup_ratio, est_ri=est_ri)


def _print_line(w: dict) -> None:
    print(f"  [msg] {w['msg_rate']:6.2f} Hz | [unique] {w['uniq_rate']:6.2f} Hz | "
          f"dups {w['n_msg'] - w['n_uniq']}/{w['n_msg']} ({w['dup_ratio'] * 100:.1f}%) | "
          f"est render_interval ≈ {w['est_ri']:.2f}")


def run_zmq(host: str, port: int, duration: float, interval: float, stats: FrameStats) -> float:
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 8)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(f"tcp://{host}:{port}")
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    print(f"[zmq] 连接 tcp://{host}:{port},等待首帧 ...")

    # 等首帧(最多 10s)
    t0 = time.monotonic()
    first = None
    while time.monotonic() - t0 < 10:
        ev = dict(poller.poll(timeout=1000))
        if sock in ev:
            first = sock.recv_multipart()
            break
    if first is None:
        print("[zmq] 10s 内未收到任何帧,请确认 sim 已启动且端口正确。")
        sock.close(linger=0)
        ctx.term()
        return 0.0
    if len(first) >= 2:
        stats.update(bytes(first[1]))
    print(f"[zmq] 收到首帧,开始统计 {duration}s ...")
    print("-" * 92)

    start = time.monotonic()
    next_report = start + interval
    try:
        while time.monotonic() - start < duration:
            ev = dict(poller.poll(timeout=1000))
            if sock in ev:
                parts = sock.recv_multipart()
                if len(parts) >= 2:          # parts[0]=meta json, parts[1]=rgb, parts[2]=depth
                    stats.update(bytes(parts[1]))
            if time.monotonic() >= next_report:
                _print_line(stats.window(interval))
                next_report = time.monotonic() + interval
    except KeyboardInterrupt:
        print("\n[zmq] 中断。")
    finally:
        sock.close(linger=0)
        ctx.term()
    return time.monotonic() - start


def run_ros2(topic: str, duration: float, interval: float, stats: FrameStats) -> float:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("measure_camera_framerate")

    def cb(msg):
        # msg.data 为 rgb8 原始字节;stale 重发 ⇒ 字节相同 ⇒ 哈希相同
        stats.update(bytes(msg.data))

    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    print(f"[ros2] 订阅 {topic} (sensor_msgs/Image),等待首帧 ...")

    t0 = time.monotonic()
    while stats.total_msgs == 0 and time.monotonic() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if stats.total_msgs == 0:
        print("[ros2] 10s 内未收到任何帧,请确认 C++ bridge 已发布该话题。")
        node.destroy_node()
        rclpy.shutdown()
        return 0.0
    print(f"[ros2] 收到首帧,开始统计 {duration}s ...")
    print("-" * 92)

    start = time.monotonic()
    next_report = start + interval
    try:
        while time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                _print_line(stats.window(interval))
                next_report = time.monotonic() + interval
    except KeyboardInterrupt:
        print("\n[ros2] 中断。")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        # Ctrl-C 会先触发 rclpy 信号处理调用 shutdown,这里再调会报
        # "rcl_shutdown already called";用 ok() 守卫避免重复关闭。
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return time.monotonic() - start


def main():
    ap = argparse.ArgumentParser(
        description="测量 sim 端相机真实帧率(消息速率 vs 唯一内容速率)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--mode", choices=["zmq", "ros2"], default="zmq",
                    help="zmq=直连 sim ZMQ 5557(定位 sim 端 stale 重发);"
                         "ros2=订阅 ROS2 话题(recorder 输入)")
    ap.add_argument("--host", default="127.0.0.1", help="sim ZMQ 主机(仅 zmq 模式)")
    ap.add_argument("--zmq-port", type=int, default=5557, help="sim ZMQ image 端口(仅 zmq 模式)")
    ap.add_argument("--ros-topic", default="/ob_camera_head/color/image_raw",
                    help="ROS2 相机话题(仅 ros2 模式)")
    ap.add_argument("--duration", type=float, default=20.0, help="统计时长(秒)")
    ap.add_argument("--interval", type=float, default=2.0, help="滚动窗口/报告间隔(秒)")
    args = ap.parse_args()

    stats = FrameStats()
    print(f"模式: {args.mode} | 统计 {args.duration}s | 每 {args.interval}s 报告")
    print("图例: [msg]=消息速率(ros2 topic hz 同口径)  [unique]=唯一内容速率(真实帧率)")
    print("      est render_interval ≈ msg/unique ≈ 3 ⇒ sim 端 render_interval=3 stale 重发")
    print("=" * 92)

    if args.mode == "zmq":
        dur = run_zmq(args.host, args.zmq_port, args.duration, args.interval, stats)
    else:
        dur = run_ros2(args.ros_topic, args.duration, args.interval, stats)

    if dur <= 0 or stats.total_msgs == 0:
        print("无数据,退出。")
        return

    s = stats.summary(dur)
    print("=" * 92)
    print(f"汇总 ({dur:.1f}s):")
    print(f"  总消息数:        {s['total_msgs']}")
    print(f"  重复消息数:      {s['total_dups']} ({s['dup_ratio'] * 100:.1f}%)")
    print(f"  消息速率:        {s['msg_rate']:.2f} Hz   (ros2 topic hz 同口径)")
    print(f"  真实帧率:        {s['uniq_rate']:.2f} Hz   (唯一内容速率)")
    print(f"  估算 render_int: ≈ {s['est_ri']:.2f}")
    print()
    if s["uniq_rate"] > 0 and s["msg_rate"] / s["uniq_rate"] > 2.0:
        print("⚠️  消息速率 >> 唯一内容速率 ⇒ 存在 stale 重发,真实帧率远低于消息速率。")
        print("    若 zmq 与 ros2 模式唯一速率都低 ⇒ stale 起源于 sim 端(未做 render_interval 门控)。")
    else:
        print("✅ 消息速率 ≈ 唯一内容速率 ⇒ 无 stale 重发。")


if __name__ == "__main__":
    main()
