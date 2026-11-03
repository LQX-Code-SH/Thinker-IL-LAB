#!/usr/bin/env python3
"""数据集 action 回放脚本：把 LeRobot 数据集中某条 episode 的 action 原样发回 dex 真机。

参考 deploy/tienkung_pro/replay.py，针对 TienKung Dex（右臂 7 + 强脑 Revo2 右手 6，
共 13 维 action）：

用途：
  - 数据采集质量回看
  - Bridge2 / 手臂+灵巧手控制链路真机健康检查（不依赖模型推理）
  - 复现某条 episode 场景

action 布局（13 维，与 convert/tienkung_dex/configs/tienkung_dex_13d_1RGB_brainco_real.json
一致）：
  [0:7]   右臂关节角 rad（shoulder_pitch/roll/yaw, elbow_pitch/yaw, wrist_pitch/roll）
          -> /arm/cmd (ros2_bridge_msgs/ArmCtrl)，电机 ID 21~27
  [7:13]  右手闭合度 [0,1]（thumb_proximal, thumb_metacarpal, index/middle/ring/pinky_proximal）
          -> /right_hand/set_motor_multi (brainco_hand_msgs/SetMotorMulti)，
          通道码 = round(1 + v*999)，1=伸直 1000=弯曲（与 constants.hand_percent_to_code 一致）

在机器人（或 lerobot-tienkung 容器）内运行：
    source /opt/ros/humble/setup.bash
    python3 /ubt_IL/scripts/deploy/tienkung_dex/replay.py --episode 0 --rate 30

前置条件：ROS2 DDS 可达（真机 Bridge2 已启动）；建议先归位（home）再回放。
安全提示：回放会让真实机械臂运动，请确认作业空间无障碍物，随时 Ctrl-C 中断。
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

# ROS2 依赖延迟到真正发送时再导入，方便宿主机上跑 --dry-run / --help。
rclpy = None
Node = object

# ── TienKung Dex 真机编址（与 ubt_sim control/tienkung_dex/constants.py 一致）──

# 右臂电机 ID，顺序 = action[0:7]：
#   shoulder_pitch / shoulder_roll / shoulder_yaw / elbow_pitch / elbow_yaw
#   / wrist_pitch / wrist_roll
RIGHT_ARM_MOTOR_IDS = [21, 22, 23, 24, 25, 26, 27]
ARM_SPD = 0.5    # 真机 SDK 实测值：cur=40 才够扛重力
ARM_CUR = 40.0

# 强脑 Revo2 通道码值域
BRAINCO_CODE_MIN, BRAINCO_CODE_MAX = 1, 1000

# 右臂关节限位 (lower, upper) rad，顺序 = RIGHT_ARM_MOTOR_IDS
# （与 Bridge2 constants.BODY_JOINT_LIMITS 一致，超限裁剪后再下发）
RIGHT_ARM_LIMITS = [
    (-2.844887, 2.844887),   # shoulder_pitch
    (-3.333579, 0.191986),   # shoulder_roll
    (-2.844887, 2.844887),   # shoulder_yaw
    (-2.548181, 0.191986),   # elbow_pitch
    (-2.844887, 2.844887),   # elbow_yaw
    (-1.32645, 1.32645),     # wrist_pitch
    (-1.32645, 1.32645),     # wrist_roll
]

TOPIC_ARM_CMD = "/arm/cmd"
TOPIC_RIGHT_HAND_CMD = "/right_hand/set_motor_multi"

# 默认数据集：本仓库最近一次 brainco 真机转换产物
DEFAULT_DATASET = Path(__file__).resolve().parents[3] / "dataset" / "tienkung_dex_real_pick_apple_brainco_13d"


def _import_ros2():
    """容器内（source /opt/ros/humble/setup.bash 后）导入 ROS2 相关模块。"""
    global rclpy, Node
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from ros2_bridge_msgs.msg import ArmCtrl, MotorCtrl
    from brainco_hand_msgs.msg import SetMotorMulti
    globals().update({"rclpy": _rclpy, "Node": _Node})
    return {"ArmCtrl": ArmCtrl, "MotorCtrl": MotorCtrl, "SetMotorMulti": SetMotorMulti}


def hand_percent_to_code(percent: float) -> int:
    """闭合度 [0,1] -> 强脑通道码（取整并夹在值域内）。"""
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def load_episode_actions(dataset_root: Path, episode: int):
    """读取指定 episode 的 action 序列，返回 ((T,13) float32 数组, info dict)。"""
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"找不到数据元信息: {info_path}")
    with open(info_path) as f:
        info = json.load(f)

    parquet = dataset_root / "data" / "chunk-000" / "file-000.parquet"
    if not parquet.is_file():
        raise FileNotFoundError(f"找不到数据文件: {parquet}")

    table = pq.read_table(parquet, columns=["episode_index", "frame_index", "action"])
    df = table.to_pandas()
    ep = df[df["episode_index"] == episode].sort_values("frame_index")
    if ep.empty:
        raise ValueError(f"数据集中没有 episode_index={episode}")

    actions = np.asarray([np.asarray(a, dtype=np.float32) for a in ep["action"].tolist()])
    if actions.shape[1] != 13:
        raise ValueError(f"action 维度异常: 期望 13 (右臂7+右手6), 实际 {actions.shape[1]}")

    # 有 action.names 时按特征名核验布局，防止误放其它数据集
    names = info.get("features", {}).get("action", {}).get("names") or []
    if names:
        pattern = "".join(
            "H" if any(k in n for k in ("finger", "thumb", "hand",
                                        "index", "middle", "ring", "pinky")) else "A"
            for n in names
        )
        if pattern != "A" * 7 + "H" * 6:
            raise ValueError(
                f"action 布局不匹配 (期望 右臂7+右手6, pattern={pattern})：{names}"
            )
    return actions, info


class DexReplayNode:
    """ROS2 节点包装。__init__ 内才创建底层 Node，避免模块导入期触发 ROS2。"""

    def __init__(self, msgs: dict):
        self._node = Node("dex_action_replay_node")
        self._ArmCtrl = msgs["ArmCtrl"]
        self._MotorCtrl = msgs["MotorCtrl"]
        self._SetMotorMulti = msgs["SetMotorMulti"]
        self.arm_pub = self._node.create_publisher(self._ArmCtrl, TOPIC_ARM_CMD, 10)
        self.hand_pub = self._node.create_publisher(self._SetMotorMulti, TOPIC_RIGHT_HAND_CMD, 10)

    def publish_frame(self, action: np.ndarray) -> None:
        # 右臂：位置模式（mode=0），rad 目标 + 限幅裁剪 + 速度/电流限幅
        # （限幅行为与 Bridge2 ros2_deploy_bridge.py 一致）
        arm_msg = self._ArmCtrl()
        arm_msg.header.frame_id = "arm"
        arm_msg.mode = 0
        for motor_id, (low, high), val in zip(
                RIGHT_ARM_MOTOR_IDS, RIGHT_ARM_LIMITS, action[0:7]):
            arm_msg.ctrl.append(
                self._MotorCtrl(
                    name=int(motor_id),
                    pos=float(min(max(float(val), low), high)),
                    spd=ARM_SPD,
                    cur=ARM_CUR,
                )
            )
        self.arm_pub.publish(arm_msg)

        # 右手：闭合度 [0,1] -> 通道码 1~1000；speed/current/pwm/duration 0=默认
        hand_msg = self._SetMotorMulti()
        hand_msg.mode = 1
        codes = [hand_percent_to_code(v) for v in action[7:13]]
        hand_msg.positions = codes
        hand_msg.speeds = [0] * len(codes)
        hand_msg.currents = [0] * len(codes)
        hand_msg.pwms = [0] * len(codes)
        hand_msg.durations = [0] * len(codes)
        self.hand_pub.publish(hand_msg)

    def destroy(self):
        self._node.destroy_node()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dataset", type=Path, default=DEFAULT_DATASET,
                   help=f"LeRobot 数据集根目录（默认 {DEFAULT_DATASET}）")
    p.add_argument("--episode", type=int, default=0, help="要回放的 episode_index（默认 0）")
    p.add_argument("--rate", type=float, default=None,
                   help="发送频率 Hz（默认取数据集 info.json 的 fps）")
    p.add_argument("--start", type=int, default=0, help="起始帧索引（默认 0）")
    p.add_argument("--end", type=int, default=-1, help="结束帧索引，-1 表示到末尾（默认 -1）")
    p.add_argument("--lead-in", type=float, default=1.0,
                   help="回放前先发送首帧并保持的秒数，让手臂平滑到达起点（默认 1.0，0=关闭）")
    p.add_argument("--dry-run", action="store_true", help="只打印不发布")
    return p.parse_args()


def run_replay(args, actions: np.ndarray, rate: float) -> None:
    """按 rate 频率回放 actions[start:end]。"""
    start = max(0, args.start)
    end = len(actions) if args.end < 0 else min(args.end, len(actions))
    if start >= end:
        raise ValueError(f"帧范围非法: start={start} >= end={end}")

    period = 1.0 / rate
    node = None
    if args.dry_run:
        print(f"[replay] dry-run episode={args.episode} frames=[{start},{end}) total={end - start} rate={rate}Hz")
    else:
        msgs = _import_ros2()
        rclpy.init()
        node = DexReplayNode(msgs)
        time.sleep(1.0)  # 等 publisher 上线（沿用 deploy 其它脚本的做法）

    print(f"[replay] episode={args.episode} frames=[{start},{end}) total={end - start} rate={rate}Hz "
          f"dry_run={args.dry_run} arm_ids={RIGHT_ARM_MOTOR_IDS} topic_arm={TOPIC_ARM_CMD} "
          f"topic_hand={TOPIC_RIGHT_HAND_CMD}")

    last_printed = None

    def send(i: int) -> None:
        nonlocal last_printed
        a = actions[i]
        if args.dry_run:
            if i != last_printed:  # lead-in 期间重复发首帧，只打印一次
                codes = [hand_percent_to_code(v) for v in a[7:13]]
                print(f"frame {i} arm={np.array2string(a[0:7], precision=3)} "
                      f"hand[0,1]={np.array2string(a[7:13], precision=2)} "
                      f"hand_code={codes}")
                last_printed = i
        else:
            node.publish_frame(a)

    try:
        # 首帧保持 lead-in 秒，让手臂平滑运动到录制起点再开始逐帧回放
        if args.lead_in > 0:
            print(f"[replay] 首帧保持 {args.lead_in}s ...")
            deadline = time.monotonic() + args.lead_in
            while time.monotonic() < deadline:
                send(start)
                time.sleep(period)

        next_t = time.monotonic()
        for i in range(start, end):
            send(i)
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # 落后于节奏，重新对齐基准防止越拖越多
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[replay] 中断，停止发送。")
    finally:
        if node is not None:
            node.destroy()
            rclpy.shutdown()

    print(f"[replay] 完成。机器人停在第 {end - 1} 帧位置（不再下发保持指令）。")


def main():
    args = parse_args()
    try:
        actions, info = load_episode_actions(args.dataset, args.episode)
    except (FileNotFoundError, ValueError) as e:
        print(f"[replay] 错误: {e}", file=sys.stderr)
        sys.exit(1)

    rate = args.rate if args.rate is not None else float(info.get("fps", 30))
    run_replay(args, actions, rate)


if __name__ == "__main__":
    main()
