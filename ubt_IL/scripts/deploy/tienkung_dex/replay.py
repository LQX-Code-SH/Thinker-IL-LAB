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

运行流程：读当前实测位姿 → 平滑过渡到首帧 → 保持首帧 lead-in → 按 rate 逐帧回放。

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

# 右手 6 个驱动关节（action[7:13]）
HAND_DIM = 6

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

# 状态反馈（过渡用的起点）：/robot_state 的 RobotState.arm.status[]（name=电机ID, pos=rad）
# 右手 MotorStatus.positions = 通道码 1~1000（与下发同域，闭合度 = (code-1)/999）
TOPIC_ROBOT_STATE = "/robot_state"
TOPIC_RIGHT_HAND_STATE = "/right_hand/motor_status"

# 默认数据集：本仓库最近一次 brainco 真机转换产物
DEFAULT_DATASET = Path(__file__).resolve().parents[3] / "dataset" / "tienkung_dex_real_pick_apple_brainco_13d"


def _import_ros2():
    """容器内（source /opt/ros/humble/setup.bash 后）导入 ROS2 相关模块。"""
    global rclpy, Node
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from ros2_bridge_msgs.msg import ArmCtrl, MotorCtrl, RobotState
    from brainco_hand_msgs.msg import MotorStatus, SetMotorMulti
    globals().update({"rclpy": _rclpy, "Node": _Node})
    return {
        "ArmCtrl": ArmCtrl,
        "MotorCtrl": MotorCtrl,
        "SetMotorMulti": SetMotorMulti,
        "RobotState": RobotState,
        "MotorStatus": MotorStatus,
    }


def _status_qos(depth: int = 10):
    """状态订阅用 BEST_EFFORT：兼容 RELIABLE / BEST_EFFORT 两种发布端。

    默认（RELIABLE）订阅只收 RELIABLE 发布端；真机各节点 QoS 不由我们决定，
    一旦 /robot_state 是 BEST_EFFORT，就会"话题可见但收不到数"。
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def smoothstep(s: float) -> float:
    """五次平滑（quintic）：起止速度/加速度均为 0，避免端点抖动。"""
    s = min(max(float(s), 0.0), 1.0)
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


def hand_percent_to_code(percent: float) -> int:
    """闭合度 [0,1] -> 强脑通道码（取整并夹在值域内）。"""
    ratio = min(max(float(percent), 0.0), 1.0)
    return int(round(BRAINCO_CODE_MIN + ratio * (BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)))


def hand_code_to_percent(code: float) -> float:
    """强脑通道码 -> 闭合度 [0,1]（hand_percent_to_code 的逆运算）。"""
    clipped = min(max(float(code), BRAINCO_CODE_MIN), BRAINCO_CODE_MAX)
    return (clipped - BRAINCO_CODE_MIN) / float(BRAINCO_CODE_MAX - BRAINCO_CODE_MIN)


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

        # 状态缓存（过渡起点）：臂按电机 ID 取，手按通道码取
        self._arm_state = None   # list[7] rad，None = 尚未收到
        self._hand_state = None  # list[6] 通道码，None = 尚未收到
        qos = _status_qos()
        self._node.create_subscription(
            msgs["RobotState"], TOPIC_ROBOT_STATE, self._on_robot_state, qos
        )
        self._node.create_subscription(
            msgs["MotorStatus"], TOPIC_RIGHT_HAND_STATE, self._on_hand_state, qos
        )

    # -- 状态反馈 -------------------------------------------------------------

    def _on_robot_state(self, msg) -> None:
        """按电机 ID 取右臂 7 维（不按位取，缺 ID 时整帧丢弃）。"""
        status = getattr(msg, "arm", None)
        pos_by_id = {int(m.name): float(m.pos) for m in getattr(status, "status", [])}
        if all(mid in pos_by_id for mid in RIGHT_ARM_MOTOR_IDS):
            self._arm_state = [pos_by_id[mid] for mid in RIGHT_ARM_MOTOR_IDS]

    def _on_hand_state(self, msg) -> None:
        values = [float(v) for v in list(getattr(msg, "positions", []))[:HAND_DIM]]
        if len(values) >= HAND_DIM:
            self._hand_state = values

    def read_state(self, timeout: float):
        """等一帧状态，返回 (右臂7 rad | None, 右手6 闭合度 | None)。"""
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
            if self._arm_state is not None and self._hand_state is not None:
                break
        arm = list(self._arm_state) if self._arm_state is not None else None
        hand = (
            [hand_code_to_percent(c) for c in self._hand_state]
            if self._hand_state is not None else None
        )
        return arm, hand

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
    p.add_argument("--transition", type=float, default=3.0,
                   help="从当前实测位姿平滑过渡到首帧的秒数（0=跳过，直接发首帧）")
    p.add_argument("--state-timeout", type=float, default=2.0,
                   help="等 /robot_state 与右手状态首帧的超时秒数（超时则跳过过渡）")
    p.add_argument("--lead-in", type=float, default=1.0,
                   help="过渡到位后再保持首帧的秒数（默认 1.0，0=关闭）")
    p.add_argument("--dry-run", action="store_true", help="只打印不发布")
    return p.parse_args()


def run_transition(node, target: np.ndarray, duration: float, rate: float,
                   state_timeout: float) -> bool:
    """把当前实测位姿平滑过渡到首帧 target（13 维）。

    起点 = ``/robot_state`` 右臂 7 维 + 右手通道码换算的闭合度；端点 = 首帧 action。
    quintic 插值（起止速度/加速度连续）按 rate 逐帧下发。

    Returns:
        True = 已完成过渡；False = 拿不到位姿，调用方应直接发首帧。
    """
    arm_now, hand_now = node.read_state(state_timeout)
    if arm_now is None:
        print(f"[replay] 警告: {state_timeout:.1f}s 内未收到 {TOPIC_ROBOT_STATE}"
              f"（或缺少右臂电机 ID {RIGHT_ARM_MOTOR_IDS}），跳过过渡，直接发首帧")
        print("[replay] 如需强制过渡：确认桥接/真机已启动、ROS_DOMAIN_ID 正确。")
        return False
    if hand_now is None:
        print(f"[replay] 警告: 未收到 {TOPIC_RIGHT_HAND_STATE}，过渡期间手指保持首帧目标")
        hand_now = [float(v) for v in target[7:13]]

    begin = np.concatenate([np.asarray(arm_now, dtype=np.float32),
                            np.asarray(hand_now, dtype=np.float32)])
    goal = np.asarray(target, dtype=np.float32)
    diff = np.abs(goal - begin)
    print(f"[replay] 过渡 {duration:.1f}s：当前 -> 首帧，"
          f"最大臂差 {float(diff[:7].max()):.3f} rad，"
          f"最大闭合度差 {float(diff[7:].max()):.3f}")

    steps = max(1, int(round(duration * rate)))
    period = 1.0 / rate
    next_t = time.monotonic()
    for k in range(1, steps + 1):
        w = smoothstep(k / steps)
        node.publish_frame(begin + (goal - begin) * w)
        next_t += period
        sleep_for = next_t - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            # 落后于节奏，重新对齐基准防止越拖越多
            next_t = time.monotonic()
    return True


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
          f"dry_run={args.dry_run} transition={args.transition}s "
          f"arm_ids={RIGHT_ARM_MOTOR_IDS} topic_arm={TOPIC_ARM_CMD} "
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
        # ① 当前实测位姿 -> 首帧：先平滑过渡，避免从任意位姿直接跳变到录制起点
        if node is not None and args.transition > 0:
            run_transition(node, actions[start], args.transition, rate,
                           args.state_timeout)
        elif args.dry_run and args.transition > 0:
            print(f"[replay] dry-run：实际运行时先从当前位姿过渡 {args.transition:.1f}s "
                  "到首帧（此处未读取状态、未发布）")

        # ② 过渡到位后再保持首帧 lead-in 秒，让手臂稳定下来
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
