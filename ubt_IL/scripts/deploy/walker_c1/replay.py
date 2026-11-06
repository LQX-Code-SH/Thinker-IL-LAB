#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数据集 action 回放：把 LeRobot 数据集某条 episode 的 action 原样发回 Walker C1 真机。

参考 deploy/tienkung_dex/replay.py，按 C1 的真机寻址改写：

    body  -> /mc/sdk/robot_command        (mc_task_msgs/RobotCommand, JointCmd 按关节名 + 位置模式)
    hands -> /mc/{left,right}_hand/command (mc_task_msgs/JointCommand, mode=command_mode)
    状态  -> /mc/sdk/robot_state + /mc/*_hand/joint_states（过渡起点）

关节名、限位、手部 command_mode 全部取自部署配置
``configs/walker_c1_{13d,26d}.json``（与推理链路同一份 SSOT）：

* action 维度 13 → walker_c1_13d.json，26 → walker_c1_26d.json，也可 --config 显式指定；
* **只下发 action 里出现的关节**（按名匹配，13 维不会碰左臂/左手），
  名字里的 ``_target`` 后缀会先剥掉再查表；
* 下发前按配置里的 limits 裁剪（手部按 ``thumb_swing`` 这类后缀匹配）。

运行流程：等状态 → 五次平滑过渡到首帧 → 保持首帧 lead-in → 按 rate 逐帧回放。

用法（机器人或 lerobot-tienkung 容器内；系统 python3 带 rclpy + pyarrow）：::

    source /opt/ros/humble/setup.bash
    source /ubt_IL/walker_s2/walker_sdk_ros2/install/setup.bash   # mc_task_msgs / mc_state_msgs
    python3 /ubt_IL/scripts/deploy/walker_c1/replay.py --list
    python3 /ubt_IL/scripts/deploy/walker_c1/replay.py --episode 3 --dry-run
    python3 /ubt_IL/scripts/deploy/walker_c1/replay.py --episode 3 --rate 30

（注意：不 source walker_sdk_ros2 会报 ``No module named 'mc_state_msgs'``。
``--dry-run`` / ``--list`` 不需要 ROS，宿主机也能跑。）

安全提示：回放会让真实机械臂运动。先确认作业空间无障碍物，建议先 --dry-run，
并先用 robot_ready.sh 把手臂摆到采集起始位附近，随时 Ctrl-C 中断。
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATASET = SCRIPT_DIR.parents[2] / "dataset" / "walker_c1_real_13d_1RGB"
CONFIG_DIR = SCRIPT_DIR / "configs"
STATE_SUFFIX = "_target"
DEFAULT_HAND_MODE = 5
DEFAULT_TOPICS = {
    "body_cmd": "/mc/sdk/robot_command",
    "body_state": "/mc/sdk/robot_state",
    # 兜底状态源：主控 50 Hz 全关节 JointState。实测 /mc/sdk/robot_state 在机器人
    # 重启/未重新切 SDK 模式时会静默（publisher 存在但 0 Hz），此时用这个算过渡起点，
    # 避免「拿不到起点 → 直接跳首帧」的突跳。
    "joint_state": "/mc/joint_states",
    "left_hand_cmd": "/mc/left_hand/command",
    "right_hand_cmd": "/mc/right_hand/command",
}


def strip_suffix(name: str) -> str:
    return name[: -len(STATE_SUFFIX)] if name.endswith(STATE_SUFFIX) else name


def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def dataset_parquet(root: Path) -> list[str]:
    files = sorted(glob.glob(str(root / "data" / "chunk-*" / "*.parquet")))
    if not files:
        raise FileNotFoundError(f"{root}/data/chunk-*/ 下没有 parquet")
    return files


def load_episode_actions(root: Path, episode: int):
    """读取指定 episode 的 action 序列，返回 ((T,D) float32, info dict)。"""
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"找不到数据元信息: {info_path}")
    with open(info_path, "r", encoding="utf-8") as f:
        info = json.load(f)

    import pandas as pd

    frames = []
    for fp in dataset_parquet(root):
        df = pq.read_table(fp, columns=["episode_index", "frame_index", "action"]).to_pandas()
        sel = df[df["episode_index"] == episode]
        if not sel.empty:
            frames.append(sel)
    if not frames:
        raise ValueError(f"数据集中没有 episode_index={episode}")
    ep = pd.concat(frames, ignore_index=True).sort_values("frame_index")
    actions = np.asarray([np.asarray(a, dtype=np.float32) for a in ep["action"].tolist()])
    return actions, info


def list_episodes(root: Path) -> None:
    import pandas as pd

    parts = [pq.read_table(fp, columns=["episode_index"]).to_pandas()
             for fp in dataset_parquet(root)]
    counts = pd.concat(parts, ignore_index=True)["episode_index"].value_counts().sort_index()
    print(f"{root}  共 {len(counts)} 集, {int(counts.sum())} 帧")
    for ep_idx, n in counts.items():
        print(f"  episode {int(ep_idx):>3}  {int(n):>4} 帧")


def resolve_plan(cfg: dict, action_names: list[str]) -> dict:
    """把 action 每一维映射到 body / 手部关节，并带上下发限位。"""
    body_groups = cfg.get("body", {}).get("groups", {})
    body_names = {n for names in body_groups.values() for n in names}
    # 配置 groups 的原始顺序（不能 sorted：手部驱动按下标解读 position，忽略 names）
    body_order = [n for names in body_groups.values() for n in names]
    body_limits = cfg.get("body", {}).get("limits", {})
    ee = cfg.get("end_effectors", {})
    ee_groups = ee.get("groups", {})
    ee_limits = ee.get("limits", {})
    hand_mode = int(ee.get("command_mode", DEFAULT_HAND_MODE))

    def limit_for(name: str):
        if name in body_limits:
            return tuple(body_limits[name])
        suffix = name.split("_", 1)[1] if "_" in name else name
        return tuple(ee_limits[suffix]) if suffix in ee_limits else None

    body: list[tuple[int, str, tuple | None]] = []
    hands: dict[str, list[tuple[int, str, tuple | None]]] = {"left": [], "right": []}
    unknown: list[str] = []
    for i, raw in enumerate(action_names):
        name = strip_suffix(raw)
        lim = limit_for(name)
        if name in body_names:
            body.append((i, name, lim))
            continue
        side = None
        for group_name, joints in ee_groups.items():
            if name in joints:
                side = "left" if group_name.startswith("left") else "right"
                break
        if side is None:
            unknown.append(raw)
        else:
            hands[side].append((i, name, lim))

    if unknown:
        raise ValueError(f"action 里有无法归类的关节（既不属 body 也不属手部）: {unknown}")
    # 真机 SDK 要求 RobotCommand 带全 14 个臂关节：只发子集会被整条忽略，
    # 关节会往默认位漂（实测 7 关节 @30Hz 下发后 roll 从 -0.564 退回 -0.282）。
    # 因此记录「全部关节名」，下发时未出现在 action 里的关节用当前实测值填充（保持不动）。
    body_all = list(body_order)
    hands_all = {side: list(joints) for side, joints in ee_groups.items()}
    return {"body": body, "hands": hands, "hand_mode": hand_mode,
            "body_all": body_all,
            "hands_all": {"left": hands_all.get("left_hand", []),
                          "right": hands_all.get("right_hand", [])}}


class ReplayNode:
    """ROS2 节点包装（__init__ 内才建 Node，导入期不触发 ROS）。"""

    def __init__(self, msgs: dict, topics: dict):
        self._node = msgs["Node"]("walker_c1_replay")
        self._JointCmd = msgs["JointCmd"]
        self._JointCommand = msgs["JointCommand"]
        self._RobotCommand = msgs["RobotCommand"]
        self.body_pub = self._node.create_publisher(
            self._RobotCommand, topics["body_cmd"], msgs["qos_pub"])
        self.left_pub = self._node.create_publisher(
            self._JointCommand, topics["left_hand_cmd"], msgs["qos_pub"])
        self.right_pub = self._node.create_publisher(
            self._JointCommand, topics["right_hand_cmd"], msgs["qos_pub"])

        self._pos: dict[str, float] = {}
        qos_sub = msgs["qos_sub"]
        self._node.create_subscription(
            msgs["RobotState"], topics["body_state"], self._on_body_state, qos_sub)
        if topics.get("joint_state"):
            self._node.create_subscription(
                msgs["JointState"], topics["joint_state"], self._on_joint_state, qos_sub)
        for topic in (topics["left_hand_cmd"], topics["right_hand_cmd"]):
            self._node.create_subscription(
                msgs["JointState"], topic.replace("/command", "/joint_states"),
                self._on_joint_state, qos_sub)

    def _on_body_state(self, msg) -> None:
        js = msg.joint_states
        for name, pos in zip(js.name, js.position):
            self._pos[name] = float(pos)

    def _on_joint_state(self, msg) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._pos[name] = float(pos)

    def read_state(self, names, timeout: float) -> dict:
        import rclpy

        for name in names:
            self._pos.pop(name, None)
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline and any(n not in self._pos for n in names):
            rclpy.spin_once(self._node, timeout_sec=0.02)
        return {n: self._pos[n] for n in names if n in self._pos}

    def publish(self, action: np.ndarray, plan: dict) -> None:
        stamp = self._node.get_clock().now().to_msg()
        # action 维度 -> (关节名, 限位)
        body_map = {name: (i, lim) for i, name, lim in plan["body"]}
        hand_maps = {side: {name: (i, lim) for i, name, lim in plan["hands"][side]}
                     for side in ("left", "right")}

        def value_of(name: str, mapping: dict) -> float:
            if name in mapping:
                i, lim = mapping[name]
                val = float(action[i])
                if lim is not None:
                    val = min(max(val, lim[0]), lim[1])
                return val
            # 不在 action 里的关节：用实测值保持（拿不到就 0.0）
            return float(self._pos.get(name, 0.0))

        msg = self._RobotCommand()
        msg.header.stamp = stamp
        for name in plan["body_all"]:
            cmd = self._JointCmd()
            cmd.name = name
            cmd.control_mode = self._JointCmd.MODE_POSITION
            cmd.position = value_of(name, body_map)
            msg.joint_cmd.append(cmd)
        if msg.joint_cmd:
            self.body_pub.publish(msg)

        for side, pub in (("left", self.left_pub), ("right", self.right_pub)):
            all_names = plan["hands_all"][side]
            if not all_names:
                continue
            hand = self._JointCommand()
            hand.header.stamp = stamp
            hand.names = list(all_names)
            hand.position = [value_of(n, hand_maps[side]) for n in all_names]
            hand.mode = [plan["hand_mode"]] * len(all_names)
            pub.publish(hand)

    def destroy(self) -> None:
        self._node.destroy_node()


def quintic(s: float) -> float:
    """五次平滑：起止速度/加速度均为 0。"""
    s = min(max(float(s), 0.0), 1.0)
    return s * s * s * (s * (s * 6.0 - 15.0) + 10.0)


def commanded_names(plan: dict) -> list[str]:
    names = [n for _, n, _ in plan["body"]]
    for side in ("left", "right"):
        names += [n for _, n, _ in plan["hands"][side]]
    return names


def run_transition(node, op, plan, target: np.ndarray, duration: float,
                   hz: float, state_timeout: float) -> None:
    """当前实测位姿 → 首帧，五次平滑插值按 hz 下发（绝对时间排程）。

    拿不到状态的关节直接以首帧为目标（该关节不动），避免从错误起点甩过去。
    """
    names = commanded_names(plan)
    now = node.read_state(names, state_timeout)
    missing = [n for n in names if n not in now]
    begin = target.copy()
    if missing:
        print(f"[replay] 警告: {state_timeout:.1f}s 内未收到 {len(missing)} 个关节状态 "
              f"{missing[:6]}{'...' if len(missing) > 6 else ''}，这些关节不做过渡")
    for group in [plan["body"], plan["hands"]["left"], plan["hands"]["right"]]:
        for i, name, _ in group:
            if name in now:
                begin[i] = now[name]

    print(f"[replay] 过渡 {duration:.1f}s：当前 -> 首帧，最大关节差 "
          f"{float(np.abs(target - begin).max()):.3f}")
    steps = max(1, int(round(duration * hz)))
    t_begin = time.monotonic()
    for k in range(1, steps + 1):
        op(begin + (target - begin) * quintic(k / steps))
        # 绝对时间排程：不能拿「收到下一条状态」当节拍，否则会被状态流带着跑飞
        deadline = t_begin + k / hz
        sleep_for = deadline - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Walker C1 LeRobot 数据集 action 回放（真机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--dataset", type=Path, default=DEFAULT_DATASET,
                   help=f"LeRobot 数据集根目录（默认 {DEFAULT_DATASET}）")
    p.add_argument("--episode", type=int, default=0, help="要回放的 episode_index（默认 0）")
    p.add_argument("--config", type=Path, default=None,
                   help="部署配置 JSON（默认按 action 维度选 configs/walker_c1_{13d,26d}.json）")
    p.add_argument("--rate", type=float, default=None,
                   help="数据回放帧率 Hz（时间轴，默认取 info.json 的 fps=30）")
    p.add_argument("--hz", type=float, default=100.0,
                   help="物理下发频率 Hz（默认 100，与 robot_ready 一致）。真机实测 30 Hz 下发"
                   "会让位置环断续、末端漂向默认位，故数据帧之间做线性插值重采样到该频率")
    p.add_argument("--start", type=int, default=0, help="起始帧（默认 0）")
    p.add_argument("--end", type=int, default=-1, help="结束帧，-1 表示到末尾")
    p.add_argument("--transition", type=float, default=3.0,
                   help="从当前位姿过渡到首帧的秒数（0=跳过，直接发首帧）")
    p.add_argument("--state-timeout", type=float, default=2.0, help="等状态首帧的超时秒数")
    p.add_argument("--lead-in", type=float, default=1.0, help="过渡到位后保持首帧的秒数")
    p.add_argument("--dry-run", action="store_true", help="只打印不发布")
    p.add_argument("--list", action="store_true", help="列出各集帧数后退出")
    return p.parse_args()


def import_ros2() -> dict:
    import rclpy
    from mc_state_msgs.msg import RobotState
    from mc_task_msgs.msg import JointCmd, JointCommand, RobotCommand
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    # 状态订阅 BEST_EFFORT：兼容 RELIABLE / BEST_EFFORT 两种发布端
    qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.VOLATILE)
    # 下发 RELIABLE depth 10（与 robot_ready.py 一致；depth 1 实测更抖）
    qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.VOLATILE)
    return {"rclpy": rclpy, "Node": Node, "JointCmd": JointCmd,
            "JointCommand": JointCommand, "RobotCommand": RobotCommand,
            "RobotState": RobotState, "JointState": JointState,
            "qos_sub": qos_sub, "qos_pub": qos_pub}


def main() -> int:
    args = parse_args()
    dataset = args.dataset.expanduser().resolve()
    try:
        if args.list:
            list_episodes(dataset)
            return 0
        actions, info = load_episode_actions(dataset, args.episode)
    except (FileNotFoundError, ValueError) as e:
        print(f"[replay] 错误: {e}", file=sys.stderr)
        return 1

    names = list(info.get("features", {}).get("action", {}).get("names") or [])
    if not names:
        print("[replay] 错误: info.json 缺 features.action.names，无法按名下发", file=sys.stderr)
        return 1
    if len(names) != actions.shape[1]:
        print(f"[replay] 错误: action.names {len(names)} 项 vs 数据 {actions.shape[1]} 维",
              file=sys.stderr)
        return 1

    dim = actions.shape[1]
    cfg_path = args.config
    if cfg_path is None:
        tag = {13: "13d", 26: "26d"}.get(dim)
        if tag is None:
            print(f"[replay] 错误: action {dim} 维无默认配置，请用 --config 指定", file=sys.stderr)
            return 1
        cfg_path = CONFIG_DIR / f"walker_c1_{tag}.json"
    cfg_path = cfg_path.expanduser().resolve()
    try:
        cfg = load_config(cfg_path)
        plan = resolve_plan(cfg, names)
    except (OSError, ValueError) as e:
        print(f"[replay] 错误: {e}", file=sys.stderr)
        return 1

    topics = dict(DEFAULT_TOPICS)
    ros_cfg = cfg.get("ros", {})
    topics["body_cmd"] = ros_cfg.get("body_cmd_topic", topics["body_cmd"])
    topics["body_state"] = ros_cfg.get("body_state_topic", topics["body_state"])
    topics["joint_state"] = ros_cfg.get("joint_state_topic", topics["joint_state"])

    start = max(0, args.start)
    end = len(actions) if args.end < 0 else min(args.end, len(actions))
    if start >= end:
        print(f"[replay] 错误: 帧范围非法 start={start} >= end={end}", file=sys.stderr)
        return 1
    rate = args.rate if args.rate is not None else float(info.get("fps", 30))

    hz = max(20.0, float(args.hz))
    print(f"[replay] dataset={dataset.name} episode={args.episode} frames=[{start},{end}) "
          f"data={rate}fps 下发={hz:.0f}Hz dim={dim} config={cfg_path.name}")
    print(f"[replay] body {len(plan['body'])} 维 / 左手 {len(plan['hands']['left'])} 维 / "
          f"右手 {len(plan['hands']['right'])} 维, hand_mode={plan['hand_mode']}")
    print(f"[replay] topic_body={topics['body_cmd']} dry_run={args.dry_run}")

    def op(a: np.ndarray) -> None:
        node.publish(a, plan)

    if args.dry_run:
        print("[replay] dry-run：不发布，仅打印前 3 帧")
        for i in range(start, min(start + 3, end)):
            body = {n: round(float(actions[i, j]), 3) for j, n, _ in plan["body"]}
            hands = {s: {n: round(float(actions[i, j]), 3) for j, n, _ in plan["hands"][s]}
                     for s in ("left", "right") if plan["hands"][s]}
            print(f"  frame {i}: body={body} hands={hands}")
        expected = ", ".join(n for _, n, _ in plan["body"])
        print(f"[replay] 将下发 body: {expected}")
        return 0

    msgs = import_ros2()
    msgs["rclpy"].init()
    node = ReplayNode(msgs, topics)
    time.sleep(1.0)  # 等 publisher 上线
    try:
        # ① 当前位姿 -> 首帧
        if args.transition > 0:
            run_transition(node, op, plan, actions[start], args.transition, hz,
                           args.state_timeout)
        # ② 保持首帧，等手臂稳定
        if args.lead_in > 0:
            print(f"[replay] 首帧保持 {args.lead_in}s ...")
            n_lead = max(1, int(round(args.lead_in * hz)))
            t_lead = time.monotonic()
            for k in range(n_lead):
                op(actions[start])
                sleep_for = (t_lead + (k + 1) / hz) - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        # ③ 回放：数据时间轴按 rate(fps) 走，物理下发按 hz，帧间线性插值重采样。
        #    30 fps 的相邻帧间隔 33 ms，硬阶梯下发会让位置环断续；插值后 ~10 ms 一个点。
        n_frames = end - start
        total_t = n_frames / rate
        steps = max(1, int(round(total_t * hz)))
        print(f"[replay] 回放 {n_frames} 帧/{total_t:.2f}s，按 {hz:.0f}Hz 插值下发 {steps} 个点 ...")
        t_begin = time.monotonic()
        for k in range(1, steps + 1):
            s_idx = start + (k / hz) * rate
            s_idx = min(max(s_idx, float(start)), float(end - 1))
            i0 = int(np.floor(s_idx))
            i1 = min(i0 + 1, end - 1)
            frac = s_idx - i0
            op(actions[i0] * (1.0 - frac) + actions[i1] * frac)
            sleep_for = (t_begin + k / hz) - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
        print(f"[replay] 回放实际耗时 {time.monotonic() - t_begin:.2f}s")
    except KeyboardInterrupt:
        print("\n[replay] 中断，停止发送。")
    finally:
        node.destroy()
        msgs["rclpy"].shutdown()

    print(f"[replay] 完成。机器人停在第 {end - 1} 帧位置（不再下发保持指令）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
