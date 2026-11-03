#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeRobot 数据集 -> ubt_sim playback 用的 trajectory.hdf5。

在训练侧（conda 环境，有 lerobot）把 LeRobot v3 数据集的 episode 转成
``ubt_sim/teleoperation/control/tienkung_dex/utils/recorder.py`` 同构的 HDF5，
之后用 ubt_sim 现成的 playback 工具回放（无需改仿真侧代码）：

    # 预览（Rerun 曲线 + 相机，不起物理仿真）
    python3 teleoperation/tools/playback_tienkung_dex.py \
        --episode dataset/tienkung_dex_from_lerobot/episode_000 --mode preview

    # 控制回放到 Isaac Sim（ROS_DOMAIN_ID=146）
    source /opt/ros/humble/setup.bash
    ROS_DOMAIN_ID=146 python3 teleoperation/tools/playback_tienkung_dex.py \
        --episode dataset/tienkung_dex_from_lerobot/episode_000 \
        --mode control --align-first

映射规则（按关节名对齐，数据集没有的填 0）：
- body 31 维：数据集的 14 个臂关节按名放入；腿/腰/头 17 维没有 -> 0。
  playback 按名查表发布，缺失关节不下发，仿真保持当前位姿。
- 手：真机是 brainco 变体（11 关节/手）。数据集 6 维（*_proximal / thumb_*）
  与 brainco 6 个通道首关节同名，直接填入；*_distal 没有 -> 0（不影响回放）。
- 回放（--mode control）使用 action_*；state 只用于 preview 曲线。
  真机没录手部 state（全 0），默认用 action 回填，--no-hand-state-from-action 关闭。
- 深度是伪彩色可视化，无法当 uint16 毫米用，一律不写；--color 可选写 RGB JPEG
  仅供 preview 观看。
"""
from __future__ import annotations

import argparse
import logging
import shutil
import sys
import time
from pathlib import Path

import h5py
import numpy as np
import pyarrow.parquet as pq

WORKSPACE = Path(__file__).resolve().parents[4]
DEFAULT_ROOT = WORKSPACE / "ubt_IL" / "dataset" / "tienkung_dex_real_pick"
DEFAULT_OUT_ROOT = WORKSPACE / "ubt_sim" / "dataset" / "tienkung_dex_from_lerobot"
SIM_CONSTANTS = (WORKSPACE / "ubt_sim" / "teleoperation" / "control"
                 / "tienkung_dex" / "constants.py")

STATE_KEY = "observation.state"
ACTION_KEY = "action"
CAMERA_KEY = "observation.images.head"

# 与 ubt_sim/teleoperation/control/tienkung_dex/constants.py 保持一致（启动时校验）
BODY_JOINTS = [
    "hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint",
    "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint",
    "hip_pitch_r_joint", "hip_roll_r_joint", "hip_yaw_r_joint",
    "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "head_yaw_joint", "head_pitch_joint",
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint",
    "wrist_pitch_l_joint", "wrist_roll_l_joint",
    "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint", "elbow_yaw_r_joint",
    "wrist_pitch_r_joint", "wrist_roll_r_joint",
]
BRAINCO_LEFT_HAND_JOINTS = [
    "left_thumb_metacarpal_joint", "left_thumb_proximal_joint",
    "left_thumb_distal_joint", "left_index_proximal_joint",
    "left_index_distal_joint", "left_middle_proximal_joint",
    "left_middle_distal_joint", "left_ring_proximal_joint",
    "left_ring_distal_joint", "left_pinky_proximal_joint",
    "left_pinky_distal_joint",
]
BRAINCO_RIGHT_HAND_JOINTS = [n.replace("left_", "right_")
                             for n in BRAINCO_LEFT_HAND_JOINTS]

logger = logging.getLogger("lerobot_to_playback")


def check_against_sim_constants() -> None:
    """若能找到 ubt_sim 的 constants.py，校验内嵌名单没有漂移。"""
    if not SIM_CONSTANTS.is_file():
        return
    sys.path.insert(0, str(SIM_CONSTANTS.parent))
    try:
        import constants as C  # noqa: PLC0415
    except Exception as e:  # 纯数据模块，失败只降级为警告
        logger.warning("无法加载 %s：%s（跳过名单校验）", SIM_CONSTANTS, e)
        return
    ok = (C.BODY_JOINTS == BODY_JOINTS
          and C.BRAINCO_LEFT_HAND_JOINTS == BRAINCO_LEFT_HAND_JOINTS
          and C.BRAINCO_RIGHT_HAND_JOINTS == BRAINCO_RIGHT_HAND_JOINTS)
    if not ok:
        raise SystemExit(f"内嵌关节名单与 {SIM_CONSTANTS} 不一致，请同步后重试")


def parse_episode_list(spec: str, total: int) -> list[int]:
    """'all' / '0,2,5-7' -> episode 索引列表。"""
    s = spec.strip().lower()
    if s in ("all", "*"):
        return list(range(total))
    eps: set[int] = set()
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            eps.update(range(int(a), int(b) + 1))
        else:
            eps.add(int(part))
    bad = [e for e in sorted(eps) if not 0 <= e < total]
    if bad:
        raise SystemExit(f"episode 索引越界: {bad}（共 {total} 条）")
    return sorted(eps)


def load_episode_numeric(meta, root: Path, ep: int):
    path = Path(meta.get_data_file_path(ep))
    if not path.is_absolute():
        path = Path(meta.root) / path
    df = pq.read_table(path, columns=["episode_index", "frame_index",
                                      STATE_KEY, ACTION_KEY]).to_pandas()
    d = df[df.episode_index == ep].sort_values("frame_index")
    state = np.stack(d[STATE_KEY].values).astype(np.float32)
    action = np.stack(d[ACTION_KEY].values).astype(np.float32)
    names = list(meta.features[STATE_KEY]["names"])
    if list(meta.features[ACTION_KEY]["names"]) != names:
        raise SystemExit(f"episode {ep}: state 与 action 维度名不一致")
    if state.shape[1] != len(names):
        raise SystemExit(f"episode {ep}: state {state.shape[1]} 维 != 名单 {len(names)}")
    return state, action, names


def map_by_name(dst_names: list[str], src_names: list[str],
                arr: np.ndarray) -> tuple[np.ndarray, list[str]]:
    """把 (N, len(src_names)) 按名重排到 dst_names，缺失列填 0。"""
    out = np.zeros((arr.shape[0], len(dst_names)), dtype=np.float32)
    hit: list[str] = []
    src = {n: j for j, n in enumerate(src_names)}
    for i, name in enumerate(dst_names):
        j = src.get(name)
        if j is not None:
            out[:, i] = arr[:, j]
            hit.append(name)
    return out, hit


def iter_video_frames(path: Path, start_s: float, count: int, fps: float):
    """从拼接 mp4 的 start_s 起顺序解出 count 帧（RGB uint8, HWC）。"""
    import av  # noqa: PLC0415

    tol = 0.5 / fps
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        if start_s > 0:
            # 指定 stream 时 PyAV 的 offset 单位是流的 time_base，不是微秒
            tb = float(stream.time_base)
            container.seek(int(round(start_s / tb)), stream=stream, backward=True)
        yielded = 0
        for frame in container.decode(stream):
            t = frame.time
            if t is not None and t < start_s - tol:
                continue
            img = frame.to_ndarray(format="rgb24")
            if img.shape[:2] != (720, 1280) and img.ndim == 3:  # 保底
                pass
            yield img
            yielded += 1
            if yielded >= count:
                return
        raise RuntimeError(f"{path.name}: 期望 {count} 帧，只解出 {yielded} 帧"
                           f"（起点 {start_s:.3f}s）")


def encode_color_jpegs(meta, ep: int, quality: int):
    """把 episode 的 head RGB 帧编码成 JPEG 字节流（BGR 约定，同 recorder）。"""
    import cv2  # noqa: PLC0415

    path = Path(meta.get_video_file_path(ep, CAMERA_KEY))
    if not path.is_absolute():
        path = Path(meta.root) / path
    t0 = float(meta.episodes[ep][f"videos/{CAMERA_KEY}/from_timestamp"])
    length = int(meta.episodes[ep]["length"])
    fps = meta.fps
    out = []
    for img in iter_video_frames(Path(path), t0, length, fps):
        ok, buf = cv2.imencode(".jpg", img[..., ::-1],
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError("JPEG 编码失败")
        out.append(buf.tobytes())
    return out


def write_hdf5(out_path: Path, *, fps: int, task: str, ep: int,
               body_state, body_action,
               hand_l_state, hand_l_action, hand_r_state, hand_r_action,
               color_jpegs) -> None:
    n = body_action.shape[0]
    zeros_b = np.zeros_like(body_state)
    with h5py.File(out_path, "w") as f:
        f.attrs.update({
            "body_joint_names": np.array(BODY_JOINTS, dtype=object),
            "left_hand_joint_names": np.array(BRAINCO_LEFT_HAND_JOINTS, dtype=object),
            "right_hand_joint_names": np.array(BRAINCO_RIGHT_HAND_JOINTS, dtype=object),
            "camera_names": '["camera_head"]' if color_jpegs else "[]",
            "capture_hz": float(fps),
            "created_at": float(time.time()),
            "dropped_frames": 0,
            "episode_index": ep,
            "fps": float(fps),
            "hand_variant": "brainco",
            "num_frames": n,
            "robot": "tienkung_dex",
            "robot_type": "tienkung_dex",
            "task": task,
        })
        f.create_dataset("timestamp", data=np.arange(n, dtype=np.float64) / fps)
        for key, data in [
            ("joint_observations/joint_position", body_state),
            ("joint_observations/joint_velocity", zeros_b),
            ("joint_observations/joint_effort", zeros_b),
            ("action_observations/joint_position", body_action),
        ]:
            f.create_dataset(key, data=data.astype(np.float32))
        for side, st, ac in [("left", hand_l_state, hand_l_action),
                             ("right", hand_r_state, hand_r_action)]:
            f.create_dataset(f"hand_observations/hand_{side}_position",
                             data=st.astype(np.float32))
            f.create_dataset(f"hand_observations/hand_{side}_velocity",
                             data=np.zeros_like(st))
            f.create_dataset(f"hand_observations/hand_{side}_effort",
                             data=np.zeros_like(st))
            f.create_dataset(f"action_observations/hand_{side}_position",
                             data=ac.astype(np.float32))
        if color_jpegs:
            # 二进制（含 NULL 字节）必须用 vlen uint8，不能用 vlen 字符串；
            # playback 侧 bytes(uint8 数组) 同样能得到原始 JPEG 字节
            vlen_u8 = h5py.vlen_dtype(np.dtype("uint8"))
            color_arr = np.empty((n,), dtype=object)
            for i, b in enumerate(color_jpegs):
                color_arr[i] = np.frombuffer(b, dtype=np.uint8)
            g_color = f.create_group("camera_observations/color_images")
            g_color.create_dataset("camera_head", data=color_arr, dtype=vlen_u8)
            # 深度是伪彩色、不能当 uint16 毫米：写空数组占位，preview 自动跳过
            depth_arr = np.empty((n,), dtype=object)
            for i in range(n):
                depth_arr[i] = np.zeros(0, dtype=np.uint8)
            g_depth = f.create_group("camera_observations/depth_images")
            g_depth.create_dataset("camera_head", data=depth_arr, dtype=vlen_u8)


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--repo_id", default="tienkung_dex_real_pick")
    p.add_argument("--root", default=str(DEFAULT_ROOT), help="LeRobot 数据集根目录")
    p.add_argument("--episodes", default="all", help="all 或 0,2,5-7")
    p.add_argument("--out-root", default=str(DEFAULT_OUT_ROOT),
                   help="输出父目录（每条 episode 一个子目录）")
    p.add_argument("--task", default=None, help="覆盖 task 标签")
    p.add_argument("--color", action="store_true",
                   help="写入 head RGB JPEG（仅供 preview，约 30-60MB/集）")
    p.add_argument("--jpeg-quality", type=int, default=90)
    p.add_argument("--no-hand-state-from-action", dest="hand_state_from_action",
                   action="store_false", default=True,
                   help="手部 state 不用 action 回填（保持数据集原值，即全 0）")
    p.add_argument("--overwrite", action="store_true", help="覆盖已存在的输出目录")
    args = p.parse_args()

    check_against_sim_constants()

    from lerobot.datasets import LeRobotDataset  # noqa: PLC0415

    ds = LeRobotDataset(args.repo_id, root=args.root)
    meta = ds.meta
    fps = int(round(meta.fps))
    episodes = parse_episode_list(args.episodes, meta.total_episodes)
    for key in (STATE_KEY, ACTION_KEY):
        if key not in meta.features:
            raise SystemExit(f"数据集缺少特征 '{key}'")
    state_names = list(meta.features[STATE_KEY]["names"])

    out_root = Path(args.out_root).expanduser().resolve()
    print(f"源: {args.root}  ({meta.total_episodes} ep, {meta.total_frames} frames, fps={fps})")
    print(f"出: {out_root}  episodes={episodes}")
    print(f"身体命中 {sum(n in state_names for n in BODY_JOINTS)}/31 维, "
          f"左手 {sum(n in state_names for n in BRAINCO_LEFT_HAND_JOINTS)}/11, "
          f"右手 {sum(n in state_names for n in BRAINCO_RIGHT_HAND_JOINTS)}/11")

    for ep in episodes:
        state, action, names = load_episode_numeric(meta, Path(args.root), ep)
        task = args.task or (meta.episodes[ep].get("tasks") or ["pick bottle"])[0]

        body_state, hit_s = map_by_name(BODY_JOINTS, names, state)
        body_action, _ = map_by_name(BODY_JOINTS, names, action)
        hand_l_state, _ = map_by_name(BRAINCO_LEFT_HAND_JOINTS, names, state)
        hand_r_state, _ = map_by_name(BRAINCO_RIGHT_HAND_JOINTS, names, state)
        hand_l_action, hit_l = map_by_name(BRAINCO_LEFT_HAND_JOINTS, names, action)
        hand_r_action, hit_r = map_by_name(BRAINCO_RIGHT_HAND_JOINTS, names, action)
        if args.hand_state_from_action:
            # 真机未录手部本体反馈（state 恒 0），用 action 近似，preview 曲线才不误导
            hand_l_state = hand_l_action.copy()
            hand_r_state = hand_r_action.copy()

        color = encode_color_jpegs(meta, ep, args.jpeg_quality) if args.color else None

        ep_dir = out_root / f"episode_{ep:03d}"
        if ep_dir.exists():
            if not args.overwrite:
                logger.error("已存在 %s（--overwrite 覆盖）", ep_dir)
                return 2
            shutil.rmtree(ep_dir)
        ep_dir.mkdir(parents=True)
        out_path = ep_dir / "trajectory.hdf5"
        write_hdf5(out_path, fps=fps, task=task, ep=ep,
                   body_state=body_state, body_action=body_action,
                   hand_l_state=hand_l_state, hand_l_action=hand_l_action,
                   hand_r_state=hand_r_state, hand_r_action=hand_r_action,
                   color_jpegs=color)
        size_mb = out_path.stat().st_size / 1e6
        print(f"episode {ep}: {body_action.shape[0]} 帧 -> {out_path} ({size_mb:.1f} MB) "
              f"body 命中 {len(hit_s)}, 手 {len(hit_l)}/{len(hit_r)}, task='{task}'"
              f"{', 含 RGB' if color else ''}")

    print("\n在 ubt_sim 侧回放：")
    print(f"  cd {WORKSPACE / 'ubt_sim'}")
    print(f"  python3 teleoperation/tools/playback_tienkung_dex.py "
          f"--episode dataset/tienkung_dex_from_lerobot/episode_{episodes[0]:03d} --mode preview")
    print(f"  ROS_DOMAIN_ID=146 python3 teleoperation/tools/playback_tienkung_dex.py "
          f"--episode dataset/tienkung_dex_from_lerobot/episode_{episodes[0]:03d} "
          f"--mode control --align-first")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
