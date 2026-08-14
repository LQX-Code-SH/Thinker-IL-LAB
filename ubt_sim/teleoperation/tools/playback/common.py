# -*- coding: utf-8 -*-
"""数据集回放共用核心：HDF5 加载、节奏控制、CLI、Rerun 预览。

机器人无关；机器人差异（HDF5 schema、ROS2 发布接口）由 ``playback.robots.*``
适配层封装。

依赖策略：
- 模块级只 import 轻量库（宿主机可 --help / --dry-run）
- rclpy / 机器人消息 / rerun 全部函数内懒导入（仿 ubt_IL/scripts/deploy/tienkung_pro/replay.py）
- cv2 由 apt 安装（带 GTK GUI），预览模式解码 JPEG/PNG 用
"""
from __future__ import annotations

import argparse
import gc
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import h5py
import numpy as np

HDF5_REL_PATH = "trajectory.hdf5"
DEFAULT_FPS = 15.0
# ubt_sim/teleoperation/tools/playback/common.py -> parents[3] == ubt_sim/
DATASET_ROOT = Path(__file__).resolve().parents[3] / "dataset"


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class CurveGroup:
    """一组随时间变化的通道曲线。label 即 Rerun 实体路径前缀。"""
    label: str
    channel_names: list  # 通道名（长度 = data 列数或 1）
    data: np.ndarray     # (N, C) 或 (N,)


@dataclass
class CameraStream:
    """一路相机流。color/depth 为 (N,) object 数组（JPEG/PNG 字节），无数据为 None。"""
    color: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None


@dataclass
class Episode:
    """一个采集 episode 的归一化视图（由适配层 load() 产出）。"""
    path: str
    robot_type: str
    num_frames: int
    fps: float
    timestamps: Optional[np.ndarray]   # (N,) 秒
    curve_groups: list                 # list[CurveGroup]
    cameras: dict                      # {name: CameraStream}
    attrs: dict                        # HDF5 根属性（walker 有，tienkung 为空）

    def group(self, label: str) -> CurveGroup:
        for g in self.curve_groups:
            if g.label == label:
                return g
        raise KeyError(f"curve group '{label}' not found")


# ============================================================================
# HDF5 读取
# ============================================================================

def read_hdf5_array(group, key: str) -> np.ndarray:
    """读取 HDF5 数组，兼容 ``<key>/data`` 子组与裸 ``<key>`` 两种布局。

    walker_s2: ``observation/timestamp/data``；tienkung_pro: ``observations/timestamp``（裸）。
    """
    obj = group.get(key)
    if isinstance(obj, h5py.Dataset):
        return obj[()]
    if isinstance(obj, h5py.Group) and "data" in obj:
        return obj["data"][()]
    raise KeyError(f"{key} (and {key}/data) not found in {group.name}")


def resolve_episode_path(episode: Optional[str], robot_type: str) -> Path:
    """解析 episode 路径。

    - None  → dataset/<robot_type>/ 下最新（时间戳目录名最大）episode
    - 目录  → <dir>/trajectory.hdf5
    - 文件  → 直接使用
    """
    if episode:
        p = Path(episode)
        if p.is_dir():
            p = p / HDF5_REL_PATH
        if not p.is_file():
            raise FileNotFoundError(f"未找到 episode 文件: {p}")
        return p

    robot_dir = DATASET_ROOT / robot_type
    if not robot_dir.is_dir():
        raise FileNotFoundError(
            f"数据集目录不存在: {robot_dir}（--episode 指定路径，或确认已采集数据）"
        )
    # 时间戳目录按字典序即时间序；兼容多段采集 <ts>/episode_XXX_<part>/trajectory.hdf5
    for d in sorted(robot_dir.iterdir(), reverse=True):
        if not d.is_dir():
            continue
        cand = d / HDF5_REL_PATH
        if cand.is_file():
            return cand
        sub = sorted(p for p in d.iterdir() if (p / HDF5_REL_PATH).is_file())
        if sub:
            return sub[-1] / HDF5_REL_PATH
    raise FileNotFoundError(f"{robot_dir} 下未找到 {HDF5_REL_PATH}")


def derive_fps(timestamps: Optional[np.ndarray], fallback: float = DEFAULT_FPS) -> float:
    """由时间戳中位间隔推导 fps（tienkung 无 fps 属性）；异常回退 fallback。"""
    if timestamps is None or len(timestamps) < 2:
        return fallback
    dt = np.diff(np.asarray(timestamps, dtype=float))
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if len(dt) == 0:
        return fallback
    return float(1.0 / np.median(dt))


# ============================================================================
# 帧处理
# ============================================================================

def parse_index(value, num_frames: int, fps: float) -> int:
    """解析帧索引："12" → 12；"12.5s" → 第 12.5 秒处的帧。"""
    s = str(value).strip()
    if s.endswith("s"):
        return int(round(float(s[:-1]) * fps))
    return int(s)


def slice_episode(episode: Episode, start: int, end: int) -> Episode:
    """截取 [start, end) 帧，返回新 Episode（不修改原对象）。"""
    if not (0 <= start < end <= episode.num_frames):
        raise ValueError(
            f"帧范围非法: [{start},{end}) 超出 [0,{episode.num_frames})"
        )
    groups = [
        CurveGroup(g.label, g.channel_names, g.data[start:end])
        for g in episode.curve_groups
    ]
    cameras = {
        name: CameraStream(
            color=c.color[start:end] if c.color is not None else None,
            depth=c.depth[start:end] if c.depth is not None else None,
        )
        for name, c in episode.cameras.items()
    }
    ts = episode.timestamps[start:end] if episode.timestamps is not None else None
    return Episode(episode.path, episode.robot_type, end - start, episode.fps,
                   ts, groups, cameras, episode.attrs)


def clamp_array(values, names: list, limits: dict):
    """按通道名裁剪到限位并计数。limits: {name: (lo, hi)}；values (N,C) 或 (N,)。

    Returns:
        (clamped_array, violation_count)
    """
    arr = np.asarray(values, dtype=float).copy()
    count = 0
    for j, name in enumerate(names):
        bounds = limits.get(name)
        if not bounds:
            continue
        lo, hi = bounds
        col = arr[:, j] if arr.ndim == 2 else arr
        mask = (col < lo) | (col > hi)
        if mask.any():
            count += int(mask.sum())
            np.clip(col, lo, hi, out=col)
    return arr, count


def interpolate_frames(episode: Episode, rate: float) -> Episode:
    """在记录帧之间线性插值到目标频率（保持总时长不变；rate <= fps 时不处理）。"""
    if rate <= episode.fps or episode.num_frames < 2:
        return episode
    n = max(2, int(round(episode.num_frames * rate / episode.fps)))
    x_old = np.arange(episode.num_frames)
    x_new = np.linspace(0.0, float(episode.num_frames - 1), n)
    groups = []
    for g in episode.curve_groups:
        d = g.data
        if d.ndim == 1:
            interp = np.interp(x_new, x_old, d)
        else:
            interp = np.column_stack(
                [np.interp(x_new, x_old, d[:, j]) for j in range(d.shape[1])]
            )
        groups.append(CurveGroup(g.label, g.channel_names, interp))
    ts = None
    if episode.timestamps is not None and len(episode.timestamps) > 1:
        ts = np.interp(x_new, x_old, episode.timestamps)
    return Episode(episode.path, episode.robot_type, n, rate, ts, groups,
                   episode.cameras, episode.attrs)


def format_vector(values, precision: int = 3) -> str:
    """数值向量 → 紧凑字符串（dry-run 打印用）。"""
    return "[" + ", ".join(f"{x:.{precision}f}" for x in np.atleast_1d(values)) + "]"


# ============================================================================
# 控制模式
# ============================================================================

def paced_loop(num_frames: int, period: float, on_frame: Callable[[int], None],
               loop: bool = False) -> None:
    """按固定周期逐帧回调：单调时钟 + 落后重对齐（仿 replay.py run_replay）。"""
    next_t = time.monotonic()
    i = 0
    try:
        while True:
            if i >= num_frames:
                if not loop:
                    break
                i = 0
                next_t = time.monotonic()
            on_frame(i)
            i += 1
            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # 落后于节奏，重新对齐基准防止越拖越多
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[control] 中断，停止发送。")


def print_domain_guidance(args) -> None:
    """打印 ROS_DOMAIN_ID 提示（不硬编码，项目内 0/146 混用）。

    sim/real 由 domain 本身决定、发布路径完全一致，脚本不做区分，
    仅提醒确认当前 domain 与目标一致。
    """
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    print(f"[control] 当前 ROS_DOMAIN_ID={domain or '(未设置，DDS 默认 0)'}")
    print("[control] 请确认该 domain 与目标一致"
          "（项目约定：真机 0 / 仿真 146；docker/env.sh 默认 0，以实际为准）")


def print_episode_summary(episode: Episode) -> None:
    cams = ", ".join(episode.cameras) or "-"
    print(f"[playback] {episode.robot_type} episode: {episode.path}")
    print(f"  frames={episode.num_frames} fps={episode.fps:.2f} cameras=[{cams}]")
    for g in episode.curve_groups:
        print(f"  {g.label}: {g.data.shape} ({len(g.channel_names)} 通道)")


def run_control(args, adapter_cls) -> None:
    path = resolve_episode_path(args.episode, adapter_cls.robot_type)
    episode = adapter_cls.load(path)
    print_episode_summary(episode)

    start = parse_index(args.start, episode.num_frames, episode.fps)
    end = (parse_index(args.end, episode.num_frames, episode.fps)
           if args.end is not None else episode.num_frames)
    if end < 0:
        end = episode.num_frames
    start = max(0, min(start, episode.num_frames))
    end = max(0, min(end, episode.num_frames))
    if start >= end:
        raise ValueError(
            f"帧范围非法: start={start} >= end={end}"
            "（--end 不含末帧，如 --end 300 播放 [start,300)）"
        )

    episode = slice_episode(episode, start, end)
    episode = adapter_cls.clamp(episode, args)

    rate = args.rate or episode.fps
    if getattr(args, "interp", False):
        episode = interpolate_frames(episode, rate)

    print_domain_guidance(args)

    period = 1.0 / rate
    print(f"[control] 回放 frames=[{start},{end}) total={end - start} "
          f"rate={rate:.2f}Hz dry_run={args.dry_run} "
          f"align_first={args.align_first}")

    if args.dry_run:
        for i in range(episode.num_frames):
            frame = adapter_cls.frame_at(episode, i)
            print(f"  frame {start + i}: {adapter_cls.describe_frame(frame)}")
        print("[control] dry-run 完成（未初始化 ROS2、未发布任何消息）。")
        return

    # 懒导入：真正发布才需要 ROS2 环境
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node(f"playback_{adapter_cls.robot_type}")
    try:
        publisher = adapter_cls.make_publisher(node, args)
        publisher.frame_period = period  # walker 发布器据此决定是否错峰发送
        time.sleep(1.0)  # 等 publisher discovery（沿用 replay.py 的做法）
        if args.align_first:
            adapter_cls.align_first(episode, publisher, args)

        def on_frame(i: int) -> None:
            publisher.publish_frame(adapter_cls.frame_at(episode, i))

        paced_loop(episode.num_frames, period, on_frame, loop=args.loop)
        print(f"[control] 完成。机器人停在第 {start + episode.num_frames - 1} 帧位置"
              "（不再下发保持指令）。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================================================================
# 预览模式（Rerun）
# ============================================================================

def run_preview(args, adapter_cls) -> None:
    path = resolve_episode_path(args.episode, adapter_cls.robot_type)
    episode = adapter_cls.load(path)
    print_episode_summary(episode)

    import cv2
    import rerun as rr

    # --web-port 单独给出即起 web 服务（不依赖 --headless）；
    # 仅默认参数（无任何 viewer 选项）才走本地弹窗。
    spawn_local = not (args.headless or args.save_rrd
                       or args.grpc_port or args.web_port)
    rr.init(f"{episode.robot_type}/{Path(episode.path).parent.name}",
            spawn=spawn_local)
    # 规避 rr.init 后阻塞 flush 挂起（lerobot_dataset_viz 同款 workaround）
    gc.collect()

    serve_grpc = args.grpc_port is not None or args.web_port is not None
    web_url = None
    remote_note = ""
    if serve_grpc:
        import urllib.parse
        grpc_port = args.grpc_port or 9876
        web_port = args.web_port or 9090
        server_uri = rr.serve_grpc(grpc_port=grpc_port)
        rr.serve_web_viewer(open_browser=False, web_port=web_port,
                            connect_to=server_uri)
        # rerun 0.23 的 web viewer 靠页面 URL 的 ?url= 参数拿数据源：
        # serve_web_viewer 不会把它注入页面（open_browser=False 时尤其要手动带）。
        # url 参数值需整体 URL 编码，JS 端解码后恢复 rerun+http://... 格式。
        # 浏览器链接延后到全部输出末尾再打印，醒目且方便点击。
        url_param = urllib.parse.quote(server_uri, safe="")
        web_url = f"http://localhost:{web_port}/?url={url_param}"
        remote_note = (f"远程浏览器需同时转发 {web_port} 与 {grpc_port} 两个端口"
                       f"（VSCode PORTS 面板 / ssh -L {web_port}:localhost:{web_port}"
                       f" -L {grpc_port}:localhost:{grpc_port}）")

    n = min(episode.num_frames, args.max_frames or episode.num_frames)
    print(f"[preview] 记录 {n}/{episode.num_frames} 帧到 Rerun ...")
    for i in range(n):
        rr.set_time("frame_index", sequence=i)
        if episode.timestamps is not None and i < len(episode.timestamps):
            rr.set_time("timestamp", timestamp=float(episode.timestamps[i]))

        for cam_name, stream in episode.cameras.items():
            if stream.color is not None and i < len(stream.color):
                jpeg = bytes(stream.color[i])
                if jpeg:
                    # recorder 存的 JPEG 是 BGR（cv2.imencode 约定），Rerun 显示 RGB
                    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        rr.log(f"cameras/color/{cam_name}", rr.Image(img[..., ::-1]))
            if stream.depth is not None and i < len(stream.depth):
                png = bytes(stream.depth[i])
                if png:
                    depth = cv2.imdecode(np.frombuffer(png, np.uint8),
                                         cv2.IMREAD_UNCHANGED)
                    if depth is not None:
                        rr.log(f"cameras/depth/{cam_name}", rr.Image(depth))

        for g in episode.curve_groups:
            for j, ch in enumerate(g.channel_names):
                val = g.data[i, j] if g.data.ndim == 2 else g.data[i]
                rr.log(f"{g.label}/{ch}", rr.Scalars(float(val)))

    if spawn_local:
        # 原生 viewer 是 wgpu/egui 应用：容器内无 GPU 窗口路径时（无 Mesa、无 vulkan
        # surface）会静默启动失败，日志留在 viewer 自己的 stderr。检测不到进程就
        # 给出 web viewer 指引（浏览器渲染，无容器 GPU 依赖）。
        time.sleep(2.0)
        import subprocess
        alive = subprocess.run(
            ["pgrep", "-f", "rerun_cli/rerun"],
            capture_output=True, text=True,
        ).stdout.strip()
        if not alive:
            print("[preview] 警告: 未检测到本地 Rerun viewer 进程（本容器无可用 GPU "
                  "窗口路径，wgpu 无法创建 adapter）。\n"
                  "  改用浏览器预览: 加参数 --web-port 9090，按打印的完整 URL 打开"
                  "（末尾 ?url= 参数不能少）。")

    if args.save_rrd:
        rr.save(args.save_rrd)
        size = Path(args.save_rrd).stat().st_size
        print(f"[preview] 已保存 {args.save_rrd} ({size} bytes)")
        return

    if not (spawn_local or serve_grpc):
        print("[preview] 冒烟记录完成（无 viewer 也无 --save-rrd）。")
        return

    print("[preview] 记录完成。时间轴 scrub / 逐帧步进 / 循环为 Rerun 内置功能，"
          "Ctrl-C 退出。")
    if web_url:
        print()
        print("=" * 78)
        print("浏览器打开（末尾 ?url= 参数不能省）:")
        print(f"  {web_url}")
        print("=" * 78)
        print(remote_note)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[preview] 退出。")


# ============================================================================
# CLI
# ============================================================================

def build_cli(adapter_cls):
    p = argparse.ArgumentParser(
        description=f"{adapter_cls.robot_type} 数据集回放（预览 / 控制）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--episode", default=None,
                   help="episode 目录或 trajectory.hdf5 路径；缺省 = dataset/<robot>/ 下最新目录")
    p.add_argument("--mode", choices=["preview", "control"], default="preview",
                   help="预览（Rerun 查看相机+曲线）/ 控制（ROS2 发布回放）")
    # 预览
    p.add_argument("--save-rrd", metavar="PATH", default=None,
                   help="记录全部帧后保存 .rrd 并退出（不弹 viewer）")
    p.add_argument("--headless", action="store_true",
                   help="不弹本地 viewer（单独使用 = 仅冒烟记录）")
    p.add_argument("--web-port", type=int, default=None,
                   help="起 web viewer 服务（浏览器打开 http://<宿主机IP>:<端口>；"
                        "默认 9090，与 --headless 是否给出无关）")
    p.add_argument("--grpc-port", type=int, default=None,
                   help="distant 模式 gRPC 代理端口（默认 9876）")
    p.add_argument("--max-frames", type=int, default=None,
                   help="仅记录前 N 帧（调试用）")
    # 控制
    p.add_argument("--start", default="0",
                   help="起始帧；'5s' = 第 5 秒（默认 0）")
    p.add_argument("--end", default=None,
                   help="结束帧索引（不含，同 replay.py 语义）；'5s' = 第 5 秒（默认末帧）")
    p.add_argument("--rate", type=float, default=None,
                   help="发布频率 Hz（默认 = 数据集 fps）")
    p.add_argument("--loop", action="store_true", help="循环播放直到 Ctrl-C")
    p.add_argument("--dry-run", action="store_true",
                   help="只打印每帧摘要，不初始化 ROS2、不发布")
    p.add_argument("--align-first", action="store_true",
                   help="播放前先把机器人平滑移动到首帧位姿")
    adapter_cls.add_cli_args(p)
    return p


def main(adapter_cls) -> None:
    parser = build_cli(adapter_cls)
    args = parser.parse_args()
    if args.mode == "preview":
        run_preview(args, adapter_cls)
    else:
        run_control(args, adapter_cls)
