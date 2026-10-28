"""ActionRecorder — 部署时记录推理输出与 ROS 指令，支持 CSV 落盘和 matplotlib 绘图。

通过环境变量 ``RECORD_ACTIONS=1`` 启用，在 WalkerRobot 内部自动挂载。
"""

from __future__ import annotations

import csv
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class ActionRecorder:
    """轻量级动作记录器：内存缓存 → CSV 落盘 → PNG 时序图。

    在 ``WalkerRobot.send_action()`` 中每步调用 ``record()``，
    在 ``disconnect()`` 中调用 ``save()`` + ``plot()``。
    """

    def __init__(
        self,
        output_dir: str | Path,
        *,
        group_features: dict[str, list[str]],
        body_group_names: dict[str, list[str]],
        hand_group_names: dict[str, list[str]],
    ) -> None:
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)

        # 6 组特征名（带 .pos 后缀），用于从 action_input 按名取值
        self._group_features = group_features
        # 真实 ROS 关节名（按 body_group 顺序），用于 CSV/图表的列标签
        self._body_group_names = body_group_names
        self._hand_group_names = hand_group_names

        # 按 group 聚合真实关节名
        self._joint_names_by_group: dict[str, list[str]] = {
            **{g: list(names) for g, names in body_group_names.items()},
            **{g: list(names) for g, names in hand_group_names.items()},
        }

        # 内存缓冲：每条记录是 (timestamp, step, group, joint_idx, joint_name, target, current)
        self._records: list[tuple[float, int, str, int, str, float, float]] = []
        # chunk 缓冲（act_async 异步推理整块下发）：每条 (ts, chunk_id, point_idx,
        # group, joint_idx, joint_name, target, current, inference_time_sec, fps, n_points)
        self._chunk_records: list[tuple] = []
        # 实际位置缓冲（_process_status 路由桥接 status 流，60Hz 降采样）：
        # 每条 (ts, group, joint_idx, joint_name, actual)
        self._actual_records: list[tuple] = []
        self._last_actual_ts: float = -1.0   # 降采样用：上次记录绝对时间戳
        self._step: int = 0
        self._start_time: float | None = None
        self._lock = threading.Lock()

        # 实时绘图状态
        self._live_running: bool = False
        self._live_thread: threading.Thread | None = None

        logger.info("ActionRecorder initialized, output dir: %s", self._output_dir)

    # ---- 公开 API ----------------------------------------------------------------

    def _rel_ts(self, timestamp: float | None) -> float:
        """把绝对时间戳转成相对 _start_time 的秒数；首次调用时锚定 _start_time。"""
        if self._start_time is None:
            self._start_time = timestamp if timestamp is not None else time.perf_counter()
        return (timestamp - self._start_time) if timestamp is not None else (time.perf_counter() - self._start_time)

    def record(
        self,
        action_input: dict[str, float],
        action_msg: dict[str, Any],
        current_state: dict[str, list[float]],
        *,
        timestamp: float | None = None,
    ) -> None:
        """记录一步单动作（sync 引擎 / 关机回原点路径）。

        Args:
            action_input: 策略推理输出（仅策略关节，key=feature_name.pos）。
            action_msg: 最终发给 Bridge2/ROS2 的 6 组 ZMQ 消息。
            current_state: 当前机器人关节状态（6 组，每组一个 list）。
            timestamp: 绝对时间戳，None 则用相对时间。
        """
        ts = self._rel_ts(timestamp)

        for group, features in self._group_features.items():
            joint_names = self._joint_names_by_group.get(group, [])
            target_vals = action_msg.get(group, [])
            current_vals = current_state.get(group, [])

            for i, feature_name in enumerate(features):
                joint_name = joint_names[i] if i < len(joint_names) else feature_name
                target = float(target_vals[i]) if i < len(target_vals) else float("nan")
                current = float(current_vals[i]) if i < len(current_vals) else float("nan")
                with self._lock:
                    self._records.append((ts, self._step, group, i, joint_name, target, current))

        self._step += 1

    def record_chunk(
        self,
        action_msg: dict[str, Any],
        current_state: dict[str, list[float]],
        *,
        timestamp: float | None = None,
    ) -> None:
        """记录一个异步推理 chunk（act_async 引擎整块下发）。

        chunk 的 C 个时刻 × 各关节全部落盘到 ``chunks.csv``，用于诊断：
        - chunk 是否持续产出（engine 是否存活、chunk_id 是否递增）
        - chunk 内容是 hold 还是真实轨迹（推理静止与否）
        - 每块推理耗时 ``inference_time_sec``

        与 ``record``（单动作）互补：act_async 模式下 rollout 期间走 chunk 路径，
        单动作 ``record`` 只在关机回原点时触发，故 chunk 必须单独录制。

        Args:
            action_msg: ``send_action_chunk`` 组装的 ZMQ 消息，每组为 ``[C, len(group)]``，
                另含 ``n_points / fps / inference_time_sec / chunk_id / ts``。
            current_state: 产 chunk 时刻的机器人关节状态（6 组，每组一个 list）。
            timestamp: 绝对时间戳（取 action_msg["ts"]），None 则用相对时间。
        """
        ts = self._rel_ts(timestamp)
        chunk_id = int(action_msg.get("chunk_id", 0))
        n_points = int(action_msg.get("n_points", 0))
        inf_time = float(action_msg.get("inference_time_sec", 0.0))
        fps = float(action_msg.get("fps", 0.0))
        if n_points == 0:
            return

        # 先在锁外构建行（C × 关节数 可能上千行），再一次性 extend，减少持锁时间
        new_rows: list[tuple] = []
        for group, features in self._group_features.items():
            joint_names = self._joint_names_by_group.get(group, [])
            rows = action_msg.get(group, [])  # [C, len(group)]，每行一个时刻
            current_vals = current_state.get(group, [])
            for pt in range(n_points):
                row = rows[pt] if pt < len(rows) else []
                for i, feature_name in enumerate(features):
                    joint_name = joint_names[i] if i < len(joint_names) else feature_name
                    target = float(row[i]) if i < len(row) else float("nan")
                    current = float(current_vals[i]) if i < len(current_vals) else float("nan")
                    new_rows.append(
                        (ts, chunk_id, pt, group, i, joint_name, target, current, inf_time, fps, n_points)
                    )
        with self._lock:
            self._chunk_records.extend(new_rows)

    def record_actual(
        self,
        status: dict[str, Any],
    ) -> None:
        """记录机器人实际关节位置（来自桥接 status 流，60Hz 降采样）。

        由 ``WalkerRobot._process_status`` 每收到一帧 status 调用。供 ``plot_chunks``
        与 ``actual.csv`` 使用，用于在统一时间轴上对比「实际执行位置」与预测/融合轨迹。

        降采样：相邻记录间隔 < 1/60s 跳过，限制内存与 CSV 体积（够对比 30Hz 预测）。

        Args:
            status: 桥接 ZMQ status 消息（6 组关节位置 + ``ts``，ts 为桥接 wall clock）。
        """
        ts_abs = float(status.get("ts", 0.0))
        if ts_abs <= 0.0:
            return
        # 60Hz 降采样（_last_actual_ts 仅在本方法即 status 接收线程访问，无需锁）
        if ts_abs - self._last_actual_ts < 1.0 / 60.0:
            return
        self._last_actual_ts = ts_abs

        ts = self._rel_ts(ts_abs)
        new_rows: list[tuple] = []
        for group, features in self._group_features.items():
            joint_names = self._joint_names_by_group.get(group, [])
            vals = status.get(group, [])
            for i, feature_name in enumerate(features):
                joint_name = joint_names[i] if i < len(joint_names) else feature_name
                actual = float(vals[i]) if i < len(vals) else float("nan")
                new_rows.append((ts, group, i, joint_name, actual))
        with self._lock:
            self._actual_records.extend(new_rows)

    def _load_fused_records(self) -> dict[tuple[str, int], list[tuple[float, float, int]]]:
        """读桥接写的 ``fused.csv`` -> ``{(group, joint_idx): [(exec_time_rel, fused, chunk_id), ...]}``。

        ``exec_time`` 为桥接绝对 wall clock，转相对 ``_start_time`` 以与 chunks/actual 对齐。
        文件不存在、``_start_time`` 未锚定或解析失败时返回 ``{}``（逐行 try 防半行写入）。
        """
        fused_csv = self._output_dir / "fused.csv"
        if not fused_csv.exists() or self._start_time is None:
            return {}
        out: dict[tuple[str, int], list[tuple[float, float, int]]] = {}
        try:
            with open(fused_csv, "r", newline="", encoding="utf-8") as f:
                reader = csv.reader(f)
                next(reader, None)  # 跳过表头
                for row in reader:
                    if len(row) < 7:
                        continue
                    try:
                        exec_abs = float(row[0])
                        cid = int(row[1])
                        group = row[3]
                        ji = int(row[4])
                        val = float(row[6])
                    except (ValueError, IndexError):
                        continue
                    out.setdefault((group, ji), []).append((exec_abs - self._start_time, val, cid))
        except OSError:
            pass
        return out

    # ---- 实时绘图（Agg 后端 → 定时写 PNG 到磁盘）-------------------------------

    def _build_joint_flat_list(self) -> list[tuple[str, int, str]]:
        """构建 (group, joint_idx, joint_name) 的扁平列表，按 group 顺序排列。"""
        joints: list[tuple[str, int, str]] = []
        for group in self._group_features:
            names = self._joint_names_by_group.get(group, [])
            for ji, jname in enumerate(names):
                joints.append((group, ji, jname))
        return joints

    # ---- 实时绘图（Agg 后端 → 定时写 PNG 到磁盘）-------------------------------

    def start_live_plot(self) -> bool:
        """启动实时绘图后台线程，用 Agg 后端定时写 ``actions_live.png``。

        每个关节一个子图，宿主端用 VS Code / 图片查看器打开即可实时查看。

        Returns:
            True 如果成功启动，False 如果 matplotlib 不可用。
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            logger.warning("matplotlib not available, live plot disabled")
            return False

        self._live_joints = self._build_joint_flat_list()
        if not self._live_joints:
            logger.warning("No joints to plot, live plot disabled")
            return False

        n_joints = len(self._live_joints)
        self._live_fig, axes = plt.subplots(
            n_joints, 1, figsize=(14, 1.8 * n_joints), sharex=True,
        )
        self._live_axes = [axes] if n_joints == 1 else list(axes)
        self._live_png_path = self._output_dir / "actions_live.png"

        # 每条关节: cmd(单动作) + obs(单动作当前) + predicted(chunk 预测块) + actual(实际位置)
        self._live_joint_lines: list[tuple] = []  # [(ax, group, ji, t_line, c_line, ck_line, a_line), ...]

        for ax, (group, ji, jname) in zip(self._live_axes, self._live_joints):
            (tl,) = ax.plot([], [], "C0-", linewidth=0.8, label="cmd")
            (cl,) = ax.plot([], [], "C1--", linewidth=0.5, alpha=0.6, label="obs")
            (ckl,) = ax.plot([], [], "C0-", linewidth=0.5, alpha=0.5, marker=".", markersize=1.0, label="predicted")
            (al,) = ax.plot([], [], "C2-", linewidth=0.7, alpha=0.9, label="actual")
            self._live_joint_lines.append((ax, group, ji, tl, cl, ckl, al))
            ax.set_ylabel(jname, fontsize=7)
            ax.legend(fontsize=6, loc="upper right")
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=6)

        self._live_axes[-1].set_xlabel("Time (s)")
        self._live_fig.suptitle("Walker S2 Actions (live)", fontsize=12, fontweight="bold")
        self._live_fig.tight_layout(pad=1.0)

        self._live_running = True
        self._live_thread = threading.Thread(
            target=self._live_plot_loop, daemon=True, name="action_live_plot",
        )
        self._live_thread.start()
        logger.info(
            "Live plot started (%d joints) → %s", n_joints, self._live_png_path,
        )
        return True

    def _live_plot_loop(self) -> None:
        """后台线程：每 250ms 用 Agg 重绘并覆盖 ``actions_live.png``。"""
        while self._live_running:
            try:
                with self._lock:
                    records = list(self._records)
                    chunk_records = list(self._chunk_records)
                    actual_records = list(self._actual_records)

                if records or chunk_records or actual_records:
                    for ax, group, ji, tl, cl, ckl, al in self._live_joint_lines:
                        # 单动作记录: (ts, step, group, ji, name, target, current)
                        ji_records = [r for r in records if r[2] == group and r[3] == ji]
                        tl.set_data([r[0] for r in ji_records], [r[5] for r in ji_records])
                        cl.set_data([r[0] for r in ji_records], [r[6] for r in ji_records])
                        # chunk 记录: (ts, chunk_id, pt, group, ji, name, target, current, inf, fps, n)
                        # x 用预计执行时间 = ts + pt/fps
                        ck_records = [r for r in chunk_records if r[3] == group and r[4] == ji]
                        ck_x = [(r[0] + r[2] / r[9]) if (r[9] and r[9] > 0) else r[0] for r in ck_records]
                        ckl.set_data(ck_x, [r[6] for r in ck_records])
                        # actual 记录: (ts, group, ji, name, actual)
                        a_records = [r for r in actual_records if r[1] == group and r[2] == ji]
                        al.set_data([r[0] for r in a_records], [r[4] for r in a_records])

                        all_ts = [r[0] for r in ji_records] + ck_x + [r[0] for r in a_records]
                        if all_ts:
                            ax.set_xlim(min(all_ts), max(all_ts) + 0.1)
                        ax.relim()
                        ax.autoscale_view(scaley=True)

                    self._live_fig.savefig(
                        self._live_png_path, dpi=100, bbox_inches="tight",
                    )

            except Exception:
                pass

            time.sleep(0.25)

        import matplotlib.pyplot as plt
        plt.close(self._live_fig)
        self._live_fig = None
        self._live_axes = []

    def stop_live_plot(self) -> None:
        """停止实时绘图线程。"""
        if not self._live_running:
            return
        self._live_running = False
        if self._live_thread is not None and self._live_thread.is_alive():
            self._live_thread.join(timeout=2.0)
            self._live_thread = None
        logger.info("Live plot stopped")

    def save(self) -> Path:
        """将记录落盘为 CSV 文件，返回文件路径。"""
        csv_path = self._output_dir / "actions.csv"

        with self._lock:
            records = list(self._records)

        fieldnames = ["timestamp", "step", "group", "joint_idx", "joint_name", "target", "current"]
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(fieldnames)
            writer.writerows(records)

        file_size_kb = csv_path.stat().st_size / 1024
        logger.info(
            "ActionRecorder saved %d records (%d steps) → %s (%.1f KB)",
            len(records), self._step, csv_path, file_size_kb,
        )
        # chunks.csv（act_async 异步推理 chunk；与 actions.csv 互补）
        with self._lock:
            chunk_records = list(self._chunk_records)
        if chunk_records:
            chunk_csv = self._output_dir / "chunks.csv"
            chunk_fields = [
                "timestamp", "chunk_id", "point_idx", "group", "joint_idx",
                "joint_name", "target", "current", "inference_time_sec", "fps", "n_points",
            ]
            with open(chunk_csv, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(chunk_fields)
                writer.writerows(chunk_records)
            n_chunks = len({r[1] for r in chunk_records})
            logger.info(
                "ActionRecorder saved %d chunk records (%d chunks) -> %s (%.1f KB)",
                len(chunk_records), n_chunks, chunk_csv, chunk_csv.stat().st_size / 1024,
            )
        # actual.csv（实际位置，60Hz 降采样；来自桥接 status 流，_process_status 路由）
        with self._lock:
            actual_records = list(self._actual_records)
        if actual_records:
            actual_csv = self._output_dir / "actual.csv"
            actual_fields = ["timestamp", "group", "joint_idx", "joint_name", "actual"]
            with open(actual_csv, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(actual_fields)
                writer.writerows(actual_records)
            logger.info(
                "ActionRecorder saved %d actual records -> %s (%.1f KB)",
                len(actual_records), actual_csv, actual_csv.stat().st_size / 1024,
            )
        return csv_path

    def plot(self, title: str = "Walker S2 Action Timeline") -> Path | None:
        """生成时序 PNG 图，每个关节一个子图，返回文件路径。"""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            logger.warning("matplotlib not available, skipping plot generation")
            return None

        # 找出有数据的关节
        with self._lock:
            records = list(self._records)
        if not records:
            logger.warning("No records to plot")
            return None

        joints = self._build_joint_flat_list()
        # 过滤有数据的关节
        active_joints: list[tuple[str, int, str]] = []
        for group, ji, jname in joints:
            vals = [r[5] for r in records if r[2] == group and r[3] == ji]
            if vals and any(not _is_nan(v) for v in vals):
                active_joints.append((group, ji, jname))

        if not active_joints:
            logger.warning("No active joints with data to plot")
            return None

        n_joints = len(active_joints)
        fig, axes = plt.subplots(n_joints, 1, figsize=(16, 2.0 * n_joints), sharex=True)
        if n_joints == 1:
            axes = [axes]

        for ax, (group, ji, jname) in zip(axes, active_joints):
            ji_records = [r for r in records if r[2] == group and r[3] == ji]
            ts_vals = [r[0] for r in ji_records]
            t_vals = [r[5] for r in ji_records]
            c_vals = [r[6] for r in ji_records]

            ax.plot(ts_vals, t_vals, "C0-", linewidth=0.6, marker=".", markersize=1.5, label="cmd")
            if c_vals and any(not _is_nan(v) for v in c_vals):
                ax.plot(ts_vals, c_vals, "C1--", linewidth=0.4, marker=".", markersize=1.0, alpha=0.6, label="obs")

            ax.set_ylabel(jname, fontsize=7)
            ax.legend(fontsize=6, loc="upper right")
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=6)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle(title, fontsize=12, fontweight="bold")
        fig.tight_layout(pad=1.0)

        png_path = self._output_dir / "actions.png"
        fig.savefig(png_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        file_size_kb = png_path.stat().st_size / 1024
        logger.info("ActionRecorder plot saved %d joints → %s (%.1f KB)", n_joints, png_path, file_size_kb)
        return png_path


    def plot_chunks(self, title: str = "Walker S2 Trajectory Comparison") -> Path | None:
        """生成三轨迹对比 PNG：预测块 / 融合目标 / 实际位置，按预计执行时间对齐。

        每个关节一个子图，统一 x 轴 = 预计执行时间（相对 ``_start_time`` 秒）：
        - predicted（蓝淡）：raw chunk target，x = gen_ts + pt/fps（按 fps 展开每点执行时刻）
        - fused（红）：``ChunkFuser.blend`` 输出（读桥接 ``fused.csv``），x = consume_ts + ramp + k/fps
        - actual（绿）：机器人实际位置（``actual.csv`` 数据），x = status ts

        预测/融合按 chunk_id 分段绘制，避免 receding-horizon 重叠段跨块连线 zigzag。
        无 chunk 记录时（未用 act_async）返回 None。
        """
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            logger.warning("matplotlib not available, skipping chunk plot")
            return None

        with self._lock:
            records = list(self._chunk_records)
            actual_records = list(self._actual_records)
        if not records:
            logger.info("No chunk records to plot (act_async not used or no chunks produced)")
            return None

        fused_by_joint = self._load_fused_records()

        # chunk 元组: (ts, chunk_id, pt, group, ji, name, target, current, inf, fps, n)
        # actual 元组: (ts, group, ji, name, actual)
        joints = self._build_joint_flat_list()
        active_joints: list[tuple[str, int, str]] = []
        for group, ji, jname in joints:
            ck_vals = [r[6] for r in records if r[3] == group and r[4] == ji]
            a_vals = [r[4] for r in actual_records if r[1] == group and r[2] == ji]
            has_chunk = bool(ck_vals) and any(not _is_nan(v) for v in ck_vals)
            has_actual = bool(a_vals) and any(not _is_nan(v) for v in a_vals)
            has_fused = (group, ji) in fused_by_joint
            if has_chunk or has_actual or has_fused:
                active_joints.append((group, ji, jname))
        if not active_joints:
            logger.warning("No active joints with data to plot")
            return None

        # chunk 边界：每块首点(point_idx==0)的预计执行时间 = ts + 0/fps = ts
        chunk_starts = sorted({r[0] for r in records if r[2] == 0})

        n_joints = len(active_joints)
        fig, axes = plt.subplots(n_joints, 1, figsize=(16, 2.0 * n_joints), sharex=True)
        if n_joints == 1:
            axes = [axes]

        for ax, (group, ji, jname) in zip(axes, active_joints):
            # predicted：按 chunk_id 分段，x = gen_ts + pt/fps（按 fps 展开执行时刻）
            jr = [r for r in records if r[3] == group and r[4] == ji]
            pred_by_cid: dict[int, list[tuple[float, float]]] = {}
            for r in jr:
                fps = r[9]
                x = r[0] + r[2] / fps if (fps and fps > 0) else r[0]
                pred_by_cid.setdefault(r[1], []).append((x, r[6]))
            first_p = True
            for cid, pts in pred_by_cid.items():
                pts.sort(key=lambda p: p[0])
                ax.plot([p[0] for p in pts], [p[1] for p in pts], "C0-",
                        linewidth=0.5, marker=".", markersize=1.2, alpha=0.5,
                        label="predicted" if first_p else None)
                first_p = False

            # fused：按 chunk_id 分段，x = exec_time_rel（已减 _start_time）
            fused_pts = fused_by_joint.get((group, ji), [])
            fused_by_cid: dict[int, list[tuple[float, float]]] = {}
            for rel, val, cid in fused_pts:
                fused_by_cid.setdefault(cid, []).append((rel, val))
            first_f = True
            for cid, pts in fused_by_cid.items():
                pts.sort(key=lambda p: p[0])
                ax.plot([p[0] for p in pts], [p[1] for p in pts], "C3-",
                        linewidth=0.6, alpha=0.85, label="fused" if first_f else None)
                first_f = False

            # actual：连续流，单线
            ar = [r for r in actual_records if r[1] == group and r[2] == ji]
            if ar:
                ax.plot([r[0] for r in ar], [r[4] for r in ar], "C2-",
                        linewidth=0.7, alpha=0.9, label="actual")

            for t0 in chunk_starts:
                ax.axvline(t0, color="gray", linewidth=0.3, alpha=0.4)

            ax.set_ylabel(jname, fontsize=7)
            ax.legend(fontsize=6, loc="upper right")
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=6)

        axes[-1].set_xlabel("Time (s, estimated execution)")
        fig.suptitle(f"{title} ({len(chunk_starts)} chunks)", fontsize=12, fontweight="bold")
        fig.tight_layout(pad=1.0)

        png_path = self._output_dir / "chunks.png"
        fig.savefig(png_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        file_size_kb = png_path.stat().st_size / 1024
        logger.info("ActionRecorder trajectory plot saved %d joints -> %s (%.1f KB)", n_joints, png_path, file_size_kb)
        return png_path


def _is_nan(v: float) -> bool:
    """检查浮点数是否为 NaN（兼容 Python 的 float('nan') 检查）。"""
    return v != v
