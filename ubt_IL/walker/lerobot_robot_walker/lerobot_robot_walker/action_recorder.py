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
        self._step: int = 0
        self._start_time: float | None = None
        self._lock = threading.Lock()

        # 实时绘图状态
        self._live_running: bool = False
        self._live_thread: threading.Thread | None = None

        logger.info("ActionRecorder initialized, output dir: %s", self._output_dir)

    # ---- 公开 API ----------------------------------------------------------------

    def record(
        self,
        action_input: dict[str, float],
        action_msg: dict[str, Any],
        current_state: dict[str, list[float]],
        *,
        timestamp: float | None = None,
    ) -> None:
        """记录一步动作。

        Args:
            action_input: 策略推理输出（仅策略关节，key=feature_name.pos）。
            action_msg: 最终发给 Bridge2/ROS2 的 6 组 ZMQ 消息。
            current_state: 当前机器人关节状态（6 组，每组一个 list）。
            timestamp: 绝对时间戳，None 则用相对时间。
        """
        if self._start_time is None:
            self._start_time = timestamp if timestamp is not None else time.perf_counter()

        ts = (timestamp - self._start_time) if timestamp is not None else (time.perf_counter() - self._start_time)

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

        # 每条关节一根实线(target) + 一根虚线(current)，每个子图一个关节
        self._live_joint_lines: list[tuple] = []  # [(ax, group, ji, t_line, c_line), ...]

        for ax, (group, ji, jname) in zip(self._live_axes, self._live_joints):
            (tl,) = ax.plot([], [], "C0-", linewidth=0.8, label="cmd")
            (cl,) = ax.plot([], [], "C1--", linewidth=0.5, alpha=0.6, label="obs")
            self._live_joint_lines.append((ax, group, ji, tl, cl))
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

                if records:
                    for ax, group, ji, tl, cl in self._live_joint_lines:
                        ji_records = [r for r in records if r[2] == group and r[3] == ji]
                        ts_vals = [r[0] for r in ji_records]
                        t_vals = [r[5] for r in ji_records]
                        c_vals = [r[6] for r in ji_records]
                        tl.set_data(ts_vals, t_vals)
                        cl.set_data(ts_vals, c_vals)
                        if ts_vals:
                            ax.set_xlim(ts_vals[0], ts_vals[-1] + 0.1)
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


def _is_nan(v: float) -> bool:
    """检查浮点数是否为 NaN（兼容 Python 的 float('nan') 检查）。"""
    return v != v
