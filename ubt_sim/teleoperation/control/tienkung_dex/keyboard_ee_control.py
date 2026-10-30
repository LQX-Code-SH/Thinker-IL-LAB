#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 键盘末端遥操作（仿真/真机同一份脚本）。

通过 pynput 捕获键盘，把增量累积到**目标变量**（``_target_pose``），
再由发送器按 ``send_hz`` 读该变量解 IK 下发关节轨迹。鼠标通道已弃用。

控制变量与读数的关系（三者严格分离）::

    _target_pose   目标变量，唯一事实源；键盘只改它，实测永不回写
    _sent_pose     已成功下发到的位姿；下发被拒时不前推，下一拍重试
    _actual_pose   实测末端缓存；只用于显示随动误差，绝不回写目标

**没有跟随误差门限**，防止"目标跑飞"靠的是输入节流：

    按键采样   ``--input-hz``          默认 1 Hz，每窗口最多消化一步
    单步位移   ``--translation-step``  默认 12 mm（= 单次下发上限）

这里只认"按下"边沿：**按一次 = 一步**；1 s 采样窗口内的多次按键合并成一步
（连点三下也只走一步）。按住不放时 X 的自动重复会补出新的按下边沿，所以
等效为每秒一步（N 秒长按实测走 N+1 步：按下瞬间所在窗口立刻兑现一步）。
于是目标速度上限就是 `translation_step × input_hz` = 12 mm/s，
而发送器 0.2 s 就能走完 12 mm —— 手臂始终跟得上目标，误差攒不起来。
目标变量一旦确定就只被键盘增量修改，**永远不会被实测位姿回写**。

**平移在基座系**（URDF pelvis 原点，x=前 / y=左 / z=上），方向不随手掌姿态
漂移：手掌前倾时按 A 仍是水平左移，不会斜着往上跑。**旋转仍在掌根帧**
（``left_hand_base`` / ``right_hand_base``，相对手，左右手各自的转向才符合直觉）::

    x = 手掌下方     y = 手掌左方     z = 手掌前方（手指指向）

键位::

    W/S       前后移动（基座 ±x，W=前）
    A/D       左右移动（基座 ±y，A=左）
    Q/E       上下移动（基座 ±z，Q=上）
    1/2       绕掌根 z 轴旋转（roll，沿手指轴的拧转）  −/+
    4/5       绕掌根 x 轴旋转（yaw，手掌左右摆）      −/+
    7/8       绕掌根 y 轴旋转（pitch，手掌上下摆）    −/+
               数字键：奇数 = 负方向，偶数 = 正方向（右手定则）
    3         切换左右臂（边沿触发；切换后目标重置为该臂的预备位期望）
    空格       当前手臂的灵巧手 张开/握拳 切换（边沿触发；两种状态都保持预备位
               手型——拇指侧摆留在 850 = 90°，握拳只弯其余自由度）
    T         启停切换（**臂和手一起停**，空格在暂停期间被忽略；不重置目标变量）
    ESC       退出

运行（容器内，需 X11 转发；headless 请在宿主机跑）::

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0            # 默认与真机同域
    /usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_ee_control.py --side right

重启想接着上次的**目标**走（不是实测）时，把日志里最后那行目标位姿传进去::

    ... --seed 0.314 -0.286 0.092 1.047 0.392 1.228
"""

import argparse
import logging
import os
import sys
import threading
import time
from queue import Queue

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

# ------------------------------------------------------------------
# pynput 可用性检查
# ------------------------------------------------------------------
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        logging.warning("未检测到 DISPLAY 环境变量，pynput 可能无法捕获键盘事件。")
        logging.warning("如果在 headless 容器内运行，请在宿主机上执行本脚本。")
    from pynput import keyboard  # noqa: E402
except Exception as e:  # noqa: BLE001
    logging.error(f"无法导入 pynput: {e}")
    logging.error("请安装: pip install pynput")
    sys.exit(1)

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

import constants as C  # noqa: E402
from dex_controller import (  # noqa: E402
    DexController,
    _add_common_args,
    _make_controller,
    _shutdown,
)

# ============================================================================
# RPY 工具（与 dex_controller / pinocchio 的 R = Rz(yaw) Ry(pitch) Rx(roll) 一致）
# ============================================================================
def _rpy_to_rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rotation_matrix_to_rpy(R):
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if abs(np.cos(pitch)) > 1e-10:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return float(roll), float(pitch), float(yaw)


# ============================================================================
# 默认参数
# ============================================================================
# 步长是"每一次按键"的增量，不是"每一个循环节拍"的增量：按键按
# ``--input-hz``（默认 1 Hz）采样，一个窗口最多走一步。平移步长取 12 mm
# 正好等于单次下发上限 ``DEFAULT_MAX_DELTA_XYZ``，于是**一次按键 = 一次下发**，
# 不会出现"目标跑出去了、发送器还要追好几拍"的情况。
DEFAULT_TRANSLATION_STEP = 0.012     # 平移步长 (m/次按键)
DEFAULT_ROTATION_STEP = 0.05         # 键盘旋转步长 (rad/次按键)
DEFAULT_POLL_HZ = 200.0              # 主循环频率（发送器节拍）
DEFAULT_INPUT_HZ = 1.0               # 按键采样频率：每 1 s 消化一次按键
DEFAULT_SEND_HZ = 30.0               # 指令下发频率
# 轨迹时长与单次最大位移要和安全阈值一起看：DexController 的限速检查是
# 速度 = max(Δs)·Δq / timer_period(0.02 s) ≤ 1.5 rad/s，五阶插值的 max(Δs)
# 由点数决定：0.12 s→6 点系数 18.26（Δq 上限 0.082 rad），0.2 s→10 点系数
# 10.33（上限 0.145 rad）。预备位下 12 mm 平移约需 Δq≈0.09，留 1.6 倍余量；
# 旧的 0.12 s / 20 mm 组合需要 Δq≈0.153（2.8 rad/s），必然被拒。
DEFAULT_DURATION = 0.2               # 每帧轨迹时长 (s)
DEFAULT_MAX_DELTA_XYZ = 0.012        # 单次下发最大平移步长 (m)
DEFAULT_MAX_DELTA_RPY = 0.10         # 单次下发最大旋转步长 (rad)
DEFAULT_ACTUAL_HZ = 20.0             # 实测末端缓存刷新频率 (Hz，只用于显示随动误差)

# 参与"按一次走一步"的键：WASDQE 平移 + 1/2、4/5、7/8 绕掌根 z/x/y 旋转。
# 其余键（空格 / 3 / T / ESC）走实时边沿判断，不参与 1 Hz 采样。
_MOVE_KEYS = frozenset((
    "w", "s", "a", "d", "q", "e",
    "1", "2", "4", "5", "7", "8",
))


class KeyboardEEController:
    """键盘末端遥操作控制器（事件队列 + 状态字典，同 LeRobot 模式）。"""

    def __init__(
        self,
        controller: DexController,
        side: str = "right",
        translation_step: float = DEFAULT_TRANSLATION_STEP,
        rotation_step: float = DEFAULT_ROTATION_STEP,
        poll_hz: float = DEFAULT_POLL_HZ,
        input_hz: float = DEFAULT_INPUT_HZ,
        send_hz: float = DEFAULT_SEND_HZ,
        duration: float = DEFAULT_DURATION,
        max_delta_xyz: float = DEFAULT_MAX_DELTA_XYZ,
        max_delta_rpy: float = DEFAULT_MAX_DELTA_RPY,
        actual_hz: float = DEFAULT_ACTUAL_HZ,
        seed_xyzrpy: list[float] | None = None,
    ):
        self._controller = controller
        self.side = side
        # 仅用于**首次**播种（--seed）：重启遥操作时接着上次的目标位姿走。
        # 切臂不沿用（左右臂数值不可互换），仍用该臂的 READY_POSE。
        self.seed_xyzrpy = list(seed_xyzrpy) if seed_xyzrpy is not None else None
        self.translation_step = translation_step
        self.rotation_step = rotation_step
        self.poll_hz = poll_hz
        self.input_hz = max(float(input_hz), 0.0)
        self.send_hz = send_hz
        self.duration = duration
        self.poll_period = 1.0 / poll_hz
        self.input_period = (1.0 / self.input_hz) if self.input_hz > 0 else 0.0
        self.send_period = 1.0 / send_hz
        self.max_delta_xyz = max_delta_xyz
        self.max_delta_rpy = max_delta_rpy
        self.actual_period = 1.0 / max(float(actual_hz), 1.0)

        self._key_event_queue: Queue = Queue()
        self.current_pressed: dict = {}

        self._hand_open: dict[str, bool] = {"left": True, "right": True}
        self._space_prev = False
        self._key3_prev = False
        self._key_t_prev = False
        self._enabled = True

        # ---- 目标变量（唯一事实源）----
        # 键盘只改 ``_target_pose``，发送器只读它；机器人实测只进
        # ``_actual_pose`` 缓存，用于显示随动误差，不回写目标。
        self._target_pose: np.ndarray | None = None   # 意图：要去的末端位姿
        self._sent_pose: np.ndarray | None = None     # 已成功下发到的位姿
        self._actual_pose: np.ndarray | None = None   # 实测缓存（限频刷新）
        self._last_actual_time = 0.0
        self._last_input_time = 0.0
        # 上一次按键采样以来按下的移动键（只记"按下"边沿）
        self._pending_keys: set = set()
        self._last_reject_warn = 0.0
        self._key_listener: keyboard.Listener | None = None
        self._running = False

    # ------------------------------------------------------------------ 回调
    def _on_key_press(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = key
        self._key_event_queue.put((key_char, True))
        # 移动键只留"按下"边沿：按键采样窗口把边沿合并成一步（见模块 docstring）
        if key_char in _MOVE_KEYS:
            self._pending_keys.add(key_char)

    def _on_key_release(self, key):
        try:
            key_char = key.char
        except AttributeError:
            key_char = key
        self._key_event_queue.put((key_char, False))
        if key == keyboard.Key.esc:
            self._running = False

    def _drain_key_events(self):
        while not self._key_event_queue.empty():
            key, is_pressed = self._key_event_queue.get_nowait()
            self.current_pressed[key] = is_pressed

    def _is_pressed(self, key) -> bool:
        return self.current_pressed.get(key, False)

    def _apply_key_step(self, keys: set) -> None:
        """把一次按键采样累加到目标变量：一个采样窗口最多走一步。

        平移直接在**基座系**累加（W/S = ±x 前后，A/D = ±y 左右，Q/E = ±z 上下），
        方向恒定、不随手掌姿态变化；旋转按**掌根系**合成，绕手转才符合左右手
        各自的直觉：1/2 绕手指轴 z，4/5 绕手掌下方 x，7/8 绕手掌左方 y。
        """
        step = self.translation_step
        d_base = np.zeros(3)  # x=前, y=左, z=上
        if "w" in keys:
            d_base[0] += step
        if "s" in keys:
            d_base[0] -= step
        if "a" in keys:
            d_base[1] += step
        if "d" in keys:
            d_base[1] -= step
        if "q" in keys:
            d_base[2] += step
        if "e" in keys:
            d_base[2] -= step

        # 旋转 delta（掌根系：dd[0] 绕 x, dd[1] 绕 y, dd[2] 绕 z；奇数键负、偶数键正）
        rot_step = self.rotation_step
        dd = np.zeros(3)
        if "1" in keys:
            dd[2] -= rot_step          # roll  负
        if "2" in keys:
            dd[2] += rot_step          # roll  正
        if "4" in keys:
            dd[0] -= rot_step          # yaw   负
        if "5" in keys:
            dd[0] += rot_step          # yaw   正
        if "7" in keys:
            dd[1] -= rot_step          # pitch 负
        if "8" in keys:
            dd[1] += rot_step          # pitch 正

        if not (np.any(np.abs(d_base) > 1e-9) or np.any(np.abs(dd) > 1e-9)):
            return

        cand = self._target_pose.copy()
        cand[:3] += d_base
        if np.any(np.abs(dd) > 1e-9):
            R_t = _rpy_to_rotation_matrix(cand[3], cand[4], cand[5])
            cand[3], cand[4], cand[5] = _rotation_matrix_to_rpy(
                R_t @ _rpy_to_rotation_matrix(*dd)
            )
        self._target_pose = cand
        self._log_step()

    def _log_step(self) -> None:
        """每走一步打一行：目标位姿 + 与实测末端的随动误差（只显示，不干预）。"""
        line = (f"[{self.side}] {time.strftime('%H:%M:%S')} 目标 xyz=("
                f"{self._target_pose[0]:+.3f}, {self._target_pose[1]:+.3f}, "
                f"{self._target_pose[2]:+.3f})"
                f" rpy=({self._target_pose[3]:+.3f}, {self._target_pose[4]:+.3f}, "
                f"{self._target_pose[5]:+.3f})")
        if self._actual_pose is not None:
            d = self._target_pose[:3] - self._actual_pose[:3]
            line += (f"  随动 水平 {np.hypot(d[0], d[1]) * 100:.1f} cm"
                     f" / 竖直 {d[2] * 100:+.1f} cm")
        print(line)

    def _seed_target_from_ready(self, why: str = "", pose=None) -> bool:
        """把目标变量重置为**预备位目标的期望末端**（``READY_POSE_TARGET`` 的
        正运动学），不用实测位姿。

        目标是 ``READY_POSE_TARGET`` 而不是设计姿态 ``READY_POSE``：加仿真重力前馈
        之后手臂真的停在"下发的那个目标"上，而预备位刻意下发的是旧版下垂姿态
        （见 constants）。用设计姿态播种会高出 0.13 m，第一次按键目标就得先补这
        13 cm——2026-10-27 实测就是 [0.4338,-0.4415,0.3082] vs 实测
        [0.4301,-0.4232,0.1766]。

        目标变量是纯命令流：一旦给出就只被键盘增量修改，实测永远不回写它。
        ``pose`` 给定时改用该位姿作初值（``--seed``，重启遥操作时接着上次的目标走）；
        切臂仍固定用预备位——左右臂的 ``--seed`` 数值不可互换。
        """
        ee_ready = (
            np.array(pose, dtype=float) if pose is not None
            else self._controller.fk_ee_pose(self.side, C.READY_POSE_TARGET)
        )
        if ee_ready is None:
            print(f"[{self.side}] 目标重置失败（无法计算预备位末端）{why}")
            self._target_pose = None
            self._sent_pose = None
            return False
        self._target_pose = np.array(ee_ready, dtype=float)
        self._sent_pose = self._target_pose.copy()
        # 实测只进显示缓存；拿不到就退回目标本身（随动误差显示 0），绝不参与目标
        ee_actual = self._controller.get_ee_pose(self.side)
        self._actual_pose = (
            np.array(ee_actual, dtype=float)
            if ee_actual is not None else self._target_pose.copy()
        )
        label = "指定种子位姿" if pose is not None else "预备位期望"
        print(f"[{self.side}] 目标已重置为{label}{why}: "
              f"xyz={[round(float(v), 4) for v in self._target_pose[:3]]} "
              f"rpy={[round(float(v), 4) for v in self._target_pose[3:]]}")
        return True

    # -------------------------------------------------------------- 主循环
    def run(self):
        self._key_listener = keyboard.Listener(
            on_press=self._on_key_press,
            on_release=self._on_key_release,
        )
        try:
            self._key_listener.start()
        except Exception as e:  # noqa: BLE001
            print(f"\nERROR: 无法启动键盘监听器: {e}")
            print("pynput 需要 X11。headless 容器请在宿主机运行本脚本，或挂载 DISPLAY：")
            print("  docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...")
            raise
        self._running = True
        self._print_banner()

        print(f"读取 {self.side} 臂预备位的期望末端姿态...")
        # 目标变量 = 单一事实源：键盘只改它，发送器只读它，实测只进 _actual_pose。
        # 初值取**预备位目标的期望末端**（READY_POSE_TARGET 的正运动学），不是实测：
        # 目标变量是纯命令流，拿实测会在有残差时把目标钉在实测那个点上。而
        # READY_POSE_TARGET 正是此刻下发的命令，加了重力前馈后两者只差毫米级
        # （一般姿态 ≤15 mm，预备位 0.1 mm 级），所以既不会一上来就跳，也不用
        # 操作者先"还"掉下垂。发送器的 _sent_pose 同源——指令流从这个期望位姿
        # 开始累进（tienkung_pro 的 move_arm 同样从命令值起步）。
        if not self._seed_target_from_ready(pose=self.seed_xyzrpy):
            print("ERROR: 无法计算预备位末端姿态，请检查 IK 初始化与机器人状态")
            self._running = False
            return
        dz0 = float(self._actual_pose[2] - self._target_pose[2])
        print(f"  当前实测 (base frame): "
              f"xyz={[round(float(v), 4) for v in self._actual_pose[:3]]}"
              f"（竖直随动误差 {dz0 * 100:+.1f} cm）")
        print(f"  按键采样: {self.input_hz:g} Hz（按一次 = 一步，"
              f"{self.translation_step * 1000:.0f} mm / "
              f"{self.rotation_step:.3f} rad；无跟随误差门限）")

        frame_count = 0
        last_send_time = time.perf_counter()
        self._last_input_time = last_send_time
        self._last_actual_time = last_send_time

        try:
            while self._running and rclpy.ok():
                loop_start = time.perf_counter()
                frame_count += 1
                self._drain_key_events()

                now = time.perf_counter()

                # --- 实测末端缓存（限频，只用于显示随动误差；不回写目标）---
                if now - self._last_actual_time >= self.actual_period:
                    ee_actual = self._controller.get_ee_pose(self.side)
                    if ee_actual is not None:
                        self._actual_pose = np.array(ee_actual, dtype=float)
                    self._last_actual_time = now

                # --- 键盘：按 --input-hz 采样，一个窗口最多走一步 ---
                # 只认"按下"边沿：窗口内的多次按下合并成一步（连点三下 = 一步），
                # 按住不放时 X 自动重复会补边沿，等效每秒一步。目标速度因此被夹在
                # translation_step*input_hz，不用看实测末端也能保证手臂跟得上
                # （所以没有跟随误差门限）。
                if now - self._last_input_time >= self.input_period:
                    keys = self._pending_keys
                    self._pending_keys = set()
                    self._last_input_time = now
                    if keys and self._enabled and self._target_pose is not None:
                        self._apply_key_step(keys)

                # --- 发送器：固定频率把目标变量推给机器人 ---
                if (self._enabled and self._target_pose is not None
                        and self._sent_pose is not None
                        and now - last_send_time >= self.send_period
                        and not self._controller.is_busy):
                    step_xyz = self._target_pose[:3] - self._sent_pose[:3]
                    step_rpy = self._target_pose[3:] - self._sent_pose[3:]
                    n_xyz = float(np.linalg.norm(step_xyz))
                    n_rpy = float(np.linalg.norm(step_rpy))
                    if n_xyz > 1e-9 or n_rpy > 1e-9:
                        # 单次步长要压到限速检查允许的范围内（推导见文件头常量注释）；
                        # 目标离得再远也分几拍走完，绝不一次性甩过去。
                        if n_xyz > self.max_delta_xyz:
                            step_xyz = step_xyz * (self.max_delta_xyz / n_xyz)
                        if n_rpy > self.max_delta_rpy:
                            step_rpy = step_rpy * (self.max_delta_rpy / n_rpy)
                        goal = self._sent_pose.copy()
                        goal[:3] += step_xyz
                        goal[3:] += step_rpy
                        ok = self._controller.move_arm_ik(
                            self.side,
                            target_xyzrpy=goal.tolist(),
                            duration_sec=self.duration,
                            wait=False,
                            require_success=False,
                            max_iter=300,
                            # 从上次下发的关节目标起步（tienkung_pro 的做法）：
                            # 仿真的隐式 PD 有 τ_g/k 的静态下垂，从实测起步会把
                            # 这段下垂每拍重走一遍，触发 1.80 rad/s 的限速拒绝。
                            ramp_from_commanded=True,
                        )
                        if ok:
                            self._sent_pose = goal
                        elif now - self._last_reject_warn >= 1.0:
                            # 不下发成功就不前推 _sent_pose，下一拍重试同一步；
                            # 目标变量也不动，操作者的意图不会丢。
                            self._last_reject_warn = now
                            print(f"[{self.side}] 下发被拒（限速/限位），下一拍重试")
                    last_send_time = now

                # --- 空格：手开合（边沿触发，始终保持预备位手型）---
                # 与手臂共用 _enabled：T 暂停时整条输出通道都停（手也不动），
                # 暂停期间按下的空格被当拍吃掉，恢复后不会补发。
                space_pressed = self._is_pressed(keyboard.Key.space)
                if space_pressed and not self._space_prev and not self._enabled:
                    print(f"[{self.side}] 控制已暂停，空格忽略（先按 T 恢复）")
                if space_pressed and not self._space_prev and self._enabled:
                    if self._hand_open[self.side]:
                        self._controller.close_hand(
                            self.side,
                            duration_sec=0.4,
                            wait=False,
                            base_pose=C.ready_hand_pose(self._controller.variant),
                        )
                        self._hand_open[self.side] = False
                        print(f"[{self.side}] 手：握拳（拇指侧摆保持预备位 90°）")
                    else:
                        self._controller.apply_ready_hand_pose(
                            duration_sec=0.4, wait=False, side=self.side
                        )
                        self._hand_open[self.side] = True
                        print(f"[{self.side}] 手：预备位张开（拇指侧摆 90°）")
                self._space_prev = space_pressed

                # --- 3：切换手臂 ---
                key3_pressed = self._is_pressed("3")
                if key3_pressed and not self._key3_prev:
                    old_side = self.side
                    self.side = "left" if self.side == "right" else "right"
                    self.current_pressed["3"] = False
                    self._pending_keys.clear()
                    print(f"切换手臂: {old_side} → {self.side}"
                          f"（目标重置为该臂的预备位期望）")
                    self._seed_target_from_ready()
                self._key3_prev = key3_pressed

                # --- T：启停切换（不碰目标变量）---
                key_t_pressed = self._is_pressed("t")
                if key_t_pressed and not self._key_t_prev:
                    self._enabled = not self._enabled
                    self._pending_keys.clear()
                    print(f"[{self.side}] 控制{'启用' if self._enabled else '暂停'}"
                          f"（目标变量保持不变）")
                    self.current_pressed["t"] = False
                self._key_t_prev = key_t_pressed

                # --- 频率控制 ---
                elapsed = time.perf_counter() - loop_start
                sleep_time = self.poll_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception:
            self._controller.get_logger().error("KeyboardEEController loop crashed", exc_info=True)
            raise
        finally:
            if self._key_listener is not None:
                self._key_listener.stop()
            print(f"\n键盘 EE 控制已停止（共 {frame_count} 帧）")

    def _print_banner(self):
        deg = lambda rad: f"{np.degrees(rad):.1f}°"  # noqa: E731
        print(f"\n{'=' * 60}")
        print("  键盘 EE 控制（TienKung Dex）")
        print(f"{'=' * 60}")
        print(f"  通道:       ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} / 手型 {self._controller.variant}")
        print(f"  手臂:       {self.side}（按 3 切换）")
        print(f"  平移步长:   {self.translation_step * 1000:.1f} mm/次按键")
        print(f"  旋转步长:   {self.rotation_step:.4f} rad/次按键  ({deg(self.rotation_step)})")
        print(f"  按键采样:   {self.input_hz:g} Hz（按一次 = 一步，无跟随门限）")
        print(f"  主循环:     {self.poll_hz} Hz   信号发送: {self.send_hz} Hz")
        print(f"  轨迹时长:   {self.duration:.3f} s")
        print(f"  最大位移:   {self.max_delta_xyz * 1000:.0f} mm / {self.max_delta_rpy:.3f} rad"
              f"（每拍下发上限）")
        print(f"  安全检查:   {'启用' if self._controller.enable_safety_check else '禁用'}")
        print(f"{'=' * 60}")
        print("  平移走基座系（x=前 y=左 z=上）；旋转用掌根帧（相对手）")
        print("  W/S:       前后（基座 x）      A/D: 左右（基座 y）")
        print("  Q/E:       上下（基座 z）      空格: 手 预备位张开/握拳")
        print("  1/2:       绕掌根 z 拧转 roll   −/+")
        print("  4/5:       绕掌根 x 左右摆 yaw  −/+")
        print("  7/8:       绕掌根 y 上下摆 pitch −/+")
        print("  数字键奇数 = 负方向，偶数 = 正方向；所有移动/旋转键按一次走一步")
        print("  3:         切换左右臂（目标重置为预备位期望）")
        print("  T:         启停切换（臂+手一起停，不重置目标）   ESC: 退出")
        print(f"{'=' * 60}\n")


# ============================================================================
# CLI
# ============================================================================
def main(args=None):
    parser = argparse.ArgumentParser(
        description="TienKung Dex 键盘末端遥操作",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 keyboard_ee_control.py --side right
  python3 keyboard_ee_control.py --side right --input-hz 2 --translation-step 0.008
  python3 keyboard_ee_control.py --side left --translation-step 0.005
  python3 keyboard_ee_control.py --poll-hz 100 --send-hz 20 --quiet
""",
    )
    _add_common_args(parser)
    parser.add_argument("--side", default="right", choices=["left", "right"], help="初始控制手臂")
    parser.add_argument("--translation-step", type=float, default=DEFAULT_TRANSLATION_STEP,
                        help=f"每次按键的平移步长 m（默认 {DEFAULT_TRANSLATION_STEP}，"
                             f"等于单次下发上限）")
    parser.add_argument("--rotation-step", type=float, default=DEFAULT_ROTATION_STEP,
                        help=f"每次按键的键盘旋转步长 rad（默认 {DEFAULT_ROTATION_STEP}）")
    parser.add_argument("--input-hz", type=float, default=DEFAULT_INPUT_HZ,
                        help=f"按键采样频率 Hz（默认 {DEFAULT_INPUT_HZ}）：一个窗口最多走一步，"
                             f"窗口内的多次按键合并")
    parser.add_argument("--poll-hz", type=float, default=DEFAULT_POLL_HZ,
                        help="主循环频率（发送器节拍）")
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ, help="指令下发频率")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION, help="每帧轨迹时长 s")
    parser.add_argument("--max-delta-xyz", type=float, default=DEFAULT_MAX_DELTA_XYZ,
                        help="单次下发最大平移累积 m")
    parser.add_argument("--max-delta-rpy", type=float, default=DEFAULT_MAX_DELTA_RPY,
                        help="单次下发最大旋转累积 rad")
    parser.add_argument("--actual-hz", type=float, default=DEFAULT_ACTUAL_HZ,
                        help="实测末端缓存刷新频率 Hz（只用于显示随动误差）")
    parser.add_argument("--seed", type=float, nargs=6, default=None,
                        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                        help="首次目标初值（基座系 m/rad）。默认取预备位期望；"
                             "重启遥操作时传上一次的目标位姿即可接着走（切臂仍回预备位）")
    parser.add_argument("--quiet", "-q", action="store_true", help="只输出 WARN/ERROR")
    cli = parser.parse_args(args)

    rclpy.init()
    controller = _make_controller(cli, "tienkung_dex_teleop", enable_ik=True)
    if cli.quiet:
        controller.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            print("ERROR: 等待机器人状态超时，请检查桥接是否在跑")
            return
        print("机器人状态已连接，启动键盘控制...")
        kb = KeyboardEEController(
            controller=controller,
            side=cli.side,
            translation_step=cli.translation_step,
            rotation_step=cli.rotation_step,
            poll_hz=cli.poll_hz,
            input_hz=cli.input_hz,
            send_hz=cli.send_hz,
            duration=cli.duration,
            max_delta_xyz=cli.max_delta_xyz,
            max_delta_rpy=cli.max_delta_rpy,
            actual_hz=cli.actual_hz,
            seed_xyzrpy=cli.seed,
        )
        kb.run()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(controller, executor, spin_thread)


if __name__ == "__main__":
    main()
