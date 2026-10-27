#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienKung Dex 双臂逆运动学（pinocchio DLS + 多种子回退）。

坐标系：目标位姿 ``[x, y, z, roll, pitch, yaw]`` 定义在 **URDF 基座系**
（``pelvis`` 原点，m/rad）。世界系换算见 ``constants.ROBOT_WORLD_POS``，
由 ``dex_controller`` 负责，本模块只认基座系。

末端的定义按手型自动选择（两者都是 wrist_roll 之后的固定连接件）：

* 因时 RH5DG2 —— ``left_hand_base`` / ``right_hand_base``（掌根，离腕法兰 44 mm）
* 强脑 Revo2 —— ``left_base_link`` / ``right_base_link``
* 都没有时回退 ``wrist_roll_{l,r}_link``（纯腕法兰）

与 ``walker_s2/utils/ik.py`` 同一套 DLS 算法（限位裁剪 + 零空间保持中立位 +
上一帧热启动 / 语义种子 / 随机种子回退），去掉 Walker S2 专属的腰部解析种子
与层级 IK。
"""

from __future__ import annotations

import os
from typing import Any, Iterable, Sequence

import numpy as np
import pinocchio as pin

from constants import ARM_JOINTS, BODY_JOINT_LIMITS

DEFAULT_EE_FRAME_CANDIDATES = {
    "left": ("left_hand_base", "left_base_link", "wrist_roll_l_link"),
    "right": ("right_hand_base", "right_base_link", "wrist_roll_r_link"),
}

WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]


def default_urdf_path(variant: str = "inspire") -> str:
    """仓库内 TienKung Dex URDF 路径（供 ROS 脚本默认值使用）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, os.pardir, os.pardir, os.pardir, os.pardir))
    name = "tienkung_dex_brainco.urdf" if variant == "brainco" else "tienkung_dex_inspire.urdf"
    return os.path.join(repo_root, "assets", "robots", "tienkung_dex", "urdf", name)


class TienkungDexIK:
    """TienKung Dex 双臂 IK 求解器。"""

    LEFT_ARM_JOINTS = list(ARM_JOINTS["left"])
    RIGHT_ARM_JOINTS = list(ARM_JOINTS["right"])
    WAIST_JOINTS = list(WAIST_JOINTS)

    def __init__(
        self,
        urdf_path: str,
        joint_limits: dict[str, tuple[float, float]] | None = None,
        joint_limit_margin: float = 0.0,
        ee_frames: dict[str, str] | None = None,
    ):
        self.urdf_path = str(urdf_path)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        self._validate_model()
        self._joint_limit_override_count = self._apply_joint_limits(
            BODY_JOINT_LIMITS if joint_limits is None else joint_limits, joint_limit_margin
        )

        self.ee_frames = ee_frames or self._resolve_ee_frames()
        self.left_ee_id = self.model.getFrameId(self.ee_frames["left"])
        self.right_ee_id = self.model.getFrameId(self.ee_frames["right"])

        self.left_arm_q_indices, self.left_arm_v_indices = self._joint_indices(self.LEFT_ARM_JOINTS)
        self.right_arm_q_indices, self.right_arm_v_indices = self._joint_indices(self.RIGHT_ARM_JOINTS)
        self.waist_q_indices, self.waist_v_indices = self._joint_indices(self.WAIST_JOINTS)

        self.joint_name_to_q_idx: dict[str, int] = {}
        self._name_mapping_built = False
        self.q = pin.neutral(self.model)
        self.q_initial: np.ndarray | None = None
        self.q_neutral_left: np.ndarray | None = None
        self.q_neutral_right: np.ndarray | None = None
        self._ik_sync_joint_names = set(
            self.LEFT_ARM_JOINTS + self.RIGHT_ARM_JOINTS + self.WAIST_JOINTS
        )
        self._left_fail_count = 0
        self._right_fail_count = 0
        self._fail_reset_threshold = 30
        self.last_fail_info: dict[str, Any] | None = None
        self._rng = np.random.RandomState(42)

    # ------------------------------------------------------------------ 校验
    def _resolve_ee_frames(self) -> dict[str, str]:
        frame_names = {frame.name for frame in self.model.frames}
        resolved: dict[str, str] = {}
        for side in ("left", "right"):
            for candidate in DEFAULT_EE_FRAME_CANDIDATES[side]:
                if candidate in frame_names:
                    resolved[side] = candidate
                    break
            else:
                raise ValueError(
                    f"URDF 缺少 {side} 臂末端 frame（试过 {DEFAULT_EE_FRAME_CANDIDATES[side]}）"
                )
        return resolved

    def _validate_model(self) -> None:
        missing = [
            name
            for name in self.LEFT_ARM_JOINTS + self.RIGHT_ARM_JOINTS + self.WAIST_JOINTS
            if not self.model.existJointName(name)
        ]
        if missing:
            raise ValueError(f"TienKung Dex IK URDF 缺少关节: {missing}")

    def _joint_indices(self, joint_names: Sequence[str]) -> tuple[list[int], list[int]]:
        q_indices, v_indices = [], []
        for name in joint_names:
            jid = self.model.getJointId(name)
            q_indices.append(self.model.joints[jid].idx_q)
            v_indices.append(self.model.joints[jid].idx_v)
        return q_indices, v_indices

    def _apply_joint_limits(self, joint_limits: dict, margin: float = 0.0) -> int:
        margin = float(margin)
        applied = 0
        for name, limits in joint_limits.items():
            if not self.model.existJointName(name):
                continue
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1:
                continue
            q_idx = joint.idx_q
            lower, upper = limits
            safe_lower = max(float(self.model.lowerPositionLimit[q_idx]), float(lower) + margin)
            safe_upper = min(float(self.model.upperPositionLimit[q_idx]), float(upper) - margin)
            if safe_lower >= safe_upper:
                raise ValueError(
                    f"{name} 的 IK 限位非法: [{safe_lower:.4f}, {safe_upper:.4f}]"
                )
            if (
                safe_lower != float(self.model.lowerPositionLimit[q_idx])
                or safe_upper != float(self.model.upperPositionLimit[q_idx])
            ):
                applied += 1
            self.model.lowerPositionLimit[q_idx] = safe_lower
            self.model.upperPositionLimit[q_idx] = safe_upper
        return applied

    # ------------------------------------------------------------ 状态同步
    def _build_name_mapping(self, joint_names: Iterable[str]) -> None:
        for name in joint_names:
            if name not in self.joint_name_to_q_idx and self.model.existJointName(name):
                jid = self.model.getJointId(name)
                self.joint_name_to_q_idx[name] = self.model.joints[jid].idx_q
        self._name_mapping_built = True

    def sync_joint_positions(self, joint_names: list[str], joint_positions: list[float]) -> None:
        """把当前关节读数灌进求解器的 q（用于热启动与 FK 求末端位姿）。"""
        if len(joint_names) != len(joint_positions):
            raise ValueError("joint_names 与 joint_positions 长度不一致")
        if not self._name_mapping_built:
            self._build_name_mapping(joint_names)

        if self.q_initial is not None:
            self.q = self.q_initial.copy()

        for name, pos in zip(joint_names, joint_positions):
            q_idx = self.joint_name_to_q_idx.get(name)
            if q_idx is None:
                continue
            if self.q_initial is None or name in self._ik_sync_joint_names:
                self.q[q_idx] = float(pos)

    def set_neutral_config(self, left_angles: Sequence[float], right_angles: Sequence[float]) -> None:
        self.q_neutral_left = np.asarray(left_angles, dtype=float)
        self.q_neutral_right = np.asarray(right_angles, dtype=float)

    def save_initial_q(self) -> None:
        self.q_initial = self.q.copy()

    def reset_runtime_state(self) -> None:
        self._left_fail_count = 0
        self._right_fail_count = 0
        self.last_fail_info = None
        if self.q_initial is not None:
            self.q = self.q_initial.copy()

    # ------------------------------------------------------------ 位姿换算
    @staticmethod
    def xyzrpy_to_se3(xyzrpy: Sequence[float]) -> pin.SE3:
        xyz = np.asarray(xyzrpy[:3], dtype=float)
        rpy = np.asarray(xyzrpy[3:6], dtype=float)
        return pin.SE3(pin.rpy.rpyToMatrix(float(rpy[0]), float(rpy[1]), float(rpy[2])), xyz)

    @staticmethod
    def se3_to_xyzrpy(se3: pin.SE3) -> np.ndarray:
        rpy = pin.rpy.matrixToRpy(se3.rotation)
        return np.concatenate([np.asarray(se3.translation, dtype=float), np.asarray(rpy, dtype=float)])

    def get_ee_pose(self, side: str) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        if side not in ("left", "right"):
            raise ValueError(f"无效的 arm side: {side}")
        frame_id = self.left_ee_id if side == "left" else self.right_ee_id
        return self.se3_to_xyzrpy(self.data.oMf[frame_id])

    def get_both_ee_poses(self) -> dict[str, np.ndarray]:
        return {"left": self.get_ee_pose("left"), "right": self.get_ee_pose("right")}

    # ------------------------------------------------------------ IK
    def _generate_random_seeds(self, n: int, side: str) -> list[np.ndarray]:
        arm_joints = self.LEFT_ARM_JOINTS if side == "left" else self.RIGHT_ARM_JOINTS
        seeds = []
        for _ in range(n):
            q_seed = self.q.copy()
            for name in arm_joints:
                jid = self.model.getJointId(name)
                q_idx = self.model.joints[jid].idx_q
                low = self.model.lowerPositionLimit[q_idx]
                high = self.model.upperPositionLimit[q_idx]
                q_seed[q_idx] = self._rng.uniform(low, high)
            seeds.append(q_seed)
        return seeds

    def _build_seed_list(self, side: str, num_random_seeds: int) -> list[np.ndarray]:
        """种子回退：上一帧热启动 → 中立位 → 随机种子。"""
        seeds = [self.q.copy()]
        neutral = self.q_neutral_left if side == "left" else self.q_neutral_right
        arm_joints = self.LEFT_ARM_JOINTS if side == "left" else self.RIGHT_ARM_JOINTS
        if neutral is not None:
            seed = self.q.copy()
            for name, value in zip(arm_joints, neutral):
                seed[self.model.joints[self.model.getJointId(name)].idx_q] = float(value)
            if not np.allclose(seed, self.q):
                seeds.append(seed)
        if num_random_seeds > 0:
            seeds.extend(self._generate_random_seeds(num_random_seeds, side))
        return seeds

    def _solve_ik_dls_core(
        self,
        target_se3: pin.SE3,
        ee_id: int,
        q_indices: list[int],
        v_indices: list[int],
        q_neutral_active: np.ndarray | None,
        seed_q: np.ndarray,
        max_iter: int,
        pos_tol: float,
        rot_tol: float,
        damping: float,
        dt: float,
        dq_max: float,
        pos_weight: float,
        rot_weight: float,
        null_weight: float,
    ) -> tuple[np.ndarray, bool, float, float, int]:
        """加权阻尼最小二乘迭代，从 seed_q 出发。"""
        n_dof = len(v_indices)
        q = seed_q.copy()
        weight = np.diag([pos_weight] * 3 + [rot_weight] * 3)
        effective_rot_tol = rot_tol / max(rot_weight, 0.01)
        use_null = q_neutral_active is not None and null_weight > 0.0
        pos_err = rot_err = float("inf")
        it = 0

        for it in range(1, max_iter + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            error = pin.log(self.data.oMf[ee_id].actInv(target_se3)).vector
            pos_err = float(np.linalg.norm(error[:3]))
            rot_err = float(np.linalg.norm(error[3:]))
            if pos_err < pos_tol and rot_err < effective_rot_tol:
                active_q = np.array([q[idx] for idx in q_indices], dtype=float)
                for idx in q_indices:
                    self.q[idx] = q[idx]
                return active_q, True, pos_err, rot_err, it

            jac_full = pin.computeFrameJacobian(self.model, self.data, q, ee_id, pin.LOCAL)
            jac_w = weight @ jac_full[:, v_indices]
            err_w = weight @ error
            jtj = jac_w.T @ jac_w + damping * np.eye(n_dof)
            dq_active = np.linalg.solve(jtj, jac_w.T @ (dt * err_w))

            if use_null:
                jac_w_pinv = np.linalg.solve(jtj, jac_w.T)
                null_projector = np.eye(n_dof) - jac_w_pinv @ jac_w
                q_active = np.array([q[idx] for idx in q_indices], dtype=float)
                dq_active = dq_active + null_weight * null_projector @ (q_neutral_active - q_active)

            dq_norm = float(np.linalg.norm(dq_active))
            if dq_norm > dq_max:
                dq_active *= dq_max / dq_norm

            for j, idx in enumerate(q_indices):
                q[idx] = np.clip(
                    q[idx] + dq_active[j],
                    self.model.lowerPositionLimit[idx],
                    self.model.upperPositionLimit[idx],
                )

        active_q = np.array([q[idx] for idx in q_indices], dtype=float)
        for idx in q_indices:
            self.q[idx] = q[idx]
        return active_q, False, pos_err, rot_err, it

    def solve_ik_single_arm(
        self,
        target_se3: pin.SE3,
        side: str,
        max_iter: int = 300,
        pos_tol: float = 5e-3,
        rot_tol: float = 5e-3,
        damping: float = 1e-4,
        dt: float = 1.0,
        dq_max: float = 0.5,
        pos_weight: float = 1.0,
        rot_weight: float = 1.0,
        null_weight: float = 0.1,
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
    ) -> tuple[np.ndarray, bool]:
        """单臂 IK。

        Returns:
            (关节角数组, 是否收敛)。``unlock_waist=True`` 时数组前面多 3 个
            腰部关节值，顺序为 ``waist_yaw/roll/pitch``。
        """
        if side == "left":
            ee_id = self.left_ee_id
            q_indices = list(self.left_arm_q_indices)
            v_indices = list(self.left_arm_v_indices)
            q_neutral_active = self.q_neutral_left
        elif side == "right":
            ee_id = self.right_ee_id
            q_indices = list(self.right_arm_q_indices)
            v_indices = list(self.right_arm_v_indices)
            q_neutral_active = self.q_neutral_right
        else:
            raise ValueError(f"无效的 arm side: {side}")

        if unlock_waist:
            q_indices = list(self.waist_q_indices) + q_indices
            v_indices = list(self.waist_v_indices) + v_indices
            if q_neutral_active is not None:
                waist_neutral = (
                    self.q_initial[self.waist_q_indices]
                    if self.q_initial is not None
                    else self.q[self.waist_q_indices]
                )
                q_neutral_active = np.concatenate([np.asarray(waist_neutral, dtype=float), q_neutral_active])

        last_active_q = None
        last = (float("inf"), float("inf"), 0)
        for seed_q in self._build_seed_list(side, num_random_seeds):
            active_q, ok, pos_err, rot_err, it = self._solve_ik_dls_core(
                target_se3, ee_id, q_indices, v_indices, q_neutral_active, seed_q,
                max_iter, pos_tol, rot_tol, damping, dt, dq_max,
                pos_weight, rot_weight, null_weight,
            )
            last_active_q = active_q
            last = (pos_err, rot_err, it)
            if ok:
                self.last_fail_info = None
                self._update_fail_count(side, True)
                return active_q, True

        self.last_fail_info = {
            "side": side,
            "pos_err": last[0],
            "rot_err": last[1],
            "iter": last[2],
            "max_iter": int(max_iter),
            "seeds_tried": 1 + (1 if q_neutral_active is not None else 0) + num_random_seeds,
        }
        self._update_fail_count(side, False)
        return last_active_q, False

    def _solve_position_dls_core(
        self,
        target_xyz: np.ndarray,
        ee_id: int,
        q_indices: list[int],
        v_indices: list[int],
        seed_q: np.ndarray,
        max_iter: int,
        pos_tol: float,
        damping: float,
        dt: float,
        dq_max: float,
    ) -> tuple[np.ndarray, bool, float, int]:
        """只解位置的阻尼最小二乘迭代（3 维任务）。"""
        q = seed_q.copy()
        pos_err = float("inf")
        it = 0

        for it in range(1, max_iter + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            placement = self.data.oMf[ee_id]
            error_world = target_xyz - placement.translation
            pos_err = float(np.linalg.norm(error_world))
            if pos_err < pos_tol:
                active_q = np.array([q[idx] for idx in q_indices], dtype=float)
                for idx in q_indices:
                    self.q[idx] = q[idx]
                return active_q, True, pos_err, it

            # 雅可比取 LOCAL，误差也转到 LOCAL，避免每次再乘一次基座旋转
            jac = pin.computeFrameJacobian(self.model, self.data, q, ee_id, pin.LOCAL)[:3, v_indices]
            error_local = placement.rotation.T @ error_world
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(3), dt * error_local)
            dq_norm = float(np.linalg.norm(dq))
            if dq_norm > dq_max:
                dq *= dq_max / dq_norm
            for j, idx in enumerate(q_indices):
                q[idx] = np.clip(
                    q[idx] + dq[j],
                    self.model.lowerPositionLimit[idx],
                    self.model.upperPositionLimit[idx],
                )

        active_q = np.array([q[idx] for idx in q_indices], dtype=float)
        for idx in q_indices:
            self.q[idx] = q[idx]
        return active_q, False, pos_err, it

    def solve_ik_position_only(
        self,
        target_xyz: Sequence[float],
        side: str,
        max_iter: int = 600,
        pos_tol: float = 3e-3,
        damping: float = 1e-4,
        dt: float = 1.0,
        dq_max: float = 0.3,
        unlock_waist: bool = False,
        num_random_seeds: int = 0,
    ) -> tuple[np.ndarray, bool]:
        """只约束末端位置的单臂 IK（3 维任务 / 7 自由度臂）。

        与 :meth:`solve_ik_single_arm` 的差别是**姿态不进代价函数**。7 自由度臂
        解 3 维任务有 4 维零空间，用当前关节角热启动时姿态会在零空间里"就近
        保持"，这正是 tiangong2dex 抓取脚本的做法。

        为什么需要它：工作空间边缘上要求的姿态可能不可达，6 自由度加权 DLS 会
        被姿态残差拖住（实测 72 组权重/阻尼/种子组合全部卡在 1-3 cm 外），而
        只解位置时同样的点在位置残差 0.0000 收敛。抓取这类"位置必须准、姿态
        可由关节空间单独标定"的场景应当用本方法。

        Returns:
            (关节角数组, 是否收敛)。``unlock_waist=True`` 时数组前面多 3 个
            腰部关节值，顺序为 ``waist_yaw/roll/pitch``。
        """
        if side == "left":
            ee_id = self.left_ee_id
            q_indices = list(self.left_arm_q_indices)
            v_indices = list(self.left_arm_v_indices)
        elif side == "right":
            ee_id = self.right_ee_id
            q_indices = list(self.right_arm_q_indices)
            v_indices = list(self.right_arm_v_indices)
        else:
            raise ValueError(f"无效的 arm side: {side}")

        if unlock_waist:
            q_indices = list(self.waist_q_indices) + q_indices
            v_indices = list(self.waist_v_indices) + v_indices

        target = np.asarray(target_xyz, dtype=float)[:3]
        last_active_q = None
        last = (float("inf"), 0)
        for seed_q in self._build_seed_list(side, num_random_seeds):
            active_q, ok, pos_err, it = self._solve_position_dls_core(
                target, ee_id, q_indices, v_indices, seed_q,
                max_iter, pos_tol, damping, dt, dq_max,
            )
            last_active_q = active_q
            last = (pos_err, it)
            if ok:
                self.last_fail_info = None
                self._update_fail_count(side, True)
                return active_q, True

        self.last_fail_info = {
            "side": side,
            "pos_err": last[0],
            "iter": last[1],
            "max_iter": int(max_iter),
            "mode": "position-only",
            "seeds_tried": 1 + (1 if self.q_neutral_right is not None else 0) + num_random_seeds,
        }
        self._update_fail_count(side, False)
        return last_active_q, False

    def solve_dual_arm(
        self,
        left_target_xyzrpy: Sequence[float] | None = None,
        right_target_xyzrpy: Sequence[float] | None = None,
        **kwargs: Any,
    ) -> tuple[dict[str, np.ndarray], bool]:
        """双臂 IK：先左后右（求右臂时左臂保持已解出的姿态）。"""
        results: dict[str, np.ndarray] = {}
        success = True
        for side, target in (("left", left_target_xyzrpy), ("right", right_target_xyzrpy)):
            if target is None:
                continue
            active_q, ok = self.solve_ik_single_arm(self.xyzrpy_to_se3(target), side, **kwargs)
            results[side] = active_q
            success = success and ok
        return results, success

    def _update_fail_count(self, side: str, success: bool) -> None:
        attr = "_left_fail_count" if side == "left" else "_right_fail_count"
        count = getattr(self, attr)
        if success:
            setattr(self, attr, 0)
            return
        count += 1
        if count >= self._fail_reset_threshold:
            # 连续失败：热启动可能陷在局部解，退回初始位
            if self.q_initial is not None:
                indices = self.left_arm_q_indices if side == "left" else self.right_arm_q_indices
                for idx in indices:
                    self.q[idx] = self.q_initial[idx]
            count = 0
        setattr(self, attr, count)
