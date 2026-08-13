#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walker S2 双臂搬箱控制器。

继承 WalkerS2Controller,使用 move_dual_ee_delta(末端相对位移,双臂同时)
规划双臂搬箱轨迹:靠近箱子 -> 下降 -> 抱起 -> [原地放下 -> 撤回]。

腕部夹具双臂夹取:双臂末端从预备姿态出发,先下降到预备位置,再内收+下降
夹住箱子(腕部夹具靠双臂位置夹持,无主动夹爪),抬起抱起,原地放下后双臂
外展松开。

================================================================
双臂协同实现说明(重要)
================================================================
双臂同时移动经 move_dual_ee_delta 实现:一次求解双臂 IK,生成**单条** 17
维轨迹(含左右臂关节),200Hz 定时器同时发布 -> 双臂真正同步运动。

为何不用两条 move_arm_ee_delta 并发:move_arm_ee_delta -> move_to_pose ->
execute_trajectory 会写入控制器**单一**的 current_trajectory 队列
(walker_s2_controller.py L1410),两条轨迹并发会互相覆盖。故双臂同时
移动必须走 move_dual_ee_delta(仍是末端 delta 语义:目标 = 当前位姿 +
delta,与 move_arm_ee_delta 同族)。

腕部夹具为被动机构,靠双臂内收夹紧(approach_and_descend 已完成)、外展
松开(release_box)。无主动夹爪动作,不走 ECAT GripCmd 通路。

================================================================
坐标系
================================================================
delta 定义在 Walker S2 URDF base frame:x 前、y 左、z 上,单位 m/rad。
正 dy = 向左,负 dy = 向右;正 dz = 抬升。

================================================================
运行前置条件(同 walker_s2_controller.py)
================================================================
1. 运控容器内启动并切换 SDK 控制器:
       rosa run t800_mc_server start_mc_client
       rosa run rosa_controllers switch_controller config_mc_walker_s2_v1_sps
2. 机器人移到安全位置(先用遥控器)
3. 控制容器内 source /home/ubt/additional/scripts/setup.sh,用 /usr/bin/python3
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)
from utils.constants import CARRY_READY_POSE
from utils.controller import WalkerS2Controller


# ============================================================================
# 默认轨迹参数(URDF base frame,单位 m)
# 数值为相对当前末端位置的 delta,可被 CLI 覆盖
# ============================================================================

# 预备姿态
DEFAULT_INIT_DURATION = 2.0        # init: 预备姿态运动时长(s)
DEFAULT_LEVEL_DURATION = 2.0       # level_ee: 姿态校正时长(s)
DEFAULT_WRIST_PITCH_OFFSET = 0.075 # level_ee: 腕部 pitch 偏置(左右异号)
DEFAULT_WRIST_YAW_OFFSET = -0.2    # level_ee: yaw 偏置(作用于 elbow_yaw)
DEFAULT_WRIST_ROLL_OFFSET = 0.0    # level_ee: roll 偏置
# 抱起箱子
DEFAULT_PREGRASP_DURATION = 2.0    # descend_to_pregrasp 时长(s)
DEFAULT_PREGRASP_DZ = -0.13       # descend_to_pregrasp: 预抓取高度下降
DEFAULT_DESCEND_DZ = -0.0        # approach_and_descend: 下降到夹取高度
DEFAULT_APPROACH_DURATION = 2.0    # approach_and_descend 时长(s)
DEFAULT_APPROACH_DY = 0.038         # approach_and_descend: 双臂 y 内收
DEFAULT_LIFT_DURATION = 2.0        # lift 时长(s)
DEFAULT_LIFT_DZ = 0.10             # lift: 抱起抬升
DEFAULT_PULL_BACK_DURATION = 2.0   # pull_back 时长(s)
DEFAULT_PULL_BACK_DX = -0.10       # pull_back: 抱起后后移(向胸部,-x)
# 放置箱子
DEFAULT_PLACE_DURATION = 2.0       # place 时长(s)
DEFAULT_RELEASE_DURATION = 2.0     # release 时长(s)
DEFAULT_RETREAT_DURATION = 2.0     # retreat 时长(s)
DEFAULT_PLACE_DZ = -DEFAULT_LIFT_DZ  # place: 放下下降(= -lift_dz,回桌面,自动同步)
DEFAULT_RETREAT_DZ = 0.10          # retreat: 撤回抬升(只抬起,不后退)
DEFAULT_DURATION = 2.0             # 全局默认: move_both_ee_delta API 后备时长(s)


class CarryBoxController(WalkerS2Controller):
    """双臂搬箱控制器:末端 delta 序列夹取桌面箱子并抱起。

    所有末端运动经 move_dual_ee_delta(双臂同时);腕部夹具靠双臂位置夹持,
    无主动夹爪。轨迹参数在构造时给定,可被 CLI 覆盖。
    """

    def __init__(
        self,
        node_name: str = "carry_box",
        approach_dy: float = DEFAULT_APPROACH_DY,
        pregrasp_dz: float = DEFAULT_PREGRASP_DZ,
        descend_dz: float = DEFAULT_DESCEND_DZ,
        lift_dz: float = DEFAULT_LIFT_DZ,
        place_dz: float | None = None,
        retreat_dz: float = DEFAULT_RETREAT_DZ,
        duration_sec: float = DEFAULT_DURATION,
        level_duration: float = DEFAULT_LEVEL_DURATION,
        pregrasp_duration: float = DEFAULT_PREGRASP_DURATION,
        approach_duration: float = DEFAULT_APPROACH_DURATION,
        lift_duration: float = DEFAULT_LIFT_DURATION,
        pull_back_duration: float = DEFAULT_PULL_BACK_DURATION,
        place_duration: float = DEFAULT_PLACE_DURATION,
        release_duration: float = DEFAULT_RELEASE_DURATION,
        retreat_duration: float = DEFAULT_RETREAT_DURATION,
        wrist_pitch_offset: float = DEFAULT_WRIST_PITCH_OFFSET,
        wrist_yaw_offset: float = DEFAULT_WRIST_YAW_OFFSET,
        wrist_roll_offset: float = DEFAULT_WRIST_ROLL_OFFSET,
        pull_back_dx: float = DEFAULT_PULL_BACK_DX,
        use_hierarchical: bool = False,
        **kwargs,
    ):
        # 搬箱需要 IK 求解末端 delta -> 关节目标
        kwargs.setdefault("enable_ik", True)
        kwargs.setdefault("ik_auto_initialize", True)
        super().__init__(node_name=node_name, **kwargs)

        self.approach_dy = approach_dy
        self.pregrasp_dz = pregrasp_dz
        self.descend_dz = descend_dz
        self.lift_dz = lift_dz
        # place_dz 未显式指定时，自动取 -lift_dz，保证放下深度始终与抬起一致。
        # 旧实现用模块常量 DEFAULT_PLACE_DZ(=-DEFAULT_LIFT_DZ)，用户改 --lift-dz 时
        # place_dz 不会跟随，导致放下深度与抬起不匹配。
        self.place_dz = place_dz if place_dz is not None else -lift_dz
        self.retreat_dz = retreat_dz
        self.duration_sec = duration_sec
        self.level_duration = level_duration
        self.pregrasp_duration = pregrasp_duration
        self.approach_duration = approach_duration
        self.lift_duration = lift_duration
        self.pull_back_duration = pull_back_duration
        self.place_duration = place_duration
        self.release_duration = release_duration
        self.retreat_duration = retreat_duration
        self.wrist_pitch_offset = wrist_pitch_offset
        self.wrist_yaw_offset = wrist_yaw_offset
        self.wrist_roll_offset = wrist_roll_offset
        self.pull_back_dx = pull_back_dx
        self.use_hierarchical = use_hierarchical

    # ============================================================
    # 双臂同时封装(核心:move_dual_ee_delta)
    # ============================================================

    def move_both_ee_delta(
        self,
        left_xyz,
        right_xyz,
        left_rpy=(0.0, 0.0, 0.0),
        right_rpy=(0.0, 0.0, 0.0),
        duration_sec=None,
        wait=True,
        use_hierarchical=None,
    ):
        """双臂末端相对位移(同时移动)。

        经 move_dual_ee_delta 一次求解双臂 IK,生成单条 17 维轨迹,200Hz
        定时器同时发布左右臂关节 -> 双臂真正同步运动。delta 均在 URDF base
        frame,各自相对该侧末端当前位姿。

        为何不用两条 move_arm_ee_delta 并发:move_arm_ee_delta 走控制器
        **单一**轨迹队列 current_trajectory(walker_s2_controller.py L1410),
        两条并发会互相覆盖。故双臂同时移动必须走 dual 版本(仍是末端 delta
        语义:目标 = 当前位姿 + delta,与 move_arm_ee_delta 同族)。

        Args:
            left_xyz / right_xyz: 末端平移 delta (dx, dy, dz),单位 m
            left_rpy / right_rpy: 末端姿态 delta (droll, dpitch, dyaw),单位 rad
            duration_sec: 运动时长
            wait: 是否阻塞
            use_hierarchical: 是否启用层级 IK(torso→shoulder→full arm);
                              None 时使用实例默认值 self.use_hierarchical
        Returns:
            bool: True=双臂 IK 收敛并下发成功
        """
        if use_hierarchical is None:
            use_hierarchical = self.use_hierarchical
        duration_sec = duration_sec or self.duration_sec
        return self.move_dual_ee_delta(
            left_delta_xyz=left_xyz,
            right_delta_xyz=right_xyz,
            left_delta_rpy=left_rpy,
            right_delta_rpy=right_rpy,
            duration_sec=duration_sec,
            wait=wait,
            require_success=True,
            use_hierarchical=use_hierarchical,
        )

    # ============================================================
    # 释放(腕部夹具靠双臂位置夹持,外展即松开)
    # ============================================================

    def release_box(self):
        """双臂外展松开腕部夹具(左 +y、右 -y,与内收反向)。"""
        self.get_logger().info(
            f"Release: left +y {self.approach_dy:+.3f}, "
            f"right -y {self.approach_dy:+.3f}"
        )
        return self.move_both_ee_delta(
            left_xyz=(0.0, self.approach_dy, 0.0),
            right_xyz=(0.0, -self.approach_dy, 0.0),
            duration_sec=self.release_duration,
        )

    # ============================================================
    # 搬箱阶段(每段均为双臂 delta)
    # ============================================================

    def level_ee_pose(self):
        """关节空间校正末端姿态:pitch(水平)+ yaw + roll(微调)。

        Cartesian IK 在 yaw≈π 区域不稳定,故用关节空间偏置,确定性下发。
        Walker S2 腕部只有 pitch/roll 关节,无 wrist_yaw;末端 yaw 微调作用
        于 elbow_yaw(最接近末端的 yaw 关节)。yaw≈±π(指向前方,±π 等价)。

        偏置符号需实测:方向反了把对应 --wrist-*-offset 取反。左右臂关节
        正方向镜像,故右臂偏置取反(左 +offset、右 -offset),使左右末端
        同向变化。若某维度左右本就同向(非镜像),告知单独改回同号。
        """
        pos = self.get_current_position()
        if pos is None:
            self.get_logger().error("level_ee_pose: no joint state")
            return False
        target = {}
        if abs(self.wrist_pitch_offset) > 1e-9:
            target["L_wrist_pitch_joint"] = float(pos[self.joint_index("L_wrist_pitch_joint")]) + self.wrist_pitch_offset
            target["R_wrist_pitch_joint"] = float(pos[self.joint_index("R_wrist_pitch_joint")]) - self.wrist_pitch_offset
        if abs(self.wrist_yaw_offset) > 1e-9:
            target["L_elbow_yaw_joint"] = float(pos[self.joint_index("L_elbow_yaw_joint")]) + self.wrist_yaw_offset
            target["R_elbow_yaw_joint"] = float(pos[self.joint_index("R_elbow_yaw_joint")]) - self.wrist_yaw_offset
        if abs(self.wrist_roll_offset) > 1e-9:
            target["L_wrist_roll_joint"] = float(pos[self.joint_index("L_wrist_roll_joint")]) + self.wrist_roll_offset
            target["R_wrist_roll_joint"] = float(pos[self.joint_index("R_wrist_roll_joint")]) - self.wrist_roll_offset
        if not target:
            self.get_logger().info("Level EE: pitch/yaw/roll offset 均 0,跳过")
            return True
        self.get_logger().info(
            f"Level EE (joint-space, 左+右-): pitch={self.wrist_pitch_offset:+.4f} "
            f"({np.degrees(self.wrist_pitch_offset):+.2f}°), "
            f"yaw={self.wrist_yaw_offset:+.4f} ({np.degrees(self.wrist_yaw_offset):+.2f}°), "
            f"roll={self.wrist_roll_offset:+.4f} ({np.degrees(self.wrist_roll_offset):+.2f}°)"
        )
        return self.move_to_pose(
            target,
            duration_sec=self.level_duration,
            wait=True,
            unlock_required_joints=True,
        )

    def descend_to_pregrasp(self):
        """下降到预备位置(只 dz,不内收)。"""
        self.get_logger().info(f"Descend to pregrasp: dz {self.pregrasp_dz:+.3f}")
        return self.move_both_ee_delta(
            left_xyz=(0.0, 0.0, self.pregrasp_dz),
            right_xyz=(0.0, 0.0, self.pregrasp_dz),
            duration_sec=self.pregrasp_duration,
        )

    def approach_and_descend(self):
        """同时内收 + 下降到夹取高度(dy + dz 一步到位)。

        base frame y 轴向左:左臂 -approach_dy(向右)、右臂 +approach_dy
        (向左)内收;同时双臂 dz = descend_dz 下降到夹取高度。经
        move_dual_ee_delta 一次求解,双臂同步完成内收+下降。
        """
        self.get_logger().info(
            f"Approach+descend: left -y {self.approach_dy:+.3f}, "
            f"right +y {self.approach_dy:+.3f}, dz {self.descend_dz:+.3f}"
        )
        return self.move_both_ee_delta(
            left_xyz=(0.0, -self.approach_dy, self.descend_dz),
            right_xyz=(0.0, self.approach_dy, self.descend_dz),
            duration_sec=self.approach_duration,
        )

    def lift_box(self):
        """双臂抬起抱箱。"""
        self.get_logger().info(f"Lift: dz {self.lift_dz:+.3f}")
        return self.move_both_ee_delta(
            left_xyz=(0.0, 0.0, self.lift_dz),
            right_xyz=(0.0, 0.0, self.lift_dz),
            duration_sec=self.lift_duration,
        )

    def pull_back(self):
        """抱起后双臂向后(-x,胸部方向)移动。"""
        self.get_logger().info(f"Pull back: dx {self.pull_back_dx:+.3f}")
        return self.move_both_ee_delta(
            left_xyz=(self.pull_back_dx, 0.0, 0.0),
            right_xyz=(self.pull_back_dx, 0.0, 0.0),
            duration_sec=self.pull_back_duration,
        )

    def place_box(self):
        """放回初始位:前移抵消 pull_back(-pull_back_dx)+ 下降(place_dz)。

        不依赖缓存:pull_back 后移 pull_back_dx,place 前移等量抵消;下降
        place_dz(默认 -lift_dz)回夹取高度。y 不变(内收状态,release 在后)。
        """
        dx = -self.pull_back_dx
        self.get_logger().info(
            f"Place: dx {dx:+.3f} (抵消 pull_back), dz {self.place_dz:+.3f}"
        )
        return self.move_both_ee_delta(
            left_xyz=(dx, 0.0, self.place_dz),
            right_xyz=(dx, 0.0, self.place_dz),
            duration_sec=self.place_duration,
        )

    def retreat(self):
        """撤回:垂直抬起脱离箱子(不后退)。"""
        self.get_logger().info(f"Retreat lift: dz {self.retreat_dz:+.3f}")
        return self.move_both_ee_delta(
            left_xyz=(0.0, 0.0, self.retreat_dz),
            right_xyz=(0.0, 0.0, self.retreat_dz),
            duration_sec=self.retreat_duration,
        )

    # ============================================================
    # 任务编排
    # ============================================================

    def _run_stages(self, stages):
        """顺序执行 [(name, fn), ...],任一失败则中止。"""
        for name, fn in stages:
            self.get_logger().info(f"=== Stage: {name} ===")
            if not fn():
                self.get_logger().error(f"Stage '{name}' failed, abort")
                return False
        return True

    def _return_home(self):
        """回搬箱预备姿态(elbow_yaw=±1.5,前臂指向前方)。"""
        self.get_logger().info("Returning to carry ready pose")
        if not self.move_to_pose(CARRY_READY_POSE, duration_sec=DEFAULT_INIT_DURATION, wait=True, unlock_required_joints=True):
            self.get_logger().warning("Failed to return to carry ready pose")

    def pick_up_box(self, init_pose=True, level=True):
        """抱箱部分:init -> level -> descend_to_pregrasp -> approach_and_descend -> lift -> [pull_back]。

        抱起箱子后停在抬起位置(夹具夹紧)。可独立调用;之后调 place_down_box
        放下,或 run_carry_task 串联。place 前移抵消 pull_back + 下降回夹取位。
        """
        if init_pose:
            self.get_logger().info("Stage 0: moving to carry ready pose")
            if not self.move_to_pose(CARRY_READY_POSE, duration_sec=DEFAULT_INIT_DURATION, wait=True, unlock_required_joints=True):
                self.get_logger().error("Failed to reach carry ready pose, abort")
                return False
        stages = []
        if level:
            stages.append(("level_ee", self.level_ee_pose))
        stages.extend([
            ("descend_to_pregrasp", self.descend_to_pregrasp),
            ("approach_and_descend", self.approach_and_descend),
            ("lift", self.lift_box),
        ])
        if abs(self.pull_back_dx) > 1e-9:
            stages.append(("pull_back", self.pull_back))
        if not self._run_stages(stages):
            return False
        self.get_logger().info("Pick-up completed")
        return True

    def place_down_box(self, retreat=True, return_home=True):
        """放箱部分:place -> release -> [retreat] -> [return_home]。

        从当前(抬起)位置放回箱子初始位置、外展松开、撤回、回预备姿态。可
        独立调用(假设箱子已在夹具)。place 前移抵消 pull_back + 下降,回到
        夹取位(不依赖缓存)。
        """
        stages = [("place", self.place_box), ("release", self.release_box)]
        if retreat:
            stages.append(("retreat", self.retreat))
        if not self._run_stages(stages):
            return False
        self.get_logger().info("Place-down completed")
        if return_home:
            self._return_home()
        return True

    def run_carry_task(self, init_pose=True, level=True, place=True, retreat=True, return_home=True):
        """编排完整搬箱流程 = pick_up_box + place_down_box。

        Args:
            init_pose: 是否先移动到预备姿态
            level: 是否校正末端姿态
            place: 是否执行放置(False=只抱起)
            retreat: 是否执行撤回
            return_home: 是否在完成后回到预备姿态
        Returns:
            bool: True=全部阶段成功
        """
        if not self.pick_up_box(init_pose=init_pose, level=level):
            return False
        if place:
            if not self.place_down_box(retreat=retreat, return_home=return_home):
                return False
        elif return_home:
            self._return_home()
        return True


# ============================================================================
# 命令行入口
# ============================================================================


def parse_args():
    parser = argparse.ArgumentParser(
        description="Walker S2 双臂搬箱控制器(末端 delta 轨迹)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--no-init", action="store_true", help="跳过预备姿态初始化")
    parser.add_argument("--no-level", action="store_true", help="跳过末端 pitch 水平校正")
    parser.add_argument(
        "--wrist-pitch-offset", type=float, default=DEFAULT_WRIST_PITCH_OFFSET,
        help=f"末端水平校正腕部 pitch 偏置(rad),默认 {DEFAULT_WRIST_PITCH_OFFSET}"
             f"(约 {np.degrees(DEFAULT_WRIST_PITCH_OFFSET):.1f}°);方向反了取反",
    )
    parser.add_argument(
        "--wrist-yaw-offset", type=float, default=DEFAULT_WRIST_YAW_OFFSET,
        help=f"末端 yaw 微调偏置(rad,作用于 elbow_yaw),默认 {DEFAULT_WRIST_YAW_OFFSET}"
             f"(约 {np.degrees(DEFAULT_WRIST_YAW_OFFSET):.1f}°);方向反了取反",
    )
    parser.add_argument(
        "--wrist-roll-offset", type=float, default=DEFAULT_WRIST_ROLL_OFFSET,
        help=f"末端 roll 微调腕部 roll 偏置(rad),默认 {DEFAULT_WRIST_ROLL_OFFSET}"
             f"(约 {np.degrees(DEFAULT_WRIST_ROLL_OFFSET):.1f}°);方向反了取反",
    )
    parser.add_argument("--no-place", action="store_true", help="只抱起,不原地放置")
    parser.add_argument("--no-retreat", action="store_true", help="最后不撤回")
    parser.add_argument("--no-home", action="store_true", help="完成后不回预备姿态")
    parser.add_argument("--pull-back-dx", type=float, default=DEFAULT_PULL_BACK_DX, help="抱起后后移量(m,-x 朝胸部),0=关")
    parser.add_argument("--pick-only", action="store_true", help="只抱箱(init+level+descend+approach+lift),不放下")
    parser.add_argument("--place-only", action="store_true", help="只放箱(place+release+retreat+return_home),需已抱起")
    parser.add_argument("--approach-dy", type=float, default=DEFAULT_APPROACH_DY, help="双臂 y 内收量(m)")
    parser.add_argument("--pregrasp-dz", type=float, default=DEFAULT_PREGRASP_DZ, help="预抓取下降(m)")
    parser.add_argument("--descend-dz", type=float, default=DEFAULT_DESCEND_DZ, help="夹取下降(m)")
    parser.add_argument("--lift-dz", type=float, default=DEFAULT_LIFT_DZ, help="抱起抬升(m)")
    parser.add_argument("--place-dz", type=float, default=None, help="原地放下下降(m);省略时自动=-lift_dz")
    parser.add_argument("--level-duration", type=float, default=DEFAULT_LEVEL_DURATION, help="level_ee 姿态校正时长(s)")
    parser.add_argument("--pregrasp-duration", type=float, default=DEFAULT_PREGRASP_DURATION, help="descend_to_pregrasp 时长(s)")
    parser.add_argument("--approach-duration", type=float, default=DEFAULT_APPROACH_DURATION, help="approach_and_descend 时长(s)")
    parser.add_argument("--lift-duration", type=float, default=DEFAULT_LIFT_DURATION, help="lift 抬起时长(s)")
    parser.add_argument("--pull-back-duration", type=float, default=DEFAULT_PULL_BACK_DURATION, help="pull_back 后移时长(s)")
    parser.add_argument("--place-duration", type=float, default=DEFAULT_PLACE_DURATION, help="place 放下时长(s)")
    parser.add_argument("--release-duration", type=float, default=DEFAULT_RELEASE_DURATION, help="release 外展时长(s)")
    parser.add_argument("--retreat-duration", type=float, default=DEFAULT_RETREAT_DURATION, help="retreat 撤回时长(s)")
    parser.add_argument("--no-lock", action="store_true", help="不锁定 head/waist")
    parser.add_argument("--hierarchical", action="store_true", help="启用层级 IK(torso→shoulder→full arm 三级求解),末端接近工作空间边缘时更易收敛")
    return parser.parse_known_args()


def main():
    cli_args, ros_args = parse_args()
    rclpy.init(args=ros_args)

    controller = CarryBoxController(
        approach_dy=cli_args.approach_dy,
        pregrasp_dz=cli_args.pregrasp_dz,
        descend_dz=cli_args.descend_dz,
        lift_dz=cli_args.lift_dz,
        place_dz=cli_args.place_dz,
        level_duration=cli_args.level_duration,
        pregrasp_duration=cli_args.pregrasp_duration,
        approach_duration=cli_args.approach_duration,
        lift_duration=cli_args.lift_duration,
        pull_back_duration=cli_args.pull_back_duration,
        place_duration=cli_args.place_duration,
        release_duration=cli_args.release_duration,
        retreat_duration=cli_args.retreat_duration,
        wrist_pitch_offset=cli_args.wrist_pitch_offset,
        wrist_yaw_offset=cli_args.wrist_yaw_offset,
        wrist_roll_offset=cli_args.wrist_roll_offset,
        pull_back_dx=cli_args.pull_back_dx,
        lock_joints=[] if cli_args.no_lock else None,
        use_hierarchical=cli_args.hierarchical,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(controller)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not controller.wait_for_state(timeout=5.0):
            controller.get_logger().error(
                "未收到机器人状态,请检查:1) 运控启动 2) switch_controller "
                "config_mc_walker_s2_v1_sps 3) DDS 中间件"
            )
            return

        if cli_args.pick_only:
            controller.pick_up_box(init_pose=not cli_args.no_init, level=not cli_args.no_level)
        elif cli_args.place_only:
            controller.place_down_box(retreat=not cli_args.no_retreat, return_home=not cli_args.no_home)
        else:
            controller.run_carry_task(
                init_pose=not cli_args.no_init,
                level=not cli_args.no_level,
                place=not cli_args.no_place,
                retreat=not cli_args.no_retreat,
                return_home=not cli_args.no_home,
            )
    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted, shutting down")
    finally:
        controller.stop()
        time.sleep(0.1)
        try:
            executor.shutdown(timeout_sec=1.0)
        except TypeError:
            executor.shutdown()
        spin_thread.join(timeout=2.0)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
