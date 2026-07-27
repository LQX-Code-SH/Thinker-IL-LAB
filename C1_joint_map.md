# C1 Joint Map for ubt_sim

本文档记录从 C1 新整机 URDF 中解析出来的关节分组，后续用于编写 `ubt_sim/source/ubt_sim/devices/c1/config.py`。

来源 URDF：

```text
ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3_no_sixforce_mesh.urdf
```

当前 URDF 统计：

```text
movable joints = 53
fixed joints   = 16
total joints   = 69
```

分组统计：

```text
waist       3
head        2
left_arm    7
right_arm   7
left_hand   11
right_hand  11
left_leg    6
right_leg   6
```

## Arm Joints

### Left Arm

```python
C1_LEFT_ARM_JOINTS = [
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_pitch_joint",
    "L_elbow_yaw_joint",
    "L_wrist_pitch_joint",
    "L_wrist_roll_joint",
]
```

### Right Arm

```python
C1_RIGHT_ARM_JOINTS = [
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_pitch_joint",
    "R_elbow_yaw_joint",
    "R_wrist_pitch_joint",
    "R_wrist_roll_joint",
]
```

## Hand Joints

C1 新三代手每只手有 11 个 `revolute` 手指关节。Tiankung 每只手是 12 个手指关节，并且有 mimic 映射，所以 C1 不能直接复用 Tiankung 的 hand action mapping。

### Left Hand

```python
C1_LEFT_HAND_JOINTS = [
    "L_thumb_cmp_joint",
    "L_thumb_mpp_joint",
    "L_thumb_ip_joint",
    "L_index_mpp_joint",
    "L_index_ip_joint",
    "L_middle_mpp_joint",
    "L_middle_ip_joint",
    "L_ring_mpp_joint",
    "L_ring_ip_joint",
    "L_little_mpp_joint",
    "L_little_ip_joint",
]
```

### Right Hand

```python
C1_RIGHT_HAND_JOINTS = [
    "R_thumb_cmp_joint",
    "R_thumb_mpp_joint",
    "R_thumb_ip_joint",
    "R_index_mpp_joint",
    "R_index_ip_joint",
    "R_middle_mpp_joint",
    "R_middle_ip_joint",
    "R_ring_mpp_joint",
    "R_ring_ip_joint",
    "R_little_mpp_joint",
    "R_little_ip_joint",
]
```

## Head and Waist Joints

### Head

```python
C1_HEAD_JOINTS = [
    "head_yaw_joint",
    "head_pitch_joint",
]
```

### Waist

```python
C1_WAIST_JOINTS = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]
```

## Leg Joints

### Left Leg

```python
C1_LEFT_LEG_JOINTS = [
    "L_hip_pitch_joint",
    "L_hip_roll_joint",
    "L_hip_yaw_joint",
    "L_knee_pitch_joint",
    "L_ankle_pitch_joint",
    "L_ankle_roll_joint",
]
```

### Right Leg

```python
C1_RIGHT_LEG_JOINTS = [
    "R_hip_pitch_joint",
    "R_hip_roll_joint",
    "R_hip_yaw_joint",
    "R_knee_pitch_joint",
    "R_ankle_pitch_joint",
    "R_ankle_roll_joint",
]
```

## Joint Limits Summary

### Waist

| Joint | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `waist_yaw_joint` | -2.792 | 2.792 | 63 | 10.47 |
| `waist_pitch_joint` | -0.017 | 1.221 | 165 | 8.9 |
| `waist_roll_joint` | -0.75 | 0.75 | 110 | 10.47 |

### Head

| Joint | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `head_yaw_joint` | -1.658 | 1.658 | 4.5 | 5.24 |
| `head_pitch_joint` | -0.523 | 0.785 | 4.5 | 5.24 |

### Hands

| Joint Pattern | Lower | Upper | Effort | Velocity |
|---|---:|---:|---:|---:|
| `L_thumb_cmp_joint` | -0.96 | 0 | 1.35 | 2.09 |
| `R_thumb_cmp_joint` | 0 | 0.96 | 1.35 | 2.09 |
| `L/R_thumb_mpp_joint` | 0 | 1.04 | 1.35 | 2.09 |
| `L/R_thumb_ip_joint` | 0 | 1.05 | 1.35 | 2.09 |
| `L/R_*_mpp_joint` for index/middle/ring/little | 0 | 1.46 | 1.35 | 2.09 |
| `L/R_*_ip_joint` for index/middle/ring/little | 0 | 1.62 | 1.35 | 2.09 |

## SDK Cross-Check (2026-07-13)

已用官方 SDK 文档 `【CC-API】Astron优必选SDK二次开发文档【对内】.docx` 逐项核对本文件里的
joint order。文档权威结论如下，用于对齐真机。

### 一致项（仿真代码 == SDK 文档）

- **左右手 6D SDK 顺序**：`thumb_swing, thumb_mcp, index_mcp, middle_mcp, ring_mcp, little_mcp`
  （文档 `/mc/{left,right}_hand/command` 示例，逐字匹配 `action_process.py`）。
- **头 2D**：`head_yaw_joint, head_pitch_joint`。
- **腰**：Astron waist=3（S2=2），本文件 `waist_yaw/pitch/roll` 维度正确。
- **命令都是按名字下发**：身体 `RobotCommand{ JointCmd[] }`（每个 `JointCmd.name`）、
  手 `JointCommand{ names[], position[], mode[] }`。所以下发顺序不影响真机，**关节名字必须与真机一致**。
  `JointCmd.MODE_POSITION = 2`；未控制的关节控制量填 0（置零位）。

### ⚠️ 待真机确认的差异：臂第 4 关节名字

同一个肘部槽位，两处命名不一致：

| 来源 | 臂第 4 关节名 |
|---|---|
| URDF / 仿真 / 本文件 | `elbow_pitch_joint` |
| SDK 运动学文档 `CalcEndPose` q 顺序注释 | `elbow_roll_joint` |

SDK 文档给出的单臂 7D 顺序：
`shoulder_pitch → shoulder_roll → shoulder_yaw → elbow_roll → elbow_yaw → wrist_pitch → wrist_roll`。
除第 4 个名字外全部一致。因为真机是按名字下发，如果真机实际关节名是 `elbow_roll_joint`，
用 `elbow_pitch_joint` 下发那一维会被真机忽略。

**上真机第一件事**：订阅 `/mc/sdk/robot_state`，dump `joint_states.name`，核对臂第 4 关节真名
（`elbow_pitch` vs `elbow_roll`），并顺带核对全部臂/腰关节名。低层 `RobotCommand` 文档示例只演示了
head 关节名，没有列臂关节名，所以臂名字必须以真机 `robot_state` 为准。

### 备注

仿真的 26 维 ZMQ 输入 `left_arm[7] + left_hand[6] + right_arm[7] + right_hand[6]` 是**仿真内部约定**，
不是真机线格式。真机臂/腰/头走 `RobotCommand`（按名），手走 `/mc/{left,right}_hand/command`（按名 6D）。
桥接真机时按关节名字映射即可，不受这个拼接顺序影响。

## Notes for C1 Config

1. `C1_HOME_POSE` 可以先全部设为 `0.0`，但腿部如果要站立，后续可能需要参考 MJCF 或 SDK 的默认站姿。
2. 仿真初期可以只控制双臂、双手、头和腰，腿部先用 home pose 固定。
3. C1 手没有沿用 Tiankung 的 mimic 配置。后续 `devices/c1/action_process.py` 应该先按 22 个手指关节独立映射。
4. 当前整机 URDF 还有两个缺失 mesh：`L_sixforce_link.STL` 和 `R_sixforce_link.STL`。它们影响腕部外观/碰撞，不影响 joint map。
5. Isaac Lab 运行时加载的是 USD，不是这份 URDF；机器人当前使用的 USD 见
   `README_C1_LEROBOT.md` 的目录结构说明。
