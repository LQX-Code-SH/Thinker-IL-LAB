# TienKung Dex 遥操作与数据采集

本目录脚本跑在 **ROS2 侧**（系统 Python 3.10），不依赖 Isaac Sim；
仿真进程由 `scripts/start_sim.sh` 启动，两者通过 ZMQ 5755-5758 端口通信。

## 启动顺序

```bash
# 0) 容器内
cd /ubt_sim
source /opt/ros/humble/setup.bash

# 1) 启动仿真 + ROS2-ZMQ 桥接（自动）
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# 2) 遥操作（另开终端）：关节空间键盘控制
/usr/bin/python3 teleoperation/control/tienkung_dex/teleop_controller.py --step 0.02
#    BrainCo 变体：加 --variant brainco

# 3) 数据采集（再开终端）
bash teleoperation/control/tienkung_dex/save_data.sh            # Enter 开始/停止，s 保存，q 退出
bash teleoperation/control/tienkung_dex/save_data.sh --variant brainco   # BrainCo 双手（11 关节/手）
```

也可以用外部遥操作端（Thinker / 空间鼠标）直接向
`/tienkung_dex/{body,left_hand,right_hand}_command` 发布 `sensor_msgs/JointState`
（`name` 支持省略 `_joint` 后缀），此时不需要 `teleop_controller.py`。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 指令 | `/tienkung_dex/body_command` | `JointState` | 31 个身体关节目标（rad） |
| 指令 | `/tienkung_dex/left_hand_command` | `JointState` | 左手目标（rad）：Inspire 18 / BrainCo 11 |
| 指令 | `/tienkung_dex/right_hand_command` | `JointState` | 右手目标（rad）：Inspire 18 / BrainCo 11 |
| 状态 | `/tienkung_dex/body_state` | `JointState` | 实测位置/速度 |
| 状态 | `/tienkung_dex/left_hand_state` / `right_hand_state` | `JointState` | 手部实测 |
| 图像 | `/sensor/camera/head/color/raw` | `sensor_msgs/Image` (`rgb8`) | 640×360 |
| 图像 | `/sensor/camera/head/depth/raw` | `sensor_msgs/Image` (`16UC1`, mm) | 无效值 65535 |
| 复位 | `/sim/cmd_reset` | `std_msgs/Bool` | `reset.py` 发布 |

## 采集产物

```
dataset/tienkung_dex/episode_0001.hdf5
├── joint_observations/{joint_position, joint_velocity, joint_effort}    # 31
├── hand_observations/{hand_left_*, hand_right_*}                        # Inspire 18+18 / BrainCo 11+11
├── action_observations/{joint_position, hand_left_position, hand_right_position}
├── camera_observations/images/head_rgb                                  # PNG 字节流
├── camera_observations/depth_images/head_depth                          # uint16 mm
└── timestamp
```

`attrs` 内含 `robot / hand_variant / body_joint_names / left_hand_joint_names / right_hand_joint_names / capture_hz / task_name`。

> 27 维训练口径对应「14 臂 + 13 手通道」，V3 手部为 18 关节，转换脚本需按 `*_joint_names`
> 选取通道后重排，具体映射在 `ubt_IL` 侧实现（本期不涉及）。

## 已知限制

- 手部 `mimic` 关节（每手 5 个，`thumb_pip`、各指 `*_dip`）由 `action_process` 按 URDF 倍率
  展开，ROS 侧无需下发，但状态里会以实测值出现；
- 相机外参默认取 `camera_head_link` 原点，若画面朝向不符请改
  `config/tienkung_dex/parlor.yaml` 的 `cameras.head_rgb.pos/rot`；
- 尚未接入真机 SDK 的电机 ID 映射，指令直接使用仿真关节名。
