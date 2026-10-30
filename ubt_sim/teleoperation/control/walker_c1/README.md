# Walker C1 控制与采集（ROS 侧）

真机同款 SDK 话题面：仿真与真机只差 `ROS_DOMAIN_ID`（仿真 146 / 真机 0），
本目录的脚本两边通用。

> 本分支**未移植** PICO 头显遥操作（`pico_*.py`、头显图传、xrobotoolkit 依赖）；
> 遥操作替代路径为：`reset.py` 回预备位 + 在线 IK 抓放（`pick_place_controller.py`）
> 或直接按 `/mc/sdk/robot_command` / `/mc/{left,right}_hand/command` 下发指令。

## 文件

| 文件 | 作用 |
|------|------|
| `constants.py` | 31 身体关节表、手部 SDK 6 维表、任务复位姿态与时序常量（不 import Isaac） |
| `robot_controller.py` | 运动原语：palm FK/IK（ikpy + `right_arm.urdf`，腰锁定）、多初值选解、平滑插值、闭环修正、开合手、`go_ready` |
| `dual_arm_ik.py` | 整机 URDF 运行时裁链 + scipy `least_squares` 双臂 IK（限位/跳变/肘角约束） |
| `pick_place_controller.py` | 在线 IK 抓放（读 `/sim/object_state` 现场规划）+ HDF5 录制 + CLI |
| `reset.py` | 四种复位：`task` / `sim-task` / `sim-scene` / `sdk-home` |
| `save_data.py` / `save_data.sh` | 手动/无人值守/批量采集，落 `dataset/walker_c1/<epoch>/trajectory.hdf5` |
| `utils/recorder.py` | `WalkerC1DataRecorder(Node)`：15 Hz 定频采样 + 通用 `EpisodeRecorder` 契约 |
| `right_arm.urdf` | 裁剪版右臂 URDF（`base_link → waist → R 臂 → palm`，腰 locked） |

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
export ROS_DOMAIN_ID=146

# 复位到任务预备位
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode task

# 单集抓放（含录制）
/usr/bin/python3 teleoperation/control/walker_c1/pick_place_controller.py --save

# 手动采集
bash teleoperation/control/walker_c1/save_data.sh

# 批量抓放采集
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

## 数据契约

与 `teleoperation/control/common/hdf5_recorder.py` 一致：

* `joint_observations/*`：31 个身体关节（顺序 = `constants.BODY_JOINT_NAMES`）
* `hand_observations/*`：左右手各 6 个 SDK 压缩通道
* `action_observations/*`：最近一次下发指令（未下发时用当前状态填充）
* `camera_observations/{color_images,depth_images}/camera_head`
* 根属性 `robot_type=walker_c1`（回放适配层首选字段）

## 复位姿态

* 预备位：`TASK_RESET_BODY_POSE`（臂 -0.291 / roll ±0.30 / 肘 -1.155 / 腕 -0.361，头 pitch 0.50）
* 分段时序：抬肩（`TASK_RESET_ARM_CLEAR_POSE`）→ 收肘（`TASK_RESET_ELBOW_CLEAR_POSE`）→ 预备位
* 手部：SDK 6 维全 0 = 张开中间位

## 注意

* `right_arm.urdf` 与 `dual_arm_ik.py` 的整机 URDF 是两套：前者单臂 IK（抓放），
  后者双臂 IK（可加肘角约束防奇异）。两处路径都支持 `UBT_SIM_ASSETS_ROOT` 覆盖。
* 采样频率默认 15 Hz，与其它机型一致；`--record-hz` 可调。
* 采集/抓放的录制统一走 `EpisodeRecorder`，不要再用旧的 `puppet/...` schema。
