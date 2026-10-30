# Walker C1（Astron）仿真使用说明

Walker C1 为全机 53 DOF 人形（双臂 14 + 双手 11×2 + 头 2 + 腰 3 + 双腿 12），
上体可控 41 关节；本仓库实现其 Isaac Sim 仿真、ROS2-ZMQ 桥接、在线 IK 抓放与数据采集。

| 项 | 值 |
|----|----|
| 任务 ID | `UBTSim-WalkerC1-Parlor-v0` |
| USD | `assets/robots/walker_c1/usd/walker_c1_with_hands.usd`（原始无手版在 `usd/original/`） |
| URDF（IK/限位） | `assets/robots/walker_c1/urdf/walker_c1_with_hands.urdf`（65 joint / 53 驱动） |
| ZMQ 端口 | 5655 指令 / 5656 状态 / 5657 原始 RGB / 5658 JPEG |
| 消息包 | `teleoperation/msgs/walker_c1/` → `/opt/ubt_sim/walker_c1_msgs`（与 Walker S2 独立） |

> **端口冲突**：C1 与 Walker S2 共用 5655-5658，两者不能同时运行。
> **数据契约**：26 维外部指令 = 左臂 7 + 左手 SDK 6 + 右臂 7 + 右手 SDK 6；
> 手部按真机 SDK 的 6 维压缩通道落盘（仿真侧展开到 11 个关节）。

## 1. 环境准备

```bash
cd ubt_sim/docker
bash run.sh build          # 首次构建镜像
bash run.sh init           # 编译各机型消息包（含 walker_c1_msgs，与 walker_sdk 隔离）
bash run.sh shell          # 进入容器
```

容器内（`/ubt_sim` 即仓库的 `ubt_sim/`）：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
export ROS_DOMAIN_ID=146          # 真机为 0
```

## 2. 启动仿真

```bash
# 方式一：C1 专用脚本（100 Hz 物理步进）
bash scripts/start_c1_pick_place_sim.sh

# 方式二：统一启动器
UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 bash scripts/start_sim.sh

# 只看场景（不启桥接、不接控制）
UBT_SIM_LOAD_ONLY=1 UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 bash scripts/start_sim.sh
```

启动后会依次打印 ZMQ 端口、action 关节顺序、手部关节限位与增益摘要。

**桥接自检**（另开终端，同一 `ROS_DOMAIN_ID`）：

```bash
ros2 topic list | grep -E "mc/sdk|sim/"
ros2 topic echo --once /mc/sdk/robot_state          # 31 个身体关节
ros2 topic echo --once /mc/left_hand/joint_states   # SDK 6 维
ros2 topic echo --once /sim/object_state            # 苹果/根位姿/sim_step
```

## 3. 复位

```bash
# 任务预备位（ROS 话题，分三段：抬肩 → 收肘 → 预备位）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode task

# ZMQ 直发（不依赖桥接）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode sim-task

# 场景复位（等价于在仿真窗口按 R）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode sim-scene

# 厂商 ROSA 上体复位（真机；仿真无需）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode sdk-home --dry-run
```

仿真窗口内 `R` = 场景复位，`Space` = 请求保存当前 episode（与 ROS 侧录制器配合）。

## 4. 数据采集

```bash
# 手动：Enter 开始/停止，s 保存，q 退出
bash teleoperation/control/walker_c1/save_data.sh

# 无人值守：每集 30 秒、共 5 集
bash teleoperation/control/walker_c1/save_data.sh --auto-start --duration 30 --episodes 5

# 批量抓放采集：每集「复位 → 抓苹果 → 放盘 → 回预备位」，成功才落盘
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

落盘位置：`dataset/walker_c1/<epoch秒>/trajectory.hdf5`（目录 0777、文件 0666）。

HDF5 结构（与其它机型统一的录制契约）：

```
joint_observations/{joint_position, joint_velocity, joint_effort}   # 31 身体关节
hand_observations/hand_{left,right}_{position,velocity,effort}      # SDK 6 维
action_observations/{joint_position, hand_left_position, hand_right_position}
camera_observations/color_images/camera_head                        # PNG 字节流
camera_observations/depth_images/camera_head                        # uint16 毫米（--no-depth 可关）
timestamp
```

根属性含 `robot_type=walker_c1`、`body_joint_names`、`left/right_hand_joint_names`、
`camera_names`、`fps`、`capture_hz`。

## 5. 数据回放

```bash
# 预览（Rerun 查看相机 + 关节曲线），缺省用 dataset/walker_c1 下最新一集
/usr/bin/python3 teleoperation/tools/playback_walker_c1.py --mode preview

# 控制回放（向真机同款话题发布）
/usr/bin/python3 teleoperation/tools/playback_walker_c1.py \
    --episode dataset/walker_c1/<epoch秒>/trajectory.hdf5 --mode control

# 只校验数据不发布
/usr/bin/python3 teleoperation/tools/playback_walker_c1.py --mode control --dry-run
```

## 6. 在线 IK 抓放

```bash
# 单集（含录制）
bash scripts/run_c1_pick_place_once.sh

# N 集批采：每集重启仿真栈，按 SUCCESS + trajectory saved 判定
bash scripts/run_c1_online_ik_batch.sh 3 5
```

抓放流程：读 `/sim/object_state` 的苹果位姿 → 双臂 IK 规划 → 下探抓取 →
抬升校验 → 搬运到盘子上方 → 释放 → 回预备位。成功判定为苹果最终位置到盘心
水平距离 ≤ `--task-dist-tol`（默认 0.12 m）且高度 > 0.9 m。

## 7. 常见问题

| 现象 | 排查 |
|------|------|
| `gym.make` 找不到任务 ID | `source/ubt_sim/__init__.py` 是否包含 `from .task.walker_c1_parlor import *` |
| 桥接启动即退出，报 `RMW implementation not installed` | RMW 与镜像不匹配：本仓库镜像只装了 `rmw_cyclonedds_cpp`，用 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；若镜像装了 FastRTPS 则用 `rmw_fastrtps_cpp`（`start_sim.sh` 已自动探测） |
| 指令发出去机器人不动 | 检查 `/mc/sdk/robot_command` 是否用了 `elbow_pitch`（`elbow_roll` 也可，桥接有别名） |
| 图像话题无数据 | 未加 `--enable_cameras`；或 C++ 图像桥未编译（`bash teleoperation/bridges/build_cpp_bridge.sh`） |
| 采集 HDF5 帧数不一致 | 录制节点在图像未到齐时会跳拍，属预期；若整集 0 帧检查相机话题 |
| 苹果抓不住 / 弹飞 | 手部增益必须保持 `10/1`、`effort_limit_sim=2`（力控）；不要按老调试值放大 |
| 与 Walker S2 同时启动后指令串流 | 二者端口相同（5655-5658），同一时间只能跑一个 |
