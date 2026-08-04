# Walker C1 PICO 遥操作（头、双臂、双手）

该实现不使用 GMR，也不会向腰部或腿部发送命令。

## 数据与控制链路

- PICO 头显相对姿态 → `head_yaw_joint`、`head_pitch_joint`
- 左右手柄相对位姿 → 左右手掌目标 → C1 双臂 IK
- 左右手柄扳机 → 两只手的 6D Walker SDK 手指命令
- `/pico/joint_states` → Thinker Studio 预览
- `/mc/sdk/robot_command`、`/mc/{left,right}_hand/command` → 仿真或真机

仿真与真机使用相同 ROS2 topic；`ROS_DOMAIN_ID=146` 是仿真，`ROS_DOMAIN_ID=0` 是真机。

## 安全操作

- 默认 `--mode preview`，不发布机器人控制命令。
- 按住右手柄 B（`key_two`）才会捕获锚点并跟随；松开立即停止发布。
- 左手柄摇杆按下会锁定急停。松开所有按钮后按右 A 清除锁定，但不会自动恢复运动。
- PICO 位姿非法、机器人反馈不完整或 IK 残差超限时保持上一目标。
- IK 解相对上一帧跳变超过 0.35 rad 时拒绝该帧，防止冗余关节分支切换。
- 双肘没有弯到 `-0.40 rad` 以下时拒绝使能，避免从全零伸直奇异姿态开始。
- PICO 时间戳连续 0.25 秒不更新时自动解除使能并停止发布。
- `--source mock` 禁止用于真机模式。
- 真机命令同时要求 `--enable-command --confirm-real-robot`。

## 云端无 PICO 验证

先启动 Walker C1 仿真及其 ROS2-ZMQ bridge，确认以下命令能收到完整关节名：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
ROS_DOMAIN_ID=146 ros2 topic echo /mc/sdk/robot_state --once
```

只发布 Thinker Studio 预览：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode preview --source mock
```

确认预览和安全状态正确后，才允许模拟数据控制仿真：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode sim --source mock --enable-command
```

遥操作前需要先进入弯肘准备姿态：

```bash
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task
```

模拟数据会从当前机器人姿态捕获锚点，做小幅、限速的对称双臂运动，同时测试头部和手指命令。

## Ubuntu + PICO 设备验证

XRoboToolkit 依赖 CPython 3.10。仓库中的 wheel 是 Linux x86_64 版本：

```text
xgmr_tmp/pico/pico_teleop/deps/xrobotoolkit_sdk-1.0.2-cp310-cp310-linux_x86_64.whl
```

在最终 Ubuntu 环境中安装该 wheel，并确保 `libPXREARobotSDK.so` 可由动态链接器找到。启动 Thinker Studio/`xrobotoolkit-pc-service` 和 PICO 串流后，先运行：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode preview --source sdk
```

如果 SDK 动态库不在仓库默认位置，可设置：

```bash
export PICO_SDK_LIB_DIR=/path/to/directory/containing/libPXREARobotSDK.so
```

真实 PICO 的第一轮工作只检查 `/pico/joint_states`、坐标方向、按钮、频率和断连，不发送机器人命令。确认后再在仿真中加 `--mode sim --enable-command`。

## 真机前置条件

真机必须在机器人附近的 Ubuntu 电脑运行，不使用公网云端控制回路。执行命令前还必须完成仓库根目录 `C1_REAL_ROBOT_READONLY_CHECK.md` 的只读核对，特别是实机反馈关节名及肘关节命名。

命令入口保留为：

```bash
ROS_DOMAIN_ID=0 ./run_pico_teleop.sh \
  --mode real --source sdk --enable-command --confirm-real-robot
```

在真实 PICO 坐标校准和真机只读检查完成以前，不要执行该命令。
