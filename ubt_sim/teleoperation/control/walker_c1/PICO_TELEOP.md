# Walker C1 PICO 遥操作与数据采集

该实现不使用 GMR，也不会向腰部或腿部发送命令。当前任务配置只控制右臂和右手；头部、
左臂和左手持续保持 `reset.py --mode task` 的标准姿态，不读取按 B 瞬间的反馈值作为固定目标。

## 数据与控制链路

- PICO 头显水平朝向 → 只用于建立操作者坐标基准，机器人头部不跟随
- 右手柄相对位姿 → 右手掌目标 → C1 右臂 IK
- 右手柄扳机 → 右手 6D Walker SDK 手指命令
- `/pico/joint_states` → Thinker Studio 预览
- `/mc/sdk/robot_command`、`/mc/{left,right}_hand/command` → 仿真或真机

仿真与真机使用相同 ROS2 topic；`ROS_DOMAIN_ID=146` 是仿真，`ROS_DOMAIN_ID=0` 是真机。

## 安全操作

- 默认 `--mode preview`，不发布机器人控制命令。
- 按住右手柄 B（`key_two`）才会捕获锚点并跟随；松开立即停止发布。
- 左手柄摇杆按下会锁定急停。松开所有按钮后按右 A 清除锁定，但不会自动恢复运动。
- PICO 位姿非法、机器人反馈不完整或 IK 残差超限时保持上一目标。
- IK 解相对上一帧跳变超过 0.35 rad 时拒绝该帧，防止冗余关节分支切换。
- 受控右肘没有弯到 `-0.40 rad` 以下时拒绝使能，避免从全零伸直奇异姿态开始。
- PICO 时间戳必须有效；当前允许操作者主动静止，静止时保持现有姿态，不按短超时断开。
- 进程启动后，右手柄必须产生过有效 pose 更新才允许使能；全零/非法 pose 会被拒绝。
- `--source mock` 禁止用于真机模式。
- 真机命令同时要求 `--enable-command --confirm-real-robot`。

## 连续采集多个 episode

Space 事件由 Isaac Sim 主窗口捕获，因此新增功能第一次使用前必须重启仿真。启动带录制的遥
操作：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh \
  --mode sim --source sdk --enable-command --record
```

推荐操作循环：

1. reset 完成后按住右手柄 B，开始右臂/右手遥操作，同时自动开始录制。
2. 完成一次任务后松开 B，在 Isaac Sim 主窗口按一次空格；无需连续按第二次。
3. 程序停止当前条、保存 HDF5，然后依次执行场景 reset 和标准 task reset。
4. 等日志显示 `ready for next episode`，再按 B 开始下一条；保存和 reset 期间的空格会被忽略。

默认保存目录：

```text
/ubt_sim/dataset/walker_c1_pico/<毫秒时间戳>/trajectory.hdf5
```

每条文件包含以头部 RGB 帧同步的左右臂/手反馈和对应 command action，字段与
`Walker_C1_26_1RGB.json` 的 26 维 LeRobot 转换配置一致。当前左侧 action 恒为标准 reset
姿态，右侧 action 是仿真实际收到的遥操作命令。若误按空格且尚无相机帧，则不会生成空文件，
但仍会 reset。HDF5 的 `record_hz` 是根据图像时间戳测得的实际采样率，
`requested_record_hz` 是采样上限；CPU 仿真中实际速率可能明显低于 30 Hz，转换数据时应使用
实际速率，不能仅把数据硬标为 30 Hz。R 键保留为原来的纯仿真 reset，不保存 episode。

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

模拟数据会从当前机器人姿态捕获右臂锚点，做小幅、限速的右臂和右手运动；头部与左侧保持
标准 task reset 姿态。

## Ubuntu + PICO 设备验证

XRoboToolkit 依赖 CPython 3.10。厂家 wheel 是 Linux x86_64 版本；Python 导入名是
`xrobotoolkit_sdk`（`robot` 后只有一个 `t`）。

```text
xgmr_tmp/pico/pico_teleop/deps/xrobotoolkit_sdk-1.0.2-cp310-cp310-linux_x86_64.whl
```

在最终 Ubuntu 环境中安装该 wheel，并确保 `libPXREARobotSDK.so` 可由动态链接器找到。启动 Thinker Studio/`xrobotoolkit-pc-service` 和 PICO 串流后，先运行：

容器内不要用 Isaac Python 3.11。把 wheel 和 `libPXREARobotSDK.so` 放到
`/opt/pico-sdk` 后，可不用 pip，直接准备隔离 runtime：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./prepare_pico_runtime.sh /opt/pico-sdk /opt/pico-runtime
```

`run_pico_teleop.sh` 会自动发现 `/opt/pico-runtime` 并设置动态库路径和
`PYTHONPATH`：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode preview --source sdk
```

如果 SDK runtime 不在默认位置，可设置：

```bash
export PICO_SDK_LIB_DIR=/path/to/runtime
export PICO_SDK_PYTHON_DIR=/path/to/runtime/python
```

PICO 应用只需开启 `Head`、`Controller` 和 `Data & Control -> Send`；`Hand` 可选。
当前控制链不读取腰部或腿部绑带，`PICO Motion Tracker` 保持 `None`，不需要
Full Body 校准。

真实 PICO 的第一轮工作只检查 `/pico/joint_states`、坐标方向、按钮、频率和断连，不发送机器人命令。确认后再在仿真中加 `--mode sim --enable-command`。

注意：preview 只发布 `/pico/joint_states`，不会让 Isaac Sim 机器人运动。控制仿真前必须先
执行 `reset.py --mode task`，随后使用：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode sim --source sdk --enable-command
```

## 真机前置条件

真机必须在机器人附近的 Ubuntu 电脑运行，不使用公网云端控制回路。执行命令前还必须完成仓库根目录 `C1_REAL_ROBOT_READONLY_CHECK.md` 的只读核对，特别是实机反馈关节名及肘关节命名。

命令入口保留为：

```bash
ROS_DOMAIN_ID=0 ./run_pico_teleop.sh \
  --mode real --source sdk --enable-command --confirm-real-robot
```

在真实 PICO 坐标校准和真机只读检查完成以前，不要执行该命令。
