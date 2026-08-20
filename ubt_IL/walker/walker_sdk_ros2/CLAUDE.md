# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指引。

## 仓库性质

这是一个 ROS2 colcon 工作区，以 `robot_control` Python 包为中心，提供Walker S2 人形机器人低层 SDK 接口的**消息定义**、**C++ 示例节点**和 **Python 控制包**。可作为 git submodule 集成到更大的 ROS2 工作区中。

主要交付物是 `robot_control` Python 包——外部项目通过 `from robot_control import RobotController` 使用。消息包和 C++ demo 是辅助编译依赖。

## 目录结构

```
robot_control/                     ← 仓库根 = ament_python 包
├── package.xml                    ← robot_control ament_python 包
├── setup.py / setup.cfg
├── robot_control/                 ← Python 源码（主要交付物）
├── ros2/                          ← 辅助 ROS2 包
│   ├── depend_msgs/               ← 7 个消息包
│   └── example/                   ← C++ 示例包（ubt_ros2_example）
├── CLAUDE.md
└── .gitignore
```

## 构建（两阶段）

消息包必须先编译，因为 robot_control 和 example 包依赖它们：

```bash
# 阶段 1：消息包
colcon build --packages-select \
  shm_msgs mc_state_msgs mc_task_msgs emb_task_msgs sys_task_msgs rosa_msgs ecat_task_msgs
source install/setup.bash

# 阶段 2：robot_control + example 包
colcon build --packages-select robot_control ubt_ros2_example
source install/setup.bash

# 运行 Python 控制器
ros2 run robot_control robot_control --print-state

# 运行 C++ 示例（可执行文件名 = .cpp 文件名去掉扩展名）
ros2 run ubt_ros2_example <executable_name>
```

构建完成后，Python 包可通过标准 import 使用：
```python
from robot_control import RobotController
from robot_control.camera import Camera
```

## 作为 submodule 集成

将本仓库作为 git submodule 添加到父工作区：

```bash
# 在父工作区中
cd parent_workspace/src/
git submodule add <repo_url> robot_control

# 构建方式同上（colcon 会自动发现 ros2/ 下的包）
colcon build --packages-select \
  shm_msgs mc_state_msgs mc_task_msgs emb_task_msgs sys_task_msgs rosa_msgs ecat_task_msgs
source install/setup.bash
colcon build --packages-select robot_control ubt_ros2_example
source install/setup.bash
```

## 新增示例

在 `ros2/example/src/` 下任意深度添加含 `main()` 的 `.cpp` 文件即可。[CMakeLists.txt](ros2/example/CMakeLists.txt) 使用 `file(GLOB_RECURSE)` 自动扫描——新文件会被自动发现，target 名 = 文件名（不含 `.cpp`）。

**构建缺口：** `upper_body_action_client.cpp` 和 `upper_body_service_client.cpp` 使用了 `nlohmann/json.hpp`，但未在 CMakeLists.txt 或 package.xml 中声明——需系统自行安装该头文件。

## 架构：两层控制

### 高层（任务式运动）

发送 JSON 格式的运动规划，控制 16 自由度上半身（2 腰部 + 7 左臂 + 7 右臂）。两个等效接口：

| 接口 | 类型 | 话题 | 优势 |
|---|---|---|---|
| Action | `mc_task_msgs/action/ArmTask` | `/mc/manipulation/action` | 实时进度反馈 |
| Service | `mc_task_msgs/srv/WalkerMotion` | `/mc/manipulation/service` | 更简单，发后即忘 |

两者均使用 `task_name="move_components_json"`、`mode=1`（轨迹优化器）、`vel_scale` 范围 [0.2, 0.8]。**第一个路点必须是当前关节位置**（发送前从 `/mc/whole_joint_states` 读取）。

### 低层（500Hz 直控关节）

| 部位 | 话题 | 消息 | mode 值 |
|---|---|---|---|
| 头部、腰部、手臂 | `/mc/sdk/robot_command` | `mc_task_msgs/RobotCommand`（内含 `JointCmd[]`） | `JointCmd::MODE_POSITION=2` |
| 手部（v3: 6自由度, v4: 7自由度） | `/mc/{left,right}_hand/command` | `mc_task_msgs/JointCommand`（并行数组） | `5` |

**关键区分：** `JointCmd`（无 mand）是嵌在 `RobotCommand` 内的单关节结构体；`JointCommand`（有 mand）是手部用的并行数组消息。两者的 mode 值**完全不同**——`JointCmd` 中 `MODE_POSITION=2`，`JointCommand` 中 `POSITION_MODE=1`。互相复制 mode 值会静默出错。

**v3 vs v4 手部：** 相同话题，互斥的硬件版本。V3 = 6 关节（无 `thumb_pip`），V4 = 7 关节（含 `thumb_pip`）。

### 语音

| 功能 | 接口 | 话题 |
|---|---|---|
| TTS 语音合成 | `sys_task_msgs/action/Tts`（`type=1`） | `/sys/speech/tts` |
| 音频文件播放 | `sys_task_msgs/action/Tts`（`type=0`） | `/sys/speech/tts` |
| ASR 识别结果 | topic（`sys_task_msgs/msg/Asr`） | `/sys/speech/asr` |
| ASR 开关 | service（`std_srvs/srv/SetBool`） | `/sys/asr/enable` |
| 麦克风原始数据 | topic（`std_msgs/msg/Int16MultiArray`，8声道 16kHz） | `/sys/speech/mic_source` |

### 大寰夹爪 / 电缸（`ecat_task_msgs`）

| 功能 | 话题 | 消息 | QoS |
|---|---|---|---|
| 左夹爪命令 | `/ecat/left_grip/cmd` | `ecat_task_msgs/msg/GripCmd` | 命令端通常 RELIABLE；若 Bare DDS 兼容异常可试 BEST_EFFORT |
| 左夹爪状态 | `/ecat/left_grip/state` | `ecat_task_msgs/msg/GripStatus` | BEST_EFFORT + VOLATILE |
| 右夹爪命令 | `/ecat/right_grip/cmd` | `ecat_task_msgs/msg/GripCmd` | 命令端通常 RELIABLE；若 Bare DDS 兼容异常可试 BEST_EFFORT |
| 右夹爪状态 | `/ecat/right_grip/state` | `ecat_task_msgs/msg/GripStatus` | BEST_EFFORT + VOLATILE |

`GripCmd` 字段：

| 字段 | 含义 |
|---|---|
| `init` | 1: 有效 |
| `mode` | 0: position/velocity/torque；10: 推压模式（电缸） |
| `stop` | 1: 停止；0: 可运动 |
| `reset` | 1: 有效 |
| `homing` | 1: 发送回零 |
| `pos` | 目标位置，单位 m |
| `vel` | 目标速度，单位 m/s |
| `force` | 目标力，单位 N |
| `cur` | 目标电流，单位 A；电缸推压模式下复用为加速度 m/s² |

当前测试约束按电缸安全范围收紧：

```text
pos   [0, 0.05] m
force [41, 100] N
vel   [0, 0.01] m/s
acc   [0, 3]    m/s²   # 写入 GripCmd.cur
```

`robot_control/joint_test.py` 提供夹爪测试参数：

```bash
# 打印状态
python3 joint_test.py --grip left --grip-print
python3 joint_test.py --grip both --grip-print

# 移动夹爪
python3 joint_test.py --grip left --grip-move 0.02 --grip-force 50 --grip-vel 0.005 --grip-acc 1.0

# 回零 / 停止 / 监控
python3 joint_test.py --grip left --grip-home
python3 joint_test.py --grip left --grip-stop
python3 joint_test.py --grip both --grip-monitor
```

**重要排查经验：** `/ecat/*_grip/cmd` 可能已有 Bare DDS 程序以 500Hz 持续发布命令。此时 `ros2 topic pub -r 10` 或低频测试脚本会被立即覆盖，夹爪看起来“不生效”。先检查：

```bash
ros2 topic hz /ecat/left_grip/cmd
ros2 topic echo /ecat/left_grip/cmd --once
ros2 topic info /ecat/left_grip/cmd -v
```

若已有 500Hz publisher，优先停掉该发布者；临时压测可用更高频率发布并匹配 QoS：

```bash
ros2 topic pub /ecat/left_grip/cmd ecat_task_msgs/msg/GripCmd \
"{init: 1, mode: 0, stop: 0, reset: 0, homing: 0, pos: 0.0, vel: 0.005, force: 50.0, cur: 1.0}" \
-r 1000 --qos-reliability best_effort
```

状态话题发布端是 BEST_EFFORT，订阅时需要匹配：

```bash
ros2 topic echo /ecat/left_grip/state --qos-reliability best_effort
```

## QoS —— 最常见的踩坑点

机器人传感器话题使用 sensor-data QoS。订阅端**必须**匹配，否则静默收不到任何消息：

```cpp
// 状态/图像订阅 —— 所有机器人传感器流必须这样设置
rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()
// robot_state 和手部状态还需加：
.durability_volatile()

// 命令发布端 —— 使用默认 RELIABLE QoS
rclcpp::QoS(10)  // 默认 reliable，命令方向这样写即可
```

## 共享内存图像消息（`shm_msgs`）

相机图像使用固定大小 `uint8[N*1048576]` 数组（而非变长 `sensor_msgs/Image`）以支持零拷贝 DDS 传输。变体从 `Image1m` 到 `Image16m`，数字 = 缓冲区 MB 数。

| 相机 | 话题 | 消息 |
|---|---|---|
| 后背 RGBD 彩色/深度 | `/sensor/camera/body_back_rgbd/{color,depth}/raw` | `Image1m` |
| 腰前 RGBD 彩色/深度 | `/sensor/camera/waist_front_rgbd/{color,depth}/raw` | `Image1m` |
| 鱼眼 左/右 | `/sensor/camera/fisheye_{left,right}/image/raw` | `Image2m` |
| 立体 左/右 | `/sensor/camera/stereo_{left,right}/image/raw` | `Image6m` |

**有效像素字节数 = `step × height`**，不是 `data.size()`（后者始终是完整的固定缓冲区大小）。编码字符串存于 `msg->encoding.data`（固定 `char[256]`，非 `std::string`）。示例订阅者仅打印元数据——不做 OpenCV 图像解码。

## 消息包一览

| 包名 | 关键类型 | 用途 |
|---|---|---|
| `emb_task_msgs` | `BatteryState`、`InnerData` | 电池和电源板监控 |
| `mc_state_msgs` | `RobotState`（关节 + IMU + 力矩） | 传感器速率的完整机器人状态 |
| `mc_task_msgs` | `RobotCommand`/`JointCmd`、`JointCommand`、`ArmTask`、`WalkerMotion`、`GaitModeSwitch` | 运动命令与任务 |
| `sys_task_msgs` | `Asr`（msg）、`Tts`（action） | 语音识别与合成 |
| `shm_msgs` | `Image*Nm`（1m–16m） | 共享内存相机图像 |
| `rosa_msgs` | `NodeState` | 状态码（7位：类型.模块.子模块.编码） |
| `ecat_task_msgs` | `GripStatus`、`GripCmd` | 大寰 PGC-140-50 夹爪 / 电缸（`/ecat/{left,right}_grip/{state,cmd}`） |
