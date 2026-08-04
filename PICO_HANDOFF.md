# Walker C1 PICO 遥操作接力文档

更新日期：2026-08-03

本文档面向下一位继续开发或现场联调的同事，记录当前实现边界、验证结果、外部依赖、运行方法、已知风险和推荐接力顺序。

## 1. 当前结论

Walker C1 已有一版独立的 PICO 头部、双臂、双手遥操作实现，当前处于：

```text
代码完成 + 自动测试通过 + 等待真实 PICO 和真机联调
```

已经完成：

- PICO SDK 数据适配层。
- Unity/PICO 坐标到机器人基座坐标的转换。
- 基于当前机器人姿态捕获锚点的相对位姿控制。
- 从 C1 完整 URDF 动态提取左右臂运动链。
- 左右臂 7-DoF 位置和姿态 IK。
- 头部 yaw/pitch 相对控制。
- 左右扳机到 C1 6D 逻辑手指命令的映射。
- `/pico/joint_states` Thinker Studio 预览。
- 仿真和真机 ROS SDK topic 输出。
- Deadman、急停锁定、数据超时、IK 残差、解跳变、限速和工作空间保护。
- Preview/sim/real 模式与 ROS Domain 防误用检查。
- 数学、IK 和 ROS 消息通路自动测试。

尚未完成：

- 未连接真实 PICO 验证 SDK、按钮、坐标方向和刷新率。
- 未在 C1 仿真中进行人工手柄闭环遥操作验收。
- 未连接 C1 真机读取实际关节名。
- 未向 C1 真机发送任何 PICO 运动指令。
- 未验证现场 C1 肘关节使用 `elbow_pitch_joint` 还是 `elbow_roll_joint`。

因此不得把当前状态描述为“PICO 真机已经跑通”。

## 2. 设计边界

当前实现不使用 GMR。控制链为：

```text
PICO 头显/左右手柄
  -> XRoboToolkit SDK
  -> 相对位姿与坐标变换
  -> C1 左右臂 IK
  -> ROS 2 RobotCommand / JointCommand
```

控制范围：

- 头部 2 关节。
- 左臂 7 关节。
- 右臂 7 关节。
- 左手 6D 逻辑命令。
- 右手 6D 逻辑命令。

不会向腰部或腿部发送命令。

原 GMR 工程适合“PICO 人体追踪 -> 全身机器人动作重定向”；当前需求是“头显/手柄 -> 头部和双臂末端控制”，所以采用了更直接、可控的双臂 IK 路径。

## 3. 文件地图

实现目录：

```text
ubt_sim/teleoperation/control/walker_c1/
```

| 文件 | 作用 |
|---|---|
| `pico_source.py` | XRoboToolkit SDK 封装和确定性的 mock 数据源 |
| `pico_math.py` | 四元数、坐标变换、相对目标、头部角度和逐帧限速 |
| `dual_arm_ik.py` | 从 C1 完整 URDF 提取左右臂链并求解 6D IK |
| `pico_teleop.py` | ROS 节点、状态反馈、锚点、按键、安全逻辑和命令发布 |
| `pico_teleop_config.json` | 坐标基、工作空间、缩放、限速、IK 阈值和手部闭合位 |
| `run_pico_teleop.sh` | ROS/消息环境和 PICO 动态库启动封装 |
| `test_pico_math.py` | 坐标和数学单元测试 |
| `test_dual_arm_ik.py` | 双臂 FK/IK 恢复及小位移测试 |
| `test_pico_ros.py` | ROS topic、消息维度、mode、Deadman 和模式保护 smoke test |
| `PICO_TELEOP.md` | 面向操作者的运行说明 |

真机只读检查见仓库根目录：

```text
C1_REAL_ROBOT_READONLY_CHECK.md
```

## 4. ROS 接口和维度

订阅：

| Topic | 类型 | QoS |
|---|---|---|
| `/mc/sdk/robot_state` | `mc_state_msgs/RobotState` | BEST_EFFORT + VOLATILE |

发布：

| Topic | 类型 | 内容 |
|---|---|---|
| `/pico/joint_states` | `sensor_msgs/JointState` | 2头 + 14臂 + 12手 = 28D 预览 |
| `/mc/sdk/robot_command` | `mc_task_msgs/RobotCommand` | 2头 + 14臂 = 16 个按名位置命令 |
| `/mc/left_hand/command` | `mc_task_msgs/JointCommand` | 左手 6D，SDK position mode 5 |
| `/mc/right_hand/command` | `mc_task_msgs/JointCommand` | 右手 6D，SDK position mode 5 |

身体命令使用 `JointCmd.MODE_POSITION == 2`，手部 `JointCommand.mode[]` 使用厂家自定义值 `5`，二者不可混用。

## 5. 安全机制

- 默认 `--mode preview`，不发布机器人命令。
- 只有按住右 B 才捕获锚点并持续发送；松开立即解除使能。
- 左摇杆按下触发急停锁定。
- 松开其他按钮后按右 A 只能清除锁定，不会自动恢复运动。
- 必须收到头部和双臂完整反馈才允许使能。
- 双肘必须处于弯曲准备姿态，避免从伸直奇异位形启动。
- PICO 时间戳超过 0.25 秒不变化时自动解除使能。
- 非法位姿、IK 失败、残差超限或单帧解跳变超限时拒绝当前帧。
- 每帧关节和头部变化受限，末端目标受工作空间和最大位移限制。
- `--source mock` 禁止与 `--mode real` 同时使用。
- 真机命令必须同时提供 `--enable-command --confirm-real-robot`。
- `mode=sim` 要求 `ROS_DOMAIN_ID=146`；`mode=real` 要求 `ROS_DOMAIN_ID=0`。

这些是软件保护，不能替代现场急停、遥控器、限位和安全员。

## 6. 已完成测试

在 `walker-c1-ubt-sim` 容器运行：

```bash
docker exec walker-c1-ubt-sim bash -lc '
source /opt/ros/humble/setup.bash &&
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash &&
cd /ubt_sim/teleoperation/control/walker_c1 &&
/usr/bin/python3 -m unittest -v \
  test_pico_math.py test_dual_arm_ik.py test_pico_ros.py'
```

当前结果：

```text
8 tests passed
```

覆盖：

- PICO pose 坐标基转换。
- 相对目标位移和工作空间裁剪。
- 头部 yaw/pitch。
- 逐帧关节限速。
- mock 帧数据协议。
- 左右臂 FK/IK 和小位移目标。
- ROS 命令面、28D 预览、16D 身体命令和双手 6D 命令。
- 手部 mode 5。
- 右 B Deadman、弯肘姿态门槛、real/sim Domain 和 mock/real 防误用。

自动测试不能替代真实 PICO 坐标和物理机器人验证。

## 7. 新 Ubuntu 笔记本准备

代码应通过 Git 获取，不再打包整个工作区：

```bash
git clone --recurse-submodules \
  --branch walker_c1 \
  git@github.com:UBTECH-Robot/TienKung-IL-LAB.git \
  walker_c1
```

双臂 IK 需要这个 LFS URDF：

```bash
cd walker_c1
git lfs pull \
  --include="ubt_sim/assets/robots/walker_c1/walker_astron_v2_hand_v3_no_sixforce_mesh.urdf"
```

Ubuntu 24 宿主机不要直接混装项目 ROS Humble 环境，优先使用仓库 Docker。

## 8. PICO 外部依赖

厂家 PICO 二进制依赖没有提交到本仓库。当前参考副本位于开发机：

```text
/home/changzhang/VLA/xgmr_tmp/pico/pico_teleop/deps/
  xrobotoolkit_sdk-1.0.2-cp310-cp310-linux_x86_64.whl
  libPXREARobotSDK.so

/home/changzhang/VLA/xgmr_tmp/pico/pico_headless_service/
```

需要单独通过内部允许的方式传到新笔记本。不要把这些厂家二进制提交到公开仓库。

wheel 仅适配 Linux x86_64 CPython 3.10。Ubuntu 24 默认 Python 3.12 不能直接安装，推荐在项目 Humble/Python 3.10 容器中使用。

`run_pico_teleop.sh` 支持通过环境变量指定动态库目录：

```bash
export PICO_SDK_LIB_DIR=/path/to/directory/containing/libPXREARobotSDK.so
```

## 9. 推荐接力顺序

### 阶段 A：纯软件回归

1. 新笔记本 clone 仓库和指定 URDF。
2. 构建/启动 C1 仿真容器。
3. 运行第 6 节全部测试，必须保持全绿。
4. 运行 mock preview，确认 `/pico/joint_states` 名称与维度。

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode preview --source mock
```

### 阶段 B：mock 控制仿真

先将仿真机器人置于弯肘 task reset 姿态，再执行：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh \
  --mode sim --source mock --enable-command
```

验收头部、双臂、双手、Deadman、急停、超时和退出行为。

### 阶段 C：真实 PICO，只预览

1. 启动 PICO PC/headless service。
2. 确认 wheel 和 `.so` 能被 Python 3.10 加载。
3. 运行 SDK preview，不加 `--enable-command`。

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh --mode preview --source sdk
```

确认：

- 头显、左右手柄 pose 均为有限值。
- A/B/X/Y、trigger、grip、摇杆和摇杆按下映射正确。
- 时间戳持续变化且频率稳定。
- `/pico/joint_states` 坐标方向和幅度正确。
- PICO 断连后立即解除使能。

### 阶段 D：真实 PICO 控制仿真

真实 PICO preview 通过后，再在仿真中加 `--enable-command`，先小范围、低速验证左右和上下方向。

### 阶段 E：C1 真机只读

完整执行 `C1_REAL_ROBOT_READONLY_CHECK.md`，至少保存：

- `/mc/sdk/robot_state` 实际类型、QoS 和一帧消息。
- 身体全部 `joint_states.name`。
- 左右手状态 topic、名字和维度。
- 身体和手部 command topic 的订阅节点与 QoS。

重点确认 `L/R_elbow_pitch_joint` 与 `L/R_elbow_roll_joint`。

### 阶段 F：真机命令

只有阶段 A～E 全部完成、代码按真机反馈修正、现场安全条件满足后，才能设计单关节小幅验收。不要直接跳到完整 PICO 真机命令：

```bash
ROS_DOMAIN_ID=0 ./run_pico_teleop.sh \
  --mode real --source sdk --enable-command --confirm-real-robot
```

上面仅保留为最终入口，不是当前可执行步骤。

## 10. 已知风险和待确认项

1. **肘关节名**：当前 URDF/代码是 `elbow_pitch_joint`，SDK 文档另一处为 `elbow_roll_joint`，以真机反馈为准。
2. **PICO 坐标基**：`tracking_to_robot` 仅经过数学测试，必须用真实设备逐轴校验。
3. **按钮语义**：当前右 B 是 Deadman、右 A 清急停、左摇杆按下急停；需和现场交互约定确认。
4. **SDK 二进制**：wheel 是 CPython 3.10 x86_64，动态库搜索路径和 PC service 尚未在新笔记本验证。
5. **真机准备姿态**：当前弯肘门槛来自仿真任务姿态，真机必须重新确认安全准备姿态和限位。
6. **消息与控制器**：手部 mode 已按 SDK 使用 5，仍需只读确认现场话题和控制器版本。
7. **网络时延**：真机控制只能在机器人附近局域网运行，不允许公网远程闭环。
8. **断连行为**：当前停止继续发布，不主动发送额外归零命令；需确认真机控制器断流时的保持/卸力行为。

## 11. 原 GMR 参考位置

开发机原参考代码不属于本仓库：

```text
/home/changzhang/VLA/xgmr_tmp/
```

重点参考：

```text
general_motion_retargeting/xrobot_utils.py
pico/pico_teleop/deploy_real/xrobot_teleop_to_robot_w_hand.py
pico/pico_teleop/deploy_real/joint_state_direct_publisher.py
pico/pico_teleop/docs/xrobot_teleop_to_robot_w_hand_flowchart.md
general_motion_retargeting/ik_configs/smplx_to_walker_c1.json
```

从原工程复用的思想包括 SDK 读取、Unity 坐标处理、按键状态、连接状态和 ROS 预览；没有复用其全身 GMR、底盘速度、腿部控制和外部 sim2real 进程管理。

## 12. 下一位接手者的最小目标

下一阶段最有价值的完成标准是：

```text
新 Ubuntu 笔记本
  -> PICO SDK preview 收到稳定数据
  -> /pico/joint_states 方向正确
  -> 真实 PICO 小范围控制 C1 仿真
  -> C1 真机只读关节名/QoS 完成
```

在这四项完成前，不进入完整真机 PICO 遥操作。
