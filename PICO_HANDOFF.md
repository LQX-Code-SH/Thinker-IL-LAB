# Walker C1 PICO 遥操作接力文档

更新日期：2026-08-14

本文档面向下一位继续开发或现场联调的同事，记录当前实现边界、验证结果、外部依赖、运行方法、已知风险和推荐接力顺序。

## 1. 当前结论

Walker C1 已有一版独立的 PICO 头部、双臂、双手遥操作实现。以本文末尾
“2026-08-14 最终按键映射”一节为当前状态；此前提到右 B Deadman 的内容均为历史记录。

当前处于：

```text
真实 PICO 数据链路已打通 + C1 仿真已响应 + 映射/IK 第二轮现场验收进行中
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

当前仍未完成：

- 已验证真实 PICO 设备、头显/双手柄位姿、全局时间戳、右臂多方向平移/旋转和抓取苹果；
  后续仍需用多条正式采集任务验收连续性与数据质量。
- 未连接 C1 真机读取实际关节名。
- 未向 C1 真机发送任何 PICO 运动指令。
- 未验证现场 C1 肘关节使用 `elbow_pitch_joint` 还是 `elbow_roll_joint`。

因此可以描述为“真实 PICO 到 C1 仿真的链路已打通”，但不得描述为“完整映射验收完成”
或“PICO 控制 C1 真机已经跑通”。

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
| `README.md` | 唯一的操作者文档：启动、遥操作、采集、安全边界和常见问题 |

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
- 所有模式只有按住右 A 才捕获锚点并持续发送；松开立即解除使能。PICO 中必须关闭
  `Switch w/ A Button`，再手动开启 `Send`。
- 左摇杆按下触发急停锁定。
- 松开左摇杆后按右摇杆清除锁定，不会自动恢复运动。
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
- 右 A Deadman、右 B 不使能机器人、弯肘姿态门槛、real/sim Domain 和 mock/real 防误用。

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

## 13. 2026-08-06 本机 C1 仿真与自动采集复验

本机已从 `ubt-sim-isaac_latest.tar` 载入镜像
`ubt-sim-isaac:latest`，并创建独立容器：

```text
walker-c1-ubt-sim
```

已有的 `isaac_sim_ubt`（TienKung/其他任务）未停止、未进入、未修改。C1 使用
`ROS_DOMAIN_ID=146` 和 ZMQ 端口 5655--5658；验证结束后 C1 仿真进程已退出并释放端口，
容器保留为空闲状态。

这个 tar 镜像比当前仓库 Dockerfile 少了运行时初始化内容。已仅在
`walker-c1-ubt-sim` 容器层补充：

- `python3-colcon-common-extensions`
- Isaac Python 的 `deepdiff`、`pyzmq`
- `/opt/ubt_sim/walker_sdk_ros2_msgs/install` 下 5 个 Walker ROS 2 消息包
- `/opt/pico-sdk` 下的 PICO wheel、`libPXREARobotSDK.so`，以及系统 Python 3.10 中的
  `xrobotoolkit_sdk==1.0.2`

删除并重建该容器后，上述容器层安装会丢失，需要重新执行 `init` 或重新补齐。

PICO 数学、双臂 IK 和 ROS 消息面回归测试结果：

```text
8 tests passed
```

C1 headless 仿真、ROS2-ZMQ bridge、31 个身体关节反馈、左右手反馈、头部 RGB
（高度 480）和 `/sim/object_state` 均已验证可用。随后按
`README_C1_LEROBOT.md` 执行 task reset，并完成 2 条随机苹果在线 IK 自动抓放采集：

```text
2/2 success, 2 trajectory file(s) saved

ubt_sim/dataset/walker_c1_ros/1786009042927/trajectory.hdf5  # 199 帧
ubt_sim/dataset/walker_c1_ros/1786009056164/trajectory.hdf5  # 142 帧
```

两条文件均为 26D state、26D action，所有 dataset 帧数一致，全部 JPEG 可解码，
`success=True`。第一条平均约 29.1 Hz；第二条平均约 20.8 Hz，存在 CPU 仿真/相机链路
吞吐抖动。正式 PICO 采集前应先决定把目标帧率降到稳定值，或解决 30 Hz 图像掉帧。

现有 `pico_teleop.py` 尚未接入人工 episode HDF5 录制；自动抓放录制器不能直接替代
PICO 遥操作录制。

## 14. 2026-08-06 PICO mock 闭环与真实设备预检

### 14.1 mock PICO 已闭环驱动 C1 仿真

在 C1 task reset 弯肘准备姿势下执行：

```bash
ROS_DOMAIN_ID=146 ./run_pico_teleop.sh \
  --mode sim --source mock --enable-command
```

节点成功打印：

```text
PICO teleop mode=sim, COMMAND; hold right B to move
teleop armed; controller and robot anchors captured
```

仿真反馈中头部和双臂最大变化约 `0.1384 rad`，证明链路
`mock pose -> 坐标变换 -> 双臂 IK -> ROS -> ZMQ -> C1 Isaac Sim` 已闭环，不只是
ROS topic 自发自收。mock 只替代 PICO 输入，后续 IK、ROS、bridge 和仿真控制均为真实实现。

mock 不能验证真实 PICO 的连接、坐标方向、按键映射、时间戳或断连。测试进程退出后控制器
保持最后一条安全目标，不会自动 reset；下次继续前必须重新执行 `reset.py`。

### 14.2 本机 PICO 软件位置

厂家依赖已经随 Thinker Studio 安装在宿主机：

```text
/opt/thinker-studio/pico_teleop/deps/
  xrobotoolkit_sdk-1.0.2-cp310-cp310-linux_x86_64.whl
  libPXREARobotSDK.so

/opt/thinker-studio/pico_headless_service/bin/RoboticsServiceProcess
```

Thinker Studio 后端由 `thinker-studio.service` 管理，API 监听 `0.0.0.0:9999`。2026-08-06
检查时后端为 `enabled + active`，不要为了 C1 测试停止它。厂家进程控制接口：

```text
POST http://127.0.0.1:9999/api/control/robotics_service_start
POST http://127.0.0.1:9999/api/control/robotics_service_stop
GET  http://127.0.0.1:9999/api/control/services_status
```

已验证 start 能拉起：

```text
/opt/thinker-studio/pico_headless_service/bin/RoboticsServiceProcess
```

当天结束前已通过 stop API 将该厂家进程关闭，不会后台等待设备。

### 14.3 PC Service IP

当前宿主机网络：

```text
eno1（有线、默认路由）       192.168.1.4/24
wlx40a5ef508ee1（无线）      192.168.1.121/24
```

PICO 与电脑在同一 `192.168.1.0/24` 网络时，PC Service 优先填写：

```text
192.168.1.4
```

如果现场明确通过无线网卡直连且 `.4` 不通，再尝试 `192.168.1.121`。不要填写
`127.0.0.1` 或 `172.17/18/19.x` Docker 地址。

### 14.4 2026-08-06 收工时的准确状态

- PICO 尚未开机、尚未开始串流。
- `RoboticsServiceProcess` 已停止。
- `thinker-studio.service` 保持运行，这是原有系统服务。
- `walker-c1-ubt-sim` 容器保留并运行 `tail -f /dev/null`，但 Isaac Sim、C1 bridge
  均已停止，GPU 和 5655--5658 端口已释放。
- 已有 `isaac_sim_ubt` 容器未停止、未进入、未修改。
- C1 容器内的 `/opt/pico-sdk`、PICO Python 包和补装依赖都在容器层；不要删除该容器。
- 本轮有意修改的代码/文档只有本文档；两条 HDF5 位于第 13 节路径且被 `.gitignore` 忽略。

### 14.5 明天最短接力顺序

1. 先读本节，不要重建或删除 `walker-c1-ubt-sim`。
2. 用户打开 PICO 和左右手柄，启动已经会使用的 PICO 串流应用；PC Service IP 先填
   `192.168.1.4`。
3. 启动厂家 PC service：

   ```bash
   curl -fsS -X POST \
     http://127.0.0.1:9999/api/control/robotics_service_start
   ```

4. 使用原有 `start_c1_pick_place_sim.sh` 启动 C1 GUI 仿真（不要传 `--headless`），等待日志
   出现 `RateLimiter sleep_duration`，再执行一次 task reset。
5. 第一轮只做真实 PICO preview，必须省略 `--enable-command`：

   ```bash
   sudo docker exec -it \
     -e ROS_DOMAIN_ID=146 \
     -e PICO_SDK_LIB_DIR=/opt/pico-sdk \
     walker-c1-ubt-sim bash -lc '
       cd /ubt_sim/teleoperation/control/walker_c1 &&
       ./run_pico_teleop.sh --mode preview --source sdk'
   ```

6. 保持头显和手柄唤醒。按住右 B 时节点才捕获锚点并发布 `/pico/joint_states`；preview
   不会发布 C1 command。逐轴验证头显、左右手柄，随后验证 A/B/X/Y、trigger、grip、摇杆、
   时间戳刷新和断连解除使能。
7. preview 全部通过后，才运行 `--mode sim --source sdk --enable-command` 做小范围仿真控制。
8. 真正采集 PICO 示教前，先实现 C1 专用人工 episode HDF5 录制器，并处理第 13 节记录的
   30 Hz 图像吞吐抖动；不要直接复用自动抓放控制器来冒充遥操作录制。

## 15. 2026-08-07 真实 PICO 接入与离场状态（以本节为准）

### 15.1 已确认打通的链路

真实 PICO 已完成以下现场验证：

```text
PICO 192.168.1.101
  -> PC 192.168.1.4:4000
  -> RoboticsServiceProcess
  -> 127.0.0.1:60061 gRPC
  -> xrobotoolkit_sdk (CPython 3.10)
  -> pico_teleop.py
  -> ROS_DOMAIN_ID=146
  -> Walker C1 Isaac Sim
```

- PC Service 识别到设备序列号 `PA9410MGL2260118G`，日志出现
  `new RTC device connected`。
- SDK 时间戳持续刷新，头显和左右手柄数据不再 stale。
- 右 B 的按下、保持和释放均已验证，日志能看到 `teleop armed` 和
  `right B released`。
- `--mode sim --source sdk --enable-command` 已经让 C1 仿真机器人产生响应。
- 当前 C1 路径只用 `Head` 和 `Controller`；`Hand` 可选，手指闭合由手柄 trigger 映射。
  腰部和腿部绑带不参与控制，`PICO Motion Tracker` 保持 `None`，不需要 Full Body 校准。

XRoboToolkit Python 模块名曾写错。wheel 内真实名字是：

```text
xrobotoolkit_sdk
```

即 `robot` 后只有一个 `t`；旧代码错误地导入了 `xrobottoolkit_sdk`。本次已修正。

### 15.2 SDK runtime 的可复现准备

不要把 cp310 wheel 安装到 Isaac Python 3.11。容器内使用 `/usr/bin/python3`（3.10），并从
wheel 直接解出原生扩展：

```bash
docker exec -it walker-c1-ubt-sim bash -lc '
  cd /ubt_sim/teleoperation/control/walker_c1 &&
  ./prepare_pico_runtime.sh /opt/pico-sdk /opt/pico-runtime'
```

成功标志：

```text
[INFO] PICO SDK import OK: /opt/pico-runtime/python/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so
```

`run_pico_teleop.sh` 现在自动优先查找 `/opt/pico-runtime`，同时设置
`LD_LIBRARY_PATH` 和 `PYTHONPATH`，不再需要每次手工传这两个变量。厂家 wheel 和动态库仍是
外部二进制，不提交仓库；若删除容器，需先重新把它们放入 `/opt/pico-sdk`。

### 15.3 PC Service 与 PICO 应用设置

Thinker Studio 后端和厂家 PC Service：

```bash
sudo /usr/bin/systemctl start thinker-studio.service
curl -fsS -X POST http://127.0.0.1:9999/api/control/robotics_service_start
curl -fsS http://127.0.0.1:9999/api/control/services_status
```

主机应监听 TCP/UDP 4000，SDK 使用本地 gRPC 60061。PICO 应用设置：

```text
PC Service: 192.168.1.4
Head:       On
Controller: On
Hand:       Optional
Motion Tracker: None
Data & Control: Send
```

若 PICO 显示 Connection Error，先确认 API 中 `robotics_service.running=true`。若 PC Service
日志出现 `rtc device already added`，依次停止 SDK preview、通过 stop/start API 重启
RoboticsServiceProcess，再在 PICO 点 `Reconnect` 和 `Send`。不要删除配置或数据。

### 15.4 GUI、场景与标准启动顺序

两处失效 USD 相对引用已改为仓库现有场景：

```text
assets/local_scenes/tiangong_parlor/scene_v2_c1.usda
  -> ../../scenes/parlor/scene_v2.usd

assets/robots/walker_c1/c1_task_apple.usda
  -> ../../scenes/parlor/scene_v2.usd</World/apple>
```

因此不再依赖缺失的 `assets/local_scenes/tiangong_parlor/scene_v2.usd`。GUI 是现有脚本的默认
行为，不需要新增入口；不要传 `--headless`：

```bash
xhost +SI:localuser:root
docker exec -it -e DISPLAY="${DISPLAY:-:0}" walker-c1-ubt-sim bash -lc '
  cd /ubt_sim && exec ./scripts/start_c1_pick_place_sim.sh'
```

等待环境完成初始化后，必须先 reset，再启动遥操作。测试使用隔离 Domain，避免正在运行的
仿真反馈覆盖测试夹具：

```bash
docker exec -it walker-c1-ubt-sim bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
  cd /ubt_sim/teleoperation/control/walker_c1
  ROS_DOMAIN_ID=145 /usr/bin/python3 -m unittest -v \
    test_pico_math.py test_dual_arm_ik.py test_pico_ros.py
  ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task'
```

真实 PICO 预览（不会驱动 Isaac 机器人）：

```bash
docker exec -it -e ROS_DOMAIN_ID=146 walker-c1-ubt-sim bash -lc '
  cd /ubt_sim/teleoperation/control/walker_c1
  exec ./run_pico_teleop.sh --mode preview --source sdk'
```

真实 PICO 控制仿真：

```bash
docker exec -it -e ROS_DOMAIN_ID=146 walker-c1-ubt-sim bash -lc '
  cd /ubt_sim/teleoperation/control/walker_c1
  exec ./run_pico_teleop.sh --mode sim --source sdk --enable-command'
```

按住右 B 时捕获当前手柄和机器人锚点；先移动几厘米，松开 B 立即停止发布。启动前用
`pgrep -af pico_teleop.py` 确认只有一个遥操作进程。`tcp://*:5656 Address already in use`
则表示有重复的 Isaac/C1 控制进程，不是 reset 问题。

### 15.5 IK 调整与尚待现场复验

第一轮真实控制能响应，但右臂移动困难。新增诊断确认并非网络或 B 键问题：旧 IK 会在
7-DoF 冗余解支之间跳变，候选关节解相对上一帧常跳 `0.8--2.7 rad`，被 `0.35 rad`
安全阈值拒绝；少数接受帧会逐渐把肘带直，随后触发直臂奇异位形保护。

本次没有粗暴放宽跳变阈值，而是：

- 用带上一帧关节正则的 `scipy.optimize.least_squares` 替换原始全姿态 IKPy 调用。
- 手腕朝向改为软约束，仍保留位置、旋转残差和解跳变三重验收。
- `translation_scale` 从 `0.8` 降到 `0.55`。
- IK 拒绝日志现在显示具体位置残差、旋转残差或关节跳变量。

数学和小位移 IK 测试已通过；用户离场前没有完成新版连续 IK 的第二轮真实手柄验收。因此
下次必须按 15.4 的顺序 `隔离测试 -> task reset -> sim teleop`，小范围验证左右臂连续性，
确认后才能把“真实 PICO 仿真遥操作”标记为最终完成。

### 15.6 Thinker Studio 版本说明

当前安装包为 `2.0.29` 且 dpkg 状态是 `half-configured`，界面里没有 C1；下载目录已有
`thinker-studio_2.0.69_amd64.deb`，新版包包含 C1 前端资源。本次未执行升级。对比确认
2.0.29 与 2.0.69 中的 `RoboticsServiceProcess`、`libPXREAGRPCServer.so` 哈希完全相同，
因此升级只为 Thinker Studio UI/C1 资源，不是解决 PICO 数据链路的必要条件。自定义
`pico_teleop.py` 控制 Isaac Sim 不依赖 Thinker Studio 中是否显示 C1。

### 15.7 2026-08-07 最终离场状态

- `/opt/pico-runtime` 已由新脚本重新生成，正确模块导入验证通过。
- 使用 `ROS_DOMAIN_ID=145` 隔离运行全部 8 项测试，结果 `OK`。
- 随后已向 Domain 146 发布 `reset.py --mode task`；但检查离场状态时容器内已没有
  `sim_runner.py` 和 5655--5658 监听端口，因此下次仍需重新启动 GUI 仿真并再次 reset。
- 容器内没有残留 `pico_teleop.py` 进程，右 B 当前不会向仿真发送命令。
- `thinker-studio.service` 正在运行；`RoboticsServiceProcess` 正在运行，PID `125493`。
  后端状态里的 `pico_connection=false` 是旧 Thinker 状态文件，不代表本次自定义 SDK 验证失败。
- 新连续 IK 只完成自动测试，尚未做第二轮真实 PICO 手柄验收。
- 本节所述代码和文档修改目前只保存在本地工作区，按用户离场要求没有 commit、没有 push。
  下次先审查 `git status`/diff；不要提交根目录未跟踪的 `ubt-sim-isaac_latest.tar` 和
  `xgmr_tmp.zip`，然后再提交推送代码与文档。

## 16. 2026-08-12 现场续测（以本节为当前状态）

### 16.1 设备、网络与启动方式

本控制链只需要 PICO 头显和左右手柄，不需要腰绑带或腿部 Motion Tracker：

```text
Head:           On
Controller:     On
Hand:           Optional
Motion Tracker: None
Data & Control: Send
PC Service:     192.168.1.4
```

本次沿用设备 `PA9410MGL2260118G`。电脑地址仍为有线 `192.168.1.4`、无线
`192.168.1.121`。PICO 曾显示 `10.10.50.58`，电脑只能经 `Meta` 隧道路由 ping 通它，
PICO 无法从该网段反连 `192.168.1.4:4000`，所以不能据“ping 成功”判断 PC Service 可用。
PICO 切到与电脑相同的 `Yzx17F-5G 1` 后地址变为 `192.168.1.100`，并建立到电脑的会话。

普通用户不能访问 Docker socket。逐条 `pkexec docker exec ...` 会反复弹管理员密码；推荐
只认证一次并复用持续容器 shell：

```bash
pkexec /usr/bin/docker exec -it \
  -e ROS_DOMAIN_ID=146 \
  -e PYTHONPATH=/opt/pico-runtime/python \
  -e LD_LIBRARY_PATH=/opt/pico-runtime \
  walker-c1-ubt-sim bash
```

不要在该 shell 中用 `exec ./run_pico_teleop.sh ...` 替换 shell，否则 Ctrl-C 停遥操作时持续
终端也会退出、需要重新认证。直接运行脚本即可。

### 16.2 真实数据和三轴平移结论

SDK 已读到持续刷新的头显、左右手柄 7D pose、全局时间戳和右 B。有效连续轨迹结果为：

```text
身体向右 -> SDK 主要 +X
身体向上 -> SDK 主要 +Y
推导前方 -> SDK 约 +Z
```

当前配置：

```json
"tracking_to_robot": [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
```

因此 SDK `+Z` -> 机器人前方 `+X`，SDK `+X` -> 机器人右方 `-Y`，SDK `+Y` ->
机器人上方 `+Z`。本次没有修改平移矩阵。真实闭环中已确认“右手向前约 5 cm，机器人右手
向前”，进一步证明前向平移正确。

早先静止端点测试曾把“前方”误测为 `+X`，后续确认当时手柄流冻结且头显有明显整体移动，
该样本无效，不能据此交换 X/Z。

### 16.3 手柄流保护的现场修正

现场出现过全局时间戳/头显仍刷新，但单路或双路手柄 pose 保持上一帧/归零的状态。旧实现
只检查 `get_time_stamp_ns()`，会用旧 pose 捕获锚点。

已新增 `ControllerPoseLiveness`：进程启动后，左右手柄必须分别至少变化一次，才允许按 B
捕获锚点；全零 pose 因零四元数被拒绝。第一版曾用“1 秒完全不变即 stale”，实测发现厂家
SDK 会对静止手柄重复完全相同 pose，导致只动右手时静止左手被误判、机器人不动。现已撤销
运行期 1 秒超时：只要求启动后证明过一次活跃，允许随后静止。

同时 `PicoSource.close()` 会调用厂家 SDK `close()`；正常 Ctrl-C 可看到：

```text
CANCELLED
client cancel server stream start
client cancel server stream end
uninitialize sdk
```

这能降低异常退出后的 `rtc device already added`/旧帧会话概率。修改后数学、IK、ROS 和
liveness 共 9 项测试通过。

### 16.4 仍待解决：手腕旋转和右臂 IK

真实闭环首次复验结果：

- 右手向前，机器人向前：正确。
- 用户描述右手柄向左水平转动时，机器人手掌向右转：旋转方向疑似反向。
- 右移时机器人表现不对；日志连续出现右臂 IK 解跳变：`0.351--0.857 rad > 0.350 rad`，
  随后肘部趋直并触发准备姿态保护。

判断：平移矩阵已经过真实验证，不应为此改动。更可能是 Unity/PICO 四元数到机器人右手系
的姿态转换有符号/乘法方向问题，错误的手腕目标又把 7-DoF IK 带到不连续分支。应先校准
手腕旋转，再决定是否调整 IK 正则/肘形策略，不能简单放宽解跳变阈值。

原 GMR `coordinate_transform_unity_data()` 对人体/手部使用：

```python
rotation_matrix = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
orientation = rotation_quat * source_quaternion
```

而当前 C1 `pose7_to_robot()` 对控制器使用 `basis @ R @ basis.T`。两者不是同一个姿态变换
表达式；需要用真实控制器的三种纯旋转逐项确定，不能直接照抄 GMR 人体关节公式。

本次尝试采样“右手柄前端水平向左约 30°”，但动作期间 SDK 日志出现 `device missing`，采样
只得到约 `1.3°` 旧帧变化，因此样本无效、没有据此修改旋转代码。PICO 随后电量耗尽，本次
暂停于此。

### 16.5 数据会话稳定性与恢复顺序

PICO 在 `192.168.1.100` 时 20 次 ping 无丢包，延迟约 `3--84 ms`，并可见到电脑 4000
端口的连接。但 SDK 仍可能报告 `device missing` 并把时间戳冻结在最后一帧。测试动作后若
摘下头显查看电脑/回复消息，佩戴传感器也可能使 PICO 应用休眠并停止 Send；后续标定必须
全程佩戴头显、应用保持前台，优先用语音回复。

可靠恢复顺序：

1. 正常关闭现有 SDK 客户端（调用 `sdk.close()`/Ctrl-C）。
2. 通过 API stop/start RoboticsServiceProcess。
3. PICO 保持佩戴，点 `Reconnect`，确认 Head/Controller On，再点 Send。
4. 新 SDK 会话必须看到 `device found PA9410MGL2260118G`。
5. 至少两次读取确认时间戳持续增加，再开始标定或仿真控制。

### 16.6 下次从这里继续

本节的旧步骤已由下面第 17 节覆盖。不要重新执行本节所写的“先做手腕旋转标定”；旋转
手性已在后续真实样本中确认并修复。下一次直接从第 17.5 节继续。

当前没有进入 C1 真机命令模式。人的肘部没有被追踪，即使手掌末端正确，机器人肘形也可能
不同；这是 7-DoF IK 冗余/肘形策略问题，不是三轴平移映射问题。

## 17. 2026-08-12 最终现场状态：旋转手性已修，待加入操作者基准坐标

本节是当前最新接力状态，覆盖第 16.4 和 16.6 节中的待办判断。

### 17.1 动作语义澄清

早先用户说“沿水平轴向左转”时，实际是绕手柄自身近似水平的轴旋转，助手曾误解成“手柄
射线在水平面内向左扫”。这导致一度错误怀疑四元数顺序或数据异常。后续所有标定都改用
无歧义描述：

- 头显 yaw：像看向左边一样转头，头保持竖直，旋转轴穿过头顶。
- 手柄 yaw：手柄位置不变，让手柄射线像激光笔一样从正前方扫向左边。
- 手柄 roll：绕手柄射线/长轴拧转，必须同时说明从哪一端观察正负方向。

用户约定：在逐步调试时，回复 `1` 表示已接受并完成当前指令、可以采样。需要用户描述视觉
结果时仍应明确要求回复“向左/向右/没动”等现象，不要把 `1` 当成方向反馈。

### 17.2 四元数顺序和旋转手性的真实标定

厂家 SDK 的人体/追踪器文档明确为 `[x,y,z,qx,qy,qz,qw]`；头显和控制器接口文档只写
7D pose。真实头显纯 yaw 消除了握持姿态歧义：

```text
动作：头保持竖直，向左看
测得角度：35.25 deg
PICO 世界轴：[0.0854, 0.9947, 0.0570]
```

旋转轴有 `99.47%` 落在 PICO `+Y`，确认头显同样使用 `qx,qy,qz,qw`，且数据稳定。

随后采样右手柄射线水平向左扫：

```text
测得角度：30.42 deg
PICO 世界旋转向量：[0.00176, 0.53099, -0.00106]  # 约 +Y
旧代码机器人旋转向量：[0.00106, 0.00176, -0.53099]  # 约 -Z，方向反了
```

这与用户最初看到的“向左扫，机器人向右转”完全一致。根因是 PICO/Unity 左手系四元数被
直接交给了右手系旋转矩阵公式。位置向量无需取逆，姿态矩阵必须先转置再换基。已修改
`pico_math.py::pose7_to_robot()`：

```python
position_robot = basis @ position_unity
rotation_robot = basis @ rotation_unity.T @ basis.T
```

已新增回归测试 `test_unity_left_yaw_becomes_robot_left_yaw`，要求 Unity 绕 `+Y` 的左转
变成机器人绕 `+Z` 的左转。`py_compile` 通过，数学、IK、ROS 共 **10 项测试全部通过**。

真实 PICO → C1 仿真复验中，用户确认“手柄射线向左扫”的旋转方向已经正确；日志正常捕获
锚点、正常在松开 B 后 disarm，未出现 IK 拒绝或解跳变。因此不要撤销上述 `.T` 修复。

### 17.3 最新发现：位移必须相对操作者朝向，而不是房间全局轴

修复旋转手性后，用户继续做平行移动并观察到：

```text
手柄向用户右侧平移 -> 机器人手向前
手柄向身体内侧平移 -> 机器人手向左
```

用户还反馈：绕“水平向外”的轴向左转时，机器人绕同一类轴但方向相反；手柄射线水平向左
扫则已正确。

当前 `pose7_to_robot()` 把 SDK 的房间/追踪全局坐标直接变成机器人基座坐标；按 B 捕获锚点
时没有消除操作者相对房间原点的初始 yaw。操作者换了站立朝向以后，身体坐标中的“右”就
可能是房间坐标中的“前”，因此出现上述轴串换。这不是已经确认的 `tracking_to_robot` 三轴
矩阵本身错误，也不是四元数 `.T` 修复错误。

下一项代码工作是：按下 B 捕获锚点时，从头显姿态提取操作者初始 yaw，并把控制器的相对
平移和相对旋转都表达在该 yaw 对齐坐标系中：

```text
H = 只保留锚点头显 yaw 的 Rz(theta)
delta_position_operator = H.T @ delta_position_world
delta_rotation_operator = H.T @ delta_rotation_world @ H
```

然后用 `delta_position_operator` 和 `delta_rotation_operator` 驱动机器人锚点。应避免把头显
pitch/roll 混进平移基准，否则用户低头会让水平移动带出竖直分量。头部关节自己的相对 yaw/
pitch 也要做回归测试，但不要把头显绝对朝向直接设置成机器人头部绝对朝向。

该“操作者头显基准坐标归一化”截至本次结束 **尚未实现**，明天从这里开始。不要为了快速
通过测试而交换平移 X/Z，也不要撤销姿态转置。

### 17.4 本次结束时的安全/进程状态

- `pico_teleop.py --mode sim --enable-command` 已用 Ctrl-C 正常停止，厂家 SDK 已 `close()`；
  当前没有 PICO 控制命令进程。
- 只进入过 C1 仿真命令模式，**从未进入 C1 真机命令模式**。
- C1 仿真在结束前仍运行，但隔夜后必须重新检查，不能假定 GUI/ROS 反馈仍活着。
- PICO 最后可用地址为 `192.168.1.100`；PC Service 填 `192.168.1.4`；设备序列号
  `PA9410MGL2260118G`。
- 源码工作区本来就有其他未提交/未跟踪内容；继续保留用户改动，不要 reset、clean 或提交
  大文件。本次相关修改也尚未提交。

### 17.5 明天的准确继续顺序

1. 先实现“锚点头显 yaw 对齐”的纯数学 helper；补测试至少覆盖：操作者面向机器人、相对
   房间左转 90° 后向身体右侧平移、同条件下手柄 yaw/roll。测试中明确区分世界坐标与
   操作者坐标。
2. 将 helper 接入 `relative_target()`/锚点结构；保留 `pose7_to_robot()` 中姿态 `.T`。
3. 在 Domain 145 跑 `py_compile` 和全部测试；当前基线是 10 项。
4. 检查仿真和 `/mc/sdk/robot_state`；若隔夜失效则重启仿真。Domain 146 执行
   `reset.py --mode task`。
5. PICO 与 PC 同一局域网，保持佩戴、XRoboToolkit 前台；Reconnect，Head/Controller On，
   Motion Tracker None，Send。新 SDK 会话确认时间戳持续增加且左右手柄各至少变化一次。
6. 真实闭环只测小动作：身体右/前/上各 5 cm，然后手柄射线 yaw 左/右各 15°，最后 roll。
   每项单独按 B 捕获新锚点并松开；若异常立刻松 B。
7. 只有坐标归一化通过后，再处理可能残留的 7-DoF 肘形/IK 连续性问题；不要直接放宽
   `max_ik_solution_jump_rad`。

### 17.6 明天恢复本次 Codex 会话

本机已用 `codex-cli 0.146.1` 的实际 `--help` 确认恢复命令。最方便的方式：

```bash
cd /home/zhangchang/TienKung-IL-LAB
codex resume --last
```

`--last` 会在当前工作目录关联的记录中直接继续最近会话。若之后又在本仓库开启了其他会话，
用本次明确的 session ID 最稳妥：

```bash
cd /home/zhangchang/TienKung-IL-LAB
codex resume 019ff526-5e30-74e0-8b3e-f513f879d385
```

也可附带开场提示，避免恢复后从旧的第 16 节开始：

```bash
codex resume 019ff526-5e30-74e0-8b3e-f513f879d385 \
  "先读 PICO_HANDOFF.md 第17节，从操作者头显基准坐标归一化继续；不要进入真机模式"
```

若 session ID 找不到，运行 `codex resume --all` 打开包含其他工作目录的会话选择器。普通
`codex resume` 会打开当前目录的会话选择器。

## 18. 2026-08-13 实机逐轴标定：确认 OpenXR 坐标并修复底层换基

本节是当前最新结论，覆盖第 16.2、17.2、17.3 和 17.5 节中关于 `+Z` 前方、Unity 左手系、
必须保留姿态 `.T` 以及“头显朝向归一化尚未实现”的旧判断。第 17 节保留为排查历史，继续
工作时以本节为准。

### 18.1 佩戴状态与采样有效性

现场明确复现：摘下头显后，即使全局时间戳和头显 pose 仍持续更新，左右控制器 pose 也可能
冻结为最后一帧。用户固定头显位置并不能避免该问题，原因是佩戴传感器/应用生命周期，而
不是头部物理移动。标定期间必须保持佩戴、XRoboToolkit 在前台；冻结样本不能用于推断坐标。

本次保持佩戴后，前、右、上三段分别得到 90、90、65 个持续变化的右手柄样本。有效实测：

```text
身体正前方推出：旧机器人坐标 [-0.1858, +0.0322, +0.0149] m
身体右侧平移：  旧机器人坐标 [+0.0175, -0.1675, +0.0009] m
竖直向上抬起：  旧机器人坐标 [+0.0069, -0.0063, +0.1177] m
```

前、右两段头显 yaw 基准只差 `0.2 deg`，因此前向反号不是用户站立方向变化造成。换回 SDK
原始轴后，三组动作分别主要为 `-Z`、`+X`、`+Y`。这与 OpenXR 右手坐标完全一致：

```text
SDK -Z = 身体前方
SDK +X = 身体右方
SDK +Y = 身体上方
```

### 18.2 根因和代码修复

旧配置把 SDK 当成 Unity 左手系并假设 `+Z` 向前：

```json
[[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
```

该矩阵行列式为 `-1`，随后又对四元数旋转矩阵取转置。这个“反射 + 逆旋转”会偶然让 yaw、
pitch 的部分动作看起来正确，但造成前移反向，并留下 roll 符号问题。

现已按 PICO 实测的 OpenXR 右手系改为 proper rotation：

```json
"tracking_to_robot": [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
```

`pose7_to_robot()` 现在对位置和姿态使用同一个普通换基：

```python
position_robot = basis @ position_openxr
rotation_robot = basis @ rotation_openxr @ basis.T
```

并强制 `det(basis) == +1`，防止旧的反射矩阵再次混入。按新换基重新解释本次实测，身体前、
右、上分别成为机器人 `+X`、`-Y`、`+Z`，三轴互不串换。

### 18.3 操作者头显基准与测试状态

第 17.3 节提出的“按锚点头显 yaw 归一化”现已实现：`horizontal_heading()` 提取水平朝向，
`relative_target()` 和 `yaw_pitch_delta()` 都在该坐标中计算相对平移/旋转；不要重复实现。

新增/更新回归测试覆盖：OpenXR 前右上三轴、proper basis、OpenXR 左转 yaw、不同站立 yaw
下的身体右移、roll 轴符号和头部 pitch。结果：

```text
py_compile: OK
test_pico_math.py + test_dual_arm_ik.py + test_pico_ros.py: 16 tests, OK
```

当前只完成“PICO 原始数据 -> 数学映射”的实机校准，尚未完成新版映射下的 C1 仿真闭环
验收，不能对外称全部修好。下一轮只需短时佩戴，按 B 分项做小动作：前/右/上各约 5 cm，
yaw/pitch/roll 各约 10--15 deg；同时读取 PICO 映射目标、机器人末端实际 pose 和 IK 日志。
若映射目标正确但机器人视觉动作异常，应转入右臂 IK 连续性/肘形排查，不再改坐标矩阵。

### 18.4 本次结束时状态

- 只读 PICO SDK 已正常 `close()`；没有 PICO 遥操作命令进程。
- C1 GUI 仿真仍在原持久终端中运行，机器人此前已 task reset；继续前仍需检查 ROS 状态。
- `RoboticsServiceProcess` 本次通过 API 恢复，PID 当时为 `64457`；隔日不可假定 PID 不变。
- PC Service IP 仍为 `192.168.1.4`；设备序列号仍为 `PA9410MGL2260118G`。
- 从未进入真机模式；后续继续只允许 `--mode sim`。
- 原始连续采样暂存在容器 `/tmp/pico_calibration_20260813.npz`；关键数值已写入本节。

### 18.5 抓取阶段停住不是遥操作时限

抓到苹果后机器人停止的日志根因是 PICO 数据断流：先反复出现
`teleop disarmed: PICO timestamp stopped`，随后 SDK 明确报告
`device missing PA9410MGL2260118G`，最长连续 stale 超过 100 秒。代码没有按 B 或遥操作的
总时长限制；只要数据持续更新，B 可以一直按住。

短时 Wi-Fi/RTC 抖动曾频繁达到约 `0.27--1.7 s`。先把原 `0.25 s` 放宽到 `2.0 s`，但用户
明确要求仿真调试不使用时间戳超时保护，因此最终配置为 `source_stale_timeout_s=0.0`；非正值
表示完全禁用该超时。掉线时遥操作保持最后一个目标和锚点，不会因时间自动 disarm；数据恢复
后继续使用同一锚点。机械姿态、IK、无效初始时间戳和 B 键松开保护仍保留。真正
`device missing` 时没有新动作数据，机器人只能保持姿态；必须恢复 PICO 的
`Reconnect -> Send` 才能继续跟随。

### 18.6 IK 连续性改进（稳定基线提交后的工作区修改）

现场卡顿的主要日志证据不是仿真频率：`/mc/sdk/robot_state` 稳定约 `51 Hz`，而右臂 IK
连续因 `solution jump 0.46--1.25 rad > 0.35 rad` 拒绝，随后肘部趋直触发姿态保护。旧逻辑
任一臂失败会直接 `return`，连头部、另一臂和手指命令也一起冻结，形成明显的停—跳。

稳定坐标版本已先提交并推送为 `2da8e6b`。在该基线上继续做两项尚待实机复验的修改：

- IK 从当前已命令 seed pose 向最终控制器目标做有界笛卡尔小步，位置每周期最多 `0.012 m`，
  旋转最多 `0.08 rad`，避免远目标直接切换到另一个 7-DoF 解分支。
- 左右臂独立容错；一臂无解或过直时只保持该臂，另一臂、头部、手指仍继续发布。

操作者主动静止不属于掉线：控制器位姿允许无限期不变，时间戳超时也已禁用。静止时机器人
应保持现有目标；以上改动只改善移动过程中的 IK 连续性。

### 18.7 右侧单臂遥操作与 Space 分段采集

用户最终要求机器人头部、左臂和左手不参与遥操作，并且必须保持 `TASK_RESET_BODY_POSE`
中的标准 reset 数值，不能捕获按 B 瞬间可能有抖动的实测值。当前配置为：

```json
"teleop_sides": ["right"],
"head_tracking_enabled": false
```

头部固定为 `[head_yaw=0, head_pitch=0.50]`，左臂固定为 reset 七关节值，左手固定张开；
只有右臂和右手由 PICO 控制。右臂仍在按 B 时捕获当前位置作为相对遥操作锚点。

新增 `pico_episode_recorder.py` 及仿真 Space 事件链路。带 `--record` 启动时，第一次按 B 自动
开始录制；完成任务后在 Isaac Sim 主窗口按 Space，按“停控制 → 写 HDF5 → 场景 reset →
task reset → 等待松开 B”的顺序处理。默认文件位于
`/ubt_sim/dataset/walker_c1_pico/<timestamp>/trajectory.hdf5`，格式兼容
`Walker_C1_26_1RGB.json`。Space 支持修改了 Isaac Sim 进程内代码，必须重启仿真才能生效。
R 键仍是纯场景 reset，不保存。

本轮未启动 PICO/PC Service，未运行会向仿真域发布命令的 ROS 集成测试。语法/配置检查通过，
纯数学、IK 和离线 HDF5 格式测试共 16 项通过。运行中的旧仿真尚未加载 Space 新代码。

### 18.8 现场采集验证与 Space 可靠性修复

2026-08-13 现场验证右臂/右手遥操作正常，并成功写出 8 个 HDF5。逐文件检查确认所有数据集
帧数对齐、数值有限、时间戳严格递增、JPEG 全部可解码且右臂 action 有实际变化。发现第 3 个
文件与第 2 个文件 SHA256 完全相同：保存后旧缓存没有立即清空，reset 期间再次按 Space 会把
上一条重复写出。修复后完成事件只在 recorder 正在录制时受理，取快照时原子清空缓存，同一条
数据不可能被消费两次；历史重复文件保留，未自动删除。

仿真日志还显示保存端曾把同一 reset 消息连续发布 3 次，导致 Isaac 环境连续 reset，延长了
等待时间，操作者因此感觉要按两次 Space。ROS 链路使用可靠 QoS，现已改为只发布一次 reset。
一次 Space 已足够；需要等待 `ready for next episode` 后再开始下一条。

现场 8 条的实际相机同步采样率约为 5.6--5.8 Hz，而非请求上限 30 Hz。新文件把时间戳测得的
速率写为 `record_hz`，并将请求上限另存为 `requested_record_hz`。转换训练数据时应使用实际
速率，不能简单标 30 Hz，否则动作节奏会被压快。当前离线与 ROS 测试共 20 项通过。

### 18.9 PICO Remote Vision 可选模式（视频已现场连通）

> 本节记录 Remote Vision 的实现过程。最初设计保留右 B Deadman，但现场确认 B 同时切换
> 平面/双目画面；最终按键以 18.10 为准：所有模式统一使用右 A Deadman。

#### 18.9.1 需求与最终方案

本轮需求是增加一种显式可选的采集方式：操作者不用看电脑屏幕，而是在 PICO 内直接看 Isaac
头部相机画面；任务完成后用手柄结束、保存并 reset。视频实现保持可选；现场验收后按用户要求
把所有模式的 Deadman 统一从 B 改为 A，扳机和 Space 行为不变。

最终采用两条相互独立的网络链路：

```text
位姿/按钮：PICO -> PC:4000 -> RoboticsServiceProcess -> gRPC:60061
           -> xrobottoolkit_sdk -> pico_teleop.py

仿真画面：PICO -> PC:13579 发送 OPEN_CAMERA
           <- PC 回连 PICO 指定的视频端口，发送 H.264
```

因此视频不经过 Thinker Studio PC Service。PC Service 仍只负责原来的头显/手柄位姿；即使
Remote Vision 失败，默认旧模式也没有新的运行依赖。

#### 18.9.2 调研过程与没有采用的路径

1. 先检查本机 `xrobottoolkit_sdk` 和 `pico_source.py`。SDK 能读取右 `Trigger`、右 `Grip`、
   A、B 和摇杆按下；右 Trigger 已控制机器人手指，Grip 原先只读取、没有绑定功能。手柄上带
   方框图标的 Capture 键没有暴露在当前 Python SDK，不能可靠用作 reset。
2. 随后检查 PC Service 动态库中的视频相关符号。虽然生成接口中能看到类似
   `StartPlayVideo` 的名字，但本机 headless server 没有对应的有效实现，不能从当前 Python
   SDK 直接把 ROS 图像塞进 PC Service；该路径被放弃。
3. 最后核对 XRoboToolkit 官方源码，确认 PICO Unity Client 本身已有 `Remote Vision`，官方
   用独立的 `XRoboToolkit-Orin-Video-Sender` 把 ZED H.264 发送到头显。因此在 C1 项目里实现
   同一协议，用 Isaac ROS Image 替代 ZED 图像，是与现有 PICO APK 最兼容的路径。

官方依据：

- [XRoboToolkit Unity Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)：
  `Remote Vision -> ZEDMINI -> Listen` 的使用方法。
- [XRoboToolkit Orin Video Sender](https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender)：
  13579 控制端口、`OPEN_CAMERA` 和 H.264 TCP 发送实现。

#### 18.9.3 已复现的官方协议

PICO 先连接电脑 TCP 13579。每条控制消息的外层是 4 字节大端 body 长度，body 内层为：

```text
int32 little-endian command_length
UTF-8 command                         # OPEN_CAMERA / CLOSE_CAMERA
int32 little-endian data_length
bytes data
```

`OPEN_CAMERA` 的 data 是版本 1 `CameraRequestData`：

```text
CA FE                                 # magic
01                                    # version
7 * int32 little-endian:
  width, height, fps, bitrate,
  enableMvHevc, renderMode, videoPort
uint8 cameraNameLength + cameraName
uint8 picoIpLength + picoIp
```

收到请求后，电脑回连 PICO 请求中的视频端口。每个 H.264 access unit 按
`4 字节大端 payload 长度 + payload` 发送。当前实现只接受 H.264；若 PICO 请求 HEVC，会明确
报错而不是错误地发送另一种码流。请求的宽高必须是偶数，端口、分辨率、帧率和码率也会校验/
限幅。视频目标使用控制连接的真实 peer IP，避免 PICO 上报了错误网卡地址时连错目标。

#### 18.9.4 代码实现

新增文件：

| 文件 | 作用 |
|---|---|
| `pico_headset_view.py` | Remote Vision 控制服务、ROS Image 转换、FFmpeg 编码和视频回连 |
| `test_pico_headset_view.py` | 协议、图像、H.264 分帧和真实编码离线测试 |

`pico_headset_view.py` 的处理流程：

1. 只有启用 `--headset-mode` 才检查 `ffmpeg`/`libx264`、绑定 `0.0.0.0:13579` 并订阅
   `/sensor/camera/head/color/raw`。
2. ROS 回调只替换“最新一帧”，不在遥操作主循环里编码；慢编码时直接丢弃旧画面，避免视频
   处理阻塞 PICO 位姿读取和 ROS 控制。
3. 支持 `rgb8`、`bgr8`、`rgba8`、`bgra8`、`mono8`，并正确处理 ROS Image 每行的 `step`
   padding。图像按 PICO 请求尺寸等比例缩放并用黑边补齐，不拉伸。
4. FFmpeg 使用 `libx264 + ultrafast + zerolatency + baseline + yuv420p + 无 B 帧`，插入 AUD、
   SPS/PPS，并定期产生关键帧。
5. `AnnexBAccessUnitParser` 处理跨 pipe read 边界的 3/4 字节 Annex-B start code，以 AUD 把
   连续 H.264 字节流重新组成 PICO 期望的 access unit，再加大端长度发送。
6. `CLOSE_CAMERA`、控制连接断开或进程退出时会停止编码、关闭视频 socket 和后台线程；再次
   `Listen` 可以建立新会话。新版客户端可能先发 `AUDIO_SESSION`，本实现明确忽略音频协商，
   只提供视频。

`pico_teleop.py` 新增参数：

```text
--headset-mode              开启头显画面和 Grip 完成动作；自动启用 --record
--remote-vision-port 13579  Remote Vision 控制端口
--grip-reset-hold-s 1.0     Grip 有效长按时间
```

`--headset-mode` 只允许 `--mode sim --enable-command`，防止把仿真 reset 逻辑误带入 preview
或真机模式。标准启动命令为：

```bash
./run_pico_teleop.sh --mode sim --source sdk --enable-command --headset-mode
```

#### 18.9.5 Grip 保存/reset 与防误触

`pico_episode_recorder.py` 原先只有 Space ROS callback 能结束 episode。现在公共入口为
`request_complete(trigger)`，Space 和 Grip 都调用它，之后共用完全相同的原子流程：

```text
只消费一次当前 buffer
-> 停止遥操作并阻止新 episode
-> 写 trajectory.hdf5
-> 发布一次 /sim/cmd_reset
-> reset.py --mode task
-> 发布 /sim/episode_ready
```

HDF5 新增 `completion_trigger` attribute，用于区分 `Space key` 和
`right Grip long press`，不改变原有数据集路径与 LeRobot 转换兼容性。

Grip 使用模拟量迟滞：`>=0.80` 才算按下，`<=0.55` 才算完全释放。有效动作必须是：

```text
先松开 A 和 Grip -> 再单独按住 Grip 1 秒 -> 保存/reset -> 松开 Grip
```

按住 A 遥操作时的 Grip 不累计；如果松开 A 时 Grip 已经按住，该次 Grip 会锁定为无效，必须
先释放再重新按。这样正常抓握、抓取过程和临时松 A 暂停不会自动结束 episode。Grip 完成入口
对所有带 `--record` 的模式生效，不仅限于 `--headset-mode`。

按钮兼容性：

| 输入 | 普通模式 | `--headset-mode` |
|---|---|---|
| 右 A | 按住遥操作，松开保持 | 相同；必须关闭 PICO 的 `Switch w/ A Button` |
| 右 B | 不控制机器人 | 不控制机器人；仅由 Remote Vision UI 切换显示方式 |
| 右 Trigger | 右手开合 | 相同 |
| 左摇杆按下 | 锁定急停 | 相同 |
| 右摇杆按下 | 清除急停 | 相同 |
| Space | 保存并 reset（带 `--record`） | 仍可作为备用 |
| 右 Grip | 带 `--record` 时，释放 A/Grip 后长按 1 秒保存并 reset | 相同 |

#### 18.9.6 视频依赖边界

- 不加 `--headset-mode` 时不会构造 `PicoHeadsetView`，因此不绑定 13579、不订阅第二路相机、
  不启动线程、不执行 FFmpeg 预检。
- `--record` + Space 继续可用，同时增加右 Grip 长按的同一保存/reset 入口。
- 不带 `--record` 时 Grip 不触发完成动作。
- 右 A Deadman 是用户确认后的统一按键变更，与是否启用视频无关。

#### 18.9.7 已做测试与尚未完成的验收

本机已通过 4 项新增离线测试：

1. 构造并解析真实格式的 `OPEN_CAMERA`/`CameraRequestData`。
2. 在任意 chunk 边界（包括拆开的 start code）恢复 H.264 access unit。
3. 验证带 row padding 的 BGR ROS 图像能正确转为 RGB。
4. 实际调用本机 FFmpeg/libx264 编码 3 帧，并确认得到至少 3 个 AUD 分隔的 H.264 access unit。

新增测试与 `test_pico_math.py` 合计 17 项通过，`git diff --check` 通过。宿主 Python 缺少
`h5py`/`ikpy`，因此本轮没有在宿主重复运行依赖它们的 IK/HDF5 测试；此前容器内相关回归为
20 项通过。本轮没有向仿真或真机发布控制命令。

以下项目仍必须现场完成，不能提前宣称头显画面最终验收：

- 当前运行容器内是否存在 `ffmpeg` 且包含 `libx264`；宿主机已确认存在，但宿主不能代替容器。
- 真实 PICO 是否能连接电脑 13579 并成功接受电脑回连的视频 socket。
- 当前安装 APK 的 `ZEDMINI` profile 请求的实际宽高、FPS、码率、H.264/HEVC 和显示布局。
- 图像方向、宽高比、端到端延迟；旧版 APK 可能把 ZED 画面按 side-by-side 解释，需要根据实测
  决定是否在发送端复制左右眼画面。
- Remote Vision 打开后 B 会切换平面/双目画面，现场已确认影响操作体验；项目已不再用 B
  控制机器人，改用右 A，避免画面切换污染同一条 episode。
- Grip 模拟量阈值、长按一次只保存一条、场景与 task reset、HDF5 `completion_trigger`。

#### 18.9.8 下次从这里继续的最短步骤

1. 先按日常 README 启动 PC Service、仿真并执行 `reset.py --mode task`。
2. 在现有容器 shell 中只检查一次编码器，避免反复 `docker exec`/密码：

   ```bash
   command -v ffmpeg
   ffmpeg -hide_banner -encoders 2>/dev/null | grep libx264
   ```

3. 若存在编码器，直接启动：

   ```bash
   ./run_pico_teleop.sh --mode sim --source sdk --enable-command --headset-mode
   ```

   预期先看到：

   ```text
   PICO Remote Vision ready on 0.0.0.0:13579
   PICO teleop mode=sim, COMMAND
   ```

4. PICO 原有 Tracking 继续 `Reconnect -> Head On -> Controller On -> Send`。随后在
   `Remote Vision` 选择 `ZEDMINI -> Listen`，填写电脑 IP `192.168.1.4`，点 `Confirm`。
5. 预期电脑日志依次出现 `control connected`、`opening PICO video ...`、
   `video streaming started`。确认 PICO 的 `Switch w/ A Button` 未勾选，手动开启 `Send`，
   再按住 A 做小范围遥操作。
6. 做一条短任务，松开 A 和 Grip，再单独长按 Grip 1 秒；确认只生成一个 HDF5、场景和机器人
   均 reset，并检查 `completion_trigger=right Grip long press`。

若新模式启动时直接报告找不到 FFmpeg/libx264，只需在容器内补编码器；不要修改 PC Service、
PICO SDK 或旧遥操作代码。若能出现 `control connected` 但没有 `opening PICO video`，优先保存
收到的请求字节并核对 APK profile；若已经 `video streaming started` 但头显黑屏，则排查 H.264
布局/decoder，而不是回头改坐标映射或 IK。

#### 18.9.9 文档入口

- 日常启动、PC Service、仿真、reset、旧/新遥操作命令：
  `ubt_sim/teleoperation/control/walker_c1/README.md`
- 历史排查、实测结论和跨会话继续点：本文件 `PICO_HANDOFF.md`

### 18.10 2026-08-14 最终按键映射

头显画面现场连通后发现：Remote Vision 本地把右 B 绑定为平面/双目切换。第一次按 B 已经开始
机器人动作，但画面同时变成不需要的布局；第二次按 B 才切回。录制器不会因此生成两条文件，
但两次 B 之间的错误观察/动作会进入同一 episode，因此不能继续把 B 当 Deadman。

曾考虑修改 Unity 客户端、固定单目画面并重新打 APK；该方案需要 Unity/Android 构建、签名和
重新安装，当前没有必要。XRoboToolkit 已提供 `Switch w/ A Button` 选项：未勾选时，A 不切换
`Send`。现场确认该选项一直未勾选，因此采用下面的最终方案：

```text
PICO: Switch w/ A Button = Off，手动点一次 Send

按住右 A       捕获锚点并控制机器人
松开右 A       停止发布新动作并解除锚点
右 B           不参与机器人控制，仅保留 Remote Vision 本地功能
右 Trigger     控制右手开合
左摇杆按下     锁定急停
右摇杆按下     解除急停（仍需重新按 A 才恢复控制）
松开 A/Grip 后长按右 Grip 1 秒
                保存当前 episode，并执行场景 reset + task reset
Isaac Space     与 Grip 共用相同保存/reset 流程，作为电脑端入口
```

这一按键映射对 preview/sim/real 和是否启用 `--headset-mode` 都一致。Grip 保存/reset 仅在启用
录制器（`--record`，或隐含它的 `--headset-mode`）时生效；未启用录制时不会抢占 Grip。

## 19. 2026-08-18 PICO 裸手 tracking 与 C1 11DoF 灵巧手方案

### 19.1 目标和不可破坏边界

第一阶段只跑通 PICO 原生裸手 skeleton，不直接控制机械臂或灵巧手：

```text
PICO PXR/OpenXR hand tracking
  -> 现有 XRoboToolkit Unity Client 状态 JSON
  -> 现有 PC Service / libPXREARobotSDK.so
  -> xrobotoolkit_sdk Python receiver
  -> 26-joint HandTrackingData + 限频日志/调试 marker
```

必须保留现有 controller teleoperation。`pico_teleop.py` 的右 A Deadman、controller 锚点、
`tracking_to_robot`、相对位姿、IK、安全门槛和 ROS 命令发布不重写；hand tracking 默认关闭，
只通过新调试入口显式开启。第一阶段不实现 optimizer，也不把裸手数据发布成机器人命令。

### 19.2 已确认的现有 PICO 输入链路

当前头显应用是 XRoboToolkit Unity Client。其上游源码使用 Unity 2022.3.16f1 和内嵌
PICO Unity Integration SDK 3.1.2：

- controller：`PXR_Input.GetControllerPredictPosition/Rotation()`；
- hand：`PXR_HandTracking.GetJointLocations(HandType, ref HandJointLocations)`；
- UI 的 `Head`、`Controller`、`Hand` 开关彼此独立，选中的数据一起放入同一状态 JSON；
- PICO 与 PC 继续使用现有 XRoboToolkit 网络连接，PC Service 再通过本机 gRPC
  `127.0.0.1:60061` 向 `libPXREARobotSDK.so`/Python binding 提供数据。

因此 MVP 不需要另建 UDP/TCP 协议，也不需要 MediaPipe、额外相机、数据手套或把 controller
绑在手腕上。头显端应启用系统 Hand Tracking，在 XRoboToolkit 中打开 `Hand` 和 `Send`。

本项目现有 `xrobotoolkit_sdk 1.0.2` 已实测导出：

```text
get_left_hand_tracking_state()   -> 26 x 7
get_right_hand_tracking_state()  -> 26 x 7
get_left_hand_is_active()        -> int, 0 low / 1 high
get_right_hand_is_active()       -> int, 0 low / 1 high
```

26×7 每项为 `x,y,z,qx,qy,qz,qw`，位置单位 meter，quaternion 顺序明确为 `xyzw`。

### 19.3 OpenXR 26-joint 顺序

必须按 PICO 官方/OpenXR 顺序保留全部数据：

| index | joint |
|---:|---|
| 0 | palm |
| 1 | wrist |
| 2–5 | thumb metacarpal / proximal / distal / tip |
| 6–10 | index metacarpal / proximal / intermediate / distal / tip |
| 11–15 | middle metacarpal / proximal / intermediate / distal / tip |
| 16–20 | ring metacarpal / proximal / intermediate / distal / tip |
| 21–25 | little metacarpal / proximal / intermediate / distal / tip |

XRoboToolkit 上游 `test_hand_isactive.py` 曾把 index 0 的 palm 注释成 wrist，该注释不符合
OpenXR 标准，项目实现必须使用 wrist index 1。

### 19.4 坐标系策略

现有 controller 现场标定确认 SDK pose 使用 OpenXR 右手系：

```text
+X 向右，+Y 向上，-Z 向前，quaternion xyzw
```

XRoboToolkit Unity 的 `TrackingData.GetHandJsonData()` 直接发送 `Posef.Position/Orientation`；
controller 预测 pose 也直接发送 PICO native pose 字段，没有在两者之间额外换轴。因此 hand MVP
对全部 26 个 joint 复用 controller 已验收的 proper basis：

```json
[[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
```

输出轴为机器人 `+X 前、+Y 左、+Z 上`。position 使用 `C p`，orientation 使用
`C R C^T`，再明确输出 `xyzw`；禁止靠逐轴猜符号处理。第一轮现场仍要分别验证前、右、上和平移/
旋转方向，因为当前只能从源码确认 reference-space 路径，不能替代真机观测。

### 19.5 tracking validity 的真实能力边界

PICO Unity 端实际取得并发送每关节：

- `locationStatus`：orientation/position valid/tracked 位；
- `radius`；
- 整手 `isActive` 和 `handScale`。

但当前 `xrobotoolkit_sdk 1.0.2` 的 pybind 只复制每关节 pose 和整手 `isActive`，丢弃了
`locationStatus`、`radius` 和 `handScale`。MVP 不伪造 per-joint confidence：只把 `isActive > 0`、
26×7 shape、有限值和非零 quaternion 共同作为保守 validity；tracking 丢失时不发布有效 marker，
以后也不得进入 retarget/robot command。

需要逐关节 validity 时的最小升级是重新编译 `XRoboToolkit-PC-Service-Pybind`，在现有 JSON
解析处暴露已经收到的 `s/r/scale` 字段。无需改 Unity 网络协议，也不应先大改 PC Service。

### 19.6 MVP 文件边界

计划/实施文件：

| 文件 | 作用 |
|---|---|
| `pico_hand_tracking.py` | 26 joint 名称、`HandTrackingData`、校验、同 controller 换基、finger helper |
| `pico_source.py` | 仅增加默认关闭的可选 hand read；原 controller frame 字段和读取保持兼容 |
| `pico_hand_debug.py` | 独立 receiver，约 8 Hz 摘要/全量打印，可选 ROS marker，不发布机器人命令 |
| `run_pico_hand_tracking.sh` | 复用现有 CPython 3.10、PICO `.so` 和 ROS 环境 |
| `test_pico_hand_tracking.py` | joint 顺序、shape、xyzw、坐标、validity、flexion、pinch 与 source 兼容测试 |

预留但第一阶段不实现：

```python
robot_hand_q = retarget_hand(human_hand_data)
```

先提供 `get_finger_flexion()`、`get_thumb_opposition()`、`get_fingertip_positions()`。后续 26→11DoF
应先在 Isaac Sim 使用左右手 URDF 的真实 11 个关节名，再区分真机 SDK 当前 6D 逻辑命令和是否存在
未公开的独立 11DoF 接口。

### 19.7 C1 手控制现状与后续选项

Isaac Sim 每只手已有 11DoF：拇指 3DoF，其余四指各 2DoF。`action_process.py` 已支持两种输入：

1. 现有 Astron SDK 风格 6D：thumb swing + thumb/index/middle/ring/little MCP；每个非 swing 值在仿真
   展开到相应两级关节；
2. 直接按 `L_/R_*_joint` 名称发送完整 11DoF dict，仿真会逐关节执行。

后续控制人的手有三个递进选项：

1. **几何规则映射（推荐先做）**：由各指骨段夹角得到 flexion，thumb-tip 距离/方向得到 opposition，
   标定 open/fist/pinch 后映射到 11DoF joint limits；可解释、易加限速与丢失保护。
2. **运动学优化 retargeting**：以 fingertip/palm 相对位置为目标，求 11DoF 约束优化；精度更好，但需
   机器人手 FK、关节限位、连续性正则和自碰约束。
3. **学习式映射**：采集人手 skeleton 与期望机器人姿态训练模型；只有前两种稳定并积累数据后再考虑。

任何方案进入机器人控制前都必须增加：显式 enable/deadman、首次捕获锚点、tracking timeout、
joint limit、逐帧限速、异常跳变拒绝，以及失跟踪保持/张开策略。第一阶段完全不发送这些命令。

### 19.8 MVP 现场验收顺序

1. 不启动 `--enable-command`，先运行独立 hand debug receiver。
2. PICO 系统 Hand Tracking 开启；XRoboToolkit `Hand On -> Send`，controller 可关闭或保留。
3. 确认左右手各 26 点、wrist index 1、meter、xyzw，摘要日志限制在 5–10 Hz。
4. 分别做真实手前/右/上移动和手腕旋转，检查 robot-axis 输出方向。
5. 弯食指，确认 index chain 和 `get_finger_flexion()` 明显变化。
6. 做 thumb-index pinch，确认两 fingertip 距离明显减小。
7. 把手移出视野，确认 `is_tracked=False`，marker 删除且没有机器人命令。
8. 单独回归原 `run_pico_teleop.sh` controller 模式，确认行为和之前一致。

当前本机没有运行 PC Service/PICO 数据，离线调用 SDK 得到 timestamp 0、左右 `isActive=0` 和
26×7 全零；这只证明接口和 shape 存在，不算真实 hand tracking 验收。

### 19.9 本轮实现与测试结果

MVP 代码已经按 19.6 的边界实现：

- controller 默认路径不导入 hand 模块；只有 `PicoSource(hand_tracking_enabled=True)` 才加载/检查
  4 个 hand API。
- `read_hands()` 只读取全局 timestamp 和左右手 skeleton/quality，不读取 head、controller pose、
  trigger 或按键，所以现场可以不拿 controller。
- `HandTrackingData` 保留 26 个命名 joint 的 position、`quaternion_xyzw`、保守 validity、左右手、
  timestamp、单位和 reference-space 描述。
- hand debug 默认 8 Hz 输出 wrist、五指 tip、食指 flexion 和 thumb-index distance；
  `--all-joints` 输出全部 26 点。
- `--ros-markers` 发布 `/pico/hand_tracking/markers` 的 wrist + 五指 tip spheres；失跟踪时发送
  DELETE，不发布任何 C1 body/hand command。
- `retarget_hand()` 只保留显式接口并抛出 `NotImplementedError`，防止第一阶段误发未经标定的
  机器人手指值。

容器内执行：

```bash
/usr/bin/python3 -B -m unittest -v \
  test_pico_hand_tracking.py test_pico_math.py test_dual_arm_ik.py \
  test_pico_ros.py test_pico_episode_recorder.py test_pico_headset_view.py
```

结果为 `33 tests passed`，覆盖新增 hand parsing/helper/source 隔离和原 controller ROS、IK、录制、
视频回归。宿主 CPython 3.10 的 hand/math/video 24 项也通过；`git diff --check`、无写入语法检查和
shell `bash -n` 通过。

无 PC Service/PICO 状态实际启动 receiver 时，左右手均安全显示：

```text
is_tracked=False quality=0
wrist/tips/flexion/pinch=unavailable
```

SDK 能正常关闭；进程未发布机器人命令。真实设备验收仍未完成，下一次最短步骤：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
./run_pico_hand_tracking.sh
```

随后在 PICO 端开启系统 Hand Tracking，XRoboToolkit 选择 `Hand On -> Send`。先完成 19.8 的
26 点、方向、rotation、食指和 pinch 验收，再运行 `--ros-markers`；在此之前不得进入 26→11DoF
控制实现。

## 20. 2026-08-18 26-joint → C1 11DoF 离线重定向

本节是第 19 节之后的下一阶段，取代 19.6/19.9 中“`retarget_hand()` 只抛异常”的历史状态。
这一阶段不连接真实 PICO，也不把 mock transport 接到真机。

### 20.1 资产真值纠正与自动审计

运行时主资产是：

```text
ubt_sim/assets/robots/walker_c1/Collected_walker_c1_v1_sensorKpkd/
  Collected_walker_astron_v1_sensorKpkd/walker_astron_v1_sensorKpkd_hands.usd
```

URDF 只能交叉参考，不能作为实际运行关节的最终依据。本轮用 `pxr.Usd` 直接打开上述 USD crate，
遍历 `/walker_astron_v1/joints`，并检查了：

- prim type 必须是 `PhysicsRevoluteJoint`；
- `physics:jointEnabled=true` 且 `excludeFromArticulation=false`；
- `PhysicsDriveAPI:angular`、body0/body1、axis、lower/upper limit；
- `physxJoint:maxJointVelocity`；
- `devices/walker_c1/config.py` 的左右 hand list 和 `ActionsCfg` 显式顺序；
- `action_process.py` 对完整 11DoF named dict 的入口。

新增 `inspect_c1_hand_usd.py` 会自动重复这套核对，并把主 USD、控制顺序和独立 retarget config
三方比较；任何 joint 缺失、顺序或限位不一致都会返回非零退出码：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
/isaac-sim/python.sh -B inspect_c1_hand_usd.py
```

当前实测输出 `OK: 22 driven revolute joints match the retarget config and control order.`。主 USD
`metersPerUnit=1`、`upAxis=Z`，关节 limit 和 max velocity 以 degree 存储，代码审计时明确换算为 radian。

### 20.2 主 USD 确认的单手 11DoF

控制顺序不是 USD 的字典/遍历顺序，而是 `WALKER_C1_LEFT_HAND_JOINTS`/
`WALKER_C1_RIGHT_HAND_JOINTS` 传给 `JointPositionActionCfg` 的显式顺序：

| i | semantic | left joint | left `[q_min,q_max]` rad | right joint | right `[q_min,q_max]` rad | parent→child / physical role |
|---:|---|---|---|---|---|---|
| 0 | thumb_cmp | `L_thumb_cmp_joint` | `[-0.96,0]` | `R_thumb_cmp_joint` | `[0,0.96]` | palm→thumb_cmp；镜像 base swing/opposition，现有 `thumb_swing` 也只映射到此轴 |
| 1 | thumb_mpp | `L_thumb_mpp_joint` | `[0,1.04]` | `R_thumb_mpp_joint` | `[0,1.04]` | thumb_cmp→thumb_mpp；近端屈曲 |
| 2 | thumb_ip | `L_thumb_ip_joint` | `[0,1.05]` | `R_thumb_ip_joint` | `[0,1.05]` | thumb_mpp→thumb_ip；远端/IP 屈曲 |
| 3 | index_mpp | `L_index_mpp_joint` | `[0,1.46]` | `R_index_mpp_joint` | `[0,1.46]` | palm→index_mpp；近端/base 屈曲 |
| 4 | index_ip | `L_index_ip_joint` | `[0,1.62]` | `R_index_ip_joint` | `[0,1.62]` | index_mpp→index_ip；压缩人手 PIP/DIP 后的远端屈曲 |
| 5 | middle_mpp | `L_middle_mpp_joint` | `[0,1.46]` | `R_middle_mpp_joint` | `[0,1.46]` | palm→middle_mpp；近端/base 屈曲 |
| 6 | middle_ip | `L_middle_ip_joint` | `[0,1.62]` | `R_middle_ip_joint` | `[0,1.62]` | middle_mpp→middle_ip；压缩后的远端屈曲 |
| 7 | ring_mpp | `L_ring_mpp_joint` | `[0,1.46]` | `R_ring_mpp_joint` | `[0,1.46]` | palm→ring_mpp；近端/base 屈曲 |
| 8 | ring_ip | `L_ring_ip_joint` | `[0,1.62]` | `R_ring_ip_joint` | `[0,1.62]` | ring_mpp→ring_ip；压缩后的远端屈曲 |
| 9 | little_mpp | `L_little_mpp_joint` | `[0,1.46]` | `R_little_mpp_joint` | `[0,1.46]` | palm→little_mpp；近端/base 屈曲 |
| 10 | little_ip | `L_little_ip_joint` | `[0,1.62]` | `R_little_ip_joint` | `[0,1.62]` | little_mpp→little_ip；压缩后的远端屈曲 |

22 个 USD joint 都没有 mimic，均有独立 angular drive。USD 的
`physxJoint:maxJointVelocity=119.748 deg/s=2.09 rad/s`；Isaac Lab actuator config 虽设置
`velocity_limit_sim=3`，retargeter 使用主 USD 更保守的 2.09 rad/s。

### 20.3 文件与接口

| 文件 | 作用 |
|---|---|
| `pico_hand_retarget_config.json` | 主 USD 关节模型以及 sign/scale/offset/range/limit/smoothing/sensitivity |
| `pico_hand_retarget.py` | 几何特征、纯 normalized mapping、11D 映射和 stateful safety filter |
| `pico_hand_mock.py` | 八类完整 26-joint mock skeleton，不依赖 PICO |
| `inspect_c1_hand_usd.py` | 主 USD + control list + retarget config 自动一致性检查 |
| `pico_hand_offline_demo.py` | mock→retarget→ROS bridge→Isaac 的自动循环播放器 |
| `run_pico_hand_offline_demo.sh` | 容器内离线 demo 入口 |
| `test_pico_hand_retarget.py` | mapping、mock、安全滤波、payload 单元测试 |

公开接口现在为：

```python
mapper = C1HandRetargeter()
robot_hand_q = retarget_hand(hand_data, retargeter=mapper, dt_s=0.02)
assert robot_hand_q.shape == (11,)
```

实时循环必须复用同一个 `mapper`，否则 smoothing、velocity limit 和 previous-valid hold 没有跨帧状态。
receiver 默认仍不导入 retarget 模块；只有显式调用 `retarget_hand()` 才 lazy import，原 controller
teleoperation 无变化。

### 20.4 可解释几何映射

对 index/middle/ring/little，每条 OpenXR chain 先计算：

1. 相邻 bone direction 的三个夹角，作为合并后的 MCP/PIP/DIP bend；
2. metacarpal→fingertip 距离除以各 bone length 总和，作为 tip reach/curl；
3. bend 和 curl 按 config 权重归一化为 total flexion；
4. proximal bend 与 total flexion 混合成 robot mpp；distal bends 与 total flexion 混合成 robot ip。

因此不是把人的某个关节角直接复制给机器人；人手 3 个 bend 被压缩为 C1 的 2 个 joint。thumb 使用
两段 bend 生成 mpp/ip，thumb-tip 与 index-tip 距离除以 palm width 生成 opposition，驱动镜像的 cmp，
并给 thumb mpp/ip 和 index 一个小的 pinch assist。所有几何量对 wrist 平移/旋转不敏感。

### 20.5 安全滤波

`C1HandRetargeter` 当前具备：

- 主 USD `q_min/q_max` 最终裁剪；
- 每 joint 2.09 rad/s 逐帧速度限制；
- 第一个有效帧默认也从 open-hand 初值按限速进入，避免首次握拳目标直接跳变；
- 可配置 exponential smoothing（默认 time constant 0.08 s）；
- 非法 joint、NaN、zero-length bone 或 `is_tracked=false` 统一进入 tracking-loss 分支；
- 默认精确保持 previous valid command；首次就是 invalid 时输出 open hand；
- 可选 `invalid_behavior=open/neutral`，fallback 同样经过 smoothing/velocity limit；
- 可配置 hold timeout 后再转 open/neutral，默认无限 hold。

这一层只返回 NumPy `(11,)`，不会自行发布 ROS/ZMQ 命令。真正接 PICO 控制前仍要在更上层增加显式
hand-control enable/deadman，不能把“收到了 skeleton”直接等同于“允许控制机器人”。

### 20.6 mock 与 Isaac 离线播放器

完整 mock pose：

```text
open_hand
fist
index_flex
ring_flex
little_flex
thumb_index_pinch
tripod
cylindrical_grasp
```

它们都生成左右手完整 26-joint `HandTrackingData`，position 单位 meter，便于不连接 PICO 时重复测试。
先只看目标向量：

```bash
./run_pico_hand_offline_demo.sh --dry-run --side both --cycles 1
```

正常 C1 仿真和自动 ROS bridge 已启动时：

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 ./run_pico_hand_offline_demo.sh --side both
```

循环顺序固定为
`open→fist→open→pinch→tripod→open→ring flex→open→little flex→open→cylindrical grasp`。
`ring_flex` 只驱动无名指 `mpp/ip`，`little_flex` 只驱动小指 `mpp/ip`；两者前后插入 open，
用于明确验证主 USD 中两根手指及控制通道彼此独立。默认通过
`/mc/{left,right}_hand/command` 发送完整 11-name `JointCommand`；现有 bridge 的 `hand_cb()` 不限制
6D SDK names，会原样转成 full named dict，`action_process._expand_hand_command()` 再按实际 sim joint
名称执行。不能同时运行 controller teleoperation 或其他 hand publisher。

### 20.7 A：已经离线验证的软件部分

- 用主 USD crate 直接验证 22 个 joint type、drive、parent/child、limits、velocity 和左右镜像符号；
- 控制 order 与 `ActionsCfg` 列表严格一致，不使用 USD traversal 顺序；
- 八种 mock、左右手均输出 finite `(11,)` 且不越主 USD limits；
- open 与 fist 的 11D 范数明显不同；
- index_flex 只有 index mpp/ip 明显变化；
- pinch 主要变化 thumb+index，tripod 再加入 middle，cylindrical 覆盖五指；
- left/right thumb cmp 符号镜像，其余屈曲量一致；
- invalid tracking 精确保持上一有效命令，NaN 也走相同保护；
- smoothing、2.09 rad/s 限速、open/neutral fallback 均有测试；
- demo dry-run 的每只手 payload 是按 control order 排列的完整 11-name dict；
- 原 hand receiver 仍是 opt-in，原 controller 入口和坐标映射文件未改。

主 USD 自动审计、mock/retarget/dry-run 已实际运行。用户随后在现有 Isaac Sim 中完成 GUI 视觉验收：
open/fist/pinch/tripod/cylindrical 均能运动；新增 `ring_flex` 与 `little_flex` 能分别独立驱动无名指和
小指，确认主 USD articulation、full 11-name payload 和现有 action processor 的运行路径一致。

### 20.8 B：只有真实 PICO 才能验证的部分

2026-08-18 已用真实 PICO 验证：左右手均能进入 `is_tracked=true quality=1`，PC 连续收到完整 26 点；
右手实测 wrist、五 fingertip、`index flexion` 和 thumb-index distance 均为有效动态数值，单手丢失
时对应数据安全变为 unavailable。以下项目仍需现场继续确认/调参：

- PICO 手 skeleton 与 controller 是否完全共享同一 reference space；
- 实际手腕前/右/上与 rotation 的方向和 quaternion convention；
- 真实用户 open/fist 的 bend 分布是否匹配当前 normalization reference；
- 真实 pinch distance/palm width 阈值、拇指 opposition sensitivity；
- 左右手个体尺寸、遮挡、抖动和 `isActive` 丢失时的现场行为；
- 从真实 skeleton 驱动仿真时的主观自然度，以及最终 sign/scale/offset/smoothing 微调；
- 若需要 per-joint validity，升级 pybind 后对 PICO `locationStatus` 的现场验证；
- 真机是否存在独立 11DoF SDK 接口。当前只确认 Isaac 支持 full 11DoF，不能把仿真接口能力外推到真机。

真实 PICO 接入前仍应先完成 19.8 的纯 skeleton 验收；接入后只调
`pico_hand_retarget_config.json`，不要回头重写已经验收的 controller 坐标逻辑。

### 20.9 本轮测试记录

主 USD 审计：

```bash
/isaac-sim/python.sh -B \
  /ubt_sim/teleoperation/control/walker_c1/inspect_c1_hand_usd.py
```

本轮实际使用临时 `usd-core` 的同一 `pxr.Usd` API 在宿主读取主 USD，临时环境在审计后已删除；上面是
容器内可重复执行的等价命令。结果：22 个 driven revolute joints 的 type、independent/no-mimic、parent/child、q limits、velocity、
左右符号、config 和 control order 全部一致。

宿主 CPython 3.10 离线回归：

```bash
python3 -B -m unittest -v \
  test_pico_hand_retarget.py test_pico_hand_tracking.py \
  test_pico_math.py test_pico_headset_view.py
```

结果 `44 tests passed`，其中包含把 demo full 11-name payload 直接送入现有
`action_process._extract_command_targets()` 的隔离测试，确认没有退化为旧 6D 展开。另外：

- demo 的 ROS transport 强制核对仿真 `ROS_DOMAIN_ID=146`，Domain 0 或未设置时拒绝发布；

- `run_pico_hand_offline_demo.sh --dry-run --side left/right` 均成功输出八类 mock 的 full 11D；
- `git diff --check`、新增 Python AST/JSON parsing、两个 shell `bash -n` 全部通过；
- `pico_teleop.py`、`pico_math.py`、`pico_teleop_config.json`、C1 `config.py/action_process.py` 均为零 diff；
- 宿主单独运行 `test_dual_arm_ik.py` 因宿主未装 `ikpy` 在 collection 阶段退出；这是环境依赖，
  同一原测试此前已在容器 33 项 controller/IK/ROS/recorder/video 回归中通过，本轮未修改 IK/controller。

## 21. 2026-08-18 真实 PICO → C1 11DoF 仿真 teleop

在真实 PICO 左右手 skeleton 均通过接收验收后，新增独立入口 `pico_hand_live.py` 与
`run_pico_hand_live.sh`。它复用 `PicoSource(hand_tracking_enabled=true)` 和持久化
`C1HandRetargeter`，但不修改 `pico_teleop.py`。当前只控制手指；wrist pose 继续保留，但不发布
机械臂或 body 命令。手指映射验收后再新增可选 `hand+arm` 模式。

### 21.1 模式和命令边界

- `--mode preview` 是默认值：实时输出左右 `q11_rad`、五指 normalized flexion 和 pinch distance，
  不加载 ROS、不创建 publisher；
- `--mode sim` 必须同时显式给出 `--enable-command` 且 `ROS_DOMAIN_ID=146`；
- 代码没有 `real` 模式，不能从此入口切到真机；
- 只发布 `/mc/{left,right}_hand/command` 的完整 11-name `JointCommand`，不发布 body/arm 命令；
- 必须停止 offline demo、controller teleoperation 和其他 hand publisher 后再运行。

直接测试右手：

```bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --mode sim --side right --enable-command
```

程序永远从 `DISARMED` 启动。按 `a` 后，选定手需要连续 tracking valid 0.5 秒才进入 `ARMED`；
按 `d` 立即停止发布，`q`/`Ctrl+C` 退出。任一选定手 invalid 时立即停止发布，持续 0.30 秒后撤销
解锁；tracking 恢复不会自动重新解锁。每次刚进入 ARMED 都重置 mapper，从 open command 按主 USD
2.09 rad/s 限速进入当前真实手姿态。

### 21.2 软件验证

`test_pico_hand_live.py` 覆盖：显式 arm、连续有效 dwell、双手任一 invalid 拒绝 arm、tracking loss
立即停发/超时 disarm、恢复不自动 re-arm、operator disarm、双手 full named 11D payload、invalid 时
空 payload、preview 拒绝 command flag、sim 双重确认和 Domain 146 guard。与 tracking/retarget 测试
合并运行共 37 项通过；shell `bash -n`、Python bytecode compile 和 `git diff --check` 通过。

用户已按 21.1 使用真实 PICO 完成 PICO skeleton → 11DoF → Isaac Sim 现场验收，并确认手指 teleop
动作表现正常。当前阶段因此冻结为可回退基线；下一阶段在独立可选模式中加入 wrist→arm，不回改
已经验收的 finger mapping。后续若需微调，只改 `pico_hand_retarget_config.json`。

## 22. 2026-08-18 右手腕 → 右臂＋右手 11DoF 联合仿真控制

手指-only 现场验收后，已从基线提交 `cf8a0a8` 向前增加独立的 `hand-arm` 可选模式。第一版按用户
要求只允许右侧，使左手可操作键盘。它没有修改或导入 `pico_teleop.py` 的运行入口，也没有改动
`pico_math.py`、`pico_teleop_config.json`、C1 `config.py` 或 `action_process.py`；原 controller
teleoperation 和默认 `--control hand` 行为保持不变。

### 22.1 数据与控制链路

```text
PICO headset pose + right OpenXR 26-joint hand skeleton
  -> PicoSource.read_hand_frame()（不调用任何 controller getter）
  -> 已验收的 tracking_to_robot proper rotation
  -> wrist relative pose + 启动时 robot right-palm FK anchor
  -> 现有 ArmIK / workspace / residual / joint-step 限制
  -> /mc/sdk/robot_command：只含右臂 7 个 JointCmd

同一 skeleton
  -> 已验收的 C1HandRetargeter
  -> /mc/right_hand/command：完整 11-name JointCommand
```

新增 `pico_hand_arm.py` 只复用现有纯数学和 `ArmIK`，没有复制或改写 controller mapping。手 skeleton
已经由 `PicoSource` 转为机器人轴 `+X forward, +Y left, +Z up`；headset raw pose 用同一
`tracking_to_robot` 转换。锚定和相对目标继续使用 `horizontal_heading()` 与 `relative_target()`：

- 按键时的真实 wrist pose 对齐当前机器人 right palm FK，不发送绝对 tracking-origin offset；
- 平移沿用 `translation_scale=0.55`、0.35 m 最大位移与 right workspace；
- 旋转沿用相对 orientation 和操作者水平 heading；
- IK 沿用每帧 0.012 m/0.08 rad 目标推进、残差/分支跳变限制以及 0.025 rad/joint 命令 slew；
- right elbow 必须 `<= -0.40 rad`，IK 命令继续限制为 `<= -0.50 rad`。

### 22.2 边界与失效保护

- 只有 `--control hand-arm --mode sim --side right --enable-command` 被接受；preview、left、both、
  ROS Domain 非 146 或 real mode 均不可进入；
- 只创建 right-arm body command 和 right-hand publisher，不发布 head/left arm/waist/leg joint；
- 必须收到新鲜 `/mc/sdk/robot_state`，再检查右肘、头显、wrist 和整手 tracking；
- 启动前额外要求五指 normalized flexion/opposition 最大值不超过默认 0.45，避免按 `a` 时闭拳导致
  灵巧手立即闭合；
- 程序永远从 `DISARMED` 启动；`a` 后仍需 0.5 s 连续有效才锚定，`d` 立即停发；
- arm IK 与 hand payload 作为同一帧事务：任何 tracking/RobotState/IK 拒绝都让二者同时不发布；
- 失效不足 0.30 s 时不发布但保留锚点；达到超时即清除锚点并 disarm，恢复不会自动重新启动；
- arm joint slew 和 hand smoothing/主 USD 2.09 rad/s velocity limit 保留，即使短暂丢失后恢复也不会
  产生未经限速的关节命令跳变。

### 22.3 启动姿态与键盘

按 `a` 时建议操作者面向前方，右手腕静止放在舒适中位，右手张开；机器人必须先执行 task reset，
确保右肘处于弯曲可解的姿态。wrist 使用相对锚定，所以启动姿态不标准不会立即产生绝对 pose 跳变，
但它会成为后续平移/旋转的零点；手腕过度扭转可能缩小可用活动范围并增加 IK 拒绝。头显水平朝向
定义前/右方向，因此启动时转头会改变这次 session 的操作坐标。

键盘是当前唯一 enable 输入：`a` 请求启动、`d` 停发、`q` 退出。联合模式还支持空格键完整复位：
先立即 disarm 并清除 wrist anchor，再向可靠 ROS topic `/sim/cmd_reset` 发布一次 `Bool(true)`，等待
0.2 秒后用当前 `/usr/bin/python3` 执行现有 `reset.py --mode task`。后者沿用已经验证的
arm-clear → elbow-clear → task-reset 三阶段路径，同时把左右手张开。成功或失败都保持 disarmed；
成功后还会清空旧 RobotState，必须收到新的 post-reset state、张开右手并重新按 `a` 才能控制。

若以后要完全离开键盘，推荐新增
独立 USB 脚踏/HID deadman；左手手势可作为非安全辅助命令，但左手操作键盘时容易误识别；语音不应
作为唯一急停。右手 pinch 不适合做 enable，因为它本身就是 11DoF retarget 的有效动作。

### 22.4 运行与验证

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
cd /ubt_sim/teleoperation/control/walker_c1
ROS_DOMAIN_ID=146 /usr/bin/python3 reset.py --mode task

ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command
```

本轮宿主离线回归共 62 项通过，包含：anchor 零位无跳变、0.55 平移比例、wrist orientation、现有
0.025 rad slew、IK reject 不推进 seed、RobotState/弯肘前置条件、invalid wrist、右侧-only/domain/
explicit sim guard、open-hand gate、单次场景 reset edge、现有 task-reset 子进程调用、reset 失败保持
disarmed，以及原 hand tracking/retarget/controller 坐标数学回归。
Python 语法检查使用独立 `/tmp` pycache 通过，shell `bash -n` 和 `git diff --check` 通过。

### 22.5 真实 PICO 现场验收与已知限制

用户已佩戴真实 PICO，在不使用 controller 的情况下运行右侧 `hand-arm` 模式：右手腕驱动右臂、
同一 26-joint skeleton 驱动右手 11DoF。用户对手臂平移/旋转的现场反馈为整体正常，并已实际把固定
苹果抓起，确认 PICO → wrist relative target → ArmIK → 右臂 ROS command 与 skeleton → 11DoF hand
command 能在 Isaac 中同步闭环。当前主观结论是“可以抓起，但仍比较难”，不能写成抓取 mapping 已
最终调好，也没有进入真机命令模式。

现场还明确复现了闭拳自遮挡：真实手完全闭合时，日志出现 thumb-index distance 约
`0.003–0.011 m`、多指 normalized flexion `1.0`，robot middle/ring 等关节达到主 USD
`1.46/1.62 rad` 上限；随后 PICO 整手 `isActive` 间歇或连续 invalid。当前 Python binding 只有整手
active 状态，无法在这种情况下证明 wrist 仍有效，因此安全门停止新命令并在默认 0.30 秒后 disarm
是正确行为，不得用缓存 skeleton 伪装成有效 tracking。恢复后必须重新按 `a`。

已验证的自动苹果抓取使用约 `0.70–0.85 rad`，比裸手完全闭拳产生的极限 q11 温和。现场操作优先
使用半闭合 C 形手势（normalized flexion 约 `0.4–0.7`、主要 robot finger q 约 `0.7–0.9 rad`），
避免真实手自遮挡和机器人过度挤压。短暂遮挡可显式运行：

```bash
ROS_DOMAIN_ID=146 ./run_pico_hand_live.sh \
  --control hand-arm --mode sim --side right --enable-command \
  --tracking-timeout-s 2.5
```

2.5 秒只改变自动 disarm 的 grace period：invalid 帧仍不发布新目标、C1HandRetargeter 保留上一安全
手命令、arm anchor 不更新；超时仍 disarm。不能无限延长，也不能据此宣称闭拳 tracking 已解决。
后续若要降低抓取难度，应单独标定 live teleop 的抓取输出上限/灵敏度，使真人半握映射到已经验证的
苹果 grip 范围；不要放宽右肘安全限制，也不要改动已验收的 controller 坐标映射。
