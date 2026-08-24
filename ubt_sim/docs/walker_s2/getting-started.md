# Walker S2 EDU 探索者仿真使用说明

## 概述

Walker S2 EDU 仿真基于 NVIDIA Isaac Sim（Isaac Lab 2.2.0）构建，通过 ROS2-ZMQ 桥接将仿真状态与图像以**与真机一致的 ROS2 话题**对外发布，实现 sim-to-real 一致的遥操作与数据采集。整套环境运行在 Docker 容器内便于部署。

![Walker S2 EDU 仿真界面预览](../assets/walker-s2仿真界面预览.png)

架构（三进程，Python 不可混用）：

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  遥操作/采集脚本
```

- **仿真** `/isaac-sim/python.sh`：加载 `UBTSim-WalkerS2-PartSorting-v0`，四路相机 RGB 直发 ZMQ 5657。
- **桥接** `/usr/bin/python3`：ZMQ↔ROS2 双向转发，`start_sim.sh` 自动拉起。
- **采集** `/usr/bin/python3`：发布指令、录制 15Hz HDF5。

### ZMQ 端口（见 `walker_s2_bridge_config.yaml`）

| 端口 | 用途 |
| --- | --- |
| 5655 | 控制指令（bridge PUB） |
| 5656 | 机器人状态（bridge SUB） |
| 5657 | 相机图像（仿真直接发布，四路相机 RGB，采集时直连此端口） |
| 5658 | JPEG 图像（仿真发布） |

## 仿真模块

| 模块 | 路径 | 说明 |
| --- | --- | --- |
| Docker 编排 | `docker/run.sh` | 镜像构建、容器生命周期、桥接启停 |
| 环境变量 | `docker/env.sh` | 容器名、镜像、`ROS_DOMAIN_ID` |
| 仿真启动器 | `scripts/start_sim.sh` | 按 `UBT_SIM_TASK` 识别机器人、拉起桥接、启动 `sim_runner.py` |
| 仿真入口 | `scripts/sim_runner.py` | Isaac Sim 主循环、键盘复位、profiling |
| 任务定义 | `source/ubt_sim/task/walker_*/` | walker S2 场景定义 |
| 仿真配置 | `config/walker_s2/` | `part_sorting.yaml` / `pick-part.yaml` / `parlor.yaml`（场景/相机/零件参数） |
| ROS2-ZMQ 桥接 | `teleoperation/bridges/walker_s2/` | 桥接脚本 + `walker_s2_bridge_config.yaml` |
| 遥操作与采集 | `teleoperation/control/walker_s2/` | `walker_s2_controller.py` / `pick_part.py` / `carry_box.py` / `keyboard_ee_control.py` / `save_data.sh` |
| 数据输出 | `dataset/walker_s2/<时间戳>/trajectory.hdf5` | 采集产物 |


## 快速开始

### 1. 构建环境（宿主机）

```bash
cd ubt_sim/docker
bash run.sh build      # 构建 ROS2 + Isaac Sim 镜像
bash run.sh start      # 创建/启动容器 ubt-sim
bash run.sh init       # 容器安装相关依赖,只需首次构建容器时执行一次
bash run.sh check      # 校验环境（GPU / ROS2 / 消息包 / numpy<2 等）
```

如需区分真机/仿真的 ROS2 域，用 `ROS_DOMAIN_ID` 启动容器（默认 0）：

```bash
ROS_DOMAIN_ID=0 bash run.sh start
```

### 2. 启动仿真（容器内）

```bash
bash run.sh bash                       # 进入容器（自动 source ROS2 + Walker SDK 环境）
UBT_SIM_TASK=UBTSim-WalkerS2-PickPart-v0 bash /ubt_sim/scripts/start_sim.sh   # 启动仿真 + 自动拉起桥接
```
Walker S2 EDU 现有任务场景（通过`UBT_SIM_TASK`选择场景）：

| 任务 ID | 场景 | 说明 |
| --- | --- | --- |
| `UBTSim-WalkerS2-PartSorting-v0` | 仓库 `2_small_warehouse2.usd` | 零件分拣：4 个零件 + 收纳箱（默认任务，下文说明对应此场景） |
| `UBTSim-WalkerS2-PickPart-v0` | 仓库（同上） | 单零件抓取：桌面只保留单个零件，配合 `pick_part.py` 采集 |

运行效果（抓放任务）：

<video src="../assets/walker.mp4" controls muted width="100%"></video>
- 默认加载 `UBTSim-WalkerS2-PartSorting-v0`（仓库零件分拣）场景，桌面上有 4 个零件（`part_a_ori` / `part_a_red` / `part_b_blue` / `part_b_ori`）和一个收纳箱。
- **按 `R` 键**可复位机器人/场景；也可通过采集脚本 `--reset-scene` 发布 `/sim/cmd_reset`。
- 关闭仿真窗口时，`start_sim.sh` 的退出陷阱会自动停止桥接进程。



常用环境变量（可叠加）：

| 变量 | 作用 | 默认 |
| --- | --- | --- |
| `UBT_SIM_TASK` | 任务名（同时决定机器人类型） | `UBTSim-TienkungPro-Parlor-v0`，Walker S2 EDU 见上方任务场景列表 |
| `--headless` | 无窗口运行 | 无显示器/服务器场景最常用 |
| `--perf_stats` | 打印性能统计 | 排查帧率 |
| `--device cpu\|cuda\|cuda:N` | 运行设备 | 非 load_only 换卡时用 |


### 3. 数据采集（容器内，使用系统 Python 3.10）

```bash
# 机器人抓取并录制 
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/pick_part.py --save --reset-scene --robot-init

# 批量循环采集（默认 400 次）
bash /ubt_sim/teleoperation/control/walker_s2/save_data.sh
```

-  `--part <名>`指定零件，默认抓取单个 part_a_red 零件；
-  `--save` 开启录制，录制 17 维身体关节 + 双手 + 夹爪 + 四路相机 RGB 与时间戳到 HDF5；
-  `--reset-scene` 运行时先初始化场景 `--robot-init` 机器人初始位置；
- **`save_data.sh`** 批量采集脚本：注意需根据不同任务修改内部参数，当前默认单零件400次抓取任务；
- **`pick_part.py`** 执行一次零件抓放操作：实时读取 `/sim/part_states` 零件位置 → pregrasp → 夹取 → 抬升 → 放入箱中 → 归位；
- 产物路径：`/ubt_sim/dataset/walker_s2/<时间戳>/trajectory.hdf5`。

详细参数见下方列表：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--part <名>` | `part_a_red` | 零件名（`part_a_ori` / `part_a_red` / `part_b_blue` / `part_b_ori`） |
| `--all-parts` / `--parts <名...>` | — | 依次抓取四个零件 / 自定义抓取顺序 |
| `--side left\|right` | `right` | 抓取手臂 |
| `--reset-scene` | 关 | 抓取前发布 `/sim/cmd_reset` 重置场景 |
| `--robot-init` | 关 | 抓取前移动到 READY_POSE（等价 `state --init`；前置操作，不写入录制数据） |
| `--randomize-parts` / `--no-randomize-parts` | 开 | 多零件抓取前随机化零件位置 |
| `--save` | 关 | 录制 HDF5（配合 `--save-only-success`，默认仅任务成功落盘） |
| `--capture-hz <Hz>` | `15.0` | 写入 HDF5 的采集频率 |
| `--unlock-waist` | 关 | 是否解锁腰部 IK |



### 4. 机器人控制器介绍（容器内运行，使用系统 Python 3.10）

统一入口 `walker_s2_controller.py` 机器人脚本，可用于真机/仿真场景机器人控制。

```bash
# 示例：机器人位置初始化动作（分段移动到 READY_POSE 预备姿态）
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py --init
# 更多控制可以通过 `--help` 参数查看
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py --help 
# 查看某个子命令的完整参数
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py <子命令> --help 
```

子命令一览：

| 子命令 | 说明 | 常用参数 |
| --- | --- | --- |
| `state` | 关节/夹爪/末端状态、单关节移动、预备姿态、夹爪控制 | `--print-state` `--init` `--grip-open` `--move-joint --joint <名> --pos <rad>` |
| `joint` | 关节/手部调试 | `--print` `--move JOINT=ANGLE` `--hand-open` `--monitor` |
| `endpoint` | 末端/TCP 位姿测试 | `--no-move` `--side left` |
| `home` | 分段回 home 全零位 + 张开双手 | 无参数 |
| `analyze` | 关节阶跃/正弦响应分析 + CSV | `--joint <名> --step <rad>` `--sine` `--listen` `--csv <路径>` |
| `camera` | 相机话题信息/预览/保存 | `--preview` `--save --count 5` `--topic <话题>` |

常用示例：

```bash
/usr/bin/python3 walker_s2_controller.py state --print-state                # 查看关节/夹爪/末端状态
/usr/bin/python3 walker_s2_controller.py state --init                       # 分段移动到预备姿态
/usr/bin/python3 walker_s2_controller.py joint --move R_elbow_yaw_joint=0.5 # 单关节移动到 0.5 rad（回车确认）
/usr/bin/python3 walker_s2_controller.py home                               # 分段回 home 全零位 + 张开双手
/usr/bin/python3 walker_s2_controller.py analyze --joint R_elbow_yaw_joint --step 0.5  # 关节阶跃分析
/usr/bin/python3 walker_s2_controller.py camera --preview                   # 相机实时预览
```

#### 相机测试（camera 子命令）

Walker S2 EDU 发布四路相机（头部双目 stereo_left/right + 腕部 wrist_left/right）ZMQ（5657）和 ROS2 话题，可使用以下工具订阅相机预览，也可使用 `ros2 topic echo` 订阅ROS话题查看：

```bash
# 实时预览（cv2.imshow，需 X11 图形显示；SSH 进入容器需 -X 转发，按 Q/ESC 退出）
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py camera --preview

# 查看/保存四路相机图像
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py camera --save --count 5
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--topic` | bridge config → `/sensor/camera/stereo/color/raw` | 订阅的相机话题 |
| `--msg-type` | `Image2m` | 消息类型（`Image8k`/`Image512k`/`Image1m`/`Image2m`/`Image4m`/`Image6m`/`Image8m`/`sensor_msgs/Image`） |
| `--preview` | 关 | cv2.imshow 实时预览（按 Q/ESC 退出，需 X11） |
| `--save` | 关 | 保存帧为 PNG（`camera_frame_<时间戳>.png`） |
| `--count <N>` | `0`（无限） | 保存/预览帧数上限，达到即退出 |
| `--interval` | `1.0 <秒>` | 打印/保存循环间隔 |
| `--no-print` | 关 | 不打印帧信息（所有模式生效） |


### 5. 数据集预览与回放（容器内）

读取 `dataset/walker_s2/<时间戳>/trajectory.hdf5`，预览相机+关节曲线（Rerun），或把录制动作直发回放（仿真/真机同话题）。

![Walker S2 EDU 数据预览](../assets/walker数据预览.png)

```bash
# 预览HDF5数据集
/usr/bin/python3 /ubt_sim/teleoperation/tools/playback_walker_s2.py \
    --episode dataset/walker_s2/1786681892 --mode preview --web-port 9090

# 真机动作（仿真/真机）
/usr/bin/python3 /ubt_sim/teleoperation/tools/playback_walker_s2.py \
    --episode dataset/walker_s2/1786681892 --mode control 
```

完整参数与真机说明见 `teleoperation/tools/playback/README.md`。


## 常用 run.sh 命令

```text
build         构建镜像
start         创建/启动容器
stop          停止容器（先停桥接）
restart       重启容器
bash          进入容器 shell（自动 source ROS2 + Walker SDK）
rm            删除容器
init          容器内安装依赖、构建 ROS2 消息与 C++ 图像桥接
check         校验环境
bridge-start  单独启动 ROS2-ZMQ 桥接
bridge-stop   单独停止桥接
```

## Walker S2 EDU 特殊说明

### GPU 渲染 + CPU PhysX

Walker S2 EDU 默认采用 **GPU 渲染 + CPU PhysX** 架构：`start_sim.sh` / `sim_runner.py` 将渲染/AppLauncher 设备设为 `cuda:0`，但 Isaac Lab physics/env 设备设为 `cpu`。这是为了规避 Isaac Sim 5.0 + Isaac Lab 2.2 在 Walker S2 EDU articulation 初始化时 `get_dof_velocities()` 触发的 PhysX GPU tensor device mismatch（`getVelocities: expected device 0, received device -1`）。不要默认改回 GPU physics；如需实验可显式设置 `UBT_SIM_WALKER_S2_PHYSICS_DEVICE=cuda:0`。

### Walker SDK ROS2 消息包

桥接与采集脚本依赖 Walker SDK ROS2 消息（`rosa_msgs` / `shm_msgs` / `mc_task_msgs` / `mc_state_msgs` / `ecat_task_msgs`），由 `bash run.sh init` 构建到 `/opt/ubt_sim/walker_sdk_ros2_msgs/install`，`run.sh bash` 进入容器时自动 source。DDS 中间件使用 `rmw_cyclonedds_cpp`。

## 故障排查

- **`bash run.sh check` 报错**：按提示重跑 `bash run.sh init`；确认 `numpy<2`、`bodyctrl_msgs`、Walker SDK ROS2 消息包均已就绪。
- **采集脚本报 rclpy / 消息类型导入失败**：确认用的是 `/usr/bin/python3` 而非 Isaac Sim Python；确认已 `bash run.sh init` 构建消息包。
- **`save_data.sh` 连续失败熔断**：多为桥接未就绪。先确认仿真窗口在运行、桥接进程存在（`pgrep -f walker_s2_ros2_zmq_bridge`），或单独 `bash run.sh bridge-start`。
- **仿真启动报 Walker SDK messages not built**：容器内执行 `cd /ubt_sim/docker && bash run.sh init`。
- **X11 不可用 / 仿真窗口不显示**：宿主机执行 `xhost +`，确认 `DISPLAY` 已透传；无显示器时仅可用 `UBT_SIM_LOAD_ONLY=1` 预览。
- **键盘遥操作无法捕获键鼠**：headless 容器内无法使用 pynput，请在宿主机（有图形界面）执行 `keyboard_ee_control.py`。
