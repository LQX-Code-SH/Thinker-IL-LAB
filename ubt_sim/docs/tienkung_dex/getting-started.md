# 天工行者 Dex （TienKung Dex）仿真使用说明

## 概述

天工行者 Dex（TienKung Dex，V3 模型）仿真基于 NVIDIA Isaac Sim 5.0（Isaac Lab 2.2.0）构建，通过 ROS2-ZMQ 桥接将仿真状态与图像以**与真机一致的 ROS2 话题**对外发布，实现 sim-to-real 一致的遥操作与数据采集。整套环境运行在 Docker 容器内，支持 Inspire RH5DG2（67 revolute）与 BrainCo Revo2（53 revolute）两种灵巧手。

![天工行者 Dex 仿真界面预览](../assets/TienKung_DEX.png)

架构（三进程，Python 不可混用）：

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  遥操作/采集脚本
```


### ZMQ 端口（见 `bridge_config.yaml`）

| 端口 | 用途 |
| --- | --- |
| 5755 | 控制指令（bridge PUB → 仿真 SUB） |
| 5756 | 机器人状态（仿真 PUB → bridge SUB） |
| 5757 | 相机原图（仿真发布 RGB + uint16 深度，bridge 交给 C++ 图像桥转 ROS 话题） |
| 5758 | JPEG 直连（仿真发布，调试用） |



## 快速开始

> 命令分两段：构建/进入容器在**宿主机**执行；启动仿真与采集在**容器内**执行（容器内根路径为 `/ubt_sim`）。

### 1. 构建并启动容器（宿主机）

```bash
cd ubt_sim/docker
bash run.sh build      # 构建 ROS2 + Isaac Sim 镜像
bash run.sh start      # 创建/启动容器 ubt-sim
bash run.sh init       # 编译 ros2_bridge_msgs / 因时 / 强脑消息包与 C++ 图像桥，首次执行一次
bash run.sh check      # 校验环境（GPU / ROS2 / 消息包 / numpy<2 等）
```

> `run.sh init` 不能跳过：Dex 桥接找不到 `/opt/ubt_sim/tienkung_dex_msgs/install/setup.bash` 会直接退出。
> 纯仿真只用 `127.0.0.1`；**真机联调前**须把 `docker/fastdds_no_shm.xml` 白名单改成实机直连网卡 IP（改完重启容器生效）。

### 2. 启动仿真（容器内）

```bash
# 只加载场景（不接 ROS，先确认位姿/相机）
UBT_SIM_LOAD_ONLY=1 UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# 完整模式：同时拉起 ROS2-ZMQ 桥接（5755-5758）
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh

# 参数原样透传给 sim_runner.py：无窗口 + 性能统计
UBT_SIM_TASK=UBTSim-TienkungDex-Parlor-v0 bash scripts/start_sim.sh --headless --perf_stats
```

两个场景（手部变体）分别对应独立任务：

| 任务 ID | 手部 | 关节数 | 配置 |
|---------|------|--------|------|
| `UBTSim-TienkungDex-Parlor-v0` | Inspire RH5DG2 | 67 | `config/tienkung_dex/parlor.yaml` |
| `UBTSim-TienkungDex-Brainco-Parlor-v0` | BrainCo Revo2 | 53 | `config/tienkung_dex/brainco_parlor.yaml` |

`start_sim.sh` 会从任务名推断变体（含 `Brainco` → brainco），并把 `--variant` 传给桥接。**按 `R` 键**可复位机器人/场景；关闭仿真窗口时退出陷阱会自动停止桥接。

运行效果（桌面抓苹果）：

<video src="../assets/tienkung_dex抓取视频.mov" controls muted width="100%"></video>

常用环境变量：

| 变量 | 默认 | 说明 |
|------|------|------|
| `UBT_SIM_TASK` | `UBTSim-TienkungPro-Parlor-v0` | 设为 `UBTSim-TienkungDex-Parlor-v0` 启用 DEX |
| `UBT_SIM_NUM_ENVS` | 1 | 并行环境数 |
| `UBT_SIM_LOAD_ONLY` | 未设置 | 仅渲染，不接 ROS（默认 `--device cpu`） |
| `UBT_SIM_NO_BRIDGE` | 未设置 | 置 1 时不启动桥接 |
| `UBT_SIM_TIENKUNG_DEX_CMD_PORT` / `_STATUS_PORT` / `_IMAGE_PORT` / `_JPEG_PORT` | 5755/5756/5757/5758 | ZMQ 端口（须与 `bridge_config.yaml` 一致） |
| `ROS_DOMAIN_ID` | 146 | 仿真 146 / 真机 0；话题相同，只靠 domain 分流 |

> `run.sh bash` 进入的 shell 会把 `ROS_DOMAIN_ID` 预设为 `docker/env.sh` 的值（默认 0），从那里启动请显式 `export ROS_DOMAIN_ID=146`，避免与真机串台。

### 3. 数据采集（容器内，使用系统 Python 3.10）

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
export ROS_DOMAIN_ID=146          # 真机改成 0

# 键盘关节遥操作（1~6 切分组，[/] 选关节，w/s 增减，o/c 手张开/闭合，q 退出）
/usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_joint_control.py --step 0.02
/usr/bin/python3 teleoperation/control/tienkung_dex/keyboard_joint_control.py --step 0.02 --variant brainco

# 交互录制：Enter 开始/停止，s 保存并下一集，q 退出
bash teleoperation/control/tienkung_dex/save_data.sh

# 无人值守：每集 30s，共 5 集
bash teleoperation/control/tienkung_dex/save_data.sh --auto-start --duration 30 --episodes 5 --task-name "pour water"

# 批量采集：每集自动「复位 → 预备位 → 抓 → 放到盘上」并落盘，失败集不保存
bash teleoperation/control/tienkung_dex/save_data.sh --batch 20
```

- 批量模式由 `pick_apple.py --save` 在**同一个进程**里跑动作 + 录制，录制区间掐在「预备位收尾完成 → 张手放到盘上」；`--batch` 之后的参数原样透传给 `pick_apple.py`。
- **成败按仿真真值判**：`/sim/task_completed` 给出苹果中心到盘心的距离，小于 `--task-dist-tol`（默认 0.15 m）才落盘并算成功，否则计入失败；收不到该话题（真机 / 旧桥接）时跳过判定。连续失败 10 次（`MAX_CONSECUTIVE_FAIL`）熔断整批。
- **苹果随机化**（`--randomize-apple`）：每集把苹果随机挪 `x=0~10 cm`、`y=0~-10 cm`（世界系），抓取航点跟着平移，用于采集有位置多样性的数据。
- BrainCo 变体给 `save_data.sh` 加 `--variant brainco`（影响手部列数与 attrs）。
- 腿部关节默认锁定（真机由行走控制器持有），控制器/播放类子命令可加 `--include-legs` 解锁。
- 更多控制（状态/IK/手部/相机/末端遥操作）与完整话题表见
  [control/tienkung_dex/README.md](../../teleoperation/control/tienkung_dex/README.md)。

产物：一集一个时间戳目录 `/ubt_sim/dataset/tienkung_dex/<时间戳>/trajectory.hdf5`（与 `tienkung_pro` / `walker_s2` 同布局，目录 0777、文件 0666），内含 31 维身体关节、双手关节、动作、头部 RGB+深度与时间戳；录制频率默认 15 Hz，`--capture-hz` / `--no-depth` 可调。两个手部变体靠 attrs 里的 `hand_variant` 区分。

### 4. 相机测试（容器内）

```bash
# 实时预览（cv2.imshow，需 X11，按 Q/ESC 退出）
/usr/bin/python3 teleoperation/control/tienkung_dex/tienkung_dex_controller.py camera --preview
# 保存 5 帧
/usr/bin/python3 teleoperation/control/tienkung_dex/tienkung_dex_controller.py camera --save --count 5

# 或订阅 ROS2 图像话题，保存 3 张 JPEG
/usr/bin/python3 /ubt_sim/teleoperation/image/test_ros_image.py
```

相机话题为 `/ob_camera_head/color/image_raw`（rgb8）与 `/ob_camera_head/depth/image_raw`（16UC1，毫米）。画面朝向由 `head_pitch_joint` 决定（正值为低头，预备位低头 45°），不要往 `cameras.*.rot` 里加安装俯角。

### 5. 数据集预览与回放（容器内）

```bash
# 预览（Rerun）；不给 --episode 就取 dataset/tienkung_dex 下最新的时间戳目录
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py --mode preview --web-port 9090

# 控制（向桥接话题回放，仿真/真机同话题）
/usr/bin/python3 teleoperation/tools/playback_tienkung_dex.py \
    --episode dataset/tienkung_dex/<时间戳>/trajectory.hdf5 --mode control
```

完整参数与真机说明见 `teleoperation/tools/playback/README.md`。

## 常用 run.sh 命令

```text
build         构建镜像
start         创建/启动容器
stop          停止容器
restart       重启容器
bash          进入容器 shell（自动 source ROS2 + Walker SDK；Dex 消息包需手动 source）
rm            删除容器
init          容器内安装依赖、构建 ROS2 消息与 C++ 图像桥接
check         校验环境
bridge-start  单独启动 ROS2-ZMQ 桥接
bridge-stop   单独停止桥接
```

> `bridge-start` / `bridge-stop` 目前只识别 `tienkung_pro` 的桥接进程；Dex 的桥接由 `start_sim.sh` 自动拉起与停止，需要单独起时执行
> `/usr/bin/python3 teleoperation/bridges/tienkung_dex/tienkung_dex_ros2_zmq_bridge.py --variant inspire`。

## 故障排查

- **仿真启动报 USD 缺失**：确认已用 Git LFS 拉全资产（`git lfs pull`）；`usd/` 下的 USD 已随仓库提供，仅修改 URDF 后需要按 [assets README](../../assets/robots/tienkung_dex/README.md) 重新生成，且 `--variant` 与任务 ID 必须匹配。
- **`bash run.sh check` / 桥接报消息包缺失**：容器内重跑 `bash run.sh init`；确认 `numpy<2`、`ros2_bridge_msgs`、因时/强脑手消息包均已就绪。
- **采集脚本报 rclpy / 消息类型导入失败**：确认用的是 `/usr/bin/python3` 而非 Isaac Sim Python。
- **`save_data.sh --batch` 连续失败熔断**：多为桥接未就绪。先确认仿真窗口在运行、`pgrep -f tienkung_dex_ros2_zmq_bridge` 有进程；`run.sh bridge-start` 只起 Pro 桥，Dex 需从 `start_sim.sh` 启动或手动起桥接脚本。
- **轨迹走完头/臂还在追**：仿真主循环约 11 Hz（`--perf_stats` 可看详情，实时率 ≈0.11），按墙钟时长下发的轨迹换算到仿真时间会超出关节速度上限；这是时间基准问题，真机不受影响。
- **键盘遥操作/预览无窗口**：需要 X11 与 `DISPLAY`，宿主机执行 `xhost +`；无显示器时改用 `save_data.sh --batch` 或脚本化子命令。
- **真机收不到话题**：检查 `docker/fastdds_no_shm.xml` 白名单是否为实机直连网卡 IP，并确认 `ROS_DOMAIN_ID=0`。
- **上真机前**：仿真机上的消息包是按 SDK 用法重建的，真机要 source 厂商自带包；`Ctrl+C` 只停止下发，已发出的目标仍由驱动器保持，务必留硬件急停。详见 [control/tienkung_dex/README.md](../../teleoperation/control/tienkung_dex/README.md) 的「上真机前」。
