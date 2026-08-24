# 天工行者（Walker TienKung）仿真使用说明

## 概述

天工行者（Walker TienKung）仿真基于 NVIDIA Isaac Sim（Isaac Lab 2.2.0）构建，通过 ROS2-ZMQ 桥接将仿真状态与图像以**与真机一致的 ROS2 话题**对外发布，实现 sim-to-real 一致的遥操作与数据采集。整套环境运行在 Docker 容器内。

![天工行者仿真界面预览](../assets/tienkung-pro仿真界面预览.png)

架构（三进程，Python 不可混用）：

```
Isaac Sim (Py3.11)  ──ZMQ──►  ROS2-ZMQ Bridge (Py3.10)  ──ROS2──►  遥操作/采集脚本
```

- **仿真** `/isaac-sim/python.sh`：加载 `UBTSim-TienkungPro-Parlor-v0`，JPEG 直发 ZMQ 5558。
- **桥接** `/usr/bin/python3`：ZMQ↔ROS2 双向转发，`start_sim.sh` 自动拉起。
- **采集** `/usr/bin/python3`：发布指令、录制 15Hz HDF5。

> 仿真用 Isaac Sim Python 3.11；桥接/采集用系统 Python 3.10（rclpy 所在）。

## 仿真模块

| 模块 | 路径 | 说明 |
| --- | --- | --- |
| Docker 编排 | `docker/run.sh` | 镜像构建、容器生命周期、桥接启停 |
| 环境变量 | `docker/env.sh` | 容器名、镜像、`ROS_DOMAIN_ID` |
| 仿真启动器 | `scripts/start_sim.sh` | 识别机器人、拉起桥接、启动 `sim_runner.py` |
| 仿真入口 | `scripts/sim_runner.py` | Isaac Sim 主循环、键盘复位、profiling |
| 任务定义 | `source/ubt_sim/task/tienkung_pro_parlor/` | 客厅抓苹果场景与奖励 |
| 运行时配置 | `config/tienkung_pro/parlor.yaml` | 场景/相机/设备参数 |
| ROS2-ZMQ 桥接 | `teleoperation/bridges/tienkung_pro/` | 桥接脚本 + `bridge_config.yaml` |
| 遥操作与采集 | `teleoperation/control/tienkung_pro/` | `reset.py` / `pick_place_save_data.py` / `save_data.sh` |
| 相机客户端 | `teleoperation/image/image_client.py` | 直连 JPEG 流（5558）调试 |
| 数据输出 | `dataset/tienkung_pro/<时间戳>/trajectory.hdf5` | 采集产物 |

### ZMQ 端口（见 `bridge_config.yaml`）

| 端口 | 用途 |
| --- | --- |
| 5555 | 控制指令（bridge PUB） |
| 5556 | 机器人状态（bridge SUB） |
| 5557 | 相机原图像（C++ 桥接处理） |
| 5558 | JPEG 直连（仿真发布，无需桥接） |

## 快速开始

> 命令分两段：构建/进入容器在**宿主机**执行；启动仿真与采集在**容器内**执行（容器内根路径为 `/ubt_sim`）。

### 1. 构建并启动容器（宿主机）

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
bash run.sh bash                       # 进入容器（自动 source ROS2 环境）
bash /ubt_sim/scripts/start_sim.sh     # 启动仿真，并自动拉起 ROS2-ZMQ 桥接
```

- 仿真加载 `UBTSim-TienkungPro-Parlor-v0`（客厅抓苹果）场景。
- **按 `R` 键**可复位机器人（1 秒去抖，见 `KeyboardResetController`）。
- 关闭仿真窗口时，`start_sim.sh` 的退出陷阱会自动停止桥接进程。

运行效果（抓放任务）：

<video src="../assets/tienkung.mp4" controls muted width="100%"></video>

常用环境变量（可叠加）：

| 变量 | 作用 | 默认 |
| --- | --- | --- |
| `UBT_SIM_TASK` | 任务名（同时决定机器人类型） | `UBTSim-TienkungPro-Parlor-v0` |
| `--headless` | 无窗口运行 | 无显示器/服务器场景最常用 |
| `--perf_stats` | 打印性能统计 | 排查帧率 |
| `--device cpu\|cuda\|cuda:N` | 运行设备 | 非 load_only 换卡时用 |

### 3. 数据采集（容器内，使用系统 Python 3.10）

```bash
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung_pro/reset.py                  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung_pro/pick_place_save_data.py   # 单次抓放 + 录制
bash /ubt_sim/teleoperation/control/tienkung_pro/save_data.sh                # 批量循环（默认 400 次）
```

- **`reset.py`**：仅复位机器人位姿。
- **`pick_place_save_data.py`**：执行一次完整抓放（随机苹果位置 → 抓取 → 放置 → 归位），以 15Hz 录制双臂/双手关节、动作、头部 RGB+深度图像与时间戳；**任务成功**（苹果进盘，距离 < 0.12m）才落盘，失败则丢弃并以退出码 1 退出。
- **`save_data.sh`**：循环调用 `pick_place_save_data.py`批量采集数据集。
- 产物路径：`/ubt_sim/dataset/tienkung_pro/<时间戳>/trajectory.hdf5`。
- 采集数据集时，可使用--headless 运行提高渲染速度，提升数据集采集质量。

### 4. 相机测试（容器内）

```bash
# 测试相机直连
python3 /ubt_sim/teleoperation/image/image_client.py 
# 测试 ROS2 图像，保存3张 JPEG
/usr/bin/python3 /ubt_sim/teleoperation/image/test_ros_image.py
```

直连仿真的 JPEG 图像流（`127.0.0.1:5558`，由仿真进程直接发布，无需桥接），弹窗显示头部相机画面并打印延迟/丢帧统计。需仿真已在运行。

### 5. 数据集预览与回放（容器内）

读取 `dataset/tienkung_pro/<时间戳>/trajectory.hdf5`，预览相机+关节曲线（Rerun），或把录制动作直发回放（仿真/真机同话题）。

![天工行者数据预览](../assets/tienkung数据预览.png)

```bash
# 预览HDF5 文件
/usr/bin/python3 /ubt_sim/teleoperation/tools/playback_tienkung_pro.py \
    --episode dataset/tienkung_pro/1786638182 --mode preview --web-port 9090

# 动作回放（仿真/真机）
/usr/bin/python3 /ubt_sim/teleoperation/tools/playback_tienkung_pro.py \
    --episode dataset/tienkung_pro/1786638182 --mode control
```

完整参数与真机说明见 `teleoperation/tools/playback/README.md`。

## 常用 run.sh 命令

```text
build         构建镜像
start         创建/启动容器
stop          停止容器（先停桥接）
restart       重启容器
bash          进入容器 shell（自动 source ROS2）
rm            删除容器
init          容器内安装依赖、构建 ROS2 消息与 C++ 图像桥接
check         校验环境
bridge-start  单独启动 ROS2-ZMQ 桥接
bridge-stop   单独停止桥接
```

## 故障排查

- **`bash run.sh check` 报错**：按提示重跑 `bash run.sh init`；确认 `numpy<2`、`bodyctrl_msgs`、Walker SDK ROS2 消息包均已就绪。
- **采集脚本报 rclpy / 消息类型导入失败**：确认用的是 `/usr/bin/python3` 而非 Isaac Sim Python；确认已 `bash run.sh init` 构建消息包。
- **`save_data.sh` 连续失败熔断**：多为桥接未就绪。先确认仿真窗口在运行、桥接进程存在（`pgrep -f tienkung_pro_ros2_zmq_bridge`），或单独 `bash run.sh bridge-start`。
- **X11 不可用 / 仿真窗口不显示**：宿主机执行 `xhost +`，确认 `DISPLAY` 已透传；无显示器时仅可用 `UBT_SIM_LOAD_ONLY=1` 预览。
