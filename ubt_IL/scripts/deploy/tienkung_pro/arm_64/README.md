# 真机本体 ARM_64 部署流程

> 在 Jetson AGX Orin 上**不依赖 Docker**，用 conda 环境 `env_vla`（Python 3.12）运行
> Walker TienKung LeRobot 的推理 / 训练部署。本目录为自包含部署包：环境构建脚本、host 版部署脚本、
> 本地编译的 PyTorch wheel，以及本文档。
>
> - 适用目录：`ubt_IL/scripts/deploy/tienkung_pro/arm_64/`
> - 目标设备：Jetson AGX Orin（JetPack 6 / L4T R36.4.0 / Ubuntu 22.04 / glibc 2.35 / CUDA 12.6）
> - `PROJECT_ROOT` = `/home/nvidia/vla/Thinker-IL-LAB/ubt_IL`（脚本内由本目录的 `../../../..` 解析得到）
> - 设计原则：host 版脚本用 `PROJECT_ROOT` + CLI 参数覆盖原脚本里硬编码的容器路径
>   （`/ubt_IL/...`、`/lerobot/.venv/bin/...`），**不 sudo、不建软链、不改原脚本**。

---

## 1. 运行架构（双 Python 栈）

本部署包的核心是**两个 Python 栈分工**：env_vla (3.12) 跑 LeRobot 推理 / 训练，系统 python3.10 跑
只能用 cp310 的硬件相关服务（pyorbbecsdk 相机、ROS2 桥接）。两者通过 ZMQ 解耦，互不污染。

```
env_vla (conda, Python 3.12):  LeRobot + tienkung 插件 + torch
   │  lerobot-rollout 推理 / lerobot-train 训练 / image_client 通路验证
   │
   ├── 5558 ◄── ImageServer (系统 py3.10 + pyorbbecsdk) ──► 相机
   │
   └── 5559 ──► Bridge2 (系统 py3.10 + ROS2) ──ROS2 DDS──► 天工行者硬件
       5560 ◄──┘
```

| ZMQ 端口 | 方向 | 用途 | 由谁启动 |
|----------|------|------|----------|
| 5558 | ImageServer → env_vla | 相机 JPEG 帧 | `image_server_host.sh`（手动，常驻） |
| 5559 | env_vla → Bridge2 | 动作指令 (action) | 插件在 rollout 时自动拉起 Bridge2 |
| 5560 | Bridge2 → env_vla | 机器人状态 (status) | 同上 |

**两个栈的归属：**

- **env_vla (3.12)**：`rollout_host.sh`、`train_host.sh`、`image_client_host.sh`。装 torch / LeRobot / tienkung 插件，运行前需 `conda activate env_vla`。
- **系统 python3 `/usr/bin/python3` (3.10.12) + `/opt/ros/humble`**：`image_server_host.sh`、`robot_ready.sh`，以及 Bridge2（`ros2_deploy_bridge.py`，由插件用 `/usr/bin/python3` 自动启动，**无需在 env_vla 装 rclpy**）。

**state/action 为 26 维**，顺序为 `[左臂7 | 右臂7 | 左手6 | 右手6]`（由 `TienKungRobotConfig.all_joints` 决定，对应模型 action/observation 张量映射）。

---

## 2. 文件清单

| 文件 | 作用 | 运行栈 |
|------|------|--------|
| `setup_env.sh` | 一键构建 conda 环境 `env_vla`（Python 3.12 + 本地 wheel + LeRobot + tienkung 插件），末尾自检导入 | 构建器（conda） |
| `rollout_host.sh` | 策略推理部署（`lerobot-rollout`） | env_vla (3.12) |
| `train_host.sh` | ACT 模型训练（`lerobot-train`，可选） | env_vla (3.12) |
| `image_server_host.sh` | 相机图像服务：相机 → ZMQ 5558 (JPEG)，供 env_vla 侧 ImageServerCamera 连接 | 系统 python3.10 + pyorbbecsdk |
| `image_client_host.sh` | 相机通路验证客户端启动器（无界面 / 可 `--show`） | env_vla (3.12) |
| `image_client.py` | 上述客户端实现：连 ZMQ 5558 收帧、统计 fps/延迟并给结论 | env_vla (3.12) |
| `robot_ready.sh` | 机器人复位到预设位置（推理前预备动作） | 系统 python3.10 + ROS2 |
| `torch-2.7.1a0+gite2d141d-cp312-cp312-linux_aarch64.whl` | 本地编译的 PyTorch wheel（Jetson 专属） | - |
| `torchvision-0.22.1+59a3e1f-cp312-cp312-linux_aarch64.whl` | 本地编译的 torchvision wheel | - |
| `Log/` | OrbbecSDK 运行日志输出目录（运行时自动创建） | - |
| `README.md` | 本文档 | - |

---

## 3. 使用流程

### 3.1 前置条件

- 项目代码位于 `/home/nvidia/vla/Thinker-IL-LAB/`。
- 已安装 conda（miniconda / miniforge）。
- 宿主机已有 ROS2 Humble（`/opt/ros/humble`，含 rclpy、bodyctrl_msgs、cv_bridge）。
- 系统 python3.10 已装 `pyorbbecsdk2` / `cv2` / `zmq` / `numpy`（位于 `~/.local`）。缺失时补装相机相关依赖：
  ```bash
  python3 -m pip install evdev pyorbbecsdk2
  ```
- **真机部署**：把 `ubt_IL/docker/fastdds_no_shm.xml` 中的 `192.168.41.99` 改为**本机与机器人网线直连那张网卡的 IP**（`127.0.0.1` 保留），否则 ROS2 DDS 无法与真机通信。

### 3.2 构建环境（首次）

```bash
cd /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64
bash setup_env.sh          # 创建 conda env_vla (python=3.12)，装 wheel + LeRobot + 插件并自检
```

`setup_env.sh` 可用环境变量覆盖：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CONDA_BASE` | `conda info --base` 或 `~/miniconda3` | conda 安装根目录 |
| `ENV_NAME` | `env_vla` | conda 环境名 |
| `PIP_MIRROR` | 清华 TUNA | pip 镜像 |

构建完成后 `conda activate env_vla`，`which python` 应指向 env_vla（python 3.12）。

### 3.3 标准部署顺序（每次推理）

> 顺序固定：**先起相机服务 → 再起推理**。相机服务跑在系统 python3.10，需在独立终端或后台常驻。

```bash
# ① 相机服务（系统 python3.10，独立终端 / 后台常驻）
bash image_server_host.sh

# ②（可选）机器人复位到预设位置（系统 python3.10 + ROS2）
#    默认读 /tmp/tienkung_bridge_config.json（由 Bridge2 在 rollout 启动时写出）；可 --config-file 覆盖
bash robot_ready.sh

# ③ 策略推理（env_vla 3.12，新终端）
conda activate env_vla
bash rollout_host.sh
```

`rollout_host.sh` 常用覆盖：

```bash
POLICY_PATH=/home/nvidia/vla/Thinker-IL-LAB/ubt_IL/model/Pick_up_tiangong_all_act/checkpoints/last/pretrained_model \
DURATION=60 bash rollout_host.sh

# 部署 13-DOF 模型（右臂7+右手6，JOINT_CONFIG 须与训练 DOF 一致）
JOINT_CONFIG=tienkung_13 \
POLICY_PATH=/home/nvidia/vla/Thinker-IL-LAB/ubt_IL/model/sim_pick_place_right13_act/checkpoints/last/pretrained_model \
DURATION=60 bash rollout_host.sh

DISPLAY_CAM=false bash rollout_host.sh        # SSH 无 X 时关相机显示
```

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | `$PROJECT_ROOT/model/Pick_up_tiangong_all_act/checkpoints/last/pretrained_model` | ACT checkpoint |
| `JOINT_CONFIG` | `tienkung_26` | 关节 DOF 配置（`tienkung_26`=全26；`tienkung_13`=右臂7+右手6），须与训练 DOF 一致 |
| `STRATEGY` | `base` | 推理策略（`base` 自主执行；`sentry`/`highlight`/`dagger` 用于录制/交互） |
| `TASK` | `sim_pick_place` | 任务描述（注入 policy 的任务条件） |
| `ZMQ_HOST` | `127.0.0.1` | image_server 地址（真机相机在机器人上则改其 IP） |
| `DURATION` | `60` | 运行时长（秒） |
| `FPS` | `30` | 控制环频率（与训练 fps 对齐） |
| `DISPLAY_CAM` | `true` | 相机显示（SSH 无 X 设 `false`） |

### 3.4 验证相机通路

在启动 rollout 前，用 `image_client`（env_vla 侧）连 5558 收帧确认通路，无需图形界面：

```bash
bash image_client_host.sh --count 60               # 收 60 帧后退出并给结论
bash image_client_host.sh --show                   # 弹窗预览（需 X：ssh -X/-Y 或本机 DISPLAY=:0）
bash image_client_host.sh --address 192.168.41.2   # 连真机上的 image_server
```

输出 `=> 相机通路 OK ✓` 即正常。

### 3.5 模型训练（可选）

```bash
conda activate env_vla
bash train_host.sh
STEPS=100000 BATCH_SIZE=8 bash train_host.sh
```

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CONFIG_PATH` | `$PROJECT_ROOT/scripts/deploy/train_config_tiangong_all.json` | 训练配置 |
| `DATASET_ROOT` | `$PROJECT_ROOT/dataset` | LeRobotDataset root（repo_id 同名文件夹的父目录） |
| `DATASET_REPO_ID` | `Pick_up_the_apple_all` | 与实际数据集目录名一致（覆盖配置文件里的 repo_id） |
| `OUTPUT_DIR` | `$PROJECT_ROOT/model/Pick_up_tiangong_all_act` | 输出目录 |
| `STEPS` | `500000` | 训练步数 |
| `BATCH_SIZE` | `8` | 批大小 |
| `SEED` | `10000` | 随机种子 |
| `DEVICE` | `cuda` | 训练设备 |
| `HF_HUB_OFFLINE` | `1` | 离线模式 |

> **注意**：配置文件里的 `repo_id=Pick_up_tiangong_all` 与实际数据集目录名 `Pick_up_the_apple_all` 不一致；
> 脚本默认用 `DATASET_REPO_ID=Pick_up_the_apple_all` 覆盖以匹配实际目录。若自行改动需保持二者一致。

### 3.6 数据集回放（可选）

```bash
/usr/bin/python3 $PROJECT_ROOT/scripts/deploy/replay.py \
    --dataset $PROJECT_ROOT/dataset/Pick_up_the_apple_all --episode 0 --rate 30
```

---

## 4. 注意事项与排错

- **Python 3.12**：LeRobot 0.5.2 + tienkung 插件原生运行，免源码补丁；勿降级 3.10。
- **本地 wheel**：torch/torchvision 用本目录的 cp312 wheel（Jetson 专属，链接本机 glibc 2.35）。
  勿用 PyPI 的 aarch64 wheel（CPU-only、无 CUDA），也勿用需 GLIBC_2.38 的预编译 wheel（本机 2.35 会崩）。
- **numpy 必须 <2**：本地 torch 按 numpy 1.x 编译，装 2.x 会触发 "compiled using NumPy 1.x" 崩溃。
  `setup_env.sh` 已强制 `numpy==1.26.4`，勿手动升级。
- **运行栈别混**：`image_server_host.sh`、`robot_ready.sh` 走系统 python3.10，**不需要** `conda activate env_vla`；
  `rollout_host.sh`、`train_host.sh`、`image_client_host.sh` 走 env_vla (3.12)，需先 activate。
- **相机显示**：`DISPLAY_CAM=true` 与 `image_client --show` 需 X 环境；SSH 无 `DISPLAY` 时设 `false` 或用 `ssh -X/-Y`。
- **conda activate 失效**：非交互 shell 中先 `source $CONDA_BASE/etc/profile.d/conda.sh` 再 `conda activate env_vla`（脚本内已处理）。
- **磁盘**：env_vla 含 torch 约 1.5G，eMMC 紧张；pip 已全程 `--no-cache-dir`。
