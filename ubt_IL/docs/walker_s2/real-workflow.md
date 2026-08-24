# Walker S2 Edu 探索者 真机工作流（Real-Workflow）

> 适用于 **真机数据采集 -> Walker S2 Edu 探索者 真机部署** 的完整闭环：真机采集 -> 数据转换 -> 模型训练 -> 策略评估 -> 真机部署。
> 对应的代码位于 `ubt_IL/scripts/{convert,train,deploy}/walker_s2/`。

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：真机数据采集](#1-前置真机数据采集)
- [2. 启动容器](#2-启动容器)
- [3. 数据转换（HDF5 -> LeRobot）](#3-数据转换hdf5--lerobot)
  - [3.1 转换命令](#31-转换命令)
  - [3.2 数据可视化](#32-数据可视化)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（真机）](#6-模型部署真机)
  - [6.1 推理部署](#61-推理部署)
  - [6.2 推理服务器（常驻预热/外部调用）](#62-推理服务器常驻预热外部调用)
- [7. 常见问题](#7-常见问题)

## 0. 工作流总览

```
A[真机遥操作采集] --> B[HDF5 数据转换] --> C[模型训练] --> D[离线 MSE 评估] --> E[真机部署/ 推理服务器] --> F[真机任务验证] 
```

## 1. 前置：真机数据采集

目前使用Thinker Studio 遥操数采平台进行数据采集，官方提供 [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) 遥操数采平台，可进行数据采集。具体参见官网使用文档，可直接导出lerobot v3.0 数据集。也可把采集到的真机 HDF5 放到 `/ubt_IL/dataset/hdf5/`（或任意 `SRC_ROOT` 指定目录）后进入转换。

## 2. 启动容器

训练 / 转换 / 部署均在容器 `lerobot-tienkung` 内执行：

```bash
cd ubt_IL/docker
bash run.sh build      # 首次
bash run.sh start
bash run.sh bash       # 进入容器，后续转换/训练/评估命令均在容器内执行
```

> 容器内存在双 `python` 环境，ROS相关使用`/usr/bin/python3`，lerobot相关脚本使用 `/lerobot/.venv/bin/python`（默认）。
> 真机部署时容器在 **机器人Jetson板** 上构建，构建脚本会自动识别当前主板类型构建相应的arm容器。

## 3. 数据转换（HDF5 -> LeRobot）

### 3.1 转换命令

转换脚本：[`ubt_IL/scripts/convert/walker_s2/convert.sh`](../../scripts/convert/walker_s2/convert.sh)，
核心转换器为 `ubt_IL/scripts/convert/common/convert_to_lerobot.py`，
转换配置文件路径：`/ubt_IL/scripts/convert/walker_s2/configs/`，可根据训练需求选择/修改配置文件。

**真机 10D（右臂 + 头 + 右夹爪，2 RGB）示例：**

```bash
CONFIG=/ubt_IL/scripts/convert/walker_s2/configs/walker_s2_real_10d_2RGB.json \
SRC_ROOT=/ubt_IL/dataset/walker-s2-pick-part-real-hdf5 \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=Walker_S2_pick_part_real_10_2RGB \
TASK_NAME=walker_s2_pick_part \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh

# 静止帧裁剪版本（去除抓取等待期的静止段，避免推理陷入局部静止）
TRIM_STATIONARY=1 \
CONFIG=/ubt_IL/scripts/convert/walker_s2/configs/walker_s2_real_10d_2RGB.json \
SRC_ROOT=/ubt_IL/dataset/walker-s2-pick-part-real-hdf5 \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=Walker_S2_pick_part_real_10_2RGB \
TASK_NAME=walker_s2_pick_part \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh

```

#### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `SRC_ROOT` | `/ubt_IL/dataset/walker-s2-real-data` | HDF5 源目录（真机数据默认位置） |
| `TGT_PATH` | `/ubt_IL/dataset` | LeRobot 数据集输出根目录 |
| `CONFIG` | `configs/walker_s2_real_19d_1RGBD.json` | 转换配置文件（**该默认文件仓库未提供**，真机请显式指定 `configs/` 下实际存在的配置） |
| `REPO_ID` | `Walker_S2_real_19_1RGBD` | 输出数据集 repo_id（本地目录名 + HF repo 名） |
| `ROBOT_TYPE` | `walker_s2` | 机器人类型（写入 meta） |
| `TASK_NAME` | `walker_s2_real` | 任务名（写入 meta） |
| `FPS` | `15` | 数据集帧率（`auto` 取源 HDF5 频率） |
| `VCODEC` | `h264` | 视频编码（`h264`/`jpeg`/`png`） |
| `HDF5_REL_PATH` | `hdf5/metadata_aligned.hdf5` | 源 HDF5 相对路径 |
| `RESAMPLE_FPS` | 空 | 非空则重采样到目标帧率 |
| `LABEL_ROOT` | 空 | 标签根目录（有任务标签时使用） |
| `TRIM_STATIONARY` | 空 | 非空则裁剪静止帧（`--trim-stationary`） |
| `STATIONARY_KEY` | `action` | 静止判定键名 |
| `STATIONARY_WINDOW` | 自动（`0.3 × fps`） | 静止判定窗口 |
| `STATIONARY_THRESH` | `0.03` | 静止判定阈值 |
| `STATIONARY_CAP` | `8` | 连续静止帧数上限 |
| `STATIONARY_MIN_RUN` | `3` | 有效运行段最短帧数 |
| `STATIONARY_RANGE_EPS` | `1e-3` | 静止区间容差 |

> 其他透传参数：追加 `--overwrite` 覆盖已存在数据集；`--save_one true` 单文件模式。


### 3.2 数据可视化

转换完成后，用 LeRobot 可视化工具检查数据集：

```bash
# 在容器内：
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <数据集名称> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<数据集名称>
```

示例：

```bash
# 可视化第 3.1 节转换出的数据集
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id Walker_S2_real_10_2RGB \
  --episode-index 0 \
  --root /ubt_IL/dataset/Walker_S2_real_10_2RGB
```
示例预览：
![Walker 真机数据预览](../assets/walker真机数据预览.jpg)

> 重点检查：图像分辨率、state/action 维度与顺序、grip 量程、是否存在空 episode。

## 4. 模型训练

训练脚本：`/ubt_IL/scripts/train/walker_s2/train.sh`
训练配置：`/ubt_IL/scripts/train/walker_s2/configs/`，根据训练需求选择/修改配置文件。

```bash
# 显式指定真机训练配置文件路径，或覆盖配置文件常用参数
# 单零件抓取策略：使用 pick_part 配置 + 对应数据集
CONFIG=/ubt_IL/scripts/train/walker_s2/configs/train_config_pick_part_real_10d_2RGB.json \
DATASET_REPO_ID=Wlaker_Pick_part_real_10d_2RGB \
OUTPUT_DIR=/ubt_IL/model/Wlaker_Pick_part_real_10d_2RGB_act \
STEPS=50000 SAVE_FREQ=5000 BATCH_SIZE=8 \
bash /ubt_IL/scripts/train/walker_s2/train.sh

# 断点续训（CONFIG 指向 checkpoint 内 train_config.json）
CONFIG=/ubt_IL/model/Wlaker_Pick_part_real_10d_2RGB_act/checkpoints/last/pretrained_model/train_config.json \
RESUME=true \
bash /ubt_IL/scripts/train/walker_s2/train.sh
```

**不使用 train.sh**（直接调用 `lerobot-train`，覆盖参数用 `--xxx=yyy` 形式，需在 `/ubt_IL/lerobot` 目录下执行）：

```bash
# 从头训练
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_pick_part_real_10d_2RGB.json \
  --output_dir=/ubt_IL/model/Wlaker_Pick_part_real_10d_2RGB_act \
  --steps=50000 --save_freq=5000 --batch_size=8

# 断点续训
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Wlaker_Pick_part_real_10d_2RGB_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

#### 覆盖参数

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CONFIG` | `$SCRIPT_DIR/configs/train_config_walker_s2_sim_act_10_2RGB.json` | 训练配置 JSON（**默认为仿真配置**，真机请显式指定 real 配置，如 `train_config_pick_part_real_10d_2RGB.json`） |
| `DATASET_REPO_ID` | 取 config | 数据集 repo_id |
| `DATASET_ROOT` | 取 config | 数据集根目录 |
| `OUTPUT_DIR` | 取 config | 模型输出目录 |
| `STEPS` | 取 config | 总训练步数 |
| `SAVE_FREQ` | 取 config | checkpoint 保存频率 |
| `BATCH_SIZE` | 取 config | 批大小 |
| `SEED` | 取 config | 随机种子 |
| `DEVICE` | 取 config | 训练设备（`cuda`/`cpu`） |
| `RESUME` | `false` | `true` 则从 checkpoint 恢复训练 |
| `WANDB_ENABLE` | `false` | wandb 日志开关 |

> `HF_HUB_OFFLINE=1` 默认开启（离线训练）。训练配置仅当对应环境变量**显式设置**时覆盖 config 中的值。


## 5. 策略评估（离线 MSE）

不连机器人，在 LeRobot 数据集上做离线 MSE 评估（预测 vs 真值），并生成逐 episode 对比图。
脚本（与天工行者无疆共用）：[`ubt_IL/scripts/eval/eval_policy.py`](../../scripts/eval/eval_policy.py)。

示例：
```bash
cd /ubt_IL/lerobot

/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_pick_part_real_10d_2RGB_act/checkpoints/080000/pretrained_model \
  --dataset-path /ubt_IL/dataset/Wlaker_Pick_part_real_10d_2RGB \
  --episodes 10 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_walker_pick_part_real_10d_2RGB_act \
  --output /ubt_IL/scripts/eval/output/eval_walker_pick_part_real_10d_2RGB_act/results.json \
  --device cuda
```

示例预览：
![walker 真机离线评估](../assets/walker真机离线评估.png)

| 参数 | 说明 |
|------|------|
| `--policy-path` | pretrained_model 目录（含 `config.json` + `model.safetensors`） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，中间复用预测 chunk（模拟真机部署；`1`=逐步推理） |
| `--plot` / `--plot-dir` | 生成逐 episode 预测-vs-真值图及输出目录 |
| `--output` | 结果 JSON 保存路径 |
| `--device` | `cuda` 或 `cpu`（默认 `cuda`，不可用时自动回退 `cpu`） |

## 6. 模型部署（真机）

#### 前置条件：
 - 真机上电站立并进入开发者模式。
 - 操作流程：机器人开机后打开伺服按D启动内部运控 -> 机器人落地扶稳按A进入站立模式，机器人遥控器详细操作参考[《Walker S2 Edu 探索者 二次开发文档》](./sdk-guide.md)。
 - walker真机部署在机器人vision板上运行，需将项目和模型拷贝到vision板上并构建容器和启动容器。

### 6.1 推理部署

部署脚本：[`ubt_IL/scripts/deploy/walker_s2/rollout.sh`](../../scripts/deploy/walker_s2/rollout.sh)。

**部署步骤**：

```bash
# 0. 进入开发者模式--机器人Vision板ubt容器
ssh -p 2222 ubt@192.168.11.2/3  # 输入密码：请向您的技术支持人员获取
#向service请求进入开发者模式true为进入,false为退出
ros2 service call /sys/task/developer_mode std_srvs/srv/SetBool "{data: true}"
#true为开发者模式,false则为普通模式
ros2 topic echo /sys/state/walker_mode

# 1. 构建并启动推理容器--机器人Vision板
# 1.1 拷贝项目到vision板
scp 项目代码和模型 /home/walker/
# 1.2 进入vision板并构建容器
ssh walker@192.168.11.3 # 登陆机器人Vision板,密码请向您的技术支持人员获取
cd 项目路径/ubt_IL/docker
bash run.sh build
# 1.3 启动容器
bash run.sh start
bash run.sh bash

# 2. 初始化动作（抬起手臂到桌面上）
bash /ubt_IL/scripts/deploy/walker_s2/robot_ready.sh

# 3. 部署真机 10D 模型异步推理
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/walker_pick_part_real_10d_2RGB_act/checkpoints/080000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=1 \
FPS=13 DURATION=60 \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh

# （可选）验证相机通路,可指定相机话题/机器人配置
/usr/bin/python3 scripts/deploy/walker_s2/preview_camera.py --robot walker_s2_10d
```

**部署效果演示**（真机部署运行效果）：

<video src="../assets/walker真机部署效果.mp4" controls width="640"></video>

#### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填，无默认值** | checkpoint 目录（含 `config.json`） |
| `ROBOT_MODEL` | `walker_s2_31d` | ROBOT_MODELS 注册表键（见下表） |
| `ROBOT_CONFIG` | `configs/<ROBOT_MODEL>.json` | 机器人配置文件完整路径（自定义时覆盖） |
| `ALLOW_DIM_ONLY_POLICY` | `1` | 策略无 action names 时允许仅按维度匹配 |
| `STRATEGY` | `base` | rollout 策略类型 |
| `FPS` | `13` | 控制频率 |
| `DURATION` | `30` | 运行时长（秒） |
| `TASK` | `Walker S2 Edu 探索者 rollout` | 任务描述 |
| `PREVIEW_CAMERA` | `1` | 是否显示相机预览窗口 |
| `RECORD_ACTIONS` | `1` | 是否记录 rollout 动作（`RECORD_OUTPUT_DIR` 指定输出） |
| `INFERENCE_TYPE` | `sync` | 推理引擎类型（`sync` / `act_async`） |
| `INFERENCE_HZ` | `0.6` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数（0=不截断） |
| `ZMQ_HOST` | `127.0.0.1` | 桥接地址（Bridge2 与容器同机时保持默认；跨机部署改为机器人侧 IP） |



#### 在线评估（记录推理轨迹）
模型部署时，可添加 `RECORD_ACTIONS=1` 参数，记录 rollout 动作，用于模型推理动作分析预测轨迹，机器人执行轨迹，ACT融合轨迹。记录的文件保存目录为 `ubt_IL/scripts/deploy/output` 。

![在线推理分析曲线](../assets/在线推理分析曲线.png)

#### 机器人配置（ROBOT_MODELS 注册表）

| 注册表键 | 维度 | 末端执行器 | 对应配置文件 |
|----------|------|-----------|--------------|
| `walker_s2_10d` | 10D（右臂 7 + 头 2 + 右夹爪 1） | PGC 右夹爪 | `walker_s2_gripper_19d.json`（10D 用其子集） |
| `walker_s2_19d` | 19D（17 body + 2 gripper） | PGC 1DOF 夹爪 | [`walker_s2_gripper_19d.json`](../../scripts/deploy/walker_s2/configs/walker_s2_gripper_19d.json) |
| `walker_s2_31d` | 31D（17 body + 14 hands） | V4 灵巧手 7DOF | [`walker_s2_v4_hand_31d.json`](../../scripts/deploy/walker_s2/configs/walker_s2_v4_hand_31d.json) |


### 6.2 推理服务器（常驻预热/外部调用）

推理服务器脚本：[`ubt_IL/scripts/deploy/walker_s2/inference_server.sh`](../../scripts/deploy/walker_s2/inference_server.sh)。
推理服务客户端脚本：[`ubt_IL/scripts/deploy/walker_s2/inference_client.sh`](../../scripts/deploy/walker_s2/inference_client.sh)。

`rollout.sh` 每次冷启动（policy 加载到 CUDA + 桥接拉起）耗时长。推理服务器**常驻**预热，外部经 ZMQ
指令 `start/stop/home/status/shutdown`，并可 `load` 热切换模型，用于切换 VLA 操作任务和其他任务，如：导航，搬箱。
详细使用说明见 [`推理服务器使用文档`](inference-server.md)。

> ✅ **真机已验证（2026-08-09）**：预热 ~6s 到 READY；`start`->RUNNING；`stop`/`home`（无抖动）/`shutdown` 全通过。

```bash
# 1. 拉起服务（容器内，后台预热到 READY；引擎 paused，不动机器人）
cd /ubt_IL/scripts/deploy/walker_s2
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/walker_pick_part_real_10d_2RGB_act/checkpoints/080000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=1 FPS=15 \
bash inference_server.sh

# 2. 查询推理服务状态是否 READY,如果false可等待后再次查询
bash inference_client.sh status

# 3. 开始 / 停止推理（动！）
bash inference_client.sh start
bash inference_client.sh stop

# 4. 回原点 / 关停（均回 home_position）
bash inference_client.sh home
bash inference_client.sh shutdown
```

> `start`/`home`/`shutdown` 会让机器人运动，请在机旁执行。

#### 远程拉起服务（指定参数加载模型）

在另一台机器经客户端远程拉起（SSH + docker-exec 到 **跑容器的 Jetson**，自动轮询至 READY）：

```bash
cd /ubt_IL/scripts/deploy/walker_s2
bash inference_client.sh launch \
  --policy-path /ubt_IL/model/walker_pick_part_real_10d_2RGB_act/checkpoints/080000/pretrained_model \
  --robot-model walker_s2_10d --inference-hz 2 --fps 13 \
  --remote --host 192.168.11.3 --ssh-user walker
```

> **注意区分两台机器**：
> - **Jetson `192.168.11.3`**（SSH `walker`，端口 22/2222）：跑容器 `lerobot-tienkung`、做推理。`launch --remote` 的 `--host` 指它。
> - **机器人主控 PC `192.168.11.2`**（SSH `ubt`，端口 2222）：`dev_mode.sh` 切开发者模式 / ROS2，**不跑推理容器**，别用它做 `--host`。

#### 客户端指令

```bash
bash inference_client.sh status                       # 查状态（state/model/loop_hz/chunk_id/...）
bash inference_client.sh start                        # 开始推理（engine reset+resume -> RUNNING）
bash inference_client.sh stop                         # 结束推理（engine pause -> READY，hold 当前位姿）
bash inference_client.sh home                         # 回 home_position（先 pause 再平滑 chunk）
bash inference_client.sh load --policy-path /ubt_IL/model/<other>/.../pretrained_model --robot-model walker_s2_10d
                                                       # 运行中热切换模型（整体重建含重连）
bash inference_client.sh shutdown                     # 关服务（回 home + kill 桥接 + 退出）
bash inference_client.sh watch                        # 订阅状态 PUB 流（Ctrl-C 退）
```

#### 环境变量（inference_server.sh）

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填** | pretrained_model 目录 |
| `ROBOT_MODEL` | `walker_s2_31d` | ROBOT_MODELS 注册表键 |
| `ROBOT_CONFIG` | 空 | 自定义 robot config JSON（覆盖 ROBOT_MODEL） |
| `INFERENCE_TYPE` | `act_async` | 推理引擎类型（`start`/`stop` 需 act_async） |
| `INFERENCE_HZ` | `0.6` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数 |
| `FPS` | `13` | 控制环 FPS |
| `BLEND_HORIZON` | `10` | 桥接动作块融合重叠点数 |
| `BODY_PUBLISH_HZ` | `300` | 桥接 300Hz body 发布频率 |
| `BODY_V_MAX` | `2.0` | 桥接 rate_limit 单关节最大速度 |
| `SERVER_CMD_PORT` | `5570` | ZMQ REP 指令端口 |
| `SERVER_STATUS_PORT` | `5571` | ZMQ PUB 状态端口 |
| `PREVIEW_CAMERA` | `0` | 服务器默认不开预览窗口 |

> 单例：`/tmp/walker_inference_server.pid` 在跑则拒绝重复拉起；日志：`/tmp/walker_inference_server.log`。
> 状态机：`LOADING`(预热/重建) -> `READY`(空闲) ⇄ `RUNNING`(执行)；`home` -> `RETURNING_HOME` -> `READY`。

## 7. 常见问题

| 问题 | 原因 / 解决 |
|------|-------------|
| 转换报"配置不存在" | `convert.sh` 默认 `CONFIG=configs/walker_s2_real_19d_1RGBD.json` 在仓库中未提供；请显式指定 `configs/` 下实际存在的配置（`walker_s2_real_10d_2RGB.json`） |
| 19D 真机数据无法转换 | 仓库仅提供 10D 真机转换配置；19D 需参考 `walker_s2_gripper_19d.json` 关节顺序自行编写 `walker_s2_real_19d_*.json` |
| 数据集帧率与预期不符 | 采集 15 Hz，转换默认 `FPS=15`；如需其他频率用 `FPS=auto` 取源频率，或 `RESAMPLE_FPS` 重采样 |
| 部署时报维度不匹配 | 安全预检拒绝：policy `output_features.action.shape[0]` 与 robot config `action_order` 维度不一致。确认 `ROBOT_MODEL` 与策略维度对应（10D->`walker_s2_10d`，19D->`walker_s2_19d`，31D->`walker_s2_31d`）；无 action names 的策略需 `ALLOW_DIM_ONLY_POLICY=1` |
| 推理服务器 `start` 报错 | `start/stop` 依赖 act_async 的 pause/resume；`sync` 引擎无此语义，需 `INFERENCE_TYPE=act_async` |
| `launch --remote` 失败 | 确认 `--host` 是 **Jetson `192.168.11.3`**（SSH `walker`）而非主控 PC；SSH 用户需对 `sudo docker exec` 有权限 |
| 部署时机器人不动 | 确认 Bridge2 已启动（5561/5562/5563）；`dev_mode.sh` 已切开发者模式；`status` 检查桥接连接状态 |
| 重复拉起推理服务器失败 | 单例 pid 文件 `/tmp/walker_inference_server.pid` 存在；`shutdown` 正常退出或手动清理后再拉 |
