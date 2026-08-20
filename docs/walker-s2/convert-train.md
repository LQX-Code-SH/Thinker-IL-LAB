# Walker S2 EDU 探索者：模仿学习平台

> 对应代码：`ubt_IL/scripts/{convert,train,eval,deploy}`（容器内执行）
> 前置：已按 [快速开始](../getting-started.md) 克隆代码并初始化子模块；HDF5 数据已就绪（[仿真采集](sim-setup.md)或真机采集）

本页维护 Walker S2 EDU 探索者 转换/训练/评估/部署的**通用流程与配置参数说明**；各环节的具体命令随数据来源嵌在对应工作流章节中，按需跳转。

## 1. 构建环境（ubt_IL 容器）

容器构建流程见 [模仿学习平台 · 容器构建与使用](../il/docker.md)（`build` / `start` / `bash`，均在容器内执行后续命令）。

- 容器内双 Python 环境：ROS 相关使用 `/usr/bin/python3`，LeRobot 相关脚本使用 `/lerobot/.venv/bin/python`（默认）。
- 真机部署时容器在**机器人 Vision 板（Jetson）**上构建，构建脚本自动识别主板类型构建 arm 容器。

## 2. 数据转换（HDF5 -> LeRobot）

脚本：`/ubt_IL/scripts/convert/walker_s2/convert.sh`（核心转换器 `ubt_IL/scripts/convert/common/convert_to_lerobot.py`），将 HDF5 原始数据转换为 LeRobot v3 数据集。

**转换指令**（各变量的取值见下文配置表与环境变量表）：

```bash
CONFIG=<转换配置JSON> \
SRC_ROOT=<HDF5源目录> \
TGT_PATH=<LeRobot数据集输出根目录> \
REPO_ID=<输出数据集名> \
TASK_NAME=<任务名> \
bash /ubt_IL/scripts/convert/walker_s2/convert.sh
```

**示例**（跳转查看，含静止帧裁剪版本）：

- 仿真数据：[仿真全流程 · 数据转换](sim-workflow.md#3-数据转换hdf5---lerobot)
- 真机数据：[真机全流程 · 数据转换](real-workflow.md#3-数据转换hdf5---lerobot)

转换配置在 `/ubt_IL/scripts/convert/walker_s2/configs/` 下，包含字段筛选和映射关系配置，可根据训练需求选择/修改：

| 配置文件 | 维度 | 场景 |
|----------|------|------|
| `walker_s2_sim_10d_2RGB.json` | 10D（右臂 7 + 头 2 + 右夹爪 1） | 仿真（**推荐**） |
| `walker_s2_sim_33d_4RGB.json` | 33D（含深度） | 仿真 |
| `walker_s2_real_10d_2RGB.json` | 10D | 真机（**推荐**，规避恒定维度归一化炸裂） |

!!! warning "convert.sh 默认配置仓库未提供"
    `convert.sh` 默认 `CONFIG=configs/walker_s2_real_19d_1RGBD.json` **仓库未提供**，必须显式指定 `configs/` 下实际存在的配置（见上表）。

**常用环境变量**（标「默认」的可省略；完整列表 `bash convert.sh -h`，通用说明见 [数据转换详解](../common/data-conversion.md)）：

| 变量 | 说明 |
|------|------|
| `SRC_ROOT` | HDF5 存放根目录（默认 `/ubt_IL/dataset/walker-s2-real-data`；仿真数据请改为 sim 数据目录） |
| `TGT_PATH` | LeRobot 数据集输出根目录（默认 `/ubt_IL/dataset`） |
| `CONFIG` | 转换配置 JSON（见上表，**须显式指定**） |
| `REPO_ID` | 输出数据集名称（训练时保持一致） |
| `TASK_NAME` | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `FPS` | 目标帧率（默认 `13`（sim）/ `15`（real）；`auto` 取源 HDF5 频率） |
| `ROBOT_TYPE` | 机器人类型（默认 `walker_s2`，写入 meta） |
| `HDF5_REL_PATH` | HDF5 相对 episode 目录的路径（默认 `hdf5/metadata_aligned.hdf5`；仿真数据为 `trajectory.hdf5`） |
| `VCODEC` | 视频编码器（默认 `h264`，可选 `jpeg` / `png`） |
| `TRIM_STATIONARY` | `1` 开启静止帧裁剪（见下） |
| `RESAMPLE_FPS` | 目标帧率，设置后启用重采样（默认不重采样） |

!!! tip "静止帧裁剪"
    数据集静止帧会使模型推理陷入局部静止，可在转换命令前加 `TRIM_STATIONARY=1 \` 裁剪（判定阈值/窗口/cap 等参数见 [数据转换详解](../common/data-conversion.md#静止帧裁剪)）。

> 19D 真机数据：仓库仅提供 10D 真机转换配置；19D 需参考 `walker_s2_gripper_19d.json` 关节顺序自行编写配置。

**转换产物**：`/ubt_IL/dataset/<REPO_ID>/` 下生成 LeRobot v3 数据集（`meta/`、`chunk-000/`（data parquet）、`videos/`）。

## 3. 数据可视化

转换完成后用 `lerobot-dataset-viz` 检查数据集质量，训练前检查数据质量非常重要，避免盲目训练：

**可视化指令**
```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <REPO_ID> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<REPO_ID>
```

> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。

**示例**（跳转查看）：

可视化界面示例见 [仿真全流程 · 数据可视化](sim-workflow.md#32-数据可视化) / [真机全流程 · 数据可视化](real-workflow.md#32-数据可视化)。

> 重点检查：图像分辨率、视频帧率、state/action 维度与顺序、grip 量程、是否存在空 episode。仿真 RGB 为 [3,240,320]（10D）或 [3,480,640]（33D），真机为 [256,320] 等；模型输入需与训练数据一致，勿混用。

## 4. 模型训练（ACT）

训练脚本：`/ubt_IL/scripts/train/walker_s2/train.sh`；训练配置在 `/ubt_IL/scripts/train/walker_s2/configs/` 下。

**训练指令**（可选覆盖参数见下文表格，仅显式设置时覆盖配置文件内值）：

```bash
CONFIG=<训练配置JSON> \
DATASET_REPO_ID=<数据集repo_id> \
OUTPUT_DIR=<模型输出目录> \
STEPS=<训练步数> SAVE_FREQ=<保存间隔> BATCH_SIZE=<批大小> \
bash /ubt_IL/scripts/train/walker_s2/train.sh
```

**示例**（跳转查看）：

- 仿真训练：[仿真全流程 · 模型训练](sim-workflow.md#4-模型训练)
- 真机训练：[真机全流程 · 模型训练](real-workflow.md#4-模型训练)

**不使用 train.sh**（直接调用 `lerobot-train`，需在 `/ubt_IL/lerobot` 目录下执行）：

```bash
# Smoke Test（训练前快速验证，2 步不落盘）
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_act_10_2RGB.json \
  --steps=2 --save_checkpoint=false \
  --output_dir=/ubt_IL/model/Walker_S2_sim_act_smoke

# 继续训练 + 调参
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=150000 \
  --dataset.image_transforms.enable=true      # 开启图像增强

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true \
  --steps=100000 \
  --optimizer.lr=5e-06                        # 降低学习率微调
```

**常用覆盖参数**（仅当环境变量**显式设置**时覆盖 config 中的值；`HF_HUB_OFFLINE=1` 默认开启）：

| 环境变量 | 说明 |
|------|------|
| `CONFIG` | 训练配置 JSON（默认 `configs/train_config_walker_s2_sim_act_10_2RGB.json`）；⚠️ 默认指向**仿真配置**，真机训练必须显式指定真机配置（如 `train_config_pick_part_real_10d_2RGB.json`） |
| `DATASET_REPO_ID` / `DATASET_ROOT` | 数据集名称 / 根目录（默认 `Walker_S2_sim_10_2RGB` / `/ubt_IL/dataset`；与转换 `REPO_ID` 一致） |
| `OUTPUT_DIR` | 模型输出目录（默认 `/ubt_IL/model/Walker_S2_sim_10_2RGB_act`，checkpoint 在 `checkpoints/<step>/pretrained_model/` 下） |
| `STEPS` | 训练步数（默认 `50000`） |
| `SAVE_FREQ` | checkpoint 保存间隔（默认 `5000`） |
| `BATCH_SIZE` | 批量大小（默认 `8`） |
| `SEED` | 随机种子（默认 `1000`） |
| `DEVICE` | 训练设备（默认 `cuda`） |
| `RESUME` | 续训开关（默认 `false`；`true` 时从 checkpoint 恢复训练） |
| `WANDB_ENABLE` | wandb 日志开关（默认 `false`） |

## 5. 模型训练（Pi0.5 VLA）

Pi0.5 是 Physical Intelligence 的 ~4B 参数视觉-语言-动作（VLA）模型，基于 PaliGemma-2B 视觉语言骨干 + Gemma-300M 动作专家，使用 flow matching 生成动作。

> **硬件要求**：需要 ≥24GB 显存的 GPU（如 RTX 4090）；bf16 精度下约需 16-20GB 显存（开 gradient checkpointing）。
> **预训练模型**：Pi0.5 必须从 `lerobot/pi05_base` 预训练权重初始化，无法从头训练，首次使用前需下载。

```bash
cd /ubt_IL/lerobot

# 首次训练
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/walker_s2/configs/train_config_walker_s2_sim_pi05.json

# 继续训练 + 调参
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true --steps=10000 \
  --policy.freeze_vision_encoder=false       # 解冻 vision encoder

HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/Walker_S2_sim_pi05/checkpoints/last/pretrained_model/train_config.json \
  --resume=true --steps=5000 \
  --policy.train_expert_only=true            # 仅训练 action expert（冻结 VLM）
```

关键配置（vs ACT）：

| 配置项 | ACT | Pi0.5 |
|--------|-----|-------|
| 模型规模 | ~30M | ~4B |
| 架构 | ResNet18 + Transformer | PaliGemma-2B + Gemma-300M |
| 归一化 | MEAN_STD（全部） | QUANTILES（state/action）+ IDENTITY（visual） |
| 图像尺寸 | 原始（360×640 等） | resize 到 224×224 |
| 动作预测 | VAE + absolute | Flow matching |
| 语言输入 | 无 | task 描述（需数据有 `task` 字段） |
| batch_size | 8 | 4 |
| 训练步数 | 50,000 | 5,000 |
| optimizer | AdamW (1e-5) | AdamW (2.5e-5, cosine decay + warmup) |

## 6. 策略评估（离线 MSE）

脚本（与天工共用）：`/ubt_IL/scripts/eval/eval_policy.py`，在 LeRobot 数据集上离线推理，逐帧对比**预测动作 vs 真值动作**的 MSE，并可生成逐 episode 对比图。部署前先量化策略质量。

**评估指令**：

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path <pretrained_model 目录（含 config.json + model.safetensors）> \
  --dataset-path <LeRobot 数据集目录（含 meta/info.json）> \
  --episodes <评估 episode 数> \
  --inference-freq <每 N 步推理一次> \
  --plot --plot-dir <对比图输出目录> \
  --output <结果 JSON 路径> \
  --device cuda
```

**示例**（跳转查看）：

- 仿真模型：[仿真全流程 · 策略评估](sim-workflow.md#5-策略评估离线-mse)
- 真机模型：[真机全流程 · 策略评估](real-workflow.md#5-策略评估离线-mse)

| 参数 | 说明 |
|------|------|
| `--policy-path` | pretrained_model 目录（含 `config.json` + `model.safetensors`） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，中间复用预测 chunk（模拟真机部署；`1`=逐步推理） |
| `--plot` / `--plot-dir` | 逐 episode 预测-vs-真值图及输出目录 |
| `--output` | 结果 JSON 保存路径 |
| `--device` | `cuda` 或 `cpu`（默认 `cuda`，不可用自动回退） |

评估通过后，进入 [§7 模型部署](#7-模型部署)。

## 7. 模型部署

脚本：`/ubt_IL/scripts/deploy/walker_s2/rollout.sh`，加载训练的 checkpoint，经 ROS2-ZMQ 桥接将动作发给 Isaac Sim 仿真器或真机，部署前自动做**安全预检**（action 维度匹配），通过后才允许运动。配套工具：`robot_ready.sh`（初始化/回零）、`preview_camera.py`（相机通路验证）、`inference_server.sh`（常驻预热推理服务）。

**部署指令**（可选环境变量见下文表格，未设置时沿用脚本默认值）：

```bash
POLICY_PATH=<checkpoint 的 pretrained_model 目录> \
ROBOT_MODEL=<机器人 DOF 配置> \
INFERENCE_TYPE=<推理引擎类型> \
ZMQ_HOST=<仿真桥接/机器人地址> \
TASK=<任务描述> \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

**具体部署方法**（跳转查看）：

- 仿真部署（回注仿真器验证，建议先做）：[仿真全流程 · 模型部署](sim-workflow.md#6-模型部署回注仿真)
- 真机部署（含 ROBOT_MODELS 注册表与安全预检）：[真机全流程 · 模型部署](real-workflow.md#6-模型部署真机) · [真机部署](deploy.md)
- 推理服务器（常驻预热/外部调用）：[推理服务器](inference-server.md)

**常用环境变量**：

| 变量 | 说明 |
|------|------|
| `POLICY_PATH` | **必填，无默认值**；checkpoint 目录（含 `config.json`） |
| `ROBOT_MODEL` | ROBOT_MODELS 注册表键（默认 `walker_s2_31d`；`walker_s2_10d`=右臂+头+右夹爪 10D、`walker_s2_19d`=17 body + 2 夹爪），**须与训练 DOF 一致** |
| `INFERENCE_TYPE` | 推理引擎：`sync`（默认）/ `act_async`（异步重规划，配合 `INFERENCE_HZ`） |
| `INFERENCE_HZ` | act_async 重规划频率（默认 `0.6`） |
| `STRATEGY` | rollout 策略类型（默认 `base`=自主执行） |
| `ZMQ_HOST` | 桥接地址：回注仿真 `127.0.0.1`（默认），跨机真机部署改为机器人侧 IP |
| `FPS` | 控制频率（默认 `13`，与训练 fps 对齐） |
| `DURATION` | 运行时长（秒，默认 `30`） |
| `TASK` | 任务描述 |
| `PREVIEW_CAMERA` / `RECORD_ACTIONS` | 相机预览窗口 / rollout 动作记录开关 |

完整环境变量、ROBOT_MODELS 注册表与安全预检说明见 [真机部署](deploy.md)。

## 常见问题

| 现象 | 处理 |
|------|------|
| 转换报"配置不存在" | `convert.sh` 默认 `CONFIG` 仓库未提供；显式指定 `configs/` 下实际存在的配置 |
| 真机 19D 数据无法转换 | 仓库仅提供 10D 真机转换配置；19D 需参考 `walker_s2_gripper_19d.json` 关节顺序自行编写配置 |
| 数据集帧率与预期不符 | 采集 15Hz，转换默认 `FPS=13`（真机配置为 15）；保留原频率用 `FPS=auto`，或 `RESAMPLE_FPS` 重采样 |
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 真机 19D 数据 9 维恒定所致；用 10D 配置（右臂+头+右夹爪）转换，剔除死维度 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1` 裁剪静止帧 |
| 偶发不收敛 | 数据集量不足；增加采集次数；确认 `image_transforms` 已开启 |
