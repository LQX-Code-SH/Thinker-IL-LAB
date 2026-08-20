# 天工行者：模仿学习平台

> 对应代码：`ubt_IL/scripts/{convert,train,eval,deploy}`（容器内执行）
> 前置：已按 [快速开始](../getting-started.md) 克隆代码并初始化子模块；HDF5 数据已就绪（[仿真采集](sim-setup.md)或真机采集）

本页维护天工行者 转换/训练/评估/部署的**通用流程与配置参数说明**；各环节的具体命令随数据来源嵌在对应工作流章节中，按需跳转。

## 1. 构建环境（ubt_IL 容器）

容器构建流程见 [模仿学习平台 · 容器构建与使用](../il/docker.md)（`build` / `start` / `bash`，均在容器内执行后续命令）。


## 2. 数据转换（HDF5 -> LeRobot）

脚本：`/ubt_IL/scripts/convert/tienkung_pro/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。

**转换指令**（各变量的取值见下文配置表与环境变量表）：

```bash
CONFIG=<转换配置JSON> \
SRC_ROOT=<HDF5源目录> \
TGT_PATH=<LeRobot数据集输出根目录> \
REPO_ID=<输出数据集名> \
TASK_NAME=<任务名> \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
```

**示例**（跳转查看）：

- 仿真数据：[仿真全流程 · 数据转换](sim-workflow.md#3-数据转换hdf5---lerobot)
- 真机数据：[真机全流程 · 数据转换](real-workflow.md#3-数据转换hdf5---lerobot)

转换配置在 `/ubt_IL/scripts/convert/tienkung_pro/configs/` 下，包含字段筛选和映射关系配置，可根据训练需求选择/修改：

| 配置文件 | 维度 | 场景 |
|----------|------|------|
| `Tien_Kung_13_1RGB_sim.json` | 13D（右臂7+右手6） | 仿真（**推荐**，规避左臂死维度） |
| `tienkung_pro_26d_1RGB.json` | 26D 双臂 | 仿真（双臂场景） |
| `tienkung_pro_26d_1RGB_real.json` | 26D 双臂，仅 RGB | 真机（真机 HDF5 无深度流） |
| `tienkung_pro_13d_1RGB.json` | 13D | 真机 |

**常用环境变量**（标「默认」的可省略；完整列表 `bash convert.sh -h` 查看，通用说明见 [数据转换详解](../common/data-conversion.md)）：

| 变量 | 说明 |
|------|------|
| `SRC_ROOT` | HDF5 存放根目录（每个 episode 一个含 `trajectory.hdf5` 的子目录；默认 `/ubt_IL/dataset/tienkung_pro`） |
| `TGT_PATH` | LeRobot 数据集输出根目录（默认 `/ubt_IL/dataset`） |
| `CONFIG` | 转换配置 JSON（见上表） |
| `REPO_ID` | 输出数据集名称（训练时保持一致） |
| `TASK_NAME` | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `FPS` | 目标帧率（默认 `15`） |
| `ROBOT_TYPE` | 机器人类型（默认 `tienkung`） |
| `VCODEC` | 视频编码器（默认 `h264`） |
| `HDF5_REL_PATH` | HDF5 相对 episode 目录的路径（默认 `trajectory.hdf5`） |
| `TRIM_STATIONARY` | `1` 开启静止帧裁剪（见下） |
| `STATIONARY_DIAGNOSE` | `1` 只统计静止分布不写盘（校准阈值用） |
| `RESAMPLE_FPS` | 目标帧率，设置后启用重采样（默认不重采样） |

!!! tip "静止帧裁剪"
    数据集静止帧会使模型推理陷入局部静止，可开启 `TRIM_STATIONARY=1` 裁剪（静止游程超过 cap 时截断，默认 cap 8 帧）。在转换命令前加 `TRIM_STATIONARY=1 \` 即可。

**转换产物**：`/ubt_IL/dataset/<REPO_ID>/` 下生成 LeRobot v3 数据集（`meta/`、`chunk-000/`（data parquet）、`videos/`）。

## 3. 数据可视化

转换完成后用 `lerobot-dataset-viz` 检查数据集质量，训练前检查数据质量非常重要，避免盲目训练：

```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <REPO_ID> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<REPO_ID>
```

> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。

**示例**（跳转查看）：
可视化界面示例见 [仿真全流程 · 数据可视化](sim-workflow.md#32-数据可视化) / [真机全流程 · 数据可视化](real-workflow.md#32-数据可视化)。

## 4. 模型训练

训练脚本：`/ubt_IL/scripts/train/tienkung_pro/train.sh`；训练配置在 `/ubt_IL/scripts/train/tienkung_pro/configs/` 下。

**训练指令**（可选覆盖参数见下文表格，未设置时沿用配置文件内值）：

```bash
CONFIG_PATH=<训练配置JSON> \
OUTPUT_DIR=<模型输出目录> \
STEPS=<训练步数> SAVE_FREQ=<保存间隔> BATCH_SIZE=<批大小> \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

**示例**（跳转查看）：

- 仿真训练：[仿真全流程 · 模型训练](sim-workflow.md#4-模型训练)
- 真机训练：[真机全流程 · 模型训练](real-workflow.md#4-模型训练)

**不使用 train.sh**（直接调用 `lerobot-train`，需在 `/ubt_IL/lerobot` 目录下执行）：

```bash
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_sim_pick_place.json \
  --output_dir=/ubt_IL/model/tienkung_sim_pick_place_act \
  --steps=80000 --save_freq=20000 --batch_size=8
```

**常用覆盖参数**（未设置时沿用 config 文件内值）：

| 环境变量 | 说明 |
|------|------|
| `CONFIG_PATH` | 训练配置 JSON；⚠️ `train.sh` 默认指向**仿真配置**，真机训练必须显式指定真机配置 |
| `DATASET_REPO_ID` / `DATASET_ROOT` | 数据集名称 / 根目录（与转换 `REPO_ID` 一致；`DATASET_ROOT` 须为含 `meta/` 的数据集目录本身） |
| `OUTPUT_DIR` | 模型输出目录（checkpoint 在 `checkpoints/last/pretrained_model/` 下） |
| `STEPS` | 训练步数 |
| `SAVE_FREQ` | checkpoint 保存间隔（步） |
| `BATCH_SIZE` | 批量大小 |
| `RESUME` | 续训开关（`true` 时 `CONFIG_PATH` 指向 checkpoint 内 `train_config.json`） |

## 5. 策略评估（离线 MSE）

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在 LeRobot 数据集上离线推理，逐帧对比**预测动作 vs 真值动作**的 MSE，并可生成逐 episode 对比图。部署前先量化策略质量。

**评估指令**：

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path <ACT checkpoint 的 pretrained_model 目录> \
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
| `--policy-path` | 训练的 ACT checkpoint（`pretrained_model` 目录） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，模拟真实部署的 chunk 队列；默认取 `policy.n_action_steps`，`1` 为逐步推理 |
| `--plot` / `--plot-dir` | 逐 episode 预测-vs-真值对比图（PNG） |
| `--output` | 结果 JSON（summary + 逐 episode MSE） |
| `--device` | `cuda`（默认，不可用自动回退 `cpu`） |

评估通过后，进入 [§6 模型部署](#6-模型部署)。

## 6. 模型部署

脚本：`/ubt_IL/scripts/deploy/tienkung_pro/rollout.sh`，加载训练的 ACT checkpoint，按控制频率推理并经 ZMQ 将动作发给仿真器或真机。配套工具：`reset.py`（初始化/复位动作）、`replay.py`（回放数据集动作验证）、`image_client.py`（相机通路验证）。

**部署指令**（可选环境变量见下文表格，未设置时沿用脚本默认值）：

```bash
POLICY_PATH=<ACT checkpoint 的 pretrained_model 目录> \
JOINT_CONFIG=<关节 DOF 配置> \
ZMQ_HOST=<仿真器/真机地址> \
TASK=<任务描述> \
bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh
```

**示例**（跳转查看）：

- 仿真部署（回注仿真器验证，建议先做）：[仿真全流程 · 模型部署](sim-workflow.md#6-模型部署仿真)
- 真机部署（方案 A 远程容器 / 方案 B Jetson 板端 / 自定义 DOF）：[真机全流程 · 模型部署](real-workflow.md#6-模型部署真机)

**常用环境变量**：

| 变量 | 说明 |
|------|------|
| `POLICY_PATH` | 训练的 ACT checkpoint（`checkpoints/<step>/pretrained_model` 目录） |
| `JOINT_CONFIG` | 关节 DOF 配置：`tienkung_26`=全 26、`tienkung_13`=右臂 7+右手 6；支持自定义配置，**须与训练时 DOF 一致** |
| `STRATEGY` | 执行策略（`base`=自主执行） |
| `ZMQ_HOST` | 目标地址：仿真 `127.0.0.1`（默认），真机须显式覆盖（如 `192.168.41.2`） |
| `FPS` | 控制频率（与训练 fps 对齐，默认 `15`） |
| `DURATION` | 运行时长（秒，默认 `60`） |
| `TASK` | 任务描述 |

## 常见问题

| 现象 | 处理 |
|------|------|
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度（26D 中左臂锁死等）；改用 13D 配置转换，剔除死维度 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1` 裁剪静止帧 |
| 训练报 `FileExistsError` | 从头训练时 `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |
| 真机转换报缺深度流 | 真机 HDF5 无深度是正常的；必须用 `tienkung_pro_26d_1RGB_real.json`（仅 RGB） |
| 转换后动作维度异常 | 真机 `master` 夹爪经 invert/repeat/pad 变换到 6 维；确认用 real 配置而非 sim 配置 |
| 偶发不收敛 | 数据集量不足；适当增加采集次数；确认 `image_transforms` 已开启以增强泛化 |
