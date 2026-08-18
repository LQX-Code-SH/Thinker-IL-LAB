# 天工 Pro：数据转换与训练

> 对应代码：`ubt_IL/scripts/{convert,train,eval}`（容器内执行）
> 前置：已按 [快速开始](../getting-started.md) 克隆代码并初始化子模块；HDF5 数据已就绪（[仿真采集](sim-setup.md)或真机采集）

## 1. 构建环境（ubt_IL 容器）

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile：x86 -> humble，arm64 -> humble-arm64）
bash run.sh start      # 启动容器 lerobot-tienkung
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`。
- 通信链路：LeRobot（Python 3.12）-ZMQ `5559`/`5560`-> Bridge2（系统 Python 3.10）-ROS2 DDS-> 机器人/仿真器；相机推流走 ZMQ `5558`。

## 2. 数据转换（HDF5 -> LeRobot）

脚本：`/ubt_IL/scripts/convert/tienkung_pro/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。

转换配置在 `/ubt_IL/scripts/convert/tienkung_pro/configs/` 下，包含字段筛选和映射关系配置，可根据训练需求选择/修改：

| 配置文件 | 维度 | 场景 |
|----------|------|------|
| `Tien_Kung_13_1RGB_sim.json` | 13D（右臂7+右手6） | 仿真（**推荐**，规避左臂死维度） |
| `tienkung_pro_26d_1RGB.json` | 26D 双臂 | 仿真（双臂场景） |
| `tienkung_pro_26d_1RGB_real.json` | 26D 双臂，仅 RGB | 真机（真机 HDF5 无深度流） |
| `tienkung_pro_13d_1RGB.json` | 13D | 真机 |

```bash
# 仿真数据转换示例
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/Tien_Kung_13_1RGB_sim.json \
SRC_ROOT=/ubt_IL/dataset/tienkung_pro \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=tienkung_sim_pick_place \
TASK_NAME=tienkung_sim_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh

# 真机数据转换示例（注意使用 real 配置，仅 RGB）
SRC_ROOT=/ubt_IL/dataset/hdf5 \
TGT_PATH=/ubt_IL/dataset \
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/tienkung_pro_26d_1RGB_real.json \
REPO_ID=real_pick_place \
FPS=15 \
ROBOT_TYPE=tienkung \
TASK_NAME=real_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
```

**常用环境变量**（完整列表 `bash convert.sh -h` 查看，通用说明见 [数据转换详解](../common/data-conversion.md)）：

| 变量 | 说明 |
|------|------|
| `SRC_ROOT` | HDF5 存放根目录（每个 episode 一个含 `trajectory.hdf5` 的子目录） |
| `TGT_PATH` | LeRobot 数据集输出根目录（默认 `/ubt_IL/dataset`） |
| `CONFIG` | 转换配置 JSON（见上表） |
| `REPO_ID` | 输出数据集名称（训练时保持一致） |
| `FPS` | 目标帧率（默认 15） |
| `TASK_NAME` | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `TRIM_STATIONARY` | `1` 开启静止帧裁剪（见下） |

!!! tip "静止帧裁剪"
    数据集静止帧会使模型推理陷入局部静止，可开启 `TRIM_STATIONARY=1` 裁剪（静止游程超过 cap 时截断，默认 cap 8 帧）。在转换命令前加 `TRIM_STATIONARY=1 \` 即可。

**转换产物**：`/ubt_IL/dataset/<REPO_ID>/` 下生成 LeRobot v3 数据集（`meta/`、`chunk-000/`（data parquet）、`videos/`）。

## 3. 数据可视化

转换完成后用 `lerobot-dataset-viz` 检查数据集质量，训练前检查数据质量非常重要，避免盲目训练：

```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id tienkung_sim_pick_place \
  --episode-index 0 \
  --root /ubt_IL/dataset/tienkung_sim_pick_place
```

![tienkung 仿真数据集可视化界面](../assets/tienkung仿真数据集可视化.png)

> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。

## 4. 模型训练

训练脚本：`/ubt_IL/scripts/train/tienkung_pro/train.sh`；训练配置在 `/ubt_IL/scripts/train/tienkung_pro/configs/` 下。

```bash
# 使用默认配置训练
bash /ubt_IL/scripts/train/tienkung_pro/train.sh

# 显式指定训练配置文件路径，或覆盖配置文件常用参数
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_sim_pick_place.json \
OUTPUT_DIR=/ubt_IL/model/tienkung_sim_pick_place_act \
STEPS=80000 SAVE_FREQ=20000 BATCH_SIZE=8 \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh

# 断点续训（CONFIG_PATH 指向 checkpoint 内 train_config.json）
CONFIG_PATH=/ubt_IL/model/tienkung_sim_pick_place_act/checkpoints/last/pretrained_model/train_config.json \
RESUME=true \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

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

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_sim_pick_place_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_sim_pick_place \
  --episodes 5 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_sim \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_sim/results.json \
  --device cuda
```

![天工模型离线评估曲线](../assets/tienkung模型离线评估曲线.png)

| 参数 | 说明 |
|------|------|
| `--policy-path` | 训练的 ACT checkpoint（`pretrained_model` 目录） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，模拟真实部署的 chunk 队列；默认取 `policy.n_action_steps`，`1` 为逐步推理 |
| `--plot` / `--plot-dir` | 逐 episode 预测-vs-真值对比图（PNG） |
| `--output` | 结果 JSON（summary + 逐 episode MSE） |
| `--device` | `cuda`（默认，不可用自动回退 `cpu`） |

评估通过后，进入 [仿真工作流 §6](sim-workflow.md#6-模型部署仿真) 或 [真机工作流 §6](real-workflow.md#6-模型部署真机) 部署。

## 常见问题

| 现象 | 处理 |
|------|------|
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度（26D 中左臂锁死等）；改用 13D 配置转换，剔除死维度 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1` 裁剪静止帧 |
| 训练报 `FileExistsError` | 从头训练时 `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |
| 真机转换报缺深度流 | 真机 HDF5 无深度是正常的；必须用 `tienkung_pro_26d_1RGB_real.json`（仅 RGB） |
| 转换后动作维度异常 | 真机 `master` 夹爪经 invert/repeat/pad 变换到 6 维；确认用 real 配置而非 sim 配置 |
| 偶发不收敛 | 数据集量不足；适当增加采集次数；确认 `image_transforms` 已开启以增强泛化 |
