# 天工行者无疆 仿真数据「采集 → 转换 → 训练 → 评估 → 部署」工作流

> 适用机器人：**天工行者无疆（Walker TienKung Pro）**
> 数据来源：**Isaac Sim 仿真器采集**（`ubt_sim` 项目）
> 容器：`lerobot-tienkung`（`docker/run.sh` 构建/启动，项目挂载 `/ubt_IL`）

本文档给出TienKung2.0从**仿真 HDF5 原始数据**到 **LeRobot 数据集**、**ACT 模型训练**、**离线策略评估**、最终**仿真器部署**的完整工作流程，可快速复现。全部转换/训练/评估/部署脚本在 `ubt_IL` 容器内执行。

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：仿真数据采集（ubt_sim）](#1-前置仿真数据采集ubt_sim)
- [2. 启动容器（ubt_IL）](#2-启动容器ubt_il)
- [3. 数据转换（HDF5 → LeRobot）](#3-数据转换hdf5--lerobot)
  - [3.1 转换命令](#31-转换命令)
  - [3.2 数据可视化](#32-数据可视化)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（仿真）](#6-模型部署仿真)
- [7. 常见问题](#7-常见问题)

---

<a id="0-工作流总览"></a>

## 0. 工作流总览

```mermaid

仿真数据采集（hdf5） --> 数据转换(LeRobot 数据集) --> 模型训练[train.sh ACT 训练] --> 策略评估[eval_policy.py 评估] --> 模型部署[rollout.sh 回注仿真器]--> 仿真器部署[Isaac Sim 仿真器]

```

---

<a id="1-前置仿真数据采集ubt_sim"></a>

## 1. 前置：仿真数据采集（ubt_sim）

仿真 HDF5 数据由 **`ubt_sim`** 项目采集（详见 [ubt_sim/docs/tienkung_pro/getting-started.md](../../../ubt_sim/docs/tienkung_pro/getting-started.md)）。要点：

```bash
# ① 在 ubt_sim 容器内启动仿真（自动拉起 ROS2-ZMQ 桥接）
bash /ubt_sim/scripts/start_sim.sh          # 可加 --headless 提速采集

# ② 数据采集（系统 Python 3.10）
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung_pro/reset.py                  # 复位
bash /ubt_sim/teleoperation/control/tienkung_pro/save_data.sh                          # 批量循环采集（默认 400 次）
```

- 每次成功抓放（苹果入盘，距离 < 0.12m）才落盘，失败丢弃并以退出码 1 退出。
- 产物：`/ubt_sim/dataset/tienkung_pro/<时间戳>/trajectory.hdf5`。

采集完成后，把 HDF5 数据放到容器能访问的位置（如 `/ubt_IL/dataset/tienkung_pro/`，即第 3 节 `SRC_ROOT` 默认目录），再进入转换。

---

<a id="2-启动容器ubt_il"></a>

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次
bash run.sh start
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

---

<a id="3-数据转换hdf5--lerobot"></a>

## 3. 数据转换（HDF5 → LeRobot）

<a id="31-转换命令"></a>

### 3.1 转换命令

脚本：`/ubt_IL/scripts/convert/tienkung_pro/convert.sh`
转换配置：在`/ubt_IL/scripts/convert/tienkung_pro/configs/`下，包含字段筛选和映射关系配置，可根据训练需求选择/修改配置文件。

```bash
# 仿真数据转换
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/Tien_Kung_13_1RGB_sim.json \
SRC_ROOT=/ubt_IL/dataset/tienkung_pro \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=tienkung_sim_pick_place \
TASK_NAME=tienkung_sim_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh

# 数据集静止帧会影响模型推理，陷入局部静止，可开启TRIM_STATIONARY=1静止帧裁剪对数据集进行处理
TRIM_STATIONARY=1 \
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/Tien_Kung_13_1RGB_sim.json \
SRC_ROOT=/ubt_IL/dataset/tienkung_pro \
TGT_PATH=/ubt_IL/dataset \
REPO_ID=tienkung_sim_pick_place \
TASK_NAME=tienkung_sim_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
```

**环境变量**（标"默认"的可省略；其余可 `bash convert.sh -h` 查看）：

| 变量 | 仿真建议值 | 说明 |
|------|-----------|------|
| `SRC_ROOT` | `/ubt_IL/dataset/tienkung_pro`（默认） | 仿真 HDF5 存放根目录（每个 episode 一个含 `trajectory.hdf5` 的子目录） |
| `TGT_PATH` | `/ubt_IL/dataset`（默认） | LeRobot 数据集输出根目录 |
| `CONFIG` | `configs/Tien_Kung_13_1RGB_sim.json` | 13D 单臂+单手，1 RGB；双臂 26D 场景用 `tienkung_pro_26d_1RGB.json` |
| `REPO_ID` | `tienkung_sim_pick_place` | 输出数据集名称（训练时保持一致） |
| `FPS` | `15` | 目标帧率 |
| `ROBOT_TYPE` | `tienkung` | 机器人类型 |
| `TASK_NAME` | `tienkung_sim_pick_place` | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `VCODEC` | `h264` | 视频编码器 |
| `HDF5_REL_PATH` | `trajectory.hdf5`（默认） | HDF5 相对 episode 目录的路径 |
| `TRIM_STATIONARY` | 空（关闭）/ `1` | 静止帧裁剪：静止游程 > cap 时截断（cap 默认 8 帧） |
| `STATIONARY_DIAGNOSE` | 空 / `1` | 只统计静止分布不写盘（校准阈值用） |
| `RESAMPLE_FPS` | 空（不重采样） | 目标帧率，启用重采样 |


**转换产物**：`/ubt_IL/dataset/<REPO_ID>/` 下生成 LeRobot v3 数据集（`meta/`、`chunk-000/`（data parquet）、`videos/`）。

<a id="32-数据可视化"></a>

### 3.2 数据可视化

使用 `lerobot-dataset-viz` 在容器内可视化已转换的 LeRobot 数据集,训练前检查数据质量非常重要，避免盲目训练：

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
  --repo-id tienkung_sim_pick_place \
  --episode-index 0 \
  --root /ubt_IL/dataset/tienkung_sim_pick_place
```

![TienKung 仿真数据集可视化界面](../assets/tienkung仿真数据集可视化.png)

> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。

---

<a id="4-模型训练"></a>

## 4. 模型训练

训练脚本：`/ubt_IL/scripts/train/tienkung_pro/train.sh`
训练配置：`/ubt_IL/scripts/train/tienkung_pro/configs/`，根据训练需求选择/修改配置文件。

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

**不使用 train.sh**（直接调用 `lerobot-train`，覆盖参数用 `--xxx=yyy` 形式，需在 `/ubt_IL/lerobot` 目录下执行）：

```bash
# 从头训练
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_sim_pick_place.json \
  --output_dir=/ubt_IL/model/tienkung_sim_pick_place_act \
  --steps=80000 --save_freq=20000 --batch_size=8

# 断点续训
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/tienkung_sim_pick_place_act/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

**常用覆盖参数**（对应 `train.sh` 环境变量，未设置时沿用 config 文件内值）：

| 环境变量 | config 内默认值 | 说明 |
|------|-----|------|
| `CONFIG_PATH` | `configs/train_config_tienkung_pro_sim_pick_place.json` | 训练配置 JSON（train.sh 默认即本文件） |
| `DATASET_REPO_ID` / `DATASET_ROOT` | `tienkung_sim_pick_place` / `/ubt_IL/dataset/tienkung_sim_pick_place` | 数据集名称 / 根目录（与第 3 节转换 `REPO_ID` 一致） |
| `OUTPUT_DIR` | `/ubt_IL/model/tienkung_sim_pick_place_act` | 模型输出目录（checkpoint 在 `checkpoints/last/pretrained_model/` 下） |
| `STEPS` | `50000` | 训练步数 |
| `SAVE_FREQ` | `10000` | checkpoint 保存间隔（步） |
| `BATCH_SIZE` | `16` | 批量大小 |
| `RESUME` | `false` | 续训开关（`true` 时 `CONFIG_PATH` 指向 checkpoint 内 `train_config.json`） |



---

<a id="5-策略评估离线-mse"></a>

## 5. 策略评估（离线 MSE）

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在 LeRobot 数据集上离线推理，逐帧对比**预测动作 vs 真值动作**的 MSE，并可生成逐 episode 对比图。部署前先量化策略质量。

```bash
# 容器内执行（LeRobot venv python）
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_sim_pick_place_right13_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_sim_pick_place \
  --episodes 5 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_sim_pick_place_right13_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_sim_pick_place_right13_act/results.json \
  --device cuda

```

评估输出示例（逐 episode 预测 vs 真值对比）：

![天工行者无疆模型离线评估曲线](../assets/tienkung模型离线评估曲线.png)

**主要参数**：

| 参数 | 说明 |
|------|------|
| `--policy-path` | 训练的 ACT checkpoint（`pretrained_model` 目录） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，模拟真实部署的 chunk 队列；默认取 `policy.n_action_steps`，`1` 为逐步推理 |
| `--plot` / `--plot-dir` | 逐 episode 预测-vs-真值对比图（PNG） |
| `--output` | 结果 JSON（summary + 逐 episode MSE） |
| `--device` | `cuda`（默认，不可用自动回退 `cpu`） |

输出：控制台打印 Mean/Std/Min/Max MSE 与逐 joint MSE；`--output` 保存结果 JSON，`--plot` 保存对比图。

---

<a id="6-模型部署仿真"></a>

## 6. 模型部署（仿真）

脚本：`/ubt_IL/scripts/deploy/tienkung_pro/rollout.sh`，仿真部署使用ubt_sim模块代替机器人真机进行测试。该仿真环境与真机ROS话题部署和通信方法一致，可用于真机部署前的验证工作，避免真机动作错误造成损坏等严重后果。仿真模块容器独立运行，与模型训练推理容器在同一主机通过本地回环127.0.0.1网段进行ROS通信。仿真环境使用方法详见：[ubt_sim 天工行者无疆仿真 getting-started](../../../ubt_sim/docs/tienkung_pro/getting-started.md)。

**部署步骤**（仿真容器与推理容器分别操作）：

```bash
# 1. 启动仿真（已启动可跳过）——宿主机进仿真容器
cd ubt_sim/docker 
bash run.sh start
bash run.sh bash
bash /ubt_sim/scripts/start_sim.sh      # 容器内：启动仿真，自动拉起 ROS2-ZMQ 桥接

# 2. 启动推理容器——宿主机
cd ubt_IL/docker
bash run.sh start
bash run.sh bash

# 3. 初始化动作（抬起手臂到桌面上）——推理容器内，使用/usr/bin/python3可使用ROS环境
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/reset.py

# 部署 13-DOF 模型（右臂7+右手6，与数据转换时配置一致）
POLICY_PATH=/ubt_IL/model/tienkung_sim_pick_place_right13_act/checkpoints/last/pretrained_model \
  JOINT_CONFIG=tienkung_13 bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# （可选）验证相机通路
python /ubt_IL/scripts/deploy/tienkung_pro/image_client.py --count 60

# （可选）仿真中回放数据集动作
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /ubt_IL/dataset/tienkung_sim_pick_place --episode 0 --rate 30
```

**环境变量**：

| 变量 | 仿真建议值 | 说明 |
|------|-----------|------|
| `POLICY_PATH` | `/ubt_IL/model/tienkung_sim_pick_place_act/checkpoints/last/pretrained_model` | 训练的 ACT checkpoint |
| `JOINT_CONFIG` | `tienkung_26` | 26D；13D 模型用 `tienkung_13`，须与训练 DOF 一致 |
| `STRATEGY` | `base` | 自主执行 |
| `ZMQ_HOST` | `127.0.0.1` | **仿真器地址**（真机为 `192.168.41.2`） |
| `FPS` | `15` | 控制频率（与训练 fps 对齐） |
| `DURATION` | `60` | 运行时长（秒） |
| `TASK` | `sim pick and place` | 任务描述 |

部署后观察仿真器中机器人按策略执行抓放；可用 `reset.py` 复位、`replay.py` 回放验证。

部署效果演示：

<video controls muted loop width="100%" src="../assets/tienkung仿真部署效果.mp4"></video>

---

<a id="7-常见问题"></a>

## 7. 常见问题

| 阶段 | 现象 | 可能原因 / 处理 |
|------|------|-----------------|
| 采集 | 视频大量重复帧、帧率不足 | 减少渲染窗口摄像头数量，使用无头模式运行采集，或使用高性能多GPU设备 |
| 采集 | 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度：双臂 26D 中左臂锁死（commanded-but-frozen）、左夹爪恒 1.0，std=0 致归一化除零；用 13D 单臂配置转换（`Tien_Kung_13_1RGB_sim.json`），剔除死维度 |
| 采集 | 轨迹含长静止段（前缀/尾巴） | 静止帧使模型陷入局部静止，数据转换时加 `TRIM_STATIONARY=1` 裁剪 |
| 转换 | 报 `FileNotFoundError`（HDF5） | `SRC_ROOT` 路径错误；确认 HDF5 已从 ubt_sim 同步到容器可访问目录 |
| 训练 | 偶发不收敛 | 仿真数据集量不足；适当增加 `save_data.sh` 采集次数；确认 `image_transforms` 已开启以增强泛化 |
| 部署 | 无相机图像 | 仿真器未运行/相机流未起；用 `image_client.py` 单独验证 5558 通路 |
| 部署 | 动作卡顿 | 尝试异步推理，进行RTC轨迹融合 |
| 部署 | 动作停止 | 是否陷入局部循环，优化数据集减少停顿 |
| 部署 | 仿真器无响应 | 检查桥接是否运行，网络使用本地回环，是否连通 |
