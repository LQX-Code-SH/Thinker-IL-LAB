# 天工行者真机数据「采集 → 转换 → 训练 → 评估 → 部署」工作流

> 适用机器人：**天工行者（Walker TienKung）真机**

本文档给出从**真机 HDF5 原始数据**到 **LeRobot 数据集**、**ACT 模型训练**、**离线策略评估**、最终**部署到真机**的完整流程，可复制命令。转换/训练/评估在 `ubt_IL` 容器内执行，真机部署提供「容器版」与「Jetson host 版」两种方案。

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：真机数据采集](#1-前置真机数据采集)
- [2. 启动容器（ubt_IL）](#2-启动容器ubt_il)
- [3. 数据转换（HDF5 → LeRobot）](#3-数据转换hdf5--lerobot)
  - [3.1 转换命令](#31-转换命令)
  - [3.2 数据可视化](#32-数据可视化)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（真机）](#6-模型部署真机)
  - [方案 A：远程设备部署容器（x86 工作站 → 真机）](#6a-远程设备部署容器)
  - [方案 B：机器人本体 Jetson AGX Orin 板内部署](#6b-jetson-agx-orin-板内部署)
- [7. 常见问题](#7-常见问题)

---

<a id="0-工作流总览"></a>

## 0. 工作流总览

```mermaid
真机数据采集（hdf5） --> 数据转换(LeRobot 数据集) --> 模型训练[train.sh ACT 训练] --> 策略评估[eval_policy.py 评估] --> 模型部署[rollout.sh]--> 真机部署[机器人本体]
```

---

<a id="1-前置真机数据采集"></a>

## 1. 前置：真机数据采集

目前使用Thinker Studio 遥操数采平台进行数据采集，官方提供 [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) 遥操数采平台，可进行数据采集。具体参见官网使用文档，可直接导出lerobot v3.0 数据集。也可把采集到的真机 HDF5 放到 `/ubt_IL/dataset/hdf5/`（或任意 `SRC_ROOT` 指定目录）后进入转换。

---

<a id="2-启动容器ubt_il"></a>

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次
bash run.sh start
bash run.sh bash       # 进入容器，后续转换/训练/评估命令均在容器内执行
```

---

<a id="3-数据转换hdf5--lerobot"></a>

## 3. 数据转换（HDF5 → LeRobot）

<a id="31-转换命令"></a>

### 3.1 转换命令

脚本：`/ubt_IL/scripts/convert/tienkung_pro/convert.sh`
转换配置：在`/ubt_IL/scripts/convert/tienkung_pro/configs/`下，包含字段筛选和映射关系配置，可根据训练需求选择/修改配置文件。

```bash
# 真机数据转换
SRC_ROOT=/ubt_IL/dataset/hdf5 \
TGT_PATH=/ubt_IL/dataset \
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/tienkung_pro_26d_1RGB_real.json \
REPO_ID=real_pick_place \
FPS=15 \
ROBOT_TYPE=tienkung \
TASK_NAME=real_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh

# 数据集静止帧会影响模型推理，陷入局部静止，可开启TRIM_STATIONARY=1静止帧裁剪对数据集进行处理
TRIM_STATIONARY=1 \
SRC_ROOT=/ubt_IL/dataset/hdf5 \
TGT_PATH=/ubt_IL/dataset \
CONFIG=/ubt_IL/scripts/convert/tienkung_pro/configs/tienkung_pro_26d_1RGB_real.json \
REPO_ID=real_pick_place \
FPS=15 \
ROBOT_TYPE=tienkung \
TASK_NAME=real_pick_place \
bash /ubt_IL/scripts/convert/tienkung_pro/convert.sh
```

**环境变量**（标"默认"的可省略；其余可 `bash convert.sh -h` 查看）：

| 变量 | 真机建议值 | 说明 |
|------|-----------|------|
| `SRC_ROOT` | `/ubt_IL/dataset/hdf5` | 真机 HDF5 存放根目录（每个 episode 一个含 `trajectory.hdf5` 的子目录） |
| `TGT_PATH` | `/ubt_IL/dataset`（默认） | LeRobot 数据集输出根目录 |
| `CONFIG` | `configs/tienkung_pro_26d_1RGB_real.json` | 26D 双臂，仅头部 RGB，action 映射 `master`（夹爪 invert/repeat/pad）；13D 真机模型用 `tienkung_pro_13d_1RGB.json` |
| `REPO_ID` | `real_pick_place` | 输出数据集名称（训练时保持一致） |
| `FPS` | `15` | 目标帧率 |
| `ROBOT_TYPE` | `tienkung` | 机器人类型 |
| `TASK_NAME` | `real_pick_place` | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `VCODEC` | `h264`（默认） | 视频编码器 |
| `HDF5_REL_PATH` | `trajectory.hdf5`（默认） | HDF5 相对 episode 目录的路径 |
| `TRIM_STATIONARY` | 空（关闭）/ `1` | 静止帧裁剪：静止游程 > cap 时截断（cap 默认 8 帧） |
| `STATIONARY_DIAGNOSE` | 空 / `1` | 只统计静止分布不写盘（校准阈值用） |
| `RESAMPLE_FPS` | 空（不重采样） | 目标帧率，启用重采样 |

> 多批真机数据可转换到同一 `REPO_ID` 再合并（如 `real_merged` 即为合并场景）；13D 真机模型用 `tienkung_pro_13d_1RGB.json`。

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
# 可视化合并后的天工行者抓取数据集
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id tienkung_pick_up_merged \
  --episode-index 0 \
  --root /ubt_IL/dataset/tienkung_pick_up_merged
```

![tienkung 真机数据集可视化界面](../assets/tienkung真机数据集可视化.png)

> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。

---

<a id="4-模型训练"></a>

## 4. 模型训练

训练脚本：`/ubt_IL/scripts/train/tienkung_pro/train.sh`
训练配置文件路径：`/ubt_IL/scripts/train/tienkung_pro/configs/`，根据训练需求选择/修改配置文件。

> **注意**：`train.sh` 默认 `CONFIG_PATH` 指向**仿真配置**，真机训练必须显式指定 `CONFIG_PATH`。

```bash
# 真机 26D 训练（可覆盖配置常用参数）
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_real_pick_place.json \
DATASET_REPO_ID=tienkung_pick_up_merged \
DATASET_ROOT=/ubt_IL/dataset/tienkung_pick_up_merged \
OUTPUT_DIR=/ubt_IL/model/tienkung_pick_up_merged \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh

# 断点续训（CONFIG_PATH 指向 checkpoint 内 train_config.json）
CONFIG_PATH=/ubt_IL/model/tienkung_pick_up_merged/checkpoints/last/pretrained_model/train_config.json \
RESUME=true \
bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

**不使用 train.sh**（直接调用 `lerobot-train`，覆盖参数用 `--xxx=yyy` 形式，需在 `/ubt_IL/lerobot` 目录下执行）：

```bash
# 从头训练
cd /ubt_IL/lerobot
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/train/tienkung_pro/configs/train_config_tienkung_pro_real_pick_place.json \
  --dataset.repo_id=tienkung_pick_up_merged \
  --dataset.root=/ubt_IL/dataset/tienkung_pick_up_merged \
  --output_dir=/ubt_IL/model/tienkung_pick_up_merged

# 断点续训
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/model/tienkung_pick_up_merged/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

**常用覆盖参数**（对应 `train.sh` 环境变量，未设置时沿用 config 文件内值）：

| 环境变量 | config 内默认值 | 说明 |
|------|-----|------|
| `CONFIG_PATH` | 仿真配置（`train_config_tienkung_pro_sim_pick_place.json`） | 训练配置 JSON；**真机训练必须显式指定真机配置** |
| `DATASET_REPO_ID` / `DATASET_ROOT` | `tienkung_pick_up_merged` / `/ubt_IL/dataset/tienkung_pick_up_merged` | 数据集名称 / 根目录（`DATASET_ROOT` 须为**含 `meta/` 的数据集目录本身**，与第 3 节转换 `REPO_ID` 一致） |
| `OUTPUT_DIR` | `/ubt_IL/model/tienkung_pick_up_merged` | 模型输出目录（checkpoint 在 `checkpoints/last/pretrained_model/` 下） |
| `STEPS` | `100000` | 训练步数 |
| `SAVE_FREQ` | `10000` | checkpoint 保存间隔（步） |
| `BATCH_SIZE` | `8` | 批量大小 |
| `RESUME` | `false` | 续训开关（`true` 时 `CONFIG_PATH` 指向 checkpoint 内 `train_config.json`） |

---

<a id="5-策略评估离线-mse"></a>

## 5. 策略评估（离线 MSE）

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在 LeRobot 数据集上离线推理，逐帧对比**预测动作 vs 真值动作**的 MSE，并可生成逐 episode 对比图。部署前先量化策略质量。

```bash
# 容器内执行（LeRobot venv python）
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_pick_up_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_pick_up_merged \
  --episodes 5 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_pick_up_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_pick_up_act/results.json \
  --device cuda
```

评估输出示例（逐 episode 预测 vs 真值对比）：

![天工行者真机模型离线评估曲线](../assets/tienkung真机模型离线评估曲线.png)

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

<a id="6-模型部署真机"></a>

## 6. 模型部署（真机）

#### 前置条件：
 - 真机已上电并进入半身运控模式。
 - 操作流程：机器人开机后按A质检 -> 按D回零 -> 机器人落地扶稳长按A进入站立模式 -> F下拨长按A进入半身运控模式 -> E上拨后开始下一步，机器人遥控器详细操作参考 [天工行者操作文档](https://docs.ubtrobot.com/walker-tienkung/docs/user-guide/8)。


<a id="6a-远程设备部署容器"></a>

### 方案 A：远程设备部署容器（x86 工作站 → 真机）

#### 准备工作：

 - 确认设备和机器人同网段（如 `192.168.41.x`）。确认 `ROS_DOMAIN_ID=0`，部署机能够订阅正常机器人ROS话题。
 - 机器人端启动 ImageServer 提供 JPEG 流。。

```bash
# 0. 构建容器，检查网络配置：编辑 ubt_IL/docker/fastdds_no_shm.xml 的 interfaceWhiteList，
# 可新增/修改 <address> 为本机 IP（如 192.168.41.99），保证与机器人在同一网段，随后重启容器，
# 最后容器内 echo $FASTRTPS_DEFAULT_PROFILES_FILE, cat /opt/fastdds_no_shm.xml检查网络配置。

cd ubt_IL/docker
bash run.sh build
bash run.sh start
bash run.sh bash
bash run.sh restart

# 1. 机器人端启动相机服务（仅真机部署需要）
# 注意：tienkung_pro 机型分别有三个主控板（运控 x86、Orin1、Orin2 分别为 192.168.41.1、192.168.41.2、192.168.41.3），相机需要在Orin1启动，可使用ssh nvidia@192.168.41.2 密码：nvidia 。
scp ubt_IL/scripts/deploy/tienkung_pro/image_server.py nvidia@192.168.41.2:~
ssh nvidia@192.168.41.2 'python3 image_server.py'
python3 /ubt_IL/scripts/deploy/tienkung_pro/image_client.py --server 192.168.41.2 	# 测试相机通路（默认 127.0.0.1 为仿真地址）

# 若机器人端未安装pyorbbec相机驱动请安装相关依赖包
python3 -m pip install evdev
python3 -m pip install pyorbbecsdk2
```
#### 部署推理：

脚本：`/ubt_IL/scripts/deploy/tienkung_pro/rollout.sh`

```bash
# 1. 初始化动作（抬起手臂到桌面上）——推理容器内，使用/usr/bin/python3可使用ROS环境
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/reset.py

# 2. 真机部署：ZMQ_HOST 指向真机地址（rollout.sh 默认 127.0.0.1 仿真地址，必须显式覆盖）
POLICY_PATH=/ubt_IL/model/tienkung_pick_up_act/checkpoints/100000/pretrained_model \
JOINT_CONFIG=tienkung_26 \
ZMQ_HOST=192.168.41.2 \
TASK="pick and place" \
FPS=15 \
DURATION=60 \
bash /ubt_IL/scripts/deploy/tienkung_pro/rollout.sh

# （可选）真机回放数据集动作
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /ubt_IL/dataset/tienkung_pick_up_merged --episode 0 --rate 30
```

**环境变量**：

| 变量 | 真机建议值 | 说明 |
|------|-----------|------|
| `POLICY_PATH` | `/ubt_IL/model/tienkung_pick_up_act/checkpoints/last/pretrained_model` | 训练的 ACT checkpoint |
| `JOINT_CONFIG` | `tienkung_26` | 26D；13D 模型用 `tienkung_13`，须与训练 DOF 一致 |
| `STRATEGY` | `base` | 自主执行 |
| `ZMQ_HOST` | `192.168.41.2` | **真机地址**（脚本默认 `127.0.0.1` 为仿真地址，真机必须显式覆盖） |
| `FPS` | `15` | 控制频率（与训练 fps 对齐） |
| `DURATION` | `60` | 运行时长（秒） |
| `TASK` | `pick and place` | 任务描述 |

**部署效果演示**（真机部署运行效果）：

<video src="../assets/tienkung真机部署效果.mp4" controls width="640"></video>

<a id="6b-jetson-agx-orin-板内部署"></a>

### 方案 B：机器人本体Jetson AGX Orin 板内部署

脚本：`/ubt_IL/scripts/deploy/tienkung_pro/arm_64/`（自包含部署包，详见该目录 `README.md`）

> 目标设备：Jetson AGX Orin（192.168.41.2），conda 环境 `env_vla`（Python 3.12）跑 LeRobot，系统 python3.10 跑相机/ROS2，两者 ZMQ 解耦。

```bash
# 0. 环境初始化
ssh nvidia@192.168.41.2 密码：nvidia
mkdir /home/nvidia/vla/   # 创建工作目录
# 将项目代码复制到此处
scp *项目代码* nvidia@192.168.41.2:/home/nvidia/vla/
# 构建conda环境env_vla
cd /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64
bash setup_env.sh

# 1. 机器人端启动相机服务（仅真机部署需要）
conda activate env_vla
bash /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/image_server_host.sh

# 若机器人端未安装pyorbbec相机驱动请安装相关依赖包（仅安装一次）
python3 -m pip install evdev
python3 -m pip install pyorbbecsdk2

# （可选）机器人相机预览测试
bash /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/image_client_host.sh --show

# 2. 机器人预备动作（抬起右手）
bash /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/robot_ready.sh     # 机器人准备动作

# 3. 运行推理脚本
conda activate env_vla
# 部署 26-DOF 模型（默认）
POLICY_PATH=/home/nvidia/vla/Thinker-IL-LAB/ubt_IL/model/tienkung_pick_up_act/checkpoints/100000/pretrained_model  DURATION=60 bash /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/arm_64/rollout_host.sh

# 4. （可选）数据集回放，在真机上播放采集的动作
/usr/bin/python3 /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/scripts/deploy/tienkung_pro/replay.py --dataset /home/nvidia/vla/Thinker-IL-LAB/ubt_IL/dataset/tienkung_pick_up_the_apple_all --episode 0 --rate 30
```

#### 关键参数

| 变量            | 默认值                                                                                  | 说明                                                    |
| --------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| `POLICY_PATH` | `$PROJECT_ROOT/model/tienkung_pick_up_act/checkpoints/last/pretrained_model` | ACT checkpoint                                          |
| `JOINT_CONFIG`| `tienkung_26`                                                                           | 关节 DOF 配置（`tienkung_26`=全26；`tienkung_13`=右臂7+右手6），支持自定义配置，须与训练时 DOF 一致 |
| `STRATEGY`    | `base`                                                                                 | 推理策略（`base` 自主执行；`sentry`/`highlight`/`dagger` 用于录制或交互） |
| `TASK`        | `sim_pick_place`                                                                       | 任务描述（注入 policy 的任务条件）                      |
| `ZMQ_HOST`    | `127.0.0.1`                                                                            | image_server 地址（真机相机在机器人端则改其 IP）        |
| `DURATION`    | `60`                                                                                   | 运行时长（秒）                                          |
| `FPS`         | `30`                                                                                   | 控制环频率（与训练 fps 对齐）                           |
| `DISPLAY_CAM` | `true`                                                                                 | 相机显示（SSH 无 X 设 `false`）                         |



<a id="7-常见问题"></a>

## 7. 常见问题

| 现象 | 可能原因 / 处理 |
|------|-----------------|
| 转换报缺深度流 | 真机 HDF5 无深度是正常的：必须用 `tienkung_pro_26d_1RGB_real.json`（仅 RGB）；误用 sim 配置会报缺 `camera_head_depth` |
| 转换后动作维度异常 | 真机 `master` 夹爪经 invert/repeat/pad 变换到 6 维；确认用 real 配置而非 sim 配置 |
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度：双臂 26D 中左臂锁死（commanded-but-frozen）等维度 std=0 致归一化除零；用 13D 配置转换（`tienkung_pro_13d_1RGB.json`），剔除死维度 |
| 轨迹含长静止段（前缀/尾巴） | 静止帧使模型陷入局部静止，数据转换时加 `TRIM_STATIONARY=1` 裁剪 |
| 训练报 `FileExistsError` | `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |
| 真机部署无响应 | `ZMQ_HOST` 是否指向真机 `192.168.41.2`（脚本默认 127.0.0.1）；Bridge2 是否启动（`pgrep -f ros2_deploy_bridge`）；真机主控上电联网 |
| 真机部署动作异常 | `JOINT_CONFIG` 与训练 DOF 不一致（26D↔13D）；相机通路未验证（先 `image_client.py`） |
| 真机部署动作停止 | 是否陷入局部循环，优化数据集减少停顿 |
| Jetson 推理崩溃（numpy 相关） | numpy 被升到 2.x；重新执行 `setup_env.sh` 并保持 `numpy==1.26.4` |
| Jetson conda activate 失效 | 非交互 shell 先 `source $CONDA_BASE/etc/profile.d/conda.sh` 再 activate（脚本已内置处理） |
