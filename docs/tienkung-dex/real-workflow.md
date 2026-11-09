# 天工行者 Dex 真机全流程：采集 -> 转换 -> 训练 -> 评估 -> 部署

> 适用机器人：**Walker TienKung Dex（天工行者 Dex）**
> ✅ **真机已验证（2026-11）**：完整闭环已跑通——BrainCo 强脑手 13D（右臂 7 + 右手 6，30 fps）真机数据。
> 不同灵巧手真机操作步骤相同，仅手型 / `JOINT_CONFIG` / 转换配置不同。

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：真机数据采集](#1-前置真机数据采集)
- [2. 启动容器（ubt_IL）](#2-启动容器ubt_il)
- [3. 数据转换（HDF5 -> LeRobot）](#3-数据转换hdf5--lerobot)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（真机）](#6-模型部署真机)
- [7. 常见问题](#7-常见问题)

<a id="0-工作流总览"></a>

## 0. 工作流总览

```
真机采集 (trajectory.hdf5)
      │  右臂 7 + 右手 13（或强脑 6）
      ▼
scripts/convert/tienkung_dex/convert.sh  ──► dataset/tienkung_dex_real_<task>
      ▼
scripts/train/tienkung_dex/train.sh      ──► model/tienkung_dex_real_<task>_act
      ▼
scripts/eval/eval_policy.py
      ▼
robot_ready.sh --real → rollout.sh        ──► 真机执行
```

真机与仿真的训练和部署流程基本一致，使用相同的训练和部署脚本，仅数据集配置有些许差别。

<a id="1-前置真机数据采集"></a>

## 1. 前置：真机数据采集

目前使用 Thinker Studio 遥操数采平台进行数据采集，官方提供 [Thinker Studio](https://thinkercosmos.ubtrobot.com/#/studio) 遥操数采平台，可进行数据采集。具体参见官网使用文档，可直接导出 LeRobot v3.0 数据集。也可把采集到的真机 HDF5 放到 `/ubt_IL/dataset/hdf5/`（或任意 `SRC_ROOT` 指定目录）后进入转换。

**已验证的真机数据集**：`tienkung_dex_real_pick_apple_brainco_13d`（32 集 / 16956 帧 / 30 fps，`observation.state` 与 `action` 均 13 维 = 右臂 7 + BrainCo 手 6，头相机 `observation.images.head` 640×360 h264）。首轮 sim2real 验证曾沿用仿真训出的策略；真机采集足够后（≥ 30 集）即切换为真机数据直训。要混训仿真/真机数据时先补采再合并转换。

<a id="2-启动容器ubt_il"></a>

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次（x86 工作站构建即可）
bash run.sh start
bash run.sh check
bash run.sh bash
```

> 若要把策略直接跑在机器人本体（Thor）上、不建容器，见 [机器人本体部署](arm-deploy.md)。

<a id="3-数据转换hdf5--lerobot"></a>

## 3. 数据转换（HDF5 -> LeRobot）

**已验证路径：BrainCo 强脑手 13D 真机数据**（配置 `configs/tienkung_dex_13d_1RGB_brainco_real.json`，13 维 = 右臂 7 + 强脑手 6 维闭合度 0~1，头相机 `observation.images.head` 640×360，单目 RGB）：

```bash
# 真机 BrainCo 13D 数据转换（真机 HDF5 无深度是正常的）
CONFIG=/ubt_IL/scripts/convert/tienkung_dex/configs/tienkung_dex_13d_1RGB_brainco_real.json \
SRC_ROOT=/ubt_IL/dataset/hdf5/tienkung_dex_real \
REPO_ID=tienkung_dex_real_pick_apple_brainco_13d \
TASK_NAME=pick_the_apple \
FPS=30 \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

> `FPS=30` 与真机采集帧率一致（已验证数据集为 30 fps）。`TRIM_STATIONARY` 对真机数据**可以按需开启**：真机录制频率与主循环匹配，不存在仿真那种散布在全段的重复帧，裁掉首尾保持预备位的长静止段是有益的。

其他手型：

| 手型 / 数据集 | 转换配置 | 维度 |
|---|---|---|
| 因时 r13（pico6，6 维 0~1） | `tienkung_dex_real_pick_r13_1RGB.json` | 13D |
| 因时 20D（13 维 rad） | `tienkung_dex_20d_1RGB_inspire.json` | 20D |
| 强脑 BrainCo（6 维 0~1） | `tienkung_dex_13d_1RGB_brainco_real.json` | 13D（已验证） |

转换后可视化：

```bash
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id tienkung_dex_real_pick_apple_brainco_13d \
  --episode-index 0 \
  --root /ubt_IL/dataset/tienkung_dex_real_pick_apple_brainco_13d
```

<a id="4-模型训练"></a>

## 4. 模型训练

**已验证路径：BrainCo 13D 真机训练**（配置 `train_config_tienkung_dex_real_pick_apple_brainco_13d.json`，对应数据集 `tienkung_dex_real_pick_apple_brainco_13d`，产物模型 `tienkung_dex_real_pick_apple_brainco_13d_act`）：

```bash
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_dex/configs/train_config_tienkung_dex_real_pick_apple_brainco_13d.json \
OUTPUT_DIR=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act \
bash /ubt_IL/scripts/train/tienkung_dex/train.sh
```

> `train.sh` 默认 `CONFIG_PATH` 指向**仿真配置**，真机训练必须显式覆盖。`train_config_tienkung_dex_real_pick_apple.json` 是仿真配置的副本，只改了 `dataset.repo_id` / `dataset.root` / `policy.repo_id` / `output_dir` / `job_name`；真机数据的 `REPO_ID` 与上面转换时不一致时，改这几个字段即可。真机光照与仿真差异大时，可参照 `train_config_tienkung_pro_real_pick_place.json` 补 `image_transforms`（颜色抖动 / 随机仿射）再训。

其余与仿真完全一致（见 [仿真全流程 · 模型训练](sim-workflow.md#4-模型训练)）。

<a id="5-策略评估离线-mse"></a>

## 5. 策略评估（离线 MSE）

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_dex_real_pick_apple_brainco_13d \
  --episodes 5 --inference-freq 1 --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_brainco_13d_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_brainco_13d_act/results.json \
  --device cuda
```

真机数据量少时优先看逐维曲线是否符合直觉（手部抖动、手腕过冲），不要只看总 MSE。

<a id="6-模型部署真机"></a>

## 6. 模型部署（真机）

### 前置条件

1. **进入话题控制模式**：用遥控器切到半身策略（站立状态下右拨）。
2. **腿部由运控独占**：`robot_ready.py` 永远不向 `/leg/cmd` 下发；Bridge2 也只发 `/arm/cmd` 与手指令。
3. **手型一致**：`--variant inspire|brainco` 必须与实物一致（`HAND_VARIANT` / `JOINT_CONFIG` 要跟着改）。
4. **急停可用**：Bridge2 监听 `/power/board/key_status` 的 `is_estop`，为 true 时停发指令。
5. **因时 r13 模型走 6→13 展开**：`JOINT_CONFIG=tienkung_dex_inspire_13` 会自动把 `HAND_ACTION_MODE` 推导为 `pico6`——策略输出的 `right_hand` 是 **6 维归一化 0~1**（0=张开、1=闭合），Bridge2 用 `pico6_to_inspire13()` 展开成 13 个 0.1° 角码，状态回显同一 6 维指令（开环）。此时给 13 维会被判维度不匹配而跳过；给 rad 会几乎不动。

### 首轮：只跑预备位

```bash
export ROS_DOMAIN_ID=0
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh --real
```

真机下发的是 `READY_POSE`（**不是**仿真的 `READY_POSE_TARGET`——后者是"让指令=实际"的重力前馈补偿，真机用它会站成低 13 cm 的姿态）。

真机联调时已比对实际姿态与仿真画面确认 `READY_POSE` 正确。手臂自然下垂时手在 0.80 m，比桌面（0.987 m）低，所以必须四段走（`READY_CLEARANCE_POSE → READY_FOREARM_POSE → READY_UPPER_VERTICAL_POSE → final`，默认每段 5 s）。

### 部署：先短时、小限幅

**已验证路径：BrainCo 13D 真机部署**（`JOINT_CONFIG=tienkung_dex_brainco_13`，策略 13 维 = 右臂 7 + 强脑手 6 维 0~1；`FPS` 与数据集 fps 一致，30）：

```bash
# Bridge2 由 rollout.sh 自动拉起（--robot.bridge_enabled=true），不需要手动起；
# 启动前会先杀掉同机残留的 dex 桥接进程。
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 \
CAMERA_PORT=5558 IMAGE_ENABLED=1 \
FPS=30 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

部署效果演示：

<video src="../../assets/tienkung-dex真机部署效果.mp4" controls muted width="100%"></video>

### 手部量纲

| 模式 | `JOINT_CONFIG` | 策略侧 `right_hand` | 换算 |
|---|---|---|---|
| 强脑 13D | `tienkung_dex_brainco_13` | 6 维闭合度 0~1 | 动作 `code = round(1 + v*999)`；状态 `v = (code-1)/999` |
| 因时 20D | `tienkung_dex_20` | 13 维关节角 rad | Bridge2 逐通道 `inspire_rad_to_code` 转 0.1° 角码 |
| 因时 r13 | `tienkung_dex_inspire_13` | 6 维归一化 0~1 | `pico6_to_inspire13()` 展开成 13 个角码 |

> 绕过插件直接给 Bridge2 喂 `right_hand` 时：`brainco` / `pico6` 必须给 0~1；`rad13` 必须给 13 维 rad，给 0~1 会被当成 rad，手指几乎不动。

### 机器人本体部署（Thor / Jazzy，无容器）

上面的路径是「x86 工作站容器里跑策略、机器人侧只做执行」。如果希望策略直接跑在机器人本体的 ARM 板（NVIDIA Thor，Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12）上，用 [机器人本体部署](arm-deploy.md) 的 host 脚本，不需要 Docker。数据转换与训练仍建议在工作站/容器内完成，本体上只做部署。

### 常驻推理服务

```bash
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_real_pick_apple_brainco_13d_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 CAMERA_PORT=5558 IMAGE_ENABLED=1 FPS=30 \
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh

bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh status   # 等 READY
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh start
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh stop
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh shutdown
```

> Dex **不支持 `load`（热切换模型）**：服务端重建机器人的代码路径写死 `--robot.type=walker` 且不带 `--robot.cameras`，对 Dex 会重建出没有相机的机器人。换模型请 `shutdown` 后重新 `launch`。

<a id="7-常见问题"></a>

## 7. 常见问题

| 现象 | 排查 |
|------|------|
| 话题能看到但收不到数据 | 跨 distro 类型哈希不一致：两端 `ros2 topic info -v` 比对；或退到「Bridge2 跑在算力主机」方案 |
| `/arm/cmd` 发出但机器人不动 | 是否已进入话题控制模式（遥控器半身/全身策略 + 上拨）；`/power/board/key_status` 的 `is_estop` 是否为 true |
| 手臂动作幅度偏大 / 起步过猛 | `MAX_RELATIVE_TARGET` 是否设置；`DURATION` 是否还是首轮的短时长 |
| 姿态比预期低一截 | 用了仿真的 `READY_POSE_TARGET`；真机应走 `robot_ready.sh --real`（`READY_POSE`） |
| 手型明显不对 | `HAND_VARIANT` / `JOINT_CONFIG` 与实物手型不一致（因时 13 通道 ↔ 强脑 6 电机） |
| 因时手完全不动 / 日志报 `right_hand 维度不匹配` | r13 模型要用 `JOINT_CONFIG=tienkung_dex_inspire_13`（pico6，动作 6 维 0~1）；20D 老模型才用 `tienkung_dex_20`（rad13，动作 13 维 rad） |
| 拇指侧摆与预备位不一致 | pico6 展开的通道 10 由 `thumb_roll` 决定（全 0→码 1750），策略首帧会覆盖 `robot_ready` 的预备位码 850 |
| 相机无画面 | Bridge2 是否带 `image_enabled=true`；端口是否 5558；`/ob_camera_head/color/image_raw` 是否有流 |
| Bridge2 启动即退出 | 报「通道表不一致」说明插件与 Bridge2 的常量副本漂移；报消息包缺失说明 `/opt/tienkung_dex_msgs_ws` 没 source |
| 急停后机器人仍在小幅动 | Bridge2 只是停发新指令，已下发目标仍在驱动器里；按物理急停 |
