# 天工行者 Dex V3 真机数据「采集 → 转换 → 训练 → 评估 → 部署」工作流

> 机型：**TienKung Dex（天工行者 Dex V3）** — 右臂 7 + 右手 13（因时 Inspire）
> 机器人：`--robot.type=tienkung_dex`，`ROS_DOMAIN_ID=0`
> 桥接：`ubt_IL/tienkung_dex/ros2_deploy_bridge.py`（Bridge2）

> ⚠️ **本轮只在仿真侧验证过**。下面标「**待确认**」的条目需要真机联调时逐条核实，
> 不要当成已验证的结论。

## 目录

- [0. 工作流总览](#0-工作流总览)
- [1. 前置：真机数据采集](#1-前置真机数据采集)
- [2. 启动容器（ubt_IL）](#2-启动容器ubt_il)
- [3. 数据转换（HDF5 → LeRobot）](#3-数据转换hdf5--lerobot)
- [4. 模型训练](#4-模型训练)
- [5. 策略评估（离线 MSE）](#5-策略评估离线-mse)
- [6. 模型部署（真机）](#6-模型部署真机)
- [7. 常见问题](#7-常见问题)

## 0. 工作流总览

```
真机采集 (trajectory.hdf5)
      │  右臂 7 + 右手 13
      ▼
scripts/convert/tienkung_dex/convert.sh  ──► dataset/tienkung_dex_real_<task>
      ▼
scripts/train/tienkung_dex/train.sh      ──► model/tienkung_dex_real_<task>_act
      ▼
scripts/eval/eval_policy.py
      ▼
robot_ready.sh --real → rollout.sh        ──► 真机执行
```

真机与仿真的差别只有三处：**目标姿态**（`--real` 下发 `READY_POSE`，仿真下发带重力前馈的
`READY_POSE_TARGET`）、**相机端口**（真机 5558 经 Bridge2 转码，仿真 5758 直连）、
**Domain ID**（0 vs 146）。其余脚本、维度、顺序完全一致。

## 1. 前置：真机数据采集

真机采集走 `ubt_sim/teleoperation` 的同名控制脚本（与仿真同一套话题），产物同样是
`<epoch>/trajectory.hdf5`，attrs 需带关节名 / fps / 手型 / task。**当前仓库没有真机 Dex 数据集**，
首轮沿用仿真训出的策略做 sim2real 验证；要混训先补采 5 集以上。

采集时不进模型的部分（左臂 / 腰 / 头 / 腿）请保持与仿真一致的预备位姿态，否则转换出来的
20 维虽然能训，部署时视角与机身姿态会对不上。

## 2. 启动容器（ubt_IL）

```bash
cd ubt_IL/docker
bash run.sh build      # 首次（x86 工作站构建即可）
bash run.sh start
bash run.sh check
bash run.sh bash
```

- 容器内 `ROS_DOMAIN_ID` 默认 `0`（真机），可用 `DOMAIN_ID=146` 覆盖。
- Dex 消息包在镜像构建期编译到 `/opt/tienkung_dex_msgs_ws`；`entrypoint.sh` 会自动 source，
  没有时会尝试用 `/ubt_IL/tienkung_dex/ros2_dex_msgs/build.sh` 兜底编译。

### 网络（**待确认**）

- 真机算力主机是 **Jazzy**，本容器是 **Humble**；消息类型名/字段一致时 FastDDS 可互通。
- `docker/fastdds_no_shm.xml` 的 `interfaceWhiteList` 需包含**工作站侧**与机器人直连网卡的
  IP（41 网段已覆盖，`127.0.0.1` 保留勿动）。**该文件会被 COPY 进镜像，改完必须重建镜像**。
- 网卡 MTU 建议设 9000。
- **上线第一步**：两端各跑 `ros2 topic info -v /arm/cmd`，比对 type name + type hash。能发现
  话题但收不到数据，就是跨 distro 类型不匹配——此时把 Bridge2 挪到算力主机（Jazzy）上跑，
  LeRobot 侧用 `ZMQ_HOST=192.168.41.2`，ZMQ 与 distro 无关。

## 3. 数据转换（HDF5 → LeRobot）

```bash
# 真机数据默认只出 RGB（真机 HDF5 无深度是正常的）
SRC_ROOT=/ubt_sim/dataset/tienkung_dex_real \
REPO_ID=tienkung_dex_real_pick_apple \
TASK_NAME=tienkung_dex_real_pick_apple \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

配置仍用 `configs/tienkung_dex_20d_1RGB.json`（20 维 / 单目头 RGB）。
真机 HDF5 的组名与仿真完全一致即可直接复用；不一致时先跑
`python3 /ubt_IL/scripts/convert/tienkung_dex/inspect_hdf5.py <root>` 看结构。

`TRIM_STATIONARY` 对真机数据**可以按需开启**：真机录制频率与主循环匹配，不存在仿真那种
散布在全段的重复帧，裁掉首尾保持预备位的长静止段是有益的。

## 4. 模型训练

```bash
CONFIG_PATH=/ubt_IL/scripts/train/tienkung_dex/configs/train_config_tienkung_dex_real_pick_apple.json \
OUTPUT_DIR=/ubt_IL/model/tienkung_dex_real_pick_apple_act \
bash /ubt_IL/scripts/train/tienkung_dex/train.sh
```

> `train.sh` 默认 `CONFIG_PATH` 指向**仿真配置**，真机训练必须显式覆盖。
> `train_config_tienkung_dex_real_pick_apple.json` 是仿真配置的副本，只改了
> `dataset.repo_id` / `dataset.root` / `policy.repo_id` / `output_dir` / `job_name`；
> 真机数据的 `REPO_ID` 与上面转换时不一致时，改这几个字段即可（`OUTPUT_DIR` 也可用环境变量覆盖）。
> 真机光照与仿真差异大时，可参照 `train_config_tienkung_pro_real_pick_place.json` 补
> `image_transforms`（颜色抖动 / 随机仿射）再训。

其余与仿真完全一致（见 [sim-workflow.md §4](./sim-workflow.md#4-模型训练)）。

## 5. 策略评估（离线 MSE）

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/tienkung_dex_real_pick_apple_act/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/tienkung_dex_real_pick_apple \
  --episodes 5 --inference-freq 1 --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_act \
  --output /ubt_IL/scripts/eval/output/eval_tienkung_dex_real_pick_apple_act/results.json \
  --device cuda
```

真机数据量少时优先看逐维曲线是否符合直觉（手部抖动、手腕过冲），不要只看总 MSE。

## 6. 模型部署（真机）

### 前置条件

1. **进入话题控制模式**：用遥控器切到半身/全身策略并上拨（**待确认**：具体按键与 `/arm/cmd`
   在非该模式下是否被忽略）。
2. **腿部由运控独占**：`robot_ready.py` 永远不向 `/leg/cmd` 下发；Bridge2 也只发 `/arm/cmd`
   与手指令。
3. **手型一致**：`--variant inspire|brainco` 必须与实物一致（`HAND_VARIANT` / `JOINT_CONFIG`
   要跟着改）。
4. **急停可用**：Bridge2 监听 `/power/board/key_status` 的 `is_estop`，为 true 时停发指令。

### 首轮：只跑预备位

```bash
export ROS_DOMAIN_ID=0
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh --real
```

真机下发的是 `READY_POSE`（**不是**仿真的 `READY_POSE_TARGET`——后者是"让指令=实际"的
重力前馈补偿，真机用它会站成低 13 cm 的姿态）。

**待确认**：`READY_POSE` 的数值来自仿真的重力前馈标定，第一次上真机请比对实际姿态与仿真
画面，确认后再上策略。手臂自然下垂时手在 0.80 m，比桌面（0.987 m）低，所以必须四段走
（`READY_CLEARANCE_POSE → READY_FOREARM_POSE → READY_UPPER_VERTICAL_POSE → final`，
默认每段 5 s）。

### 部署：先短时、小限幅

```bash
# Bridge2 由 rollout.sh 自动拉起（--robot.bridge_enabled=true），不需要手动起；
# 启动前会先杀掉同机残留的 dex 桥接进程。
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 \
CAMERA_PORT=5558 IMAGE_ENABLED=1 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

- `MAX_RELATIVE_TARGET=0.01`：单步相对目标限幅 1 cm，首轮必须加；确认方向、限位都正确后再放开。
- `IMAGE_ENABLED=1`：让 Bridge2 把头相机编成 JPEG 发到 5558（`--robot.image_enabled=true`
  会透传进 Bridge2 的配置）。
- `DURATION=10`：先 10 秒，确认动作方向和幅度无误再放长。
- **相机分辨率（待确认）**：`dex_camera` 的 `width`/`height` 实际是**裁剪窗口**——只按
  `[offset_x, offset_x+width)` 切列，`height` 仅用于声明特征形状、不参与裁剪，**任何地方都
  不做缩放**。仿真头相机正好 640×360，原样送入策略；真机 `/ob_camera_head` 若不是这个尺寸，
  必须让相机侧输出（或 Bridge2 转码）出 640×360，或至少确认裁剪后就是这个尺寸。
  ⚠️ 尺寸不对**不会报错**：ACT 的 2D 位置编码按实际特征图尺寸现算、Transformer 也不带固定
  位置表，换尺寸只是换来一串不同数量的 token——不报错，但策略看到的画面分布已经变了。

> 只想单独验证话题通路时，可以手动起 Bridge2（不跑策略）：
> `source /opt/tienkung_dex_msgs_ws/install/setup.bash && /usr/bin/python3 /ubt_IL/tienkung_dex/ros2_deploy_bridge.py`
> 然后再 `ros2 topic hz /arm/cmd` / `ros2 topic info -v /arm/cmd` 核对。手动起的这份会在
> 下次 `rollout.sh` 启动时被清掉。
>
> 不带 `--config` 手动起时，手型走默认 `inspire`，且**跳过插件↔桥接的手部通道表逐字校验**
> （那条断言只在插件自动拉起、带 `--config` 时才生效）。排查"手指令落错通道"这类问题时
> 不要用这种起法——用 `rollout.sh`，或手动补上 `--config "$(cat /tmp/tienkung_dex_bridge_config.json)"`。

> **不要把 `robot_ready.py` 和 Bridge2 同时跑**——两者都写 `/arm/cmd`。正确顺序是
> `robot_ready.sh --real` 退干净之后再起 Bridge2（`rollout.sh` 也会自己拉起 Bridge2）。

### 常驻服务

```bash
ROS_DOMAIN_ID=0 \
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 CAMERA_PORT=5558 IMAGE_ENABLED=1 \
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_server.sh

bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh status   # 等 READY
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh start
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh stop
bash /ubt_IL/scripts/deploy/tienkung_dex/inference_client.sh shutdown
```

> Dex **不支持 `load`（热切换模型）**：服务端重建机器人的代码路径写死 `--robot.type=walker`
> 且不带 `--robot.cameras`，对 Dex 会重建出没有相机的机器人。换模型请 `shutdown` 后重新 `launch`。

## 7. 常见问题

| 现象 | 排查 |
|------|------|
| 话题能看到但收不到数据 | 跨 distro 类型哈希不一致：两端 `ros2 topic info -v` 比对；或退到「Bridge2 跑在算力主机」方案 |
| `/arm/cmd` 发出但机器人不动 | 是否已进入话题控制模式（遥控器半身/全身策略 + 上拨）；`/power/board/key_status` 的 `is_estop` 是否为 true |
| 手臂动作幅度偏大 / 起步过猛 | `MAX_RELATIVE_TARGET` 是否设置；`DURATION` 是否还是首轮的短时长 |
| 姿态比预期低一截 | 用了仿真的 `READY_POSE_TARGET`；真机应走 `robot_ready.sh --real`（`READY_POSE`） |
| 手型明显不对 | `HAND_VARIANT` / `JOINT_CONFIG` 与实物手型不一致（因时 13 通道 ↔ 强脑 6 电机） |
| 相机无画面 | Bridge2 是否带 `image_enabled=true`；端口是否 5558；`/ob_camera_head/color/image_raw` 是否有流 |
| Bridge2 启动即退出 | 报「通道表不一致」说明插件与 Bridge2 的常量副本漂移；报消息包缺失说明 `/opt/tienkung_dex_msgs_ws` 没 source |
| 急停后机器人仍在小幅动 | Bridge2 只是停发新指令，已下发目标仍在驱动器里；按物理急停 |
