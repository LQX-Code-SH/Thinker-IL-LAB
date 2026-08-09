# UBTECH Robot — LeRobot 部署指南

天工与 Walker S2 机器人 LeRobot 部署插件。支持 `lerobot-rollout` CLI 部署真机。

> 技术架构、端口定义、ROS2 话题、26/31 维向量格式等参见 [CLAUDE.md](./CLAUDE.md)。

## 目录

1. [快速开始](#快速开始)
2. [数据转换](#数据转换)
3. [模型训练](#模型训练)
4. [模型部署](#模型部署)

## 1. 快速开始

构建镜像并启动容器：

```bash
# 构建镜像 + 启动容器（自动启动 Bridge2）
cd docker
bash run.sh build && bash run.sh start  && bash run.sh check
bash run.sh bash           # 进入容器
```
容器启动后 `entrypoint.sh` 自动安装 lerobot 和天工插件（editable 模式），约 30 秒。查看日志确认完成：
```bash
sudo docker logs -f lerobot-tienkung  # 看到 "Installing lerobot-robot-tienkung plugin..." 即完成
# 其他容器操作
bash run.sh stop           # 停止容器
bash run.sh restart        # 重启容器
bash run.sh rm             # 删除容器
```
---

### 环境变量

环境变量集中定义在 `docker/env.sh`，可通过宿主机环境变量覆盖：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DOMAIN_ID` | `0` | ROS_DOMAIN_ID，真机默认 0 |

### 容器dds配置

真机部署请配置`ubt_IL/docker/fastdds_no_shm.xml`中的IP地址，该配置影响容器与真机的ROS通信，若无法通过ROS控制真机请修改后重启容器：

```bash
<address>127.0.0.1</address>      # 同机部署 IP走本地回环 
<address>192.168.41.99</address>  # 天工/默认远程真机网线直连，需设置为本机 IP
<address>192.168.11.99</address>  # Walker S2 直连网段，需设置为本机 IP
```
---

## 2. 数据转换

将 HDF5 采集数据转换为 LeRobot 训练格式。

### 用法

```bash
# 默认参数（仿真数据，tienkung_pro_26d_1RGB 配置）
bash scripts/convert/tienkung_pro/convert.sh

# 指定数据/输出目录
SRC_ROOT=path/to/hdf5_episodes TGT_PATH=path/to/output REPO_ID=my_dataset \
  bash scripts/convert/tienkung_pro/convert.sh

# 或直接调用 Python 脚本获得更细粒度控制
python scripts/convert/common/convert_to_lerobot.py \
  --config scripts/convert/tienkung_pro/configs/tienkung_pro_26d_1RGB.json \
  --src_root path/to/hdf5_episodes --tgt_path path/to/output \
  --repo_id my_dataset --fps 15 --robot_type tienkung --task_name pick_and_place
```

主要参数（完整列表见 `convert_to_lerobot.py --help`）：

| 参数 | 环境变量 | 默认值 | 说明 |
|------|----------|--------|------|
| `--config` | `CONFIG` | `tienkung_pro_26d_1RGB.json` | 特征映射配置文件 |
| `--src_root` | `SRC_ROOT` | `dataset/hdf5` | HDF5 源数据目录 |
| `--tgt_path` | `TGT_PATH` | `/ubt_IL/dataset` | 输出父目录 |
| `--repo_id` | `REPO_ID` | `real_pick_place` | 数据集名称 |

### 配置文件

配置文件位于 `scripts/convert/<机型>/configs/`，定义 HDF5 → LeRobot 特征映射：

| 配置文件 | 场景 | 维度 | 说明 |
|----------|------|------|------|
| `tienkung_pro_26d_1RGB.json` | 仿真 | 26 | action 从 `action/` 读取 |
| `tienkung_pro_26d_1RGB_real.json` | 真机 | 26 | action 从 `master/` 读取，含灵巧手 invert/padding |
| `tienkung_pro_gello_16d_1RGB.json` | Gello | 16 | 关节空间，单相机 |
| `walker_s2_real_19d_1RGBD.json` | Walker S2 真机 | 19 | 17D body/head/waist + 左右 1D PGC 夹爪，RGB camera_head |

### Walker S2 真机 HDF5 批量转换（19D）

容器内运行。源目录可传单个 episode，也可传整个 `walker-s2-real-data` 根目录；传根目录时脚本会批量扫描每个子目录下的 `hdf5/metadata_aligned.hdf5` 并合并成一个 LeRobot v3.0 数据集。

转换前可先检查可转换 episode 数量：

```bash
find /ubt_IL/dataset/walker-s2-real-data \
  -maxdepth 3 \
  -type f \
  -name metadata_aligned.hdf5 \
  -print | wc -l
```

批量转换全部 Walker S2 真机 HDF5（便捷封装：`bash /ubt_IL/scripts/convert/walker_s2/convert.sh`，等价于下方命令）：

```bash
PYTHONPATH=/ubt_IL/lerobot/src \
python /ubt_IL/scripts/convert/walker_s2/convert_real_to_lerobot_v3.py \
  --config /ubt_IL/scripts/convert/walker_s2/configs/walker_s2_real_19d_1RGBD.json \
  --src_root /ubt_IL/dataset/walker-s2-real-data \
  --tgt_path /ubt_IL/dataset \
  --repo_id Walker_S2_real_19_1RGBD \
  --task_name walker_s2_real \
  --robot_type walker_s2 \
  --fps 12.5 \
  2>&1 | tee /ubt_IL/dataset/Walker_S2_real_19_1RGBD_conversion.log
```

输出目录：

```bash
/ubt_IL/dataset/Walker_S2_real_19_1RGBD
```

单条 episode 测试转换示例：

```bash
PYTHONPATH=/ubt_IL/lerobot/src \
python /ubt_IL/scripts/convert/walker_s2/convert_real_to_lerobot_v3.py \
  --config /ubt_IL/scripts/convert/walker_s2/configs/walker_s2_real_19d_1RGBD.json \
  --src_root /ubt_IL/dataset/walker-s2-real-data/20260629_150738_task_1782716590_1691 \
  --tgt_path /ubt_IL/dataset \
  --repo_id Walker_S2_real_19_1RGBD_test \
  --task_name walker_s2_real \
  --robot_type walker_s2 \
  --fps auto \
  --save_one true
```

`--fps` 支持 `auto` 或显式数值，例如 `--fps 12.5`、`--fps 15`、`--fps 30`。注意：显式改成 `--fps 30` 只会把 LeRobot timestamp 和视频编码标为 30Hz，不会对原始约 12.5Hz 的采集数据做插值/补帧；帧数不变，动作节奏会被压快。若要保持真实采集时序，推荐使用 `--fps 12.5` 或 `--fps auto`。

如果输出目录已存在，脚本默认拒绝覆盖；确认要重跑并替换旧输出时再加：

```bash
--overwrite
```

转换后检查：

```bash
python - <<'PY'
import json
from pathlib import Path

root = Path("/ubt_IL/dataset/Walker_S2_real_19_1RGBD")
info = json.loads((root / "meta/info.json").read_text())

print("codebase_version:", info["codebase_version"])
print("fps:", info["fps"])
print("robot_type:", info["robot_type"])
print("total_episodes:", info["total_episodes"])
print("total_frames:", info["total_frames"])
print("state shape:", info["features"]["observation.state"]["shape"])
print("action shape:", info["features"]["action"]["shape"])
print("image shape:", info["features"]["observation.images.camera_head"]["shape"])
PY
```

### 数据可视化

使用 `lerobot-dataset-viz` 在容器内可视化已转换的 LeRobot 数据集：

```bash
bash run.sh bash
# 在容器内：
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id <数据集名称> \
  --episode-index 0 \
  --root /ubt_IL/dataset/<数据集名称>
```

示例：

```bash
# 可视化测试数据集
HF_HUB_OFFLINE=1 lerobot-dataset-viz \
  --repo-id test_pick_place \
  --episode-index 0 \
  --root /ubt_IL/dataset/test_pick_place
```
> **注意**：`--root` 须指向包含 `meta/` 目录的数据集路径（即 `repo_id` 目录本身），而非父目录。`HF_HUB_OFFLINE=1` 用于禁止访问 HuggingFace Hub。


### 真机回放数据集 action（replay.py）

将数据集的 action 原样发回真机/仿真。容器内运行，先跑 `reset.py` 复位。

```bash
# --rate 需匹配数据集 fps：真机 30，仿真 15
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/replay.py \
  --dataset /ubt_IL/dataset/real_grasp_bottle --episode 0 --rate 30
```

常用参数：`--episode N`、`--rate Hz`、`--start/--end` 限帧、`--dry-run` 只打印不发。首次试跑可加 `--rate 10 --end 100` 低速验证前 100 帧。

依赖：`/usr/bin/python3 -m pip install --user pyarrow pandas`。


---

## 3. 模型训练

在容器内运行：

```bash
# 默认配置（50k 步，ACT 模型）
bash /ubt_IL/scripts/train/tienkung_pro/train.sh

# 自定义参数
DATASET_ROOT=/ubt_IL/dataset/my_data \
STEPS=10000 \
OUTPUT_DIR=/ubt_IL/model/my_model \
  bash /ubt_IL/scripts/train/tienkung_pro/train.sh
```

可覆盖的环境变量：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DATASET_ROOT` | `/ubt_IL/dataset/real_merged` | 数据集路径 |
| `OUTPUT_DIR` | `/ubt_IL/model/real_pick_place_act` | 模型输出路径 |
| `STEPS` | `50000` | 训练总步数 |
| `BATCH_SIZE` | `8` | 批次大小 |
| `DEVICE` | `cuda` | 训练设备 |
| `SEED` | `1000` | 随机种子 |

Checkpoint 每 10,000 步保存一次，模型路径为 `{OUTPUT_DIR}/checkpoints/last/pretrained_model`。

---

## 4. 模型部署

### 前置条件

- 真机/仿真，`ROS_DOMAIN_ID=0`
- 真机部署请保证本机IP 与机器人 IP 在统一网段：例如192.168.41.99
- ImageServer 已部署到机器人端（仿真部署可跳过）：

```bash
# 将 image_server.py 传到机器人端并启动
scp ubt_IL/scripts/deploy/tienkung_pro/image_server.py nvidia@192.168.41.2:~
# 机器人端执行：
python3 image_server.py

# 验证图像流（容器内运行）
python3 ubt_IL/scripts/deploy/tienkung_pro/image_client.py
```

### 部署步骤

```bash
# 1. 机器人复位
bash run.sh bash
/usr/bin/python3 /ubt_IL/scripts/deploy/tienkung_pro/reset.py

# 2. 启动推理（推荐脚本方式）
ROBOT_MODEL=walker_s2_10d POLICY_PATH=/ubt_IL/model/Pick_part_real_10d_2RGB_act/checkpoints/100000/pretrained_model INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=15 DURATION=30 bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

也可使用 `lerobot-rollout` CLI 直接调用（真机需修改 `server_address` 为机器人实际 IP）：

```bash
/lerobot/.venv/bin/lerobot-rollout \
    --policy.path=/ubt_IL/model/test_model \
    --robot.type=tienkung --robot.bridge_enabled=true \
    --robot.cameras="{camera_head: {type: image_server, server_address: '192.168.41.2', port: 5558, width: 640, height: 360, fps: 15, display: true}}" \
    --task="pick and place" --fps=15 --duration=60
```

### Walker S2 部署（P0 基础迁移）

Walker S2 使用独立插件与 Bridge2：

- 插件目录：`/ubt_IL/walker/lerobot_robot_walker`
- Bridge2：`/ubt_IL/walker/ros2_walker_bridge.py`
- ROS2 SDK/messages：`/ubt_IL/walker/walker_sdk_ros2`
- ZMQ 端口：`5561` action、`5562` state、`5563` image
- 相机链路：Walker ROS2 `shm_msgs` → Bridge2 JPEG relay → `walker_camera`

容器启动时 `entrypoint.sh` 会尝试构建 Walker ROS2 messages，并安装 `lerobot_robot_walker` 插件。也可用环境检查确认：

```bash
cd docker
bash run.sh check
```

Walker 专用 rollout 入口，支持 19D（PGC 夹爪）和 31D（V4 灵巧手）两种配置：

```bash
# 19D PGC 夹爪模型（需要 ALLOW_DIM_ONLY_POLICY=1，见下方说明）
ROBOT_MODEL=walker_s2_gripper_19d \
ALLOW_DIM_ONLY_POLICY=1 \
POLICY_PATH=/ubt_IL/model/Walker_S2_real_19_1RGBD_act/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh

# 31D V4 灵巧手模型
ROBOT_MODEL=walker_s2_v4_hand_31d \
POLICY_PATH=/ubt_IL/model/<walker_31dim_policy>/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

部署参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_MODEL` | `walker_s2_v4_hand_31d` | 机器人模型：`walker_s2_gripper_19d` 或 `walker_s2_v4_hand_31d` |
| `POLICY_PATH` | (必填) | 模型 checkpoint 路径，指向 `pretrained_model/` 目录 |
| `ALLOW_DIM_ONLY_POLICY` | `0` | 19D 模型缺少 `action_feature_names` 时须设为 `1` |
| `JOINT_CONFIG` | `walker_s2_31d` | DOF 配置（`walker_s2_31d`=全31；`walker_s2_19d`=19D夹爪；`walker_s2_10d`=右臂+头+右夹爪） |
| `DURATION` | `30` | 运行时长（秒） |
| `FPS` | `15` | 推理帧率 |
| `STRATEGY` | `base` | 部署策略类型 |
| `PREVIEW_CAMERA` | `1` | 是否启动相机预览窗口 |

> **19D 模型注意**：当前 19D 模型 `config.json` 的 `output_features.action` 中不包含 `names` 字段，`rollout.sh` 默认拒绝仅靠维度匹配的部署。设置 `ALLOW_DIM_ONLY_POLICY=1` 前需确认训练数据 action 顺序与 `walker_s2_gripper_19d.json` 的 `action_order` 一致。
>
> **JOINT_CONFIG**：Walker 支持可变自由度部署（19D/10D/31D），通过 `JOINT_CONFIG` 环境变量选择。子集策略（如仅右臂的 10D）部署时，非激活关节自动用 `READY_POSE` / 安全位姿填充。在 `ubt_IL/walker/lerobot_robot_walker/lerobot_robot_walker/constants.py` 中注册新 `IntEnum` 即可新增自定义 DOF。

### 相机 & 部署参数 (TienKung)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `server_address` | `127.0.0.1` | ImageServer ZMQ 地址（真机改为机器人 IP） |
| `port` | `5558` | ImageServer ZMQ 端口 |
| `offset_x` | `0` | 拼接帧水平偏移（多相机时用） |
| `width` / `height` | `640` / `360` | 截取尺寸 |
| `display` | `false` | 是否弹窗实时显示 |
| `POLICY_PATH` | `.../real_pick_place_act/.../pretrained_model` | 模型路径 |
| `DURATION` | `60` | 运行时长（秒） |
| `ZMQ_HOST` | `127.0.0.1` | ZMQ 连接主机 |
| `JOINT_CONFIG` | `tienkung_26` | 关节 DOF 配置（`tienkung_26`=全26；`tienkung_13`=右臂7+右手6），须与训练时一致 |
| `FPS` | `15` | 推理帧率 |

### DOF（自由度）配置

天工支持可变自由度部署。通过 `JOINT_CONFIG` 环境变量选择 DOF 配置，插件自动将策略子集动作 + 非激活关节的静态填充值组装为完整桥接命令发送给硬件。

| 配置名 | 维度 | 关节组成 | 说明 |
|--------|------|----------|------|
| `tienkung_26` | 26 | 左臂7+右臂7+左手6+右手6 | 默认全自由度 |
| `tienkung_13` | 13 | 右臂7+右手6 | 仅右侧，适合单手操作任务 |

> 在 `ubt_IL/tienkung/lerobot_robot_tienkung/lerobot_robot_tienkung/constants.py` 中
> 定义新 `IntEnum` 并注册到 `JOINT_INDEX_ENUMS` 即可新增自定义 DOF。

### 真机本体 ARM 板部署

天工真机本体部署使用机器人 Jetson AGX Orin 板，通过 conda 环境替代 Docker 容器。
详细文档见 [scripts/deploy/tienkung_pro/arm_64/README.md](scripts/deploy/tienkung_pro/arm_64/README.md)。

---
