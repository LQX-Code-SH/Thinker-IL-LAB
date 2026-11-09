# Walker C1 EDU 共创者 模仿学习快速开始

> 对应代码：`ubt_IL/walker_c1/`（LeRobot 插件 `lerobot_robot_walker_c1`）
> 全流程：**数据准备 → 模型训练 → 策略评估 → 模型部署**

本文档给出 Walker C1 EDU 共创者 从克隆代码到仿真/真机部署的最简路径，按需执行每一步。各步骤的完整参数说明与可复制命令见工作流专项文档：

- 仿真工作流：→ [sim-workflow.md](./sim-workflow.md)（仿真采集 → 转换 → 训练 → 评估 → 回注仿真器部署）
- 真机工作流：→ [real-workflow.md](./real-workflow.md)（真机前置 half_sdk/DDS → 采集 → 转换 → 训练 → 真机部署）
- 常驻推理服务：→ [inference-server.md](./inference-server.md)
- 技术架构：→ [architecture.md](./architecture.md)（26 维契约 / 端口 / 话题 / hand 映射）

**关键信息**：

- **策略维度**：26（左臂 7 + 右臂 7 + 左手 6 + 右手 6）或 13（右臂 7 + 右手 6，左臂/左手保持 home）
- **不控制**：头 2 / 腰 3 / 腿 12（机器人自身保持当前位；仿真锚定 HOME_POSE）
- **相机**：头部单目 RGB；训练分辨率仿真 320×240、真机 640×512
- **手部**：SDK 6 个压缩通道（`thumb_swing` / `thumb_mcp` / `index_mcp` / `middle_mcp` / `ring_mcp` / `little_mcp`）

> ✅ **真机已验证（2026-11）**：13D（右臂 7 + 右手 6）、30 fps、头相机 640×512；真机数据集 `walker_c1_real_13d_1RGB`（34 集 / 11406 帧）→ 模型 `walker_c1_real_13d_act_1RGB`，真机部署闭环已跑通。

---

# 快速开始

## 1. 克隆项目

```bash
# （可选）访问 GitHub 较慢时配置代理，端口按本地代理调整
git config --global http.proxy  http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
export GIT_LFS_PROXY="http://127.0.0.1:7897"   # LFS 走代理

# 克隆仓库（先跳过 LFS 与子模块，加速克隆）
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/LQX-Code-SH/Thinker-IL-LAB.git
cd Thinker-IL-LAB
git lfs pull                 # 拉取 LFS 大文件（USD 模型/贴图等仿真资源）
git submodule update --init  # 初始化 lerobot 子模块
```

> LFS 大文件为 `ubt_sim` 仿真资源，仅做数据转换/训练/部署可不拉取 LFS；`lerobot` 子模块必须初始化。

## 2. 数据下载（可选）

当前可从 HuggingFace 下载测试数据集（[数据集仓库地址](https://huggingface.co/datasets/qingxiangliu)）；下载可使用 HuggingFace 官方工具或本项目自定义工具 `hf_manager.py` 管理数据集，完整用法见 [hf_manage.md](../hf_manage.md)。
也可跳过此部分，使用自采数据走 [第 5 节](#5-数据转换)。

```bash
cd ubt_IL/scripts/convert/common
export HF_ENDPOINT=https://hf-mirror.com          # （可选）国内加速镜像

python hf_manager.py pull qingxiangliu/<数据集仓库名>   # 以 HF 仓库实际发布的 Walker C1 EDU 共创者 数据集为准
```
> 下载的数据集保存路径位于 `/ubt_IL/dataset/`下。

## 3. 模型下载（可选）

本项目提供训练好的 ACT 模型用于部署测试，可从 HuggingFace 下载到本地（[模型仓库地址](https://huggingface.co/qingxiangliu/models)）。下载可使用 HuggingFace 官方工具或本项目自定义工具 `hf_manager.py`（数据集与模型统一管理），完整用法见 [hf_manage.md](../hf_manage.md)。自己训练时可跳过此步骤，使用后续教程完成训练策略。

```bash
cd ubt_IL/scripts/convert/common

python hf_manager.py pull qingxiangliu/<模型仓库名>   # 以 HF 仓库实际发布的 Walker C1 EDU 共创者 模型为准
```

> 下载的模型保存路径位于 `/ubt_IL/model/`下。

## 4. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；Thor 机型自动切 Dockerfile.thor）
bash run.sh start      # 启动容器 lerobot-tienkung
bash run.sh check      # 插件 / bridge / ROS2 msgs / torch 应为 [OK]，RMW 行是 [INFO]
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`；`entrypoint.sh` 自动安装 `lerobot` 与 `lerobot_robot_walker_c1`（editable），并编译/加载 `mc_task_msgs` / `mc_state_msgs`（复用 `walker_s2/walker_sdk_ros2` 工作空间）。
- 通信链路：LeRobot（Python 3.12）—ZMQ `5661`/`5662`—> Bridge2（系统 Python 3.10）—ROS2 DDS—> 机器人/仿真器；相机直连 ROS2 话题。
- 相机：头部单目（仿真 320×240 / 真机 640×512），话题与消息类型见 [architecture.md](./architecture.md)。
- 插件缺失时可手动重装：`python /ubt_IL/walker_c1/init_walker_c1.py`（应输出 `Robot type: walker_c1` / `Camera type: walker_c1_camera`）。

## 5. 数据转换

脚本：`/ubt_IL/scripts/convert/walker_c1/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。完整流程与参数说明见：

- 仿真数据：→ [sim-workflow.md §3 数据转换](./sim-workflow.md#3-数据转换)
- 真机数据：→ [real-workflow.md §4 数据转换](./real-workflow.md#4-数据转换)

> 注意：仿真数据已只读挂载在容器 `/ubt_sim`；`SRC_ROOT` 不指定时，默认目录缺失会自动回退到 `/ubt_sim/dataset/walker_c1`。默认 26 维（产物 `walker_c1_sim_26d_1RGB`），单臂任务加 `DIM=13`（产物 `walker_c1_sim_13d_1RGB`）。配置文件在 `/ubt_IL/scripts/convert/walker_c1/configs/`。

## 6. 模型训练

脚本：`/ubt_IL/scripts/train/walker_c1/train.sh`（默认 ACT、26 维仿真配置，输出 `walker_c1_sim_26d_act_1RGB`）。完整流程与参数说明见：

- 仿真训练：→ [sim-workflow.md §4 训练](./sim-workflow.md#4-训练)
- 真机训练：→ [real-workflow.md §5 训练与评估](./real-workflow.md#5-训练与评估)

> ⚠️ `train.sh` 默认为**仿真配置**，真机训练必须用 `CONFIG=` 显式指定真机配置；冒烟测试可加 `--steps=2 --save_checkpoint=false`。

## 7. 模型评估

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在数据集上离线推理，对比预测动作 vs 真值动作的 MSE，部署前先量化策略质量。完整流程与参数说明见：

- 仿真模型评估：→ [sim-workflow.md §5 评估](./sim-workflow.md#5-评估)
- 真机模型评估：→ [real-workflow.md §5 训练与评估](./real-workflow.md#5-训练与评估)

## 8. 模型部署

脚本：`/ubt_IL/scripts/deploy/walker_c1/rollout.sh`（前置：先 `robot_ready.sh` 摆到预备位；**真机必须先切 half_sdk**）。完整流程与参数说明见：

- 仿真部署：→ [sim-workflow.md §6 仿真部署](./sim-workflow.md#6-仿真部署闭环验证)
- 真机部署：→ [real-workflow.md §6 部署](./real-workflow.md#6-部署)
- 常驻推理服务：→ [inference-server.md](./inference-server.md)

> 注意：`ROBOT_MODEL` 必须与训练维度一致（26D `walker_c1_26d` / 13D `walker_c1_13d`）；真机相机需 `CAMERA_TOPIC=/sensor/camera/head_stereo_left/color/raw` + `CAMERA_MSG_TYPE=shm_msgs/Image6m`，`FPS` 与训练数据一致（30 fps 数据集用 `FPS=30`）。

## 9. 常见问题

| 现象 | 处理 |
|---|---|
| `joint_config 'walker_c1_26d' not registered` | 插件未安装：`python /ubt_IL/walker_c1/init_walker_c1.py` |
| 转换报「维度 = '31 6' 要求不符」 | 数据不是 ubt_sim walker_c1 采集；确认来源或 `CONFIG=` 显式指定 |
| 训练报相机 key 不存在 | 转换配置的 `observation.images.camera_head` 未被写入：检查 `meta/info.json` |
| rollout 预检维度不符 | 策略维度与 `ROBOT_MODEL` 不一致；确认训练用的是 26D 还是 13D 数据集 |
| 相机超时 | 话题名/`ROS_DOMAIN_ID`/QoS（必须 BEST_EFFORT）；真机是 `head_stereo_left/color/raw` + `shm_msgs/Image6m` |
| `torch.cuda.is_available()` False | 容器缺 `NVIDIA_VISIBLE_DEVICES`/`NVIDIA_DRIVER_CAPABILITIES`；用 `run.sh start` 启动 |
| 左手完全不动 | 当前采集批次左手全程 ≈0（任务只用右手），属于数据特性 |

更多分阶段排查项见 [sim-workflow.md](./sim-workflow.md) §7 与 [real-workflow.md](./real-workflow.md) §7。

> 仿真数据 `ubt_sim/dataset/walker_c1` 约 11 集 × 100 帧，仅够 smoke 与流程验证，
> 正式仿真训练需补采（建议 ≥ 50 集）；真机数据见顶部「真机已验证」说明。
