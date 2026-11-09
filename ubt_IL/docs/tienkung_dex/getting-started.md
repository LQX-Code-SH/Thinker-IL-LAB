# 天工行者 Dex（Walker TienKung Dex）模仿学习快速开始

> 对应代码：`ubt_IL/tienkung_dex/`（LeRobot 插件 `lerobot_robot_tienkung_dex`）
> 全流程：**数据准备 → 模型训练 → 策略评估 → 模型部署**

本文档给出天工行者 Dex 从克隆代码到仿真/真机部署的最简路径，按需执行每一步。各步骤的完整参数说明与可复制命令见工作流专项文档：

- 仿真工作流：→ [sim-workflow.md](./sim-workflow.md)（仿真采集 → 转换 → 训练 → 评估 → 回注仿真器部署）
- 真机工作流：→ [real-workflow.md](./real-workflow.md)（真机采集 → 转换 → 训练 → 评估 → 真机部署）

> ✅ **真机已验证（2026-11）**：因时 Inspire 与强脑 BrainCo 两款手型的真机训练、部署闭环均已跑通。

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

python hf_manager.py pull qingxiangliu/<数据集仓库名>   # 以 HF 仓库实际发布的 天工行者 Dex 数据集为准
```
> 下载的数据集保存路径位于 `/ubt_IL/dataset/`下。

## 3. 模型下载（可选）

本项目提供训练好的 ACT 模型用于部署测试，可从 HuggingFace 下载到本地（[模型仓库地址](https://huggingface.co/qingxiangliu/models)）。下载可使用 HuggingFace 官方工具或本项目自定义工具 `hf_manager.py`（数据集与模型统一管理），完整用法见 [hf_manage.md](../hf_manage.md)。自己训练时可跳过此步骤，使用后续教程完成训练策略。

```bash
cd ubt_IL/scripts/convert/common

python hf_manager.py pull qingxiangliu/<模型仓库名>   # 以 HF 仓库实际发布的 天工行者 Dex 模型为准
```

> 下载的模型保存路径位于 `/ubt_IL/model/`下。

## 4. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile）
bash run.sh start      # 启动容器 lerobot-tienkung
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`。
- 通信链路：LeRobot（Python 3.12）—ZMQ `5559`/`5560`—> Bridge2（系统 Python 3.10）—ROS2 DDS—> 机器人/仿真器。
- 相机推流：仿真走 **5758**（ubt_sim 直发 JPEG），真机走 **5558**（Bridge2 带 `--enable-image` 转码）。
- Dex 消息包（`ros2_bridge_msgs` / `inspire_hand_msgs` / `brainco_hand_msgs`）在镜像构建期编译到
  `/opt/tienkung_dex_msgs_ws`；`bash run.sh check` 中 `tienkung_dex` 相关三行应全为 `[OK]`。
- 若要在**机器人本体（Thor，无容器）**上跑策略，可跳过本节，按
  [real-workflow.md §6](./real-workflow.md#6-模型部署真机) 的「机器人本体部署」建 venv。

## 5. 数据转换

脚本：`/ubt_IL/scripts/convert/tienkung_dex/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。完整流程与参数说明见：

- 仿真数据：→ [sim-workflow.md §3 数据转换](./sim-workflow.md#3-数据转换hdf5--lerobot)
- 真机数据：→ [real-workflow.md §3 数据转换](./real-workflow.md#3-数据转换hdf5--lerobot)

> 注意：需按自己的需求增加/修改转换配置文件，路径为 `/ubt_IL/scripts/convert/tienkung_dex/configs/`（20D 因时 / 13D 强脑 / 真机单目 RGB）。仿真数据已只读挂载在容器 `/ubt_sim`，可直接作为 `SRC_ROOT`。

## 6. 模型训练

脚本：`/ubt_IL/scripts/train/tienkung_dex/train.sh`。完整流程与参数说明见：

- 仿真训练：→ [sim-workflow.md §4 模型训练](./sim-workflow.md#4-模型训练)
- 真机训练：→ [real-workflow.md §4 模型训练](./real-workflow.md#4-模型训练)

> ⚠️ `train.sh` 默认 `CONFIG_PATH` 指向**仿真配置**，真机训练必须显式指定真机配置。

## 7. 模型评估

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在数据集上离线推理，对比预测动作 vs 真值动作的 MSE，部署前先量化策略质量。完整流程与参数说明见：

- 仿真模型评估：→ [sim-workflow.md §5 策略评估](./sim-workflow.md#5-策略评估离线-mse)
- 真机模型评估：→ [real-workflow.md §5 策略评估](./real-workflow.md#5-策略评估离线-mse)

## 8. 模型部署

脚本：`/ubt_IL/scripts/deploy/tienkung_dex/rollout.sh`（会自动拉起 Bridge2；前置：先 `robot_ready.sh` 摆到预备位，再跑策略，顺序不能反）。完整流程与参数说明见：

- 仿真部署：→ [sim-workflow.md §6 模型部署（仿真）](./sim-workflow.md#6-模型部署仿真)
- 真机部署：→ [real-workflow.md §6 模型部署（真机）](./real-workflow.md#6-模型部署真机)
- 机器人本体（Thor，无容器）部署：同见 [real-workflow.md §6](./real-workflow.md#6-模型部署真机) 的「机器人本体部署」小节
- 脚本清单、全部环境变量与动作向量：→ [scripts/deploy/tienkung_dex/README.md](../../scripts/deploy/tienkung_dex/README.md)

> 注意：`JOINT_CONFIG` 必须与训练 DOF 一致（因时 20D `tienkung_dex_20` / 强脑 13D `tienkung_dex_brainco_13` / 因时 r13 13D `tienkung_dex_inspire_13`，后者由 Bridge2 自动把 6 维 0~1 展开成 13 个角码）。**预备位不是回零**，也不能一步跳到预备位——从站立位直达会让手贴着桌面从体侧扫过去。

## 9. 常见问题

| 现象 | 处理 |
|------|------|
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度；Dex 只取右臂 7 + 右手，恒定维已在转换配置里剔除 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1`（默认关闭；仿真重复帧问题见 [sim-workflow.md §3.3](./sim-workflow.md#33-静止帧裁剪trim_stationary默认关闭)） |
| 手不动 / 手指整体错位 | `HAND_VARIANT` / `JOINT_CONFIG` 与实物手型不一致，或插件与 Bridge2 的常量副本漂移（Bridge2 启动日志会报「通道表不一致」） |
| 部署无相机图像 | 仿真看 5758 是否有流；真机看 Bridge2 是否带 `--enable-image`、端口是否 5558 |
| 仿真里机器人不动 | `ROS_DOMAIN_ID` 是否与仿真一致（ubt_sim 默认 0）；`robot_ready.sh` 是否先跑过；`/sim/cmd_reset` 复位场景 |
| 手臂起步就撞桌子 | 没跑 `robot_ready.sh`，或用了 `--stage-durations` 强行动作过快 |
| 常驻服务 `load` 被拒 | Dex 不支持热切换模型，`shutdown` 后重新 `launch` |
| 训练报 `FileExistsError` | 从头训练时 `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |

更多分阶段排查项见 [sim-workflow.md](./sim-workflow.md) §7 与 [real-workflow.md](./real-workflow.md) §7。
