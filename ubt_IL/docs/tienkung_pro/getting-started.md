# 天工行者无疆（Walker TienKung Pro）模仿学习快速开始

> 对应代码：`ubt_IL/tienkung/`（LeRobot 插件 `lerobot_robot_tienkung`）
> 全流程：**数据准备 → 模型训练 → 策略评估 → 模型部署**

本文档给出天工行者无疆 从克隆代码到真机/仿真器部署的最简路径，按需执行每一步。各步骤的完整参数说明与可复制命令见工作流专项文档：

- 仿真工作流：→ [sim-workflow.md](./sim-workflow.md)（仿真采集 → 转换 → 训练 → 评估 → 回注仿真器部署）
- 真机工作流：→ [real-workflow.md](./real-workflow.md)（真机采集 → 转换 → 训练 → 评估 → 真机部署，含「远程容器版」与「Jetson host 版」两种方案）

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

> LFS 大文件为 `ubt_sim` 仿真资源，仅做真机数据转换/训练/部署可不拉取 LFS；`lerobot` 子模块必须初始化。

## 2. 数据下载（可选）

当前可从 HuggingFace 下载测试数据集（[数据集仓库地址](https://huggingface.co/datasets/qingxiangliu)）；下载可使用HuggingFace官方工具或本项目自定义工具 `hf_manager.py` 管理数据集，完整用法见 [hf_manage.md](../hf_manage.md)。
也可用跳过此部分，使用自采数据走 [第 5 节](#5-数据转换)。

```bash
cd ubt_IL/scripts/convert/common
export HF_ENDPOINT=https://hf-mirror.com          # （可选）国内加速镜像

python hf_manager.py pull qingxiangliu/tienkung_sim_pick_place   # 下载仿真测试数据集
python hf_manager.py pull qingxiangliu/tienkung_pick_up_merged   # 下载真机测试数据集
```
> 下载的数据集保存路径位于 `/ubt_IL/dataset/`下。

## 3. 模型下载（可选）

本项目提供训练好的ACT模型用于部署测试，可从 HuggingFace 下载到本地（[模型仓库地址](https://huggingface.co/qingxiangliu/models)）。下载可使用 HuggingFace 官方工具或本项目自定义工具 `hf_manager.py`（数据集与模型统一管理），完整用法见 [hf_manage.md](../hf_manage.md)。自己训练时可跳过此步骤，使用后续教程完成训练策略。

```bash
cd ubt_IL/scripts/convert/common
                       
python hf_manager.py pull qingxiangliu/tienkung_sim_pick_place_right13_act   # 下载仿真ACT抓取测试策略
python hf_manager.py pull qingxiangliu/tienkung_pick_up_act   # 下载真机ACT抓取测试策略
```

> 下载的模型保存路径位于 `/ubt_IL/model/`下。

## 4. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile：x86 → humble，arm64 → humble-arm64）
bash run.sh start      # 启动容器lerobot-tienkung
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`。
- 通信链路：LeRobot（Python 3.12）—ZMQ `5559`/`5560`—> Bridge2（系统 Python 3.10）—ROS2 DDS—> 机器人/仿真器；相机推流走 ZMQ `5558`。

## 5. 数据转换

脚本：`/ubt_IL/scripts/convert/tienkung_pro/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。完整流程与参数说明见：

- 仿真数据：→ [sim-workflow.md §3 数据转换](./sim-workflow.md#3-数据转换hdf5--lerobot)
- 真机数据：→ [real-workflow.md §3 数据转换](./real-workflow.md#3-数据转换hdf5--lerobot)

> 注意：需按照自己的需求增加/修改转换配置文件，配置文件路径为 `/ubt_IL/scripts/convert/tienkung_pro/configs/`。

## 6. 模型训练

脚本：`/ubt_IL/scripts/train/tienkung_pro/train.sh`。完整流程与参数说明见：

- 仿真训练：→ [sim-workflow.md §4 模型训练](./sim-workflow.md#4-模型训练)
- 真机训练：→ [real-workflow.md §4 模型训练](./real-workflow.md#4-模型训练)

> ⚠️ `train.sh` 默认 `CONFIG_PATH` 指向**仿真配置**，真机训练必须显式指定真机配置。

## 7. 模型评估

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在数据集上离线推理，对比预测动作 vs 真值动作的 MSE，部署前先量化策略质量。完整流程与参数说明见：

- 仿真模型评估：→ [sim-workflow.md §5 策略评估](./sim-workflow.md#5-策略评估离线-mse)
- 真机模型评估：→ [real-workflow.md §5 策略评估](./real-workflow.md#5-策略评估离线-mse)

## 8. 模型部署

脚本：`/ubt_IL/scripts/deploy/tienkung_pro/rollout.sh`（前置：Bridge2 已启动）。完整流程与参数说明见：

- 仿真部署：→ [sim-workflow.md §6 模型部署（仿真）](./sim-workflow.md#6-模型部署仿真)
- 真机部署：→ [real-workflow.md §6 模型部署（真机）](./real-workflow.md#6-模型部署真机)

> 注意：`JOINT_CONFIG` 必须与训练 DOF 一致（26D 模型 `tienkung_26`，13D 模型 `tienkung_13`）；真机部署需先进入半身运控模式（[天工行者无疆文档](https://docs.ubtrobot.com/walker-tienkung/docs/user-guide/8)）。另外真机模型部署前也可在仿真中测试模型效果避免DOF配置不匹配造成意料之外的动作。

## 9. 常见问题

| 现象 | 处理 |
|------|------|
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度（26D 中左臂锁死等）；改用 13D 配置转换，剔除死维度 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1` 裁剪静止帧 |
| 训练报 `FileExistsError` | 从头训练时 `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |
| 真机转换报缺深度流 | 真机 HDF5 无深度是正常的；必须用 `tienkung_pro_26d_1RGB_real.json`（仅 RGB） |
| 部署无相机图像 | 用 `image_client.py` 验证 ZMQ 5558 通路；真机端需先启动 `image_server.py` |
| 真机部署无响应 | `ZMQ_HOST` 是否指向真机 `192.168.41.2`（脚本默认 127.0.0.1）；Bridge2 是否启动（`pgrep -f ros2_deploy_bridge`） |
| 真机部署动作异常 | `JOINT_CONFIG` 与训练 DOF 不一致（26D↔13D） |

更多分阶段排查项见 [sim-workflow.md](./sim-workflow.md) §7 与 [real-workflow.md](./real-workflow.md) §7。
