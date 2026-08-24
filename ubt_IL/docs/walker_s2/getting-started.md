# Walker S2 EDU 探索者模仿学习快速开始

> 对应代码：`ubt_IL/scripts/{convert,train,deploy}/walker_s2/` 与 `ubt_sim/`（仿真采集工程）
> 全流程：**数据准备 -> 模型训练 -> 策略评估 -> 模型部署**

本文档给出 Walker S2 EDU 从克隆代码到真机/仿真器部署的最简路径，按需执行每一步。各步骤的完整参数说明与可复制命令见工作流专项文档：

- 仿真工作流：-> [sim-workflow.md](./sim-workflow.md)（仿真采集 -> 转换 -> 训练 -> 评估 -> 回注仿真器部署）
- 真机工作流：-> [real-workflow.md](./real-workflow.md)（真机采集 -> 转换 -> 训练 -> 评估 -> 真机部署，含推理服务器与 Jetson 远程拉起）

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

> LFS 大文件为 `ubt_sim` 仿真资源，仅做真机数据转换/训练/部署可不拉取 LFS；`lerobot` 子模块必须初始化，仅仿真采集需要 `ubt_sim`。

## 2. 数据下载（可选）

当前可从 HuggingFace 下载测试数据集（[数据集仓库地址](https://huggingface.co/datasets/qingxiangliu)）；下载可使用HuggingFace官方工具或本项目自定义工具 `hf_manager.py` 管理数据集，完整用法见 [hf_manage.md](../hf_manage.md)。
也可跳过此部分，使用自采数据走 [第 5 节](#5-数据转换)。

```bash
cd ubt_IL/scripts/convert/common
export HF_ENDPOINT=https://hf-mirror.com          # （可选）国内加速镜像

python hf_manager.py pull qingxiangliu/Wlaker_Pick_part_real_10d_2RGB   # 下载 Walker S2 EDU 抓取测试数据集
python hf_manager.py pull qingxiangliu/Walker_S2_sim_10_2RGB  # 下载 Walker S2 EDU 仿真抓取测试数据集
```
> 下载的数据集保存路径位于 `/ubt_IL/dataset/`下。

## 3. 模型下载（可选）

本项目提供训练好的ACT模型用于部署测试，可从 HuggingFace 下载到本地（[模型仓库地址](https://huggingface.co/qingxiangliu/models)）。下载可使用 HuggingFace 官方工具或本项目自定义工具 `hf_manager.py`（数据集与模型统一管理），完整用法见 [hf_manage.md](../hf_manage.md)。自己训练时可跳过此步骤，使用后续教程完成训练策略。

```bash
cd ubt_IL/scripts/convert/common

python hf_manager.py pull qingxiangliu/Walker_S2_sim_10_2RGB_act   # 下载仿真ACT抓取测试策略
python hf_manager.py pull qingxiangliu/walker_pick_part_real_10d_2RGB_act   # 下载真机ACT抓取测试策略
```

> 下载的模型保存路径位于 `/ubt_IL/model/`下。

## 4. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile：x86 -> humble，arm64 -> humble-arm64）
bash run.sh start      # 启动容器lerobot-tienkung
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`。
- 容器内存在双 `python` 环境：ROS 相关使用 `/usr/bin/python3`，lerobot 相关脚本使用 `/lerobot/.venv/bin/python`（默认）。
- 真机部署时容器在**机器人 Vision 板（Jetson）**上构建，构建脚本自动识别主板类型构建 arm 容器，详见 [real-workflow.md §6](./real-workflow.md#6-模型部署真机)。

## 5. 数据转换

脚本：`/ubt_IL/scripts/convert/walker_s2/convert.sh`，将 HDF5 原始数据转换为 LeRobot v3 数据集（产物位于 `/ubt_IL/dataset/<REPO_ID>/`）。完整流程与参数说明见：

- 仿真数据：-> [sim-workflow.md §3 数据转换](./sim-workflow.md#3-数据转换hdf5--lerobot)
- 真机数据：-> [real-workflow.md §3 数据转换](./real-workflow.md#3-数据转换hdf5--lerobot)

> ⚠️ `convert.sh` 默认 `CONFIG=configs/walker_s2_real_19d_1RGBD.json` **仓库未提供**，必须显式指定 `configs/` 下实际存在的配置。现有配置：

| 配置文件 | 维度 | 场景 |
|----------|------|------|
| [`walker_s2_sim_10d_2RGB.json`](../../scripts/convert/walker_s2/configs/walker_s2_sim_10d_2RGB.json) | 10D | 仿真（右臂 7 + 头 2 + 右夹爪 1） |
| [`walker_s2_sim_33d_4RGB.json`](../../scripts/convert/walker_s2/configs/walker_s2_sim_33d_4RGB.json) | 33D | 仿真（含深度） |
| [`walker_s2_real_10d_2RGB.json`](../../scripts/convert/walker_s2/configs/walker_s2_real_10d_2RGB.json) | 10D | 真机 |

## 6. 模型训练

脚本：`/ubt_IL/scripts/train/walker_s2/train.sh`。完整流程与参数说明见：

- 仿真训练：-> [sim-workflow.md §4 模型训练](./sim-workflow.md#4-模型训练)
- 真机训练：-> [real-workflow.md §4 模型训练](./real-workflow.md#4-模型训练)

> ⚠️ `train.sh` 默认 `CONFIG` 指向**仿真配置**（ACT 10D 2RGB），真机训练必须显式指定真机配置（如 `train_config_pick_part_real_10d_2RGB.json`）。

## 7. 模型评估

脚本：`/ubt_IL/scripts/eval/eval_policy.py`（与天工行者（Walker TienKung）共用），在数据集上离线推理，对比预测动作 vs 真值动作的 MSE，部署前先量化策略质量。完整流程与参数说明见：

- 仿真模型评估：-> [sim-workflow.md §5 策略评估](./sim-workflow.md#5-策略评估离线-mse)
- 真机模型评估：-> [real-workflow.md §5 策略评估](./real-workflow.md#5-策略评估离线-mse)

## 8. 模型部署

脚本：`/ubt_IL/scripts/deploy/walker_s2/rollout.sh`（前置：仿真已启动或真机已进入开发者模式并执行 `robot_ready.sh` 初始化）。完整流程与参数说明见：

- 仿真部署：-> [sim-workflow.md §6 模型部署（回注仿真）](./sim-workflow.md#6-模型部署回注仿真)
- 真机部署：-> [real-workflow.md §6 模型部署（真机）](./real-workflow.md#6-模型部署真机)

> 注意：
> - 部署前自动做**安全预检**（action 维度匹配），`ROBOT_MODEL` 必须与训练 DOF 一致（10D->`walker_s2_10d`、19D->`walker_s2_19d`、31D->`walker_s2_31d`），不通过则拒绝部署。
> - 回注仿真时设 `ZMQ_HOST=127.0.0.1`；真机模型部署前也可先在仿真中测试模型效果，避免 DOF 配置不匹配造成意料之外的动作。
> - 长时间常驻部署用**推理服务器**（免冷启动，`start/stop/home/load`）：`bash /ubt_IL/scripts/deploy/walker_s2/inference_server.sh`，详见 [inference-server.md](./inference-server.md)。

## 9. 常见问题

| 现象 | 处理 |
|------|------|
| 转换报"配置不存在" | `convert.sh` 默认 `CONFIG=configs/walker_s2_real_19d_1RGBD.json` 仓库未提供；显式指定 `configs/` 下实际存在的配置 |
| 真机 19D 数据无法转换 | 仓库仅提供 10D 真机转换配置；19D 需参考 `walker_s2_gripper_19d.json` 关节顺序自行编写配置 |
| 数据集帧率与预期不符 | 采集 15 Hz，转换默认 `FPS=13`（真机配置为 15）；保留原频率用 `FPS=auto`，或 `RESAMPLE_FPS` 重采样 |
| 轨迹含长静止段，部署时动作停止 | 转换时加 `TRIM_STATIONARY=1` 裁剪静止帧 |
| 部署时报维度不匹配 | 安全预检拒绝：确认 `ROBOT_MODEL` 与策略维度对应（10D->`walker_s2_10d`、19D->`walker_s2_19d`、31D->`walker_s2_31d`）；无 action names 的策略需 `ALLOW_DIM_ONLY_POLICY=1` |
| 推理服务器 `start` 报错 | `start/stop` 需 `INFERENCE_TYPE=act_async`；`sync` 引擎无 pause/resume 语义 |
| 重复拉起推理服务器失败 | 单例 pid 文件 `/tmp/walker_inference_server.pid` 存在；`shutdown` 正常退出或手动清理后再拉 |
| 部署时机器人不动 | 真机确认 Bridge2 已启动（5561/5562/5563）、`dev_mode.sh` 已切开发者模式；仿真确认 `ZMQ_HOST=127.0.0.1` 且仿真任务已启动 |
| 仿真相机与真机相机尺寸不同 | 仿真 RGB 为 [3,240,320]（10D）或 [3,480,640]（33D），真机为 [256,320] 等；模型输入需与训练数据一致，勿混用 |

更多分阶段排查项见 [sim-workflow.md](./sim-workflow.md) §7 与 [real-workflow.md](./real-workflow.md) §7。
