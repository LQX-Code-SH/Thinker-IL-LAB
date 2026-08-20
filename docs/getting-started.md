# 快速开始

本项目分为 **仿真平台（`ubt_sim`）** 与 **模仿学习平台（`ubt_IL`）** 两个子项目，均以容器方式运行。本页介绍代码获取、环境要求与全流程总览；具体操作步骤请进入对应机器人文档线。

## 代码获取

克隆仓库后，需先拉取 LFS 大文件（USD 模型、贴图等）并初始化子模块（`lerobot`）。

```bash
# （可选）配置代理：访问 GitHub 较慢时设置，端口按本地代理调整
git config --global http.proxy  http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
export GIT_LFS_PROXY="http://127.0.0.1:7897"   # LFS 走代理

# 1. 克隆仓库（先跳过 LFS 和子模块，加速克隆）
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/LQX-Code-SH/Thinker-IL-LAB.git
cd Thinker-IL-LAB

# 2. 拉取 LFS 大文件
git lfs pull

# 3. 初始化并拉取子模块（lerobot）
git submodule update --init
```

> LFS 大文件为 `ubt_sim` 仿真资源（`*.usd`、`*.urdf`、贴图等）。仅做真机数据转换 / 训练 / 部署时可跳过 LFS；`lerobot` 子模块必须初始化；仅仿真采集才需要 `ubt_sim`。

## 环境要求

| 依赖 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Linux（Ubuntu 22.04 推荐） | 两个子项目均以容器方式运行 |
| GPU | NVIDIA GPU（3090+） | Isaac Sim 渲染与模型训练/推理 |
| Docker | 已安装 [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) | 容器内使用 GPU |
| Git LFS | 已安装 | USD 模型、贴图等大文件管理 |
| 磁盘空间 | 充足（建议 100GB+ 可用） | LFS 大文件、数据集、checkpoint 占用较大 |

## 全流程一览

一句话流程：**HDF5 采集数据 -> 转换为 LeRobot 数据集 -> 容器内训练 -> 离线评估 -> 部署（仿真/真机）**。

```
仿真采集（ubt_sim）───┐
                    ├─► 数据转换 ─► 模型训练 ─► 离线评估 ─► 仿真部署验证 ─► 真机部署
真机采集（遥操作）─────┘         
```

## 按机型进入

| 机型 | 入口 | 端到端工作流 |
|------|------|--------------|
| 天工 Pro | [TienKung概览](tienkung/index.md) | [TienKung仿真工作流](tienkung/sim-workflow.md) / [TienKung真机工作流](tienkung/real-workflow.md) |
| Walker S2 | [walker-s2概览](walker-s2/index.md) | [walker-s2仿真工作流](walker-s2/sim-workflow.md) / [walker-s2真机工作流](walker-s2/real-workflow.md) |

## 可选：下载测试数据与模型

可从 HuggingFace 下载测试数据集与训练好的 ACT 模型（[数据集仓库](https://huggingface.co/datasets/qingxiangliu) / [模型仓库](https://huggingface.co/qingxiangliu/models)），自行前往 HuggingFace 官网下载使用，或者使用项目自带 `hf_manager.py` 工具管理：

```bash
cd ubt_IL/scripts/convert/common
export HF_ENDPOINT=https://hf-mirror.com          # （可选）国内加速镜像

# 下载测试数据集
python hf_manager.py pull <数据集路径/模型路径>      # 仿真测试数据集
# 下载案例
python hf_manager.py pull qingxiangliu/tienkung_pick_up_merged       # 真机测试数据集
python hf_manager.py pull qingxiangliu/tienkung_pick_up_act          # 真机 ACT 测试策略

```

### 数据集列表

**天工 Pro**

| 仓库 | 体积 | 格式 | 说明 |
|------|------|------|------|
| [`tienkung_sim_pick_place`](https://huggingface.co/datasets/qingxiangliu/tienkung_sim_pick_place) ★ | 1.1 MB | LeRobot | 仿真抓取苹果 |
| [`tienkung_pick_up_merged`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_merged) ★ | 1.6 GB | LeRobot | 真机抓取合并数据集 |
| [`tienkung_pick_up_the_apple_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_apple_all) | 426.9 MB | LeRobot | 真机抓取苹果 |
| [`tienkung_pick_up_the_bottle_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_bottle_all) | 481.0 MB | LeRobot | 真机抓取瓶子 |
| [`tienkung_pick_up_the_red_bottle_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_red_bottle_all) | 685.2 MB | LeRobot | 真机抓取红色瓶子 |
| [`tienkung_real_merged`](https://huggingface.co/datasets/qingxiangliu/tienkung_real_merged) | 379.3 MB | LeRobot | 真机抓取瓶子（国创） |

**Walker S2**

| 仓库 | 体积 | 格式 | 说明 |
|------|------|------|------|
| [`Walker_S2_sim_10_2RGB`](https://huggingface.co/datasets/qingxiangliu/Walker_S2_sim_10_2RGB) ★ | 119.2 MB | LeRobot | 仿真抓取零件 |
| [`Walker-s2-pick-part-sim`](https://huggingface.co/datasets/qingxiangliu/Walker-s2-pick-part-sim) | 4.3 GB | HDF5 | 仿真抓取零件 |
| [`Walker_S2_real_10_2RGB`](https://huggingface.co/datasets/qingxiangliu/Walker_S2_real_10_2RGB) | 157.5 MB | LeRobot | 真机抓取多零件 |
| [`Wlaker_Pick_part_real_10d_2RGB`](https://huggingface.co/datasets/qingxiangliu/Wlaker_Pick_part_real_10d_2RGB) ★ | 24.2 MB | LeRobot | 真机抓取零件 |
| [`walker-s2-pick-part-real-hdf5`](https://huggingface.co/datasets/qingxiangliu/walker-s2-pick-part-real-hdf5) | 21.6 GB | HDF5 | 真机抓取多零件 |
| [`walker_pick_part_real_hdf5`](https://huggingface.co/datasets/qingxiangliu/walker_pick_part_real_hdf5) | 9.7 GB | HDF5 | 真机抓取零件 |
| [`walker_pick_sort`](https://huggingface.co/datasets/qingxiangliu/walker_pick_sort) | 11.5 GB | HDF5 | 真机分拣零件 |

### 模型列表（ACT 策略）

| 仓库 | 体积 | 说明 |
|------|------|------|
| [`tienkung_sim_pick_place_right13_act`](https://huggingface.co/qingxiangliu/tienkung_sim_pick_place_right13_act) | 591 MB | 天工仿真 ACT 抓取测试策略 |
| [`tienkung_pick_up_act`](https://huggingface.co/qingxiangliu/tienkung_pick_up_act) | 591 MB | 天工真机 ACT 测试策略 |
| [`Walker_S2_sim_10_2RGB_act`](https://huggingface.co/qingxiangliu/Walker_S2_sim_10_2RGB_act) | 1.9 GB | Walker S2 仿真 ACT 测试策略 |
| [`walker_pick_part_real_10d_2RGB_act`](https://huggingface.co/qingxiangliu/walker_pick_part_real_10d_2RGB_act) | 1.7 GB | Walker S2 真机 ACT 抓取测试策略 |

> 下载的数据集保存于 `ubt_IL/dataset/`，模型保存于 `ubt_IL/model/`。完整用法见 [HF 数据/模型管理](common/hf-manage.md)。
