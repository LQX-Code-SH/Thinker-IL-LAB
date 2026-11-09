# Getting Started

The project consists of two sub-projects, **Simulation Platform** and **Imitation Learning Platform**, both running in containers. This page covers repository setup, requirements, and the overall workflow; follow the robot-specific documentation for detailed steps.

## Get the code

After cloning the repository, pull the LFS assets (USD models, textures, etc.) and initialize the `lerobot` submodule.

```bash
# (Optional) Configure a proxy if GitHub is slow; adjust the port to your local proxy
git config --global http.proxy  http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
export GIT_LFS_PROXY="http://127.0.0.1:7897"   # route LFS through the proxy

# 1. Clone without LFS files and submodules for a faster checkout
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/LQX-Code-SH/Thinker-IL-LAB.git
cd Thinker-IL-LAB

# 2. Pull LFS assets
git lfs pull

# 3. Initialize and update submodules (lerobot)
git submodule update --init
```

> LFS assets are `ubt_sim` simulation resources (`*.usd`, `*.urdf`, textures, etc.). If you only need real-robot data conversion, training, or deployment, you can skip LFS. The `lerobot` submodule is required; `ubt_sim` is only needed for simulation data collection.

## Requirements

| Dependency | Requirement | Notes |
|------|------|------|
| OS | Linux (Ubuntu 22.04 recommended) | Both sub-projects run in containers |
| GPU | NVIDIA GPU (RTX 3090 or better) | Isaac Sim rendering and policy training/inference |
| Docker | [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) installed | GPU access inside containers |
| Git LFS | Installed | Manages USD models, textures, and other large files |
| Disk | Sufficient space (100 GB+ recommended) | LFS assets, datasets, and checkpoints can be large |

## End-to-end workflow

In one sentence: **collect HDF5 data -> convert to a LeRobot dataset -> train in the container -> evaluate offline -> deploy (simulation or real robot)**.

```
Simulation collection (ubt_sim) ───┐
                                   ├─► data conversion ─► training ─► evaluation ─► sim validation ─► real deployment
Real-robot collection (teleop) ────┘
```

&nbsp;

> There are two ways to get familiar with the project: explore the platforms first (`ubt_sim` simulation collection / `ubt_IL` imitation learning), or jump directly into an end-to-end robot workflow (Walker TienKung Pro / Walker TienKung Dex / Walker S2 EDU / Walker C1 EDU) and learn by reproducing it.

## Browse by platform

Start from the platform level to understand the shared architecture, commands, and workflows:

| Platform | Overview |
|---------|----------|
| Simulation Platform | [Simulation Platform Guide](sim/index.md) |
| Imitation Learning Platform | [Imitation Learning Platform Guide](il/index.md) |

## Browse by robot

Follow a complete documentation track for each robot:

| Robot | Overview | End-to-end workflow |
|------|---------|--------------|
| Walker TienKung Pro | [Walker TienKung Pro Overview](tienkung/index.md) | [Walker TienKung Pro Simulation Workflow](tienkung/sim-workflow.md) / [Walker TienKung Pro Real-Robot Workflow](tienkung/real-workflow.md) |
| Walker TienKung Dex | [Walker TienKung Dex Overview](tienkung-dex/index.md) | [Walker TienKung Dex Simulation Workflow](tienkung-dex/sim-workflow.md) / [Walker TienKung Dex Real-Robot Workflow](tienkung-dex/real-workflow.md) |
| Walker S2 EDU | [Walker S2 EDU Overview](walker-s2/index.md) | [Walker S2 EDU Simulation Workflow](walker-s2/sim-workflow.md) / [Walker S2 EDU Real-Robot Workflow](walker-s2/real-workflow.md) |
| Walker C1 EDU | [Walker C1 EDU Overview](walker-c1/index.md) | [Walker C1 EDU Simulation Workflow](walker-c1/sim-workflow.md) / [Walker C1 EDU Real-Robot Workflow](walker-c1/real-workflow.md) |

## Optional: download sample data and models

Sample datasets and trained ACT models are available on HuggingFace ([datasets](https://huggingface.co/datasets/qingxiangliu) / [models](https://huggingface.co/qingxiangliu/models)). Download them from the website or manage them with the bundled `hf_manager.py` tool:

```bash
cd ubt_IL/scripts/convert/common
export HF_ENDPOINT=https://hf-mirror.com          # (Optional) mirror for faster access in China

# Download a dataset or model
python hf_manager.py pull <dataset-or-model-path>
# Examples
python hf_manager.py pull qingxiangliu/tienkung_pick_up_merged       # real-robot dataset
python hf_manager.py pull qingxiangliu/tienkung_pick_up_act          # real-robot ACT policy
```

### Datasets

**Walker TienKung Pro**

| Repository | Size | Format | Notes |
|------|------|------|------|
| [`tienkung_sim_pick_place`](https://huggingface.co/datasets/qingxiangliu/tienkung_sim_pick_place) ★ | 1.1 MB | LeRobot | Simulated apple pick-and-place |
| [`tienkung_pick_up_merged`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_merged) ★ | 1.6 GB | LeRobot | Merged real-robot pick-up dataset |
| [`tienkung_pick_up_the_apple_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_apple_all) | 426.9 MB | LeRobot | Real-robot apple pick-up |
| [`tienkung_pick_up_the_bottle_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_bottle_all) | 481.0 MB | LeRobot | Real-robot bottle pick-up |
| [`tienkung_pick_up_the_red_bottle_all`](https://huggingface.co/datasets/qingxiangliu/tienkung_pick_up_the_red_bottle_all) | 685.2 MB | LeRobot | Real-robot red-bottle pick-up |
| [`tienkung_real_merged`](https://huggingface.co/datasets/qingxiangliu/tienkung_real_merged) | 379.3 MB | LeRobot | Real-robot bottle pick-up (Guochuang) |

**Walker S2 EDU**

| Repository | Size | Format | Notes |
|------|------|------|------|
| [`Walker_S2_sim_10_2RGB`](https://huggingface.co/datasets/qingxiangliu/Walker_S2_sim_10_2RGB) ★ | 119.2 MB | LeRobot | Simulated part picking |
| [`Walker-s2-pick-part-sim`](https://huggingface.co/datasets/qingxiangliu/Walker-s2-pick-part-sim) | 4.3 GB | HDF5 | Simulated part picking |
| [`Walker_S2_real_10_2RGB`](https://huggingface.co/datasets/qingxiangliu/Walker_S2_real_10_2RGB) | 157.5 MB | LeRobot | Real-robot multi-part picking |
| [`Wlaker_Pick_part_real_10d_2RGB`](https://huggingface.co/datasets/qingxiangliu/Wlaker_Pick_part_real_10d_2RGB) ★ | 24.2 MB | LeRobot | Real-robot part picking |
| [`walker-s2-pick-part-real-hdf5`](https://huggingface.co/datasets/qingxiangliu/walker-s2-pick-part-real-hdf5) | 21.6 GB | HDF5 | Real-robot multi-part picking |
| [`walker_pick_part_real_hdf5`](https://huggingface.co/datasets/qingxiangliu/walker_pick_part_real_hdf5) | 9.7 GB | HDF5 | Real-robot part picking |
| [`walker_pick_sort`](https://huggingface.co/datasets/qingxiangliu/walker_pick_sort) | 11.5 GB | HDF5 | Real-robot part sorting |

### Models (ACT)

| Repository | Size | Notes |
|------|------|------|
| [`tienkung_sim_pick_place_right13_act`](https://huggingface.co/qingxiangliu/tienkung_sim_pick_place_right13_act) | 591 MB | Walker TienKung Pro simulated ACT policy |
| [`tienkung_pick_up_act`](https://huggingface.co/qingxiangliu/tienkung_pick_up_act) | 591 MB | Walker TienKung Pro real-robot ACT policy |
| [`Walker_S2_sim_10_2RGB_act`](https://huggingface.co/qingxiangliu/Walker_S2_sim_10_2RGB_act) | 1.9 GB | Walker S2 EDU simulated ACT policy |
| [`walker_pick_part_real_10d_2RGB_act`](https://huggingface.co/qingxiangliu/walker_pick_part_real_10d_2RGB_act) | 1.7 GB | Walker S2 EDU real-robot ACT policy |

> Datasets are stored in `ubt_IL/dataset/` and models in `ubt_IL/model/`. See [HF Data & Models](common/hf-manage.md) for details.
