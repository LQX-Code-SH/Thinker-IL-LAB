# 模仿学习平台

`ubt_IL` 是基于 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) 的模仿学习平台，覆盖从 HDF5 采集数据到可部署策略模型的完整链路。转换、训练、评估全部在 **LeRobot + ROS2** 容器内完成，各机型配置相互隔离、流程完全一致：

```
HDF5 采集数据 ─► 转换为 LeRobot v3 ─► 可视化检查 ─► 训练（ACT / Pi0.5）─► 离线 MSE 评估 ─► 部署（仿真/真机）
```

数据来源：[仿真平台](../sim/index.md) 仿真采集，或真机遥操作采集。主要用途：

- **数据转换**：将仿真 / 真机采集的 HDF5 数据统一转为 LeRobot v3 标准格式
- **策略训练**：提供 ACT（天工行者 / Walker S2 EDU 探索者）与 Pi0.5 VLA（Walker S2 EDU 探索者）训练配方
- **离线评估**：训练后先做 MSE 评估快速筛除欠拟合模型，降低真机部署风险

## 平台架构

单容器流水线，每个阶段均为「脚本 + JSON 配置」，按机型分目录维护：

| 阶段 | 位置 | 说明 |
|------|------|------|
| 数据转换 | `scripts/convert/<机型>/` | `convert.sh` + `configs/*.json`，HDF5 → LeRobot v3 |
| 可视化检查 | LeRobot 可视化器 | 回放检查动作与图像对齐，避免脏数据进训练 |
| 训练 | `scripts/train/<机型>/` | `train.sh` + `configs/*.json`，ACT / Pi0.5 |
| 评估 | `scripts/eval/eval_policy.py` | 离线 MSE 评估，各机型共用 |
| 部署 | `scripts/deploy/<机型>/` | `rollout.sh` 等推理部署脚本，衔接仿真 / 真机全流程 |
| 输出 | `dataset/` · `model/` | LeRobot 数据集与模型 checkpoint 落盘目录 |

## 可视化与评估预览

数据集可视化（LeRobot 可视化器回放）：

<div class="grid video-grid" markdown>
<div markdown>

**天工行者**

![天工仿真数据集可视化](../assets/tienkung仿真数据集可视化.png){.img-uniform}

</div>
<div markdown>

**Walker S2 EDU 探索者**

![Walker 仿真数据预览](../assets/walker仿真数据预览.jpg){.img-uniform}

</div>
</div>

离线 MSE 评估曲线：

<div class="grid video-grid" markdown>
<div markdown>

**天工行者**

![天工模型离线评估曲线](../assets/tienkung模型离线评估曲线.png){.img-uniform .img-uniform--tall}

</div>
<div markdown>

**Walker S2 EDU 探索者**

![Walker 仿真离线评估曲线](../assets/walker仿真离线评估.png){.img-uniform .img-uniform--tall}

</div>
</div>

策略部署效果（训练完成的模型在仿真 / 真机上运行）：

<div class="grid video-grid" markdown>
<div markdown>

**天工行者 仿真**

<video src="../assets/tienkung仿真部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**天工行者 真机**

<video src="../assets/tienkung真机部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**Walker S2 EDU 探索者 仿真**

<video src="../assets/walker仿真部署效果.mp4" controls muted width="100%"></video>

</div>
<div markdown>

**Walker S2 EDU 探索者 真机**

<video src="../assets/walker真机部署效果.mp4" controls muted width="100%"></video>

</div>
</div>

## 文档导航

| 子模块 | 说明 |
|--------|------|
| [容器构建与使用](docker.md) | 各机型一致的容器构建、架构自动选择与容器内 Python 环境 |
| [天工行者](../tienkung/convert-train.md) | 13D/26D 转换配置、ACT 训练、离线评估 |
| [Walker S2 EDU 探索者](../walker-s2/convert-train.md) | 10D/33D 转换配置、ACT / Pi0.5 训练、离线评估 |

通用参考：[数据转换详解](../common/data-conversion.md) · [HDF5 数据布局](../common/hdf5-layouts.md) · [HF 数据/模型管理](../common/hf-manage.md)

评估通过后进入对应机型真机全流程部署：[天工行者](../tienkung/real-workflow.md) / [Walker S2 EDU 探索者](../walker-s2/real-workflow.md)。
