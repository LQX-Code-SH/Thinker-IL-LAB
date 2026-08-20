# 数据转换详解（HDF5 -> LeRobot）

转换脚本将各机型采集的 HDF5 原始数据统一转换为 LeRobot v3 数据集（`meta/`、`chunk-000/`（data parquet）、`videos/`），供训练 / 评估 / 可视化使用。

## 脚本布局

| 机型 | 脚本 | 配置 |
|------|------|------|
| `tienkung_pro` | `ubt_IL/scripts/convert/tienkung_pro/`（`convert.sh`、`convert_grasp_bottle.sh`） | `ubt_IL/scripts/convert/tienkung_pro/configs/` |
| `walker_s2` | `ubt_IL/scripts/convert/walker_s2/`（`convert.sh`、`convert_real_to_lerobot_v3.py`） | `ubt_IL/scripts/convert/walker_s2/configs/` |

通用转换器（非机型专属）在 `ubt_IL/scripts/convert/common/`：

| 脚本 | 说明 |
|------|------|
| `convert_to_lerobot.py` | 通用 HDF5 -> LeRobot 转换器（被各机型 `*.sh` 调用） |
| `isaaclab2lerobot.py` / `isaaclab2lerobotv3.py` | IsaacLab 仿真数据转换（v2/v3） |
| `lerobot2isaaclab.py` | LeRobot -> 单 HDF5 反向转换 |
| `fix_stats_format.py` | 修复数据集 stats 格式 |
| `merge_datasets.sh` | 多批数据集合并（如多批次真机数据合并训练） |
| `hf_manager.py` | Hugging Face 数据集与模型管理（见 [HF 管理](hf-manage.md)） |
| `all_robot_h5_info*.md` | 跨机型 HDF5 布局参考文档（见 [HDF5 数据布局](hdf5-layouts.md)） |

## 环境变量

各机型 `convert.sh` 透传以下环境变量（标"默认"的可省略；其余可 `bash convert.sh -h` 查看）：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `SRC_ROOT` | 各机型默认 | HDF5 源根目录（每个 episode 一个含 HDF5 文件的子目录） |
| `TGT_PATH` | `/ubt_IL/dataset` | LeRobot 数据集输出根目录 |
| `CONFIG` | 各机型默认 | 转换配置 JSON：字段筛选 + 关节/相机映射关系 |
| `REPO_ID` | - | 输出数据集名称（本地目录名 + HF repo 名，训练时保持一致） |
| `ROBOT_TYPE` | - | 机器人类型（`tienkung` / `walker_s2`，写入 meta） |
| `TASK_NAME` | - | 任务名称（写入每帧 `task` 字段，训练时作为语言条件） |
| `FPS` | 机型默认（15/13） | 目标帧率；`auto` 取源 HDF5 频率 |
| `VCODEC` | `h264` | 视频编码器（`h264`/`jpeg`/`png`） |
| `HDF5_REL_PATH` | 机型默认 | HDF5 相对 episode 目录的路径 |
| `RESAMPLE_FPS` | 空（不重采样） | 目标帧率，启用重采样 |
| `LABEL_ROOT` | 空 | 标签根目录（有任务标签时使用） |
| `TRIM_STATIONARY` | 空（关闭） | `1` 开启静止帧裁剪（见下） |

> 其他透传参数：追加 `--overwrite` 覆盖已存在数据集；`--save_one true` 单文件模式。

## 静止帧裁剪

数据集静止帧会使模型推理陷入**局部静止**（动作停止）。转换时设 `TRIM_STATIONARY=1` 开启裁剪：静止游程超过 cap 时截断。

| 变量 | 默认 | 说明 |
|------|------|------|
| `STATIONARY_KEY` | `action` | 静止判定键名 |
| `STATIONARY_WINDOW` | 自动（`0.3 × fps`） | 静止判定窗口 |
| `STATIONARY_THRESH` | `0.03` | 静止判定阈值（**归一化阈值**，适配 grip 行程 0.05 vs 关节 0.5 的量级差异） |
| `STATIONARY_CAP` | `8` | 连续静止帧数上限 |
| `STATIONARY_MIN_RUN` | `3` | 有效运行段最短帧数 |
| `STATIONARY_RANGE_EPS` | `1e-3` | 静止区间容差 |
| `STATIONARY_DIAGNOSE` | 空 | `1` 只统计静止分布不写盘（校准阈值用） |

```bash
# 用法：在转换命令前加 TRIM_STATIONARY=1
TRIM_STATIONARY=1 \
CONFIG=... SRC_ROOT=... TGT_PATH=... REPO_ID=... \
bash /ubt_IL/scripts/convert/<机型>/convert.sh
```

## 常见问题

| 问题 | 处理 |
|------|------|
| 报 `FileNotFoundError`（HDF5） | `SRC_ROOT` 路径错误；确认 HDF5 已同步到容器可访问目录 |
| 报"配置不存在" | 显式指定 `configs/` 下实际存在的配置文件 |
| 真机数据报缺深度流（天工） | 真机 HDF5 无深度是正常的；使用 `*_real.json`（仅 RGB）配置 |
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度（锁死关节/夹爪），std=0 致归一化除零；改用低维配置转换剔除死维度 |
| 数据集帧率与预期不符 | `FPS=auto` 取源频率，或 `RESAMPLE_FPS` 重采样 |

各机型的具体转换命令与配置选择，见对应机型页：[天工行者](../tienkung/convert-train.md) / [Walker S2 EDU 探索者](../walker-s2/convert-train.md)。
