# 天工行者 Dex V3（TienKung Dex）模仿学习快速开始

> 对应代码：`ubt_IL/tienkung_dex/`（LeRobot 插件 `lerobot_robot_tienkung_dex`）
> 全流程：**数据准备 → 模型训练 → 策略评估 → 模型部署**

本文档给出天工行者 Dex 从克隆代码到仿真/真机部署的最简路径，按需执行每一步。各步骤的完整参数说明与可复制命令见工作流专项文档：

- 仿真工作流：→ [sim-workflow.md](./sim-workflow.md)（仿真采集 → 转换 → 训练 → 评估 → 回注仿真器部署）
- 真机工作流：→ [real-workflow.md](./real-workflow.md)（真机采集 → 转换 → 训练 → 评估 → 真机部署）

> **与另外两个机型的最大差异**：Dex 的数据里只有**右臂 + 右手**在动，左臂 / 腰 / 头 / 腿全程恒定。
> 因此动作空间是 **20D = 右臂 7 + 右手 13（因时 Inspire）**，其余关节不进模型，由
> `robot_ready.sh` 摆到预备位后保持不动。恒定维进数据集会触发已知的归一化炸裂
> （见 [README 常见问题](#9-常见问题)）。

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

## 2. 构建环境

```bash
cd ubt_IL/docker
bash run.sh build      # 构建容器镜像（首次；自动按平台选择 Dockerfile）
bash run.sh start      # 启动容器 lerobot-tienkung
bash run.sh bash       # 进入容器，后续命令均在容器内执行
```

- 宿主机项目挂载于容器 `/ubt_IL`。
- 通信链路：LeRobot（Python 3.12）—ZMQ `5559`/`5560`—> Bridge2（系统 Python 3.10）—ROS2 DDS—> 机器人/仿真器；相机推流见下表。
- Dex 的消息包（`ros2_bridge_msgs` / `inspire_hand_msgs` / `brainco_hand_msgs`）由镜像构建期
  编译到 `/opt/tienkung_dex_msgs_ws`，`bash run.sh check` 里 `tienkung_dex` 相关三行应全为 `[OK]`。

## 3. 数据准备

仿真采集的数据直接在 `ubt_sim/dataset/tienkung_dex/<epoch>/trajectory.hdf5`，
转换时把 `SRC_ROOT` 指过去即可：

```bash
SRC_ROOT=$PROJECT_ROOT/../ubt_sim/dataset/tienkung_dex \
bash /ubt_IL/scripts/convert/tienkung_dex/convert.sh --overwrite
```

转换配置：`/ubt_IL/scripts/convert/tienkung_dex/configs/tienkung_dex_20d_1RGB.json`
（20D，因时 13 通道）。详细参数与静止帧处理见 [sim-workflow.md §3](./sim-workflow.md#3-数据转换hdf5--lerobot)。

> ⚠️ **`TRIM_STATIONARY` 对 Dex 默认关闭**：Dex 的重复帧成因是 15 Hz 录制跑在约 11 Hz 的
> 仿真主循环上，重复散布在整段运动里，静止裁剪裁不掉，反而会切碎动作。转换配置里的
> 20 维已剔除恒定维，不依赖裁剪。

## 4. 模型训练

脚本：`/ubt_IL/scripts/train/tienkung_dex/train.sh`（默认配置
`configs/train_config_tienkung_dex_sim_pick_apple.json`，30 集 / 3 万步）。完整参数见
[sim-workflow.md §4](./sim-workflow.md#4-模型训练)。

```bash
bash /ubt_IL/scripts/train/tienkung_dex/train.sh
```

## 5. 模型评估

脚本：`/ubt_IL/scripts/eval/eval_policy.py`，在数据集上离线推理，对比预测动作 vs 真值动作的 MSE。
完整参数见 [sim-workflow.md §5](./sim-workflow.md#5-策略评估离线-mse)。

## 6. 模型部署

**顺序不能反**：先把机器人摆到预备位，再跑策略。

```bash
# ① 预备位（四段到位；仿真 ROS_DOMAIN_ID=146 / 真机 0）
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh          # 仿真
bash /ubt_IL/scripts/deploy/tienkung_dex/robot_ready.sh --real   # 真机

# ② 策略（单次 rollout）
POLICY_PATH=/ubt_IL/model/tienkung_dex_sim_pick_apple_act/checkpoints/last/pretrained_model \
JOINT_CONFIG=tienkung_dex_20 FPS=15 DURATION=60 \
bash /ubt_IL/scripts/deploy/tienkung_dex/rollout.sh
```

- 仿真部署：→ [sim-workflow.md §6](./sim-workflow.md#6-模型部署仿真)
- 真机部署：→ [real-workflow.md §6](./real-workflow.md#6-模型部署真机)
- 脚本清单与全部环境变量：→ [scripts/deploy/tienkung_dex/README.md](../../scripts/deploy/tienkung_dex/README.md)

> 注意：`JOINT_CONFIG` 必须与训练 DOF 一致（因时 `tienkung_dex_20` / 强脑 `tienkung_dex_brainco_13`）。
> **预备位不是回零**，也不能一步跳到预备位——从站立位直达会让手贴着桌面从体侧扫过去。

## 7. 相机链路

| 场景 | 端口 | 链路 |
|------|------|------|
| 仿真 | **5758** | ubt_sim 的 `AsyncJpegCameraSender` 直发 JPEG，`dex_camera` 原生剥 12 字节 `dI` 头 |
| 真机 | **5558** | Bridge2 带 `--enable-image`，把 `/ob_camera_head/color/image_raw` 编成 JPEG |

## 8. 动作向量（20D）

```
[0-6]   右臂 7 : shoulder_pitch_r, shoulder_roll_r, shoulder_yaw_r,
                 elbow_pitch_r, elbow_yaw_r, wrist_pitch_r, wrist_roll_r
[7-19]  右手13 : thumb_yaw, thumb_mcp, thumb_pip, index_yaw, index_mcp, index_pip,
                 middle_yaw, middle_mcp, middle_pip, ring_mcp, ring_pip,
                 pinky_mcp, pinky_pip
```

⚠️ 数据集手序（URDF: thumb→pinky）≠ 因时通道序（pinky→thumb），落地映射是
`[10,11,12,5,4,9,3,2,8,1,7,0,6]`。Bridge2 按关节名散列、不按位，启动时逐字比对关节名，不一致直接拒启。

## 9. 常见问题

| 现象 | 处理 |
|------|------|
| 训练时归一化炸裂（QUANTILES NaN/Inf） | 数据含恒定维度；Dex 只取右臂 7 + 右手 13，恒定维已在转换配置里剔除 |
| 部署后手不动 / 手指整体错位 | `HAND_VARIANT` 与实物手型不一致，或插件与 Bridge2 的常量副本漂移（Bridge2 启动日志会报「通道表不一致」） |
| 部署无相机图像 | 仿真看 5758 是否有流（ubt_sim 的 `AsyncJpegCameraSender`）；真机看 Bridge2 是否带 `--enable-image`、端口是否 5558 |
| 仿真里机器人不动 | `ROS_DOMAIN_ID` 是否为 146；`robot_ready.sh` 是否先跑过；`/sim/cmd_reset` 复位场景 |
| 手臂起步就撞桌子 | 没跑 `robot_ready.sh`，或用了 `--stage-durations` 强行动作过快 |
| 常驻服务 `load` 被拒 | Dex 不支持热切换模型，`shutdown` 后重新 `launch` |
| 训练报 `FileExistsError` | 从头训练时 `OUTPUT_DIR` 已存在且 `RESUME=false`；换新目录或删旧目录 |

更多分阶段排查项见 [sim-workflow.md](./sim-workflow.md) §7 与 [real-workflow.md](./real-workflow.md) §7。
