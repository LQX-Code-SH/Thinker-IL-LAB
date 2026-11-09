# Walker C1 EDU 共创者 部署

支持两种策略维度，头部单目 RGB 640×512（5:4，真机源 1920×1536 分辨率的三分之一）：

| 模型键 | 配置 | 维度 |
|---|---|---|
| `walker_c1_26d` | `configs/walker_c1_26d.json` | 26（双臂 14 + 双手 12） |
| `walker_c1_13d` | `configs/walker_c1_13d.json` | 13（右臂 7 + 右手 6；左臂/左手由 inactive_fill 保持 home） |
仿真与真机共用同一套 ROS2 话题（`mc_task_msgs` / `mc_state_msgs`），靠 `ROS_DOMAIN_ID` 隔离。

```
LeRobot (3.12) ──ZMQ 5661/5662──► ros2_walker_c1_bridge.py (3.10) ──ROS2 DDS──► 真机 / ubt_sim
                                  ▲
walker_c1_camera_relay.py ──ZMQ 5663──┘   (仿真: sensor_msgs/Image @ /sensor/camera/head/color/raw)
                                           (真机: shm_msgs/Image6m @ /sensor/camera/head_stereo_left/color/raw)
```

相机话题/类型可用 `CAMERA_TOPIC` + `CAMERA_MSG_TYPE`（rollout.sh / inference_server.sh）或
`WALKER_C1_CAMERA_TOPIC` + `WALKER_C1_CAMERA_MSG_TYPE`（插件）覆盖。`/compressed` 结尾自动
推断成压缩图，其余默认 `sensor_msgs/Image`；真机 `shm_msgs/Image6m` 必须显式给类型。

C1 真机统一使用 `/sensor/camera/head_stereo_left/color/raw`（`shm_msgs/Image6m`）：
`inference_server.sh` 与 `inference_client.py launch` 默认即此话题；`rollout.sh` 在
`WALKER_C1_REAL=1` 且未显式指定时也默认此话题。仿真推理需显式覆盖为
`/sensor/camera/head/color/raw` + `sensor_msgs/Image`。

## 文件

| 文件 | 作用 |
|---|---|
| `configs/walker_c1_26d.json` | 机器人硬件配置（26 维 action_order、双臂分组、手部限位、端口） |
| `sdk_mode.sh` / `sdk_mode.py` | 开发者模式 / 运控权限管理（`status` / `check` / `half_sdk` / `normal`） |
| `sdk_guard.sh` | 部署门禁：非 half_sdk 时拒绝 rollout / 摆位 / 推理服务 |
| `install_ccapi.sh` | 在 ubt_IL 容器内安装 CC API Python SDK（`ubt_robot` wheel） |
| `rollout.sh` | 单集/多集 rollout（每集复位 + 进程重启） |
| `robot_ready.py` / `.sh` | 三段式摆位（抬肩 → 收肘 → 预备位）+ 手张开；每段默认 ≥5 s |
| `robot_home.py` / `.sh` | 回 home（14 臂关节全 0），路径为上面三段的严格逆序（腕/肩俯仰回零 → 展开肘 → 放下肩） |
| `replay.py` | 回放数据集 action（真机）：数据按 fps 走时间轴，**物理下发固定 100 Hz 并对相邻帧线性插值**（30 Hz 直发实测末端偏差 0.56 rad，插值后 0.067 rad）；未在 action 里的关节用实测值补齐，手部按配置顺序下发 |
| `reset.sh` / `reset.py` | `--mode sim-scene`（`/sim/cmd_reset`）/ `task`（调用 robot_ready）；**必须走 `.sh`**（需系统 python3 的 rclpy） |
| `inference_server.sh` | 常驻推理服务（ZMQ 5690/5691） |
| `inference_client.py` / `.sh` | launch/load/start/stop/home/status/watch |
| `preview_camera.py` | 头部相机预览（`sensor_msgs/Image`） |
| `infer_check.py` | 单帧离线推理自检（checkpoint + 数据集） |

## 快速开始（仿真）

```bash
# ① 宿主机：启动 ubt_sim（另开终端）
#    UBT_SIM_ROBOT=walker_c1 bash ubt_sim/scripts/start_sim.sh

# ② 容器内：确认 ROS 域一致（ubt_sim 默认 0；隔离时两边都设 146）
export ROS_DOMAIN_ID=0

# ③ 预览相机
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1.jpg

# ④ 摆到预备位
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh

# 回放数据集（13 维：右臂 7 + 右手 6）
python3 /ubt_IL/scripts/deploy/walker_c1/replay.py --episode 0 --dry-run
python3 /ubt_IL/scripts/deploy/walker_c1/replay.py --episode 0 --transition 5
#    --hz 100     物理下发频率（默认 100，与 robot_ready 一致）
#    --rate 30    数据时间轴帧率（默认取 info.json 的 fps）
#    --side 仅在 robot_ready 里可用（both/left/right）

# ⑤ rollout
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

`ubt_sim` 的 C1 桥接使用 ZMQ 5655-5658，本套件使用 5661-5663，可同时运行。

## SDK 模式（真机部署前置：half_sdk）

C1 的手臂关节只在 **开发者模式 + 半身 SDK 权限** 同时成立时才接收 ROS2 指令：

| `developModeState` | 手臂指令 | 说明 |
|---|---|---|
| `NormalActive`（`normal`） | 忽略 | 内置运控控制全身，只能调 CC-API 技能 |
| `HalfSdkActive`（`half_sdk`） | 执行 | **VLA 唯一档位**：头/臂/腰由 SDK 控制，腿部由系统保持平衡（需 `legStatus=standing`） |
| `FullSdkActive`（`full_sdk`） | 执行 | 全身电机交出、内置平衡退出，必须悬吊，**不要用于 VLA** |

### 切换与查看

```bash
# 默认经 SSH 在机器人开发者容器内执行（ubt@192.168.11.3:2222，密码见现场手册）
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh status     # 工作模式/运控权限/腿部/急停
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh check      # 只校验，0=可部署，2=不满足
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh half_sdk   # develop -> 腿部 STAND -> half_sdk
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh normal     # 回到 normal（保留开发者模式）
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh work-mode joystick   # 退出开发者模式
```

`half_sdk` 是幂等的：已处于 half_sdk 时只打印状态；腿部非 `standing` 时会先调 `A000004`
技能切 STAND（`--no-leg-stand` 可关闭自动切换）。切换过程会经过
`SdkExiting/SdkStarting` 等过渡态，脚本轮询到 `HalfSdkActive` 才返回 0。

**密码/非交互**：默认由 ssh 交互提示输入。CI 或脚本化场景可
`export WALKER_C1_SDK_PASSWORD='<密码>'`（有 `sshpass` 用 `sshpass`，没有则自动走
`SSH_ASKPASS`，部署容器内无需额外装包）。其它可覆盖项：
`WALKER_C1_SDK_HOST/PORT/USER`、`WALKER_C1_SDK_CONTAINER`（ssh 落在宿主机时用
`--ssh-container <name>` 走 `sudo docker exec`）、`WALKER_C1_SDK_SSH_OPTS`。

**传输方式**：默认 `auto` —— 当前环境能 `import ubt_robot` 且能访问 `/tmp/cc_api.sock`
时本地执行，否则 SSH 到开发者容器（脚本经 stdin 送入远端 `python3`，不落盘、不改远端环境）。
也可显式 `--local`（在 CC 容器内）、`--remote`、或 `--address 0.0.0.0:51000 --token <token>` 跨机连接。

### 部署门禁

`rollout.sh`、`robot_ready.sh` / `robot_home.sh`（非 `--dry-run`）、`inference_server.sh`
启动前都会调用 `sdk_guard.sh`，非 `half_sdk` 直接拒绝：

```bash
SDK_GUARD=1        # 真机默认（WALKER_C1_REAL=1 时自动取 1）：强制校验
SDK_GUARD=auto     # 检测到真机才校验（仿真默认）
SDK_GUARD=0        # 跳过（仅离线调试）
SDK_AUTO_SWITCH=1  # 不满足时自动执行 sdk_mode.sh half_sdk，再复检
```

真机最简命令（其余全用默认：13d / task / 30 s / act_async / 1 Hz / 真机相机 / 门禁）：

```bash
POLICY_PATH=/ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

仿真/离线：

```bash
WALKER_C1_REAL=0 SDK_GUARD=0 \
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
RESET_MODE=sim-scene \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

### 在 ubt_IL 容器内安装 CC API（可选）

```bash
# 先把 C1 SDK 拷进仓库挂载目录（容器内 /ubt_IL 可见），再安装：
#   cp -r /home/walker/work/walker_c1_sdk <repo>/ubt_IL/walker_c1_sdk
bash /ubt_IL/scripts/deploy/walker_c1/install_ccapi.sh
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh --local status
```

> 注意：`/tmp/cc_api.sock` 在机器人开发者容器内，ubt_IL 容器默认看不到。安装 wheel 后仍需
> 挂载该 socket 或用 `--address/--token` 跨机连接；否则继续用默认远程模式即可。

### 排查

| 现象 | 处理 |
|---|---|
| `未检测到 ubt_robot` | 默认远程模式下不该出现；出现说明加了 `--local` 但容器没装 SDK |
| `SSH 连接失败` / 一直提示密码 | 确认 `ubt@192.168.11.3:2222` 可达、密码正确；非交互用 `WALKER_C1_SDK_PASSWORD` + `sshpass`；ubt_IL 容器内没有 ssh 时改在宿主机执行本脚本 |
| 腿部非 `standing` | 用遥控器/Web 切站立，或让脚本自动执行 `A000004`（默认已开启） |
| `developModeState=sdk_error` | 按急停 -> 退出开发者模式或重启运控，再重新 `half_sdk` |
| `workMode` 不是 `develop` | `sdk_mode.sh half_sdk` 会先自动切 develop；也可 `sdk_mode.sh develop` |
| 切换后仍不动 | `sdk_mode.sh check` 看急停/上电；确认 `ROS_DOMAIN_ID`、RMW（CycloneDDS）与主控一致 |

## 推理后端选择（walker 系列：优先异步）

- **推荐 `INFERENCE_TYPE=act_async`**（真机默认）：chunk 走桥接融合/插值/300 Hz 限速，控制环 14.9 Hz，动作连续。
- **`sync`（串行）仅用于调试**：单步目标直接下发会阶跃顿挫，因此 C1 默认加 0.15 s 单动作平滑
  （`WALKER_C1_SINGLE_ACTION_SMOOTH_S`）。**串行必须保持平滑开启**，置 0 时 `rollout.sh`
  启动会告警（仅建议临时排查使用）。
- 平滑后串行稳态约 14.9 Hz（FPS=15）/ 29.5 Hz（FPS=30），但目标更新仍受推理节奏限制，
  综合表现不如异步。

## rollout.sh 环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `POLICY_PATH` | —（必填） | `pretrained_model` 目录 |
| `ROBOT_MODEL` | 真机 `walker_c1_13d` / 仿真 `walker_c1_26d` | `ROBOT_MODELS` 注册表键（同时作为 `--robot.joint_config`），随 `WALKER_C1_REAL` 切换 |
| `ROBOT_CONFIG` | `configs/<ROBOT_MODEL>.json` | 硬件配置 JSON（按 `ROBOT_MODEL` 自动选择） |
| `ROBOT_TYPE` | `walker_c1` | 插件注册名 |
| `EPISODES` | `1` | 集数；base strategy 不支持多集，>1 时由外层循环 + 每集复位实现 |
| `DURATION` | `30` | 每集最长秒数 |
| `FPS` | `15` | 控制环 FPS |
| `TASK` | `walker c1 pick place` | 任务描述 |
| `RESET_MODE` | 真机 `task` / 仿真 `sim-scene` | 每集前复位：`sim-scene` / `task` / `both` / `none` |
| `ALLOW_DIM_ONLY_POLICY` | `1` | 1=只校验维度；0=同时校验 `action_feature_names` |
| `INFERENCE_TYPE` | 真机 `act_async` / 仿真 `sync` | `sync` / `act_async`（chunk → bridge 融合 300 Hz） |
| `INFERENCE_HZ` | 真机 `1` / 仿真 `2` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | act_async chunk 截断步数（0=不截断） |
| `PREVIEW_CAMERA` | `0` | 1=同时开预览窗口 |
| `SDK_GUARD` | 真机 `1` / 仿真 `auto` | half_sdk 门禁：`1` 强制 / `auto` 检测到真机才校验 / `0` 跳过 |
| `SDK_AUTO_SWITCH` | `0` | 1=门禁不通过时自动执行 `sdk_mode.sh half_sdk` 再复检 |
| `WALKER_C1_REAL` | **`1`** | 1=真机（默认，含 13d/task/act_async/门禁/真机相机）；`0`=仿真/离线 |
| `WALKER_C1_SINGLE_ACTION_SMOOTH_S` | `0.15` | sync 单动作平滑窗口（秒），`0`=关闭（直接下发）；仅影响 `INFERENCE_TYPE=sync` |

## 13 维（单臂）部署

```bash
ROBOT_MODEL=walker_c1_13d \
ROBOT_CONFIG=/ubt_IL/scripts/deploy/walker_c1/configs/walker_c1_13d.json \
POLICY_PATH=/ubt_IL/model/walker_c1_sim_13d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

13 维配置的 `action_order` 只含右臂+右手；`body.groups` 仍保留 `left_arm`，
`inactive_fill` 会把左臂 7 + 左手 6 填成 home 并下发（真机位置保持、仿真 home）。

## 配置要点

- `action_order` 必须与训练数据集的 `observation.state/action` 顺序完全一致（13 或 26 维）
- `body.groups.head` / `waist` 为空：头/腰/腿不下发，机器人保持当前位
- `body.lock_joints` 必须为空：`lock_joints` 语义是"跳过不下发"
- 手部限位（rad，短名）：`thumb_swing ≤0.96`、`thumb_mcp ≤1.04`、其余四指 `≤1.57`
- `zmq` 端口固定 5661/5662/5663；推理服务 5690/5691

## 常驻推理服务

```bash
# 机器人侧拉起（后台预热）
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/inference_server.sh

# 操作机侧
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh status
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh stop
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh load \
  --policy-path /ubt_IL/model/<other>/checkpoints/last/pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_26d
```

远程拉起：`--remote --host 192.168.11.3 --ssh-user ubt --container lerobot-tienkung`。

## 离线自检与评估

```bash
python /ubt_IL/scripts/deploy/walker_c1/infer_check.py \
  --checkpoint /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset /ubt_IL/dataset/walker_c1_sim_26d_1RGB --frame 0

python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
  --episodes 5 --inference-freq 15 --plot \
  --output /ubt_IL/scripts/eval/output/eval_walker_c1_sim_26d_act/results.json
```

## 真机前置

1. 机器人切 SDK 控制：`bash sdk_mode.sh half_sdk`（进入开发者模式 + 腿部 STAND + 半身 SDK；
   `status` 查看、`normal` 恢复；`full_sdk` 会交出平衡，勿用于 VLA）
2. 车端/操作机 `ROS_DOMAIN_ID` 一致（真机默认 0）
3. `docker/fastdds_no_shm.xml` 白名单加入本机网卡 IP
4. 先 `robot_ready.sh --dry-run` 确认目标位姿，再实跑
5. 首次部署建议 `DURATION=10` 短时验证
6. 真机部署请带 `WALKER_C1_REAL=1`（或 `SDK_GUARD=1`）启用 half_sdk 门禁

## 常见问题

| 现象 | 排查 |
|---|---|
| 机器人不动 | `preview_camera.py --once` 能否出图；`ROS_DOMAIN_ID`/`RMW` 是否与仿真一致；bridge 日志 `ros2_walker_c1_bridge` |
| 手臂只动一部分 | 策略维度与 `action_order` 不符（预检会退出）；或左手数据全程为 0（该任务只用了右手，正常） |
| 动作抖动 | 提高 `FPS` 与 `--inference-hz`；或切 `act_async` 走 chunk 融合 |
| 相机超时 | 确认 `sensor_msgs/Image` 话题名；QoS 必须 BEST_EFFORT |
| 抓空/苹果被打飞 | 仿真 reset 后残留状态可能不干净：**重启 sim_runner**，用 `/sim/cmd_set_object_pose` 把苹果放回训练位置，再 `robot_ready.sh` |
| `reset.py` 报 `No module named rclpy` | 用 `reset.sh`（走 `/usr/bin/python3`），不要用 venv 的 `python3` |
| 端口占用 | `ss -lntp \| grep 566`；同一主机不要同时跑两套 bridge |
