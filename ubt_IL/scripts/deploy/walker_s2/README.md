# Walker S2 部署（Rollout）

容器内执行，前置条件：Bridge2 已由容器 entrypoint 自动启动。

部署脚本：[rollout.sh](rollout.sh)

### 19D 仿真模型 -> 19D PGC 夹爪真机

```bash
cd /ubt_IL/lerobot

POLICY_PATH=/ubt_IL/model/Walker_S2_sim_act/checkpoints/last/pretrained_model \
ROBOT_MODEL=walker_s2_gripper_19d \
FPS=15 \
DURATION=30 \
ALLOW_DIM_ONLY_POLICY=1 \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

### 10D 仿真模型 -> 真机部署（act_async）

10D（右臂 + 头 + 右夹爪）仿真训练的 ACT 模型，异步 chunk 推理部署到真机。

```bash
cd /ubt_IL/lerobot

ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 \
FPS=20 DURATION=60 \
bash /ubt_IL/scripts/deploy/walker_s2/rollout.sh
```

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填，无默认值** | checkpoint 目录（含 `config.json`） |
| `ROBOT_MODEL` | `walker_s2_v4_hand_31d` | 机器人配置文件名前缀（不含 `.json`） |
| `ROBOT_CONFIG` | `configs/<ROBOT_MODEL>.json` | 机器人配置文件完整路径 |
| `STRATEGY` | `base` | rollout 策略类型 |
| `FPS` | `15` | 控制频率 |
| `DURATION` | `30` | 运行时长（秒） |
| `TASK` | `walker s2 rollout` | 任务描述 |
| `ALLOW_DIM_ONLY_POLICY` | `0` | 策略无 action names 时允许仅按维度匹配（设 `1` 开启） |

### 机器人配置

| 配置文件 | 维度 | 末端执行器 |
|----------|------|-----------|
| [walker_s2_gripper_19d.json](configs/walker_s2_gripper_19d.json) | 19D（17 body + 2 gripper） | PGC 1DOF 夹爪 |
| [walker_s2_v4_hand_31d.json](configs/walker_s2_v4_hand_31d.json) | 31D（17 body + 14 hands） | V4 灵巧手 7DOF |

### 安全预检

脚本启动后自动执行 action 维度匹配检查，不通过则拒绝部署：

1. 读取 robot config 的 `action_order`，计算期望维度
2. 读取 policy `config.json` 的 `output_features.action.shape`，获取实际维度
3. 维度不匹配 -> 报错退出
4. 维度匹配但有 action names -> 校验 names 顺序一致性
5. 维度匹配但无 action names -> 需 `ALLOW_DIM_ONLY_POLICY=1` 才放行

### 安全参数

配置文件中 `safety` 段：

- `max_relative_target: 0.02` - 单步相对目标限幅
- `disable_torque_on_disconnect: true` - 断开连接自动卸力

---

# Walker S2 离线策略评估（eval_policy.py）

不连机器人，对训练好的策略在 LeRobot 数据集上做离线 MSE 评估（预测 vs 真值），并生成逐
episode 对比图。部署到真机前先用它量化策略质量。脚本（与天工共用）：[eval_policy.py](../../eval/eval_policy.py)。

```bash
/lerobot/.venv/bin/python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/015000/pretrained_model \
  --dataset-path /ubt_IL/dataset/Walker_S2_sim_10_2RGB \
  --episodes 10 \
  --inference-freq 1 \
  --plot \
  --plot-dir /ubt_IL/scripts/eval/output/eval_Walker_S2_sim_10_2RGB_act_60k \
  --output /ubt_IL/scripts/eval/output/eval_Walker_S2_sim_10_2RGB_act_60k/results.json \
  --device cuda
```

| 参数 | 说明 |
|------|------|
| `--policy-path` | pretrained_model 目录（含 `config.json` + `model.safetensors`） |
| `--dataset-path` | LeRobot 数据集根目录（含 `meta/info.json`） |
| `--episodes` | 评估 episode 数（默认全部） |
| `--inference-freq` | 每 N 步推理一次，中间复用预测 chunk（模拟真机部署；`1`=逐步推理） |
| `--plot` / `--plot-dir` | 生成逐 episode 预测-vs-真值图及输出目录 |
| `--output` | 结果 JSON 保存路径 |
| `--device` | `cuda` 或 `cpu`（默认 `cuda`，不可用时自动回退 `cpu`） |

---

# Walker S2 推理服务器（常驻预热）

`rollout.sh` 每次都重跑冷启动（policy 加载到 CUDA + 桥接拉起），耗时长。推理服务器
**常驻**一个进程，一次性预热加载模型 + 连机器人，之后外部经 ZMQ 指令随时
`start/stop/home/status/shutdown`，并可 `load` 热切换模型，免去重复冷启动。

> ✅ **真机已验证（2026-08-09）**：预热 ~6s 到 READY；`start`→RUNNING（chunk_id 0→211，
> loop_hz≈fps=15）；`stop`/`home`（无抖动）/`shutdown` 全通过。未测 `load` 热切换。

脚本：[inference_server.sh](inference_server.sh)（服务端）+ [inference_client.py](inference_client.py) /
[inference_client.sh](inference_client.sh)（客户端）。服务端实现在
`lerobot/src/lerobot/scripts/inference_server.py`，复用 `build_rollout_context` + `ACTAsyncChunkEngine.pause/resume`，
不改动引擎/桥接/walker 核心。

### 快速开始

```bash
# 1. 拉起服务（容器内，后台预热 ~6s 到 READY；引擎 paused，不动机器人）
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Pick_part_real_10d_2RGB_act/checkpoints/100000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=15 \
bash /ubt_IL/scripts/deploy/walker_s2/inference_server.sh

# 2. 轮询状态到 READY（此后 start/stop/home 秒级响应，无需再加载）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh status

# 3. 开始 / 停止推理（动！）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh stop

# 4. 回原点 / 关停（均回 home_position）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh shutdown
```

> 容器内 `python` 不在 PATH；用 `python3` 或 `bash inference_client.sh`（后者自动选
> `/lerobot/.venv/bin/python`）。`start`/`home`/`shutdown` 会让机器人运动，请在机旁执行。

### 架构

- **预热**：复用 `build_rollout_context`（policy + robot + processor + engine），与 `lerobot-rollout` 一致。
- **启停**：`ACTAsyncChunkEngine.pause()/resume()/reset()` 映射 stop/start/新 episode；引擎后台线程整块推 chunk 给桥接（300Hz densify），主循环只喂观测。
- **控制接口**：ZMQ REP(5570) 收发指令 + ZMQ PUB(5571) 广播状态（与桥接 5561/5562/5563 同体系）。
- **load 热切换**：整体重建 context（stop 旧 engine + disconnect 旧 robot + `build_rollout_context` 新 cfg），含机器人重连；`LOADING` 期间主线程跳过观测获取。
- **home**：先 `pause` 再 `WalkerRobot._send_home_chunk`（canonical `home_position`，300Hz 单源），避免双源抖动。

### 远程拉起服务（指定参数加载模型）

本地拉起见上文「快速开始」。亦可在另一台机器经客户端远程拉起（SSH+docker-exec 到
**跑容器的 Jetson**，自动轮询至 READY）：

```bash
bash inference_client.sh launch \
  --policy-path /ubt_IL/model/Pick_part_real_10d_2RGB_act/checkpoints/100000/pretrained_model \
  --robot-model walker_s2_10d --inference-hz 2 --fps 15 \
  --remote --host 192.168.11.3 --ssh-user walker
```

> **注意区分两台机器**：
> - **Jetson `192.168.11.3`**（SSH `walker`，端口 22/2222）：跑容器 `lerobot-tienkung`、做推理。
>   `launch --remote` 的 `--host` 指它。
> - **机器人主控 PC `192.168.11.2`**（SSH `ubt`，端口 2222）：`dev_mode.sh` 切开发者模式 / ROS2，
>   **不跑推理容器**，别用它做 `--host`。

`launch --remote` 默认 `--ssh-user ubt --ssh-port 22 --container lerobot-tienkung`；连 Jetson 需
`--ssh-user walker`。SSH 用户要对 `sudo docker exec` 有权限（免密 sudo）。不带 `--remote` 则在本地
`bash inference_server.sh` 拉起。`--host` 同时是 SSH 目标和 ZMQ 轮询目标（容器 `--network=host`，
5570 即该机端口）。

### 客户端指令

```bash
bash inference_client.sh status                       # 查状态（state/model/loop_hz/chunk_id/...）
bash inference_client.sh start                        # 开始推理（engine reset+resume -> RUNNING）
bash inference_client.sh stop                         # 结束推理（engine pause -> READY，hold 当前位姿）
bash inference_client.sh home                         # 回 home_position（先 pause 再平滑 chunk）
bash inference_client.sh load --policy-path /ubt_IL/model/<other>/.../pretrained_model --robot-model walker_s2_10d
                                                       # 运行中热切换模型（整体重建含重连）
bash inference_client.sh shutdown                     # 关服务（回 home + kill 桥接 + 退出）
bash inference_client.sh watch                        # 订阅状态 PUB 流（Ctrl-C 退）
```

ZMQ 指令默认连 `127.0.0.1:5570`；容器为 `--network=host`，故宿主机/远程用 `--host <jetson_ip>` 即可直连。
`launch --remote` 需 SSH 用户对 `docker exec` 有 sudo 权限（参照 `run.sh`）。

### 状态机

`LOADING`(预热/重建) -> `READY`(空闲) ⇄ `RUNNING`(执行)；`home` -> `RETURNING_HOME` -> `READY`；
`load` -> `LOADING` -> `READY`；致命错误 -> `ERROR`；关机 -> `SHUTTING_DOWN`。

### 环境变量（inference_server.sh）

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填** | pretrained_model 目录 |
| `ROBOT_MODEL` | `walker_s2_31d` | ROBOT_MODELS 注册表键 |
| `ROBOT_CONFIG` | 空 | 自定义 robot config JSON（覆盖 ROBOT_MODEL） |
| `INFERENCE_TYPE` | `act_async` | 推理引擎类型（start/stop 需 act_async） |
| `INFERENCE_HZ` | `0.6` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数（0=不截断） |
| `FPS` | `13` | 控制环 FPS |
| `BLEND_HORIZON` | `10` | 桥接动作块融合重叠点数 |
| `BODY_PUBLISH_HZ` | `300` | 桥接 300Hz body 发布频率 |
| `BODY_V_MAX` | `2.0` | 桥接 rate_limit 单关节最大速度 |
| `SERVER_CMD_PORT` | `5570` | ZMQ REP 指令端口 |
| `SERVER_STATUS_PORT` | `5571` | ZMQ PUB 状态端口 |
| `PREVIEW_CAMERA` | `0` | 服务器默认不开预览窗口 |

> 单例：`/tmp/walker_inference_server.pid` 在跑则拒绝重复拉起。日志：`/tmp/walker_inference_server.log`。
> `start/stop` 依赖 act_async 的 pause/resume；`sync` 引擎无此语义，start 会报错。
> `--use_torch_compile` 不被服务器支持（ACT 无需 compile warmup）。

