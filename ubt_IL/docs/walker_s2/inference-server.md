# Walker S2 EDU 探索者 推理服务器（常驻预热）

> `rollout.sh` 每次冷启动都要重新加载 policy 到 CUDA + 拉起桥接，耗时长。推理服务器**常驻**一个进程，
> 一次性预热加载模型 + 连接机器人/仿真桥接，之后外部经 ZMQ 指令随时 `start/stop/home/status/shutdown`，
> 并可 `load` 热切换模型，免去重复冷启动。
>
> 适用于 **仿真回注部署** 与 **真机部署** 两种场景，差异仅在桥接地址与相机来源。

## 目录

- [1. 脚本与架构](#1-脚本与架构)
- [2. 快速开始](#2-快速开始)
- [3. 客户端指令](#3-客户端指令)
- [4. 远程拉起服务](#4-远程拉起服务)
- [5. 状态机](#5-状态机)
- [6. 环境变量](#6-环境变量)
- [7. 常见问题](#7-常见问题)

## 1. 脚本与架构

| 脚本 | 说明 |
|------|------|
| [`inference_server.sh`](../../scripts/deploy/walker_s2/inference_server.sh) | 服务端：拉起常驻推理服务（后台预热到 READY） |
| [`inference_client.sh`](../../scripts/deploy/walker_s2/inference_client.sh) | 客户端薄包装：`launch/load/start/stop/home/status/shutdown/watch` |
| [`inference_client.py`](../../scripts/deploy/walker_s2/inference_client.py) | 客户端实现（ZMQ 指令 + SSH 远程拉起） |

服务端实现在 `lerobot/src/lerobot/scripts/inference_server.py`，复用 `build_rollout_context` +
`ACTAsyncChunkEngine.pause/resume`，不改动引擎/桥接/walker 核心。

架构要点：

- **预热**：复用 `build_rollout_context`（policy + robot + processor + engine），与 `lerobot-rollout` 一致；
- **启停**：`ACTAsyncChunkEngine.pause()/resume()/reset()` 映射 stop/start/新 episode；引擎后台线程整块推 chunk 给桥接（300Hz densify），主循环只喂观测；
- **控制接口**：ZMQ REP(5570) 收发指令 + ZMQ PUB(5571) 广播状态；
- **load 热切换**：整体重建 context（stop 旧 engine + disconnect 旧 robot + 重建新 cfg），含机器人重连；`LOADING` 期间主线程跳过观测获取；
- **home**：先 `pause` 再发 `home_position` 平滑 chunk，避免双源抖动。

## 2. 快速开始

```bash
# 1. 拉起服务（容器内，后台预热 ~6s 到 READY；引擎 paused，不动机器人）
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=13 \
bash /ubt_IL/scripts/deploy/walker_s2/inference_server.sh

# 2. 轮询状态到 READY（此后 start/stop/home 秒级响应，无需再加载）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh status

# 3. 开始 / 停止推理（start 会让机器人运动，请在机旁执行）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh stop

# 4. 回原点 / 关停（均回 home_position）
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh shutdown
```

> 容器内 `python` 不在 PATH；用 `python3` 或 `bash inference_client.sh`（后者自动选
> `/lerobot/.venv/bin/python`）。

场景差异：

| 场景 | 桥接地址 | 说明 |
|------|----------|------|
| 仿真回注 | `ZMQ_HOST=127.0.0.1`（本地桥接，容器 `--network=host`） | 先在仿真容器启动 Isaac Sim 任务与桥接 |
| 真机 | 桥接由容器 entrypoint 自动启动 | 已在真机验证（2026-08-09）：预热 ~6s 到 READY，start/stop/home/shutdown 全通过，未测 `load` |

## 3. 客户端指令

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

ZMQ 指令默认连 `127.0.0.1:5570`；容器为 `--network=host`，故宿主机/远程用 `--host <目标机 IP>` 即可直连。

## 4. 远程拉起服务

本地拉起见上文「快速开始」。亦可在另一台机器经客户端远程拉起（SSH + docker-exec 到
**跑容器的 Jetson**，自动轮询至 READY）：

```bash
bash inference_client.sh launch \
  --policy-path /ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
  --robot-model walker_s2_10d --inference-hz 2 --fps 13 \
  --remote --host 192.168.11.3 --ssh-user walker
```

> **注意区分两台机器**：
> - **Jetson `192.168.11.3`**（SSH `walker`）：跑容器 `lerobot-tienkung`、做推理。
>   `launch --remote` 的 `--host` 指它。
> - **机器人主控 PC `192.168.11.2`**（SSH `ubt`，端口 2222）：`dev_mode.sh` 切开发者模式 / ROS2，
>   **不跑推理容器**，别用它做 `--host`。

`launch --remote` 默认 `--ssh-user ubt --ssh-port 22 --container lerobot-tienkung`；连 Jetson 需
`--ssh-user walker`。SSH 用户要对 `sudo docker exec` 有权限（免密 sudo）。不带 `--remote` 则在本地
`bash inference_server.sh` 拉起。`--host` 同时是 SSH 目标和 ZMQ 轮询目标。

## 5. 状态机

```
LOADING(预热/重建) -> READY(空闲) ⇄ RUNNING(执行)
home -> RETURNING_HOME -> READY
load -> LOADING -> READY
致命错误 -> ERROR
关机   -> SHUTTING_DOWN
```

## 6. 环境变量

### inference_server.sh

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | **必填，无默认值** | pretrained_model 目录（含 `config.json`） |
| `ROBOT_MODEL` | `walker_s2_31d` | ROBOT_MODELS 注册表键（`walker_s2_10d` / `walker_s2_19d` / `walker_s2_31d`） |
| `ROBOT_CONFIG` | 空 | 自定义 robot config JSON（覆盖 ROBOT_MODEL） |
| `INFERENCE_TYPE` | `act_async` | 推理引擎类型（`start`/`stop` 需 act_async） |
| `INFERENCE_HZ` | `0.6` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数（0=不截断） |
| `FPS` | `13` | 控制环 FPS |
| `BLEND_HORIZON` | `10` | 桥接动作块融合重叠点数 |
| `BODY_PUBLISH_HZ` | `300` | 桥接 300Hz body 发布频率 |
| `BODY_V_MAX` | `2.0` | 桥接 rate_limit 单关节最大速度 |
| `SERVER_CMD_PORT` | `5570` | ZMQ REP 指令端口 |
| `SERVER_STATUS_PORT` | `5571` | ZMQ PUB 状态端口 |
| `PREVIEW_CAMERA` | `0` | 服务器默认不开预览窗口（留作 rollout.sh 用） |

> 与 `rollout.sh` 的区别：不跑单次 duration rollout，而是常驻服务；`DURATION` 无意义。
> `--use_torch_compile` 不被服务器支持（ACT 无需 compile warmup）。

### inference_client.sh / inference_client.py

| 变量 / 参数 | 默认值 | 说明 |
|------|--------|------|
| `SERVER_HOST` / `--host` | `127.0.0.1` | 服务器主机（也是 SSH 目标） |
| `SERVER_CMD_PORT` / `--cmd-port` | `5570` | ZMQ REP 指令端口 |
| `DEPLOY_CONTAINER` | `lerobot-tienkung` | 远程 docker-exec 的容器名 |
| `DEPLOY_SSH_USER` / `--ssh-user` | `ubt` | SSH 用户（Jetson 用 `walker`） |
| `DEPLOY_SSH_PORT` / `--ssh-port` | `22` | SSH 端口 |

### launch 常用参数

| 参数 | 说明 |
|------|------|
| `--policy-path` | pretrained_model 目录 |
| `--robot-model` | ROBOT_MODELS 注册表键 |
| `--inference-type` / `--inference-hz` / `--fps` | 引擎类型 / 重规划频率 / 控制环 FPS |
| `--blend-horizon` / `--body-publish-hz` / `--body-v-max` | 桥接融合与限速参数 |
| `--remote` | 经 SSH + docker-exec 远程拉起（不带的本地拉起） |

## 7. 常见问题

| 问题 | 原因 / 解决 |
|------|-------------|
| 重复拉起失败 | 单例 pid 文件 `/tmp/walker_inference_server.pid` 在跑则拒绝；`shutdown` 正常退出或手动清理后再拉 |
| `start` 报错 | `start`/`stop` 依赖 act_async 的 pause/resume；`sync` 引擎无此语义，需 `INFERENCE_TYPE=act_async` |
| 客户端连不上 | 确认服务器已 READY（`status`）；远程访问用 `--host <目标机 IP>`（容器 `--network=host`，5570 即宿主机端口） |
| 日志在哪里 | `/tmp/walker_inference_server.log` |
| `load` 热切换不生效 | 未在真机验证过；整体重建含机器人重连，`LOADING` 期间指令会被跳过，等 READY 后再 `status` 确认模型已切换 |
