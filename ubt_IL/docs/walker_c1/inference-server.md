# Walker C1 EDU 共创者 常驻推理服务

把策略常驻在机器人侧（或 GPU 侧）容器内，操作机通过 ZMQ 远程控制：
启动模型、热切换 checkpoint、开始/停止执行、回 home、订阅状态。

```
操作机 inference_client.py ──ZMQ REQ 5690──► inference_server（lerobot venv）
                            ◄──ZMQ PUB 5691──   │
                                                 ├─ lerobot-rollout（26 / 13 维）
                                                 └─ ros2_walker_c1_bridge（ZMQ 5661/5662）
```

## 1. 脚本

| 文件 | 作用 |
|---|---|
| `scripts/deploy/walker_c1/inference_server.sh` | 后台拉起 `python -m lerobot.scripts.inference_server`（5690/5691） |
| `scripts/deploy/walker_c1/inference_client.py` | 指令客户端 / 状态订阅 |
| `scripts/deploy/walker_c1/inference_client.sh` | 薄包装（优先 `/lerobot/.venv/bin/python`） |

## 2. 快速开始

```bash
# 机器人侧（容器内）：先确认/切到 half_sdk（服务启动会校验，非 half_sdk 直接拒绝）
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh half_sdk

# 后台 + 预热（真机加 WALKER_C1_REAL=1 打开门禁）
# 真机已验证（2026-11）：13D 真机模型
WALKER_C1_REAL=1 \
POLICY_PATH=/ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/inference_server.sh

# 操作机侧
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh status
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh stop
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh shutdown
```

`launch` 会拉起服务并轮询到 `READY`：

```bash
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh launch \
  --policy-path /ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_13d \
  --inference-hz 2 --fps 30
```

远程拉起（SSH + docker exec 到跑容器的机器）：

```bash
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh launch --remote \
  --host 192.168.11.3 --ssh-user ubt --ssh-port 22 --container lerobot-tienkung \
  --policy-path /ubt_IL/model/.../pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_26d
```

> **推理后端建议（walker 系列）**：优先 `INFERENCE_TYPE=act_async`（默认）。`sync` 仅用于调试，
> 且必须保留单动作平滑（`WALKER_C1_SINGLE_ACTION_SMOOTH_S`，默认 0.15 s，置 0 会告警）。
>
> `--robot.type` 不硬编码：优先取 `load` 指令里的 `robot_type`，其次 `ROBOT_TYPE` 环境变量，
> 缺省 `walker`（walker_s2 行为不变）；C1 客户端会透传 `walker_c1`。

## 3. 客户端指令

| 指令 | 说明 |
|---|---|
| `launch` | 拉起服务（本地或 `--remote`）并等待 READY |
| `load` | 热切换 checkpoint / 机型（`--policy-path`、`--robot-type`、`--robot-model`） |
| `start` / `stop` | 开始 / 停止下发动作（`stop` 立即停发，C1 由运控保持当前位） |
| `home` | 平滑回 home（chunk 路径，2 s） |
| `status` | 单次查询状态 |
| `watch` | 订阅 PUB 状态流（state/loop_hz/chunk_id/robot_connected/uptime_s） |
| `shutdown` | 关闭服务 |

## 4. 状态机

```
LOADING ──► READY ⇄ RUNNING
              │         │
              ├──► RETURNING_HOME
              ├──► ERROR
              └──► SHUTTING_DOWN
```

- `READY`：模型已加载、机器人已连接、未下发动作
- `RUNNING`：按 `inference_hz` 推理并下发
- `RETURNING_HOME`：执行回 home，完成后回 `READY`

## 5. 环境变量

**服务端**（`inference_server.sh`）

| 变量 | 默认 | 说明 |
|---|---|---|
| `POLICY_PATH` | —（必填） | `pretrained_model` 目录 |
| `CAMERA_TOPIC` | `/sensor/camera/head_stereo_left/color/raw` | 相机话题（C1 真机头目左目；仿真用 `/sensor/camera/head/color/raw`） |
| `CAMERA_MSG_TYPE` | `shm_msgs/Image6m` | 相机消息类型（真机 `shm_msgs/Image6m`；仿真 `sensor_msgs/Image`） |
| `ROBOT_TYPE` | `walker_c1` | LeRobot 插件注册名 |
| `ROBOT_MODEL` | `walker_c1_13d` | `ROBOT_MODELS` 键（= `--robot.joint_config`） |
| `ROBOT_CONFIG` | 空 | 指定硬件配置 JSON 时优先于 `ROBOT_MODEL` |
| `INFERENCE_TYPE` | `act_async` | `act_async` / `sync` |
| `INFERENCE_HZ` | `1` | 重规划频率（实测约 1 Hz） |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数（0=不截断） |
| `FPS` | `15` | 控制环 FPS |
| `TASK` | `walker c1 pick place` | 任务描述 |
| `SERVER_CMD_PORT` | `5690` | REQ 指令端口 |
| `SERVER_STATUS_PORT` | `5691` | PUB 状态端口 |
| `ALLOW_DIM_ONLY_POLICY` | `1` | 预检只校验维度 |
| `BLEND_HORIZON` / `BODY_PUBLISH_HZ` / `BODY_V_MAX` | 桥接默认 | chunk 融合参数（透传 bridge） |
| `WALKER_C1_SINGLE_ACTION_SMOOTH_S` | `0.15` | **sync 单动作平滑窗口（秒）**：把每个 action 展开为 current→target 短轨迹再按 300 Hz 限速执行；`0`=关闭。FPS=15 建议 0.15，FPS=30 建议 0.1–0.15 |

**客户端**

| 变量 | 默认 |
|---|---|
| `DEPLOY_CONTAINER` | `lerobot-tienkung` |
| `SERVER_CMD_PORT` / `SERVER_STATUS_PORT` | `5690` / `5691` |
| `--host` | `127.0.0.1` |
| `--camera-topic` / `--camera-msg-type` | `/sensor/camera/head_stereo_left/color/raw` / `shm_msgs/Image6m` |

> `launch` 会把 `--camera-topic`/`--camera-msg-type` 转成 `CAMERA_TOPIC`/`CAMERA_MSG_TYPE`
> 传给服务端脚本（远程 launch 时以 `env VAR=... bash ...` 前缀注入）。相机只在服务启动时读取，
> 运行中 `load` 不会改变相机源。

## 6. FAQ

| 现象 | 处理 |
|---|---|
| `launch` 一直 LOADING | 看 `/tmp/walker_c1_inference_server.log`；多为模型路径/相机超时 |
| `status` 连不上 | `--host` 与端口；服务端是否在别的容器 |
| `load` 后仍用旧模型 | `status.watch` 看 `model` 字段；确认 `--robot-model` 与策略维度一致 |
| 报 action dim mismatch | 策略维度与 `ROBOT_MODEL` 不一致（26D / 13D）；检查训练数据集与启动参数 |
| `stop` 后仍动 | 在飞 chunk 最多 ~1 个 tick 后停发；若持续动，确认桥接已重启到新版本（`hold_trajectory` 生效） |
| PID 文件残留 | `/tmp/walker_c1_inference_server.pid`；单例守卫会自动修正 |
