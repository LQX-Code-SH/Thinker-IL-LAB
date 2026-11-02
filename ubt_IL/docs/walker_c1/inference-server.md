# Walker C1 常驻推理服务

把策略常驻在机器人侧（或 GPU 侧）容器内，操作机通过 ZMQ 远程控制：
启动模型、热切换 checkpoint、开始/停止执行、回 home、订阅状态。

```
操作机 inference_client.py ──ZMQ REQ 5690──► inference_server（lerobot venv）
                            ◄──ZMQ PUB 5691──   │
                                                 ├─ lerobot-rollout（26 维）
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
# 机器人侧（容器内，后台 + 预热）
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
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
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_26d \
  --inference-hz 2 --fps 15
```

远程拉起（SSH + docker exec 到跑容器的机器）：

```bash
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh launch --remote \
  --host 192.168.11.3 --ssh-user ubt --ssh-port 22 --container lerobot-tienkung \
  --policy-path /ubt_IL/model/.../pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_26d
```

## 3. 客户端指令

| 指令 | 说明 |
|---|---|
| `launch` | 拉起服务（本地或 `--remote`）并等待 READY |
| `load` | 热切换 checkpoint / 机型（`--policy-path`、`--robot-type`、`--robot-model`） |
| `start` / `stop` | 开始 / 停止下发动作（`stop` 会 hold 当前位） |
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
| `ROBOT_TYPE` | `walker_c1` | LeRobot 插件注册名 |
| `ROBOT_MODEL` | `walker_c1_26d` | `ROBOT_MODELS` 键（= `--robot.joint_config`） |
| `ROBOT_CONFIG` | 空 | 指定硬件配置 JSON 时优先于 `ROBOT_MODEL` |
| `INFERENCE_TYPE` | `act_async` | `act_async` / `sync` |
| `INFERENCE_HZ` | `0.6` | 重规划频率 |
| `EXECUTION_HORIZON` | `0` | chunk 截断步数（0=不截断） |
| `FPS` | `15` | 控制环 FPS |
| `TASK` | `walker c1 pick place` | 任务描述 |
| `SERVER_CMD_PORT` | `5690` | REQ 指令端口 |
| `SERVER_STATUS_PORT` | `5691` | PUB 状态端口 |
| `ALLOW_DIM_ONLY_POLICY` | `1` | 预检只校验维度 |
| `BLEND_HORIZON` / `BODY_PUBLISH_HZ` / `BODY_V_MAX` | 桥接默认 | chunk 融合参数（透传 bridge） |

**客户端**

| 变量 | 默认 |
|---|---|
| `DEPLOY_CONTAINER` | `lerobot-tienkung` |
| `SERVER_CMD_PORT` / `SERVER_STATUS_PORT` | `5690` / `5691` |
| `--host` | `127.0.0.1` |

## 6. 参数化说明

`lerobot/src/lerobot/scripts/inference_server.py` 的 `--robot.type` 不再硬编码：
优先取 `load` 指令里的 `robot_type`，其次 `ROBOT_TYPE` 环境变量，缺省 `walker`
（walker_s2 行为不变）。C1 客户端会透传 `robot_type=walker_c1`。

## 7. FAQ

| 现象 | 处理 |
|---|---|
| `launch` 一直 LOADING | 看 `/tmp/walker_c1_inference_server.log`；多为模型路径/相机超时 |
| `status` 连不上 | `--host` 与端口；服务端是否在别的容器 |
| `load` 后仍用旧模型 | `status.watch` 看 `model` 字段；确认 `--robot-model` 与策略维度一致 |
| 报 action dim mismatch | 策略不是 26 维；检查训练数据集 |
| `stop` 后仍动 | 桥接 300 Hz hold 有 ~1 个 tick 延迟；确认 `hold_trajectory` 已下发 |
| PID 文件残留 | `/tmp/walker_c1_inference_server.pid`；单例守卫会自动修正 |
