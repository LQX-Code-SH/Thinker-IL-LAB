# Walker C1 EDU: Inference Server (Warm Start)

Keep the policy resident in a container on the robot side (or GPU side). The operator machine controls it remotely over ZMQ: load a model, hot-swap checkpoints, start/stop execution, return home, and subscribe to status.

```
Operator inference_client.py ──ZMQ REQ 5690──► inference_server (lerobot venv)
                              ◄──ZMQ PUB 5691──   │
                                                  ├─ lerobot-rollout (26D / 13D)
                                                  └─ ros2_walker_c1_bridge (ZMQ 5661/5662)
```

## 1. Scripts

| File | Purpose |
|---|---|
| `scripts/deploy/walker_c1/inference_server.sh` | Start `python -m lerobot.scripts.inference_server` in the background (5690/5691) |
| `scripts/deploy/walker_c1/inference_client.py` | Command client / status subscriber |
| `scripts/deploy/walker_c1/inference_client.sh` | Thin wrapper (prefers `/lerobot/.venv/bin/python`) |

## 2. Quick start

```bash
# Robot side (inside the container): confirm/switch to half_sdk (the service validates and refuses otherwise)
bash /ubt_IL/scripts/deploy/walker_c1/sdk_mode.sh half_sdk

# Background + warm start (set WALKER_C1_REAL=1 on hardware to enable the guard)
# Verified on hardware (Nov 2026): 13D real-robot model
WALKER_C1_REAL=1 \
POLICY_PATH=/ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/inference_server.sh

# Operator side
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh status
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh stop
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh shutdown
```

`launch` starts the service and polls until `READY`:

```bash
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh launch \
  --policy-path /ubt_IL/model/walker_c1_real_13d_act_1RGB/checkpoints/last/pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_13d \
  --inference-hz 2 --fps 30
```

Remote launch (SSH + `docker exec` on the machine running the container):

```bash
bash /ubt_IL/scripts/deploy/walker_c1/inference_client.sh launch --remote \
  --host 192.168.11.3 --ssh-user ubt --ssh-port 22 --container lerobot-tienkung \
  --policy-path /ubt_IL/model/.../pretrained_model \
  --robot-type walker_c1 --robot-model walker_c1_26d
```

> **Inference backend recommendation (Walker series)**: prefer `INFERENCE_TYPE=act_async` (default). `sync` is for debugging only and must keep single-action smoothing (`WALKER_C1_SINGLE_ACTION_SMOOTH_S`, default 0.15 s; setting 0 triggers a warning).
>
> `--robot.type` is not hard-coded: it uses the `robot_type` from the `load` command first, then the `ROBOT_TYPE` environment variable, and defaults to `walker` (preserving walker_s2 behavior). The C1 client forwards `walker_c1`.

## 3. Client commands

| Command | Description |
|---|---|
| `launch` | Start the service (locally or with `--remote`) and wait for READY |
| `load` | Hot-swap checkpoint / robot model (`--policy-path`, `--robot-type`, `--robot-model`) |
| `start` / `stop` | Start / stop sending actions (`stop` stops immediately; C1 motion control holds the current pose) |
| `home` | Smoothly return home (chunk path, 2 s) |
| `status` | Query status once |
| `watch` | Subscribe to the PUB status stream (state/loop_hz/chunk_id/robot_connected/uptime_s) |
| `shutdown` | Shut down the service |

## 4. State machine

```
LOADING ──► READY ⇄ RUNNING
              │         │
              ├──► RETURNING_HOME
              ├──► ERROR
              └──► SHUTTING_DOWN
```

- `READY`: model loaded, robot connected, no actions being sent
- `RUNNING`: inference and action publishing at `inference_hz`
- `RETURNING_HOME`: returning home; goes back to `READY` when finished

## 5. Environment variables

**Server** (`inference_server.sh`)

| Variable | Default | Description |
|---|---|---|
| `POLICY_PATH` | — (required) | `pretrained_model` directory |
| `CAMERA_TOPIC` | `/sensor/camera/head_stereo_left/color/raw` | Camera topic (C1 real head left; simulation uses `/sensor/camera/head/color/raw`) |
| `CAMERA_MSG_TYPE` | `shm_msgs/Image6m` | Camera message type (real: `shm_msgs/Image6m`; simulation: `sensor_msgs/Image`) |
| `ROBOT_TYPE` | `walker_c1` | LeRobot plugin registration name |
| `ROBOT_MODEL` | `walker_c1_13d` | `ROBOT_MODELS` key (= `--robot.joint_config`) |
| `ROBOT_CONFIG` | Empty | Explicit hardware config JSON; takes precedence over `ROBOT_MODEL` |
| `INFERENCE_TYPE` | `act_async` | `act_async` / `sync` |
| `INFERENCE_HZ` | `1` | Replanning rate (about 1 Hz measured) |
| `EXECUTION_HORIZON` | `0` | Truncate chunks after N steps (0 = no truncation) |
| `FPS` | `15` | Control loop FPS |
| `TASK` | `walker c1 pick place` | Task description |
| `SERVER_CMD_PORT` | `5690` | REQ command port |
| `SERVER_STATUS_PORT` | `5691` | PUB status port |
| `ALLOW_DIM_ONLY_POLICY` | `1` | Precheck dimensions only |
| `BLEND_HORIZON` / `BODY_PUBLISH_HZ` / `BODY_V_MAX` | Bridge defaults | Chunk blending parameters (passed to the bridge) |
| `WALKER_C1_SINGLE_ACTION_SMOOTH_S` | `0.15` | `sync` single-action smoothing window (seconds): expands each action into a short current→target trajectory executed at 300 Hz; `0` disables. Recommended 0.15 at FPS=15 and 0.1–0.15 at FPS=30 |

**Client**

| Variable | Default |
|---|---|
| `DEPLOY_CONTAINER` | `lerobot-tienkung` |
| `SERVER_CMD_PORT` / `SERVER_STATUS_PORT` | `5690` / `5691` |
| `--host` | `127.0.0.1` |
| `--camera-topic` / `--camera-msg-type` | `/sensor/camera/head_stereo_left/color/raw` / `shm_msgs/Image6m` |

> `launch` converts `--camera-topic` / `--camera-msg-type` into `CAMERA_TOPIC` / `CAMERA_MSG_TYPE` for the server script (for remote launch it prefixes `env VAR=... bash ...`). The camera is read only at service startup; `load` does not change the camera source at runtime.

## 6. FAQ

| Symptom | Fix |
|---|---|
| `launch` stays in LOADING | Check `/tmp/walker_c1_inference_server.log`; usually a model path or camera timeout |
| `status` cannot connect | Check `--host` and ports; is the service in another container? |
| Old model still used after `load` | Check the `model` field with `watch`; confirm `--robot-model` matches the policy dimension |
| action dim mismatch | Policy dimension and `ROBOT_MODEL` disagree (26D / 13D); check the training dataset and launch arguments |
| Robot still moves after `stop` | In-flight chunks stop after at most ~1 tick; if motion continues, confirm the bridge was restarted with the new version (`hold_trajectory` active) |
| Stale PID file | `/tmp/walker_c1_inference_server.pid`; the singleton guard corrects it automatically |
