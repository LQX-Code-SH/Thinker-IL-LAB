# Walker S2 EDU: Inference Server (Warm Start)

> Every cold start of `rollout.sh` reloads the policy to CUDA and starts the bridge, which is slow. The inference server keeps a **resident** process that loads the model and connects to the robot/simulation bridge once. External clients can then use ZMQ commands `start/stop/home/status/shutdown` at any time and `load` models for hot-swapping, avoiding repeated cold starts.
>
> It supports both **simulation feedback deployment** and **real-robot deployment**; only the bridge address and camera source differ.

## Scripts and architecture

| Script | Description |
|------|------|
| `ubt_IL/scripts/deploy/walker_s2/inference_server.sh` | Server: starts the resident inference service (pre-warms in the background to READY) |
| `ubt_IL/scripts/deploy/walker_s2/inference_client.sh` | Thin client wrapper: `launch/load/start/stop/home/status/shutdown/watch` |
| `ubt_IL/scripts/deploy/walker_s2/inference_client.py` | Client implementation (ZMQ commands + SSH remote launch) |

The server is implemented in `lerobot/src/lerobot/scripts/inference_server.py`, reusing `build_rollout_context` + `ACTAsyncChunkEngine.pause/resume` without modifying the engine/bridge/walker core.

Architecture:

- **Warm-up**: reuses `build_rollout_context` (policy + robot + processor + engine), same as `lerobot-rollout`.
- **Start/stop**: `ACTAsyncChunkEngine.pause()/resume()/reset()` map to stop/start/new episode; the engine background thread pushes whole chunks to the bridge (300 Hz densify), while the main loop only feeds observations.
- **Control interface**: ZMQ REP (5570) for commands + ZMQ PUB (5571) for status broadcast.
- **`load` hot-swap**: rebuilds the whole context (stop the old engine + disconnect the old robot + rebuild the new cfg), including robot reconnection; the main thread skips observation acquisition during `LOADING`.
- **home**: `pause` first, then send a smooth `home_position` chunk to avoid dual-source jitter.

## Quick start

```bash
# 1. Start the service (inside the container; pre-warms in the background to READY in ~6 s; the engine is paused and the robot does not move)
ROBOT_MODEL=walker_s2_10d \
POLICY_PATH=/ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
INFERENCE_TYPE=act_async INFERENCE_HZ=2 FPS=13 \
bash /ubt_IL/scripts/deploy/walker_s2/inference_server.sh

# 2. Poll until READY (after that, start/stop/home respond in seconds without reloading)
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh status

# 3. Start / stop inference (start moves the robot; execute next to it)
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh start
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh stop

# 4. Return home / shut down (both return to home_position)
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh home
bash /ubt_IL/scripts/deploy/walker_s2/inference_client.sh shutdown
```

> Inside the container, `python` is not on PATH; use `python3` or `bash inference_client.sh` (the latter picks `/lerobot/.venv/bin/python` automatically).

Scenario differences:

| Scenario | Bridge address | Notes |
|------|----------|------|
| Simulation feedback | `ZMQ_HOST=127.0.0.1` (local bridge, container `--network=host`) | Start the Isaac Sim task and bridge in the simulation container first |
| Real robot | Bridge started automatically by the container entrypoint | Verified on hardware (2026-08-09): ~6 s warm-up to READY, start/stop/home/shutdown all pass; `load` not tested |

## Client commands

```bash
bash inference_client.sh status                       # query state (state/model/loop_hz/chunk_id/...)
bash inference_client.sh start                        # start inference (engine reset+resume -> RUNNING)
bash inference_client.sh stop                         # stop inference (engine pause -> READY, hold current pose)
bash inference_client.sh home                         # return to home_position (pause, then smooth chunk)
bash inference_client.sh load --policy-path /ubt_IL/model/<other>/.../pretrained_model --robot-model walker_s2_10d
                                                       # hot-swap the model while running (full rebuild incl. reconnect)
bash inference_client.sh shutdown                     # shut down (home + kill bridge + exit)
bash inference_client.sh watch                        # subscribe to the status PUB stream (Ctrl-C to exit)
```

ZMQ commands connect to `127.0.0.1:5570` by default; the container uses `--network=host`, so from the host or a remote machine simply use `--host <target IP>`.

## Remote launch (load a model with explicit parameters)

Launch remotely from another machine (SSH + `docker exec` on the **Jetson running the container**, polling until READY):

```bash
bash inference_client.sh launch \
  --policy-path /ubt_IL/model/Walker_S2_sim_10_2RGB_act/checkpoints/020000/pretrained_model \
  --robot-model walker_s2_10d --inference-hz 2 --fps 13 \
  --remote --host 192.168.11.3 --ssh-user walker
```

> **Know the two machines**:
> - **Jetson `192.168.11.3`** (SSH `walker`): runs the `lerobot-tienkung` container and inference. This is the `--host` for `launch --remote`.
> - **Robot control PC `192.168.11.2`** (SSH `ubt`, port 2222): `dev_mode.sh` developer mode / ROS 2. It does **not** run the inference container; do not use it as `--host`.

`launch --remote` defaults to `--ssh-user ubt --ssh-port 22 --container lerobot-tienkung`; use `--ssh-user walker` for the Jetson. The SSH user needs `sudo docker exec` permission (passwordless sudo). Without `--remote`, it launches `bash inference_server.sh` locally. `--host` is both the SSH target and the ZMQ polling target.

## State machine

```
LOADING (warm-up/rebuild) -> READY (idle) ⇄ RUNNING (executing)
home -> RETURNING_HOME -> READY
load -> LOADING -> READY
fatal error -> ERROR
shutdown -> SHUTTING_DOWN
```

## Environment variables

### inference_server.sh

| Variable | Default | Description |
|------|--------|------|
| `POLICY_PATH` | **Required, no default** | `pretrained_model` directory (containing `config.json`) |
| `ROBOT_MODEL` | `walker_s2_31d` | ROBOT_MODELS registry key (`walker_s2_10d` / `walker_s2_19d` / `walker_s2_31d`) |
| `ROBOT_CONFIG` | Empty | Custom robot config JSON (overrides ROBOT_MODEL) |
| `INFERENCE_TYPE` | `act_async` | Inference engine (`start`/`stop` require act_async) |
| `INFERENCE_HZ` | `0.6` | act_async replanning rate |
| `EXECUTION_HORIZON` | `0` | Truncate chunks after N steps (0 = no truncation) |
| `FPS` | `13` | Control loop FPS |
| `BLEND_HORIZON` | `10` | Bridge chunk blending overlap points |
| `BODY_PUBLISH_HZ` | `300` | Bridge 300 Hz body publish rate |
| `BODY_V_MAX` | `2.0` | Bridge rate-limit maximum joint velocity |
| `SERVER_CMD_PORT` | `5570` | ZMQ REP command port |
| `SERVER_STATUS_PORT` | `5571` | ZMQ PUB status port |
| `PREVIEW_CAMERA` | `0` | Server does not open the preview window by default (reserved for rollout.sh) |

> Difference from `rollout.sh`: it does not run a single duration-bounded rollout but stays resident; `DURATION` is meaningless. `--use_torch_compile` is not supported by the server (ACT needs no compile warm-up).

### inference_client.sh / inference_client.py

| Variable / argument | Default | Description |
|------|--------|------|
| `SERVER_HOST` / `--host` | `127.0.0.1` | Server host (also the SSH target) |
| `SERVER_CMD_PORT` / `--cmd-port` | `5570` | ZMQ REP command port |
| `DEPLOY_CONTAINER` | `lerobot-tienkung` | Container name for remote docker-exec |
| `DEPLOY_SSH_USER` / `--ssh-user` | `ubt` | SSH user (use `walker` for the Jetson) |
| `DEPLOY_SSH_PORT` / `--ssh-port` | `22` | SSH port |

### Common `launch` arguments

| Argument | Description |
|------|------|
| `--policy-path` | `pretrained_model` directory |
| `--robot-model` | ROBOT_MODELS registry key |
| `--inference-type` / `--inference-hz` / `--fps` | Engine type / replanning rate / control loop FPS |
| `--blend-horizon` / `--body-publish-hz` / `--body-v-max` | Bridge blending and rate-limit parameters |
| `--remote` | Launch remotely via SSH + docker-exec (without it, launch locally) |

## FAQ

| Problem | Cause / fix |
|------|-------------|
| Repeated launch fails | The singleton PID file `/tmp/walker_inference_server.pid` exists; `shutdown` cleanly or remove it manually |
| `start` fails | `start`/`stop` rely on act_async pause/resume; the `sync` engine has no such semantics, so use `INFERENCE_TYPE=act_async` |
| Client cannot connect | Confirm the server is READY (`status`); for remote access use `--host <target IP>` (container `--network=host`, so 5570 is the host port) |
| Where is the log? | `/tmp/walker_inference_server.log` |
| `load` hot-swap does not take effect | Not verified on hardware; it rebuilds everything including robot reconnection, commands are skipped during `LOADING`; wait for READY and check `status` to confirm the model switched |
