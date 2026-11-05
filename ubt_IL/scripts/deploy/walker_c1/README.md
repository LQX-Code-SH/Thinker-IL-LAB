# Walker C1 部署

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

相机话题/类型可用 `CAMERA_TOPIC` + `CAMERA_MSG_TYPE`（rollout.sh）或
`WALKER_C1_CAMERA_TOPIC` + `WALKER_C1_CAMERA_MSG_TYPE`（插件）覆盖。`/compressed` 结尾自动
推断成压缩图，其余默认 `sensor_msgs/Image`；真机 `shm_msgs/Image6m` 必须显式给类型。

## 文件

| 文件 | 作用 |
|---|---|
| `configs/walker_c1_26d.json` | 机器人硬件配置（26 维 action_order、双臂分组、手部限位、端口） |
| `rollout.sh` | 单集/多集 rollout（每集复位 + 进程重启） |
| `robot_ready.py` / `.sh` | 三段式摆位（抬肩 → 收肘 → 预备位）+ 手张开；每段默认 ≥5 s |
| `robot_home.py` / `.sh` | 回 home（14 臂关节全 0），路径为上面三段的严格逆序（腕/肩俯仰回零 → 展开肘 → 放下肩） |
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

# ⑤ rollout
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh
```

`ubt_sim` 的 C1 桥接使用 ZMQ 5655-5658，本套件使用 5661-5663，可同时运行。

## rollout.sh 环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `POLICY_PATH` | —（必填） | `pretrained_model` 目录 |
| `ROBOT_MODEL` | `walker_c1_26d` | `ROBOT_MODELS` 注册表键（同时作为 `--robot.joint_config`） |
| `ROBOT_CONFIG` | `configs/walker_c1_26d.json` | 硬件配置 JSON |
| `ROBOT_TYPE` | `walker_c1` | 插件注册名 |
| `EPISODES` | `1` | 集数；base strategy 不支持多集，>1 时由外层循环 + 每集复位实现 |
| `DURATION` | `30` | 每集最长秒数 |
| `FPS` | `15` | 控制环 FPS |
| `TASK` | `walker c1 pick place` | 任务描述 |
| `RESET_MODE` | `sim-scene` | 每集前复位：`sim-scene` / `task` / `both` / `none` |
| `ALLOW_DIM_ONLY_POLICY` | `1` | 1=只校验维度；0=同时校验 `action_feature_names` |
| `INFERENCE_TYPE` | `sync` | `sync` / `act_async`（chunk → bridge 融合 300 Hz） |
| `INFERENCE_HZ` | `2` | act_async 重规划频率 |
| `EXECUTION_HORIZON` | `0` | act_async chunk 截断步数（0=不截断） |
| `PREVIEW_CAMERA` | `0` | 1=同时开预览窗口 |

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

1. 机器人切 SDK 控制（厂商 ROSA 工具，具体命令见现场手册）
2. 车端/操作机 `ROS_DOMAIN_ID` 一致（真机默认 0）
3. `docker/fastdds_no_shm.xml` 白名单加入本机网卡 IP
4. 先 `robot_ready.sh --dry-run` 确认目标位姿，再实跑
5. 首次部署建议 `DURATION=10` 短时验证

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
