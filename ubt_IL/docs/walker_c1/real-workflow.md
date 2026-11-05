# Walker C1 真机工作流

真机与仿真共用同一套 ROS2 接口（`mc_task_msgs` / `mc_state_msgs` / `sensor_msgs/Image`），
靠 `ROS_DOMAIN_ID` 隔离。RL 部署链路只发 26 维（双臂 + 双手），头/腰/腿保持机器人当前位。

```
真机 ROS2 ──► ubt_sim/teleoperation/control/walker_c1（采集） ──► HDF5
                                                                    │
容器内：convert.sh → train.sh → eval_policy.py → robot_ready.sh + rollout.sh
                                                                    │
LeRobot (3.12) ──ZMQ 5661/5662──► ros2_walker_c1_bridge.py (3.10) ──► 真机
```

## 0. 总览

| 阶段 | 位置 | 命令 |
|---|---|---|
| 切 SDK 控制 | 机器人主控 | 厂商 ROSA 工具（见现场手册） |
| 真机采集 | 操作机 / ubt_sim 容器 | `teleoperation/control/walker_c1/save_data.sh` |
| 容器 | 操作机 | `cd ubt_IL/docker && bash run.sh start && bash run.sh bash` |
| 转换 | 容器 | `scripts/convert/walker_c1/convert.sh` |
| 训练 | 容器（GPU） | `scripts/train/walker_c1/train.sh` |
| 评估 | 容器 | `scripts/eval/eval_policy.py` |
| 部署 | 容器（机器人侧） | `robot_ready.sh` + `rollout.sh` |

## 1. 真机前置

1. **切 SDK 控制**：C1 需由厂商工具把控制权交给 SDK（`rosa` / `t800_mc_server` 一类命令，
   以现场手册为准）。未切 SDK 时 `/mc/sdk/robot_command` 不生效。
2. **ROS 域**：真机默认 `ROS_DOMAIN_ID=0`；与仿真隔离时把两边都改成 146。
3. **DDS**：**Thor 机型（C1 本体）必须与主控栈同 RMW**——机器人栈是
   `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI`（绑定 `mgbe2_0`，本机 192.168.11.3）。
   IL 容器由 `docker/run.sh` 自动注入同款环境，并挂载 `docker/cyclonedds_thor.xml`：

   ```bash
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   CYCLONEDDS_URI=file:///opt/cyclonedds_thor.xml   # 内含 mgbe2_0 + ParticipantIndex=none
   ```

   两条实测坑（改配置时注意）：
   - `<Interfaces>` 必须写在 `<General>` 里（Humble 的 CycloneDDS 0.10+），放 `<Domain>` 下只报
     `unknown element` 并回退默认网卡；网卡只写 `mgbe2_0`，不要写 `lo`（非 multicast）。
   - 域 0 内 participant 很多，默认上限会报
     `Failed to find a free participant index for domain 0`；配置里已设
     `ParticipantIndex=none`、`MaxAutoParticipantIndex=120`。

   若走 FastDDS（仿真/远程机），改 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，并把本机网卡 IP
   （Thor 为 `192.168.11.3`）加进 `docker/fastdds_no_shm.xml` 白名单：

   ```xml
   <address>127.0.0.1</address>
   <address>192.168.11.3</address>   <!-- Thor 本机 mgbe2_0 -->
   <address>192.168.11.99</address>  <!-- Walker 直连网段对端 -->
   ```

4. **状态自检**（容器内，`run.sh bash` 之后）：

   ```bash
   source /opt/ros/humble/setup.bash
   source /ubt_IL/walker_s2/walker_sdk_ros2/install/setup.bash
   export ROS_DOMAIN_ID=0
   ros2 topic hz /mc/sdk/robot_state                     # Thor 实测 ≈ 460-490 Hz
   ros2 topic hz /sensor/camera/head_stereo_left/color/raw          # shm_msgs/Image6m
   ros2 topic info -v /mc/sdk/robot_command              # 应有订阅者（SDK 控制已切）
   ```

   说明：真机 `RobotState` 只有 19 个关节（双臂 14 + 头 2 + 腰 3，无腿），bridge 按名取
   14 个臂关节，不受影响；头/腰/腿不在策略内、也不下发。

5. **安全**：机器人置于安全位；首次 `DURATION=10` 短时验证；手边留急停。

## 2. 真机采集

```bash
# ubt_sim 容器内（真机同域，不复用仿真进程）
bash teleoperation/control/walker_c1/save_data.sh
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

产物与仿真一致：`ubt_sim/dataset/walker_c1/<epoch>/trajectory.hdf5`（15 Hz、31 body + 6+6 手、1 路 RGB）。
采集端需要真机相机话题——**C1 真机没有 `/sensor/camera/head/color/raw`**，只有：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/sensor/camera/head_stereo_left/color/raw` | `shm_msgs/Image6m` | **策略输入用它**：1920×1536、yuv422(UYVY)、step=3840、6 MB 定长 |
| ~~`/sensor/camera/head_stereo_left/color/compressed`~~ | `sensor_msgs/CompressedImage` | 本机视觉栈重启后**已不再发布**（早前存在过） |
| `/sensor/camera/head_stereo_left/color/raw` | `sensor_msgs/Image` | 未压缩同源画面 |
| `head_stereo_right` / `head_fisheye_left,right` / `mono_chin` / `wrist_left,right` | 同族 | 其它视角 |

> 宽高比待定：真机原生 1920×1536（5:4），部署当前按 320×240（4:3）直接缩放，横向约拉伸 6.7%。
> 是否改为裁剪/改分辨率，等拿到真机采集数据后再定（暂不做裁剪）。

采集/部署时用 `WALKER_C1_CAMERA_TOPIC` + `WALKER_C1_CAMERA_MSG_TYPE`（部署脚本用
`CAMERA_TOPIC` / `CAMERA_MSG_TYPE`）指定。注意：`/compressed` 结尾会推断成压缩图，其余默认
`sensor_msgs/Image`，**真机的 `shm_msgs/Image6m` 必须显式给类型**，否则订阅类型不匹配。

## 3. 容器

```bash
cd ubt_IL/docker
bash run.sh build && bash run.sh start && bash run.sh check && bash run.sh bash
```

Thor 机型（L4T r38 / sm_110）由 `env.sh` 自动切到 `Dockerfile.thor`
（镜像 `lerobot-tienkung:humble-thor`，基镜像为机器人自带 `ros2_ws:humble-dyn`，
torch 从 `astron-integration` 镜像移植 2.9.0+cu130/sm_110）。**不要**用 Orin 的
`Dockerfile.arm64`（r36 基镜像 + sm_87 wheel，在 Thor 上跑不起来）。

`run.sh check` 应包含：

- `[OK] walker_c1 plugin: installed`
- `[OK] walker_c1 bridge: available`
- `[OK] walker ROS2 msgs: installed`（`mc_state_msgs` / `mc_task_msgs`，与 walker_s2 共用工作空间）
- `[OK] torch: 2.9.0+cu130 | arch: ...,sm_110,... | kernel: ok`
- `[INFO] RMW: rmw_cyclonedds_cpp | CYCLONEDDS_URI=file:///opt/cyclonedds_thor.xml`

## 4. 数据转换

```bash
# 真机数据默认落在 /ubt_sim/dataset/walker_c1（只读挂载）
bash /ubt_IL/scripts/convert/walker_c1/convert.sh --overwrite

# 若数据拷到宿主机 ubt_IL/dataset/walker_c1
SRC_ROOT=/ubt_IL/dataset/walker_c1 bash /ubt_IL/scripts/convert/walker_c1/convert.sh --overwrite
```

真机手部值域已核对：右手 `thumb_swing ≤0.68 / thumb_mcp ≤0.82 / 四指 ≤0.85`，
均在部署限位（0.96 / 1.04 / 1.57）内。

## 5. 训练与评估

```bash
bash /ubt_IL/scripts/train/walker_c1/train.sh
python /ubt_IL/scripts/eval/eval_policy.py \
  --policy-path /ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
  --dataset-path /ubt_IL/dataset/walker_c1_sim_26d_1RGB \
  --episodes 5 --inference-freq 15 --plot \
  --output /ubt_IL/scripts/eval/output/eval_walker_c1_sim_26d_act/results.json
```

仿真训练的模型可直接在真机 rollout；若仿真/真机相机域差异大，建议真机数据微调。

## 6. 部署

```bash
# ① 相机自检（真机用压缩话题；--msg-type 默认 auto，按话题名推断）
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1.jpg \
  --msg-type image6m --topic /sensor/camera/head_stereo_left/color/raw

# ② 摆到预备位（先 dry-run 看目标位姿与每段时长）
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --dry-run
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh

#    预备位含低头 head_pitch=0.65 rad（37.2°）——实测 0.50 只能看到桌面下缘，
#    0.65 才能覆盖完整桌面；可用 --head-pitch 调整，--no-head 不动头。

#    三段：抬肩 → 收肘 → 预备位；每段默认 ≥5s（共 15s + hold 0.5s），
#    日志会打印每段最大位移与峰值角速度（≈0.5 rad/s ≈28°/s）。
#    更慢/更快：
#      --min-segment-s 8   每段下限 8s
#      --segment-s 8       按比例放大（0.6/0.6/1.0 段时长，仍受下限约束）
#      --hold-s 1.0        末端保持时间

# ②' 回 home（home = 14 个臂关节全 0，手臂竖直贴身，手张开）
bash /ubt_IL/scripts/deploy/walker_c1/robot_home.sh --dry-run
bash /ubt_IL/scripts/deploy/walker_c1/robot_home.sh
#    路径 = ready 三段的严格逆序：腕/肩俯仰回零 → 展开肘 → 放下肩，每段默认 ≥5s，
#    实测到位 max |pos| ≈0.02 rad。只动臂+手，头/腰/腿不碰。
#    注意：插件里的 home_position（rollout 断开自动回位）仍是 READY_POSE，未改成零位。

# ③ 单集 rollout
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
DURATION=10 FPS=15 RESET_MODE=task \
CAMERA_TOPIC=/sensor/camera/head_stereo_left/color/raw \
CAMERA_MSG_TYPE=shm_msgs/Image6m \
  bash /ubt_IL/scripts/deploy/walker_c1/rollout.sh

# ④ 常驻推理服务（可选，见 inference-server.md）
bash /ubt_IL/scripts/deploy/walker_c1/inference_server.sh
```

`RESET_MODE`：真机没有 `/sim/cmd_reset`，用 `task`（只摆位）或 `none`。

安全相关默认值：

- `max_relative_target`：配置 JSON 里为 `null`；需要单步限幅时设为如 `0.05` rad/步
- `disable_torque_on_disconnect=true`：退出时平滑回 home（2 s chunk）
- 手部限位在插件与 bridge 双侧裁剪（插件单点定义 → `toBridgeConfig.hand_joint_limits`）

## 7. FAQ

| 现象 | 处理 |
|---|---|
| 机械臂完全不动 | 未切 SDK 控制；`/mc/sdk/robot_command` 无人消费；`ROS_DOMAIN_ID` 不一致 |
| 只有右手动 | 数据里左手全 0（该任务特性）；策略会复现，属正常 |
| 第 4 关节方向/命名不符 | SDK 文档叫 `elbow_roll`、URDF 为 `elbow_pitch`；bridge 已做别名容忍，真机若反向需现场核对 |
| 动作突变 | 设置 `max_relative_target`；降低 `FPS`；检查手部限位 |
| 相机花屏/无图 | `sensor_msgs/Image` encoding 应为 `rgb8`；QoS BEST_EFFORT |
| 退出后手臂不回位 | `disable_torque_on_disconnect` 被设 false；或 bridge 已被外部 kill |
| 推理服务连不上 | 端口 5690/5691；`inference_client.sh status` 看状态机 |
| 话题列表空 / 收不到 `/mc/*` | RMW 不一致：真机必须 `rmw_cyclonedds_cpp` + `cyclonedds_thor.xml`（见 1.3） |
| `Failed to find a free participant index` | CycloneDDS 默认 participant 上限太低，用 `ParticipantIndex=none` + `MaxAutoParticipantIndex=120` |
| 相机话题不存在/收不到图 | 真机用 `CAMERA_TOPIC=.../head_stereo_left/color/raw` + `CAMERA_MSG_TYPE=shm_msgs/Image6m`；颜色发绿/发紫说明把 UYVY 当 YUYV 解了 |
| `torch.cuda.is_available()` 为 False | 容器缺 `NVIDIA_VISIBLE_DEVICES=all`/`NVIDIA_DRIVER_CAPABILITIES=all`（`run.sh start` 已带；手工 `docker run` 需补） |
