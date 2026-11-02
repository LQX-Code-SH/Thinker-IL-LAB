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
3. **DDS**：`ubt_IL/docker/fastdds_no_shm.xml` 白名单加入本机网卡 IP：

   ```xml
   <address>127.0.0.1</address>
   <address>192.168.41.99</address>  <!-- 天工/默认远程真机 -->
   <address>192.168.11.99</address>  <!-- Walker 直连网段 -->
   ```

4. **状态自检**：

   ```bash
   source /opt/ros/humble/setup.bash
   source /ubt_IL/walker_s2/walker_sdk_ros2/install/setup.bash
   ros2 topic hz /mc/sdk/robot_state        # 应约 48 Hz
   ros2 topic hz /sensor/camera/head/color/raw
   ```

5. **安全**：机器人置于安全位；首次 `DURATION=10` 短时验证；手边留急停。

## 2. 真机采集

```bash
# ubt_sim 容器内（真机同域，不复用仿真进程）
bash teleoperation/control/walker_c1/save_data.sh
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

产物与仿真一致：`ubt_sim/dataset/walker_c1/<epoch>/trajectory.hdf5`（15 Hz、31 body + 6+6 手、1 路 RGB）。
采集端需要真机相机话题 `/sensor/camera/head/color/raw`。

## 3. 容器

```bash
cd ubt_IL/docker
bash run.sh build && bash run.sh start && bash run.sh check && bash run.sh bash
```

`run.sh check` 应包含：

- `[OK] walker_c1 plugin: installed`
- `[OK] walker_c1 bridge: available`
- `[OK] walker ROS2 msgs: installed`（`mc_state_msgs` / `mc_task_msgs`，与 walker_s2 共用工作空间）

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
# ① 相机自检
python3 /ubt_IL/scripts/deploy/walker_c1/preview_camera.py --once --save-frame /tmp/c1.jpg

# ② 摆到预备位（先 dry-run 看目标位姿）
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh --dry-run
bash /ubt_IL/scripts/deploy/walker_c1/robot_ready.sh

# ③ 单集 rollout
POLICY_PATH=/ubt_IL/model/walker_c1_sim_26d_act_1RGB/checkpoints/last/pretrained_model \
DURATION=10 FPS=15 RESET_MODE=task \
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
