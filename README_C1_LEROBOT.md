# Walker C1：采集、训练与推理

以下命令均在仓库根目录执行。当前流程使用单个头部 RGB 相机、30 FPS、26维状态和26维动作。

## 当前状态

完整链路已经跑通：HDF5采集 → LeRobot转换 → Diffusion Policy训练 → checkpoint离线推理 → C1仿真闭环控制。

当前正式数据为200条成功轨迹。初始 reset 和苹果落稳过程不录制，轨迹从准备姿势开始，并包含抓放结束后的右臂归位；左臂和头部在 reset 后保持固定。

当前 Diffusion Policy 使用 `horizon=64`、`n_action_steps=32`、`n_obs_steps=2`，训练默认 `batch_size=8`、`steps=50000`。

## 1. 启动仿真

终端1：

```bash
docker start walker-c1-ubt-sim
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/start_c1_pick_place_sim.sh
```

## 2. 采集数据

仿真启动后，先把机器人移动到准备姿势：

```bash
docker exec -it walker-c1-ubt-sim bash -lc '
source /opt/ros/humble/setup.bash && \
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash && \
ROS_DOMAIN_ID=146 /usr/bin/python3 \
  /ubt_sim/teleoperation/control/walker_c1/reset.py'
```

然后采集数据。下面示例采集200条；需要其他数量时修改 `--episodes`。`--skip-initial-reset` 表示不重复执行开头的 reset；每条轨迹仍包含抓放结束后的右臂归位：

```bash
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/run_c1_pick_place_once.sh \
  --episodes 200 --randomize --skip-initial-reset \
  --record-root /ubt_sim/dataset/walker_c1_ros
```

脚本默认保存成功轨迹到：

```text
ubt_sim/dataset/walker_c1_ros/<episode_id>/trajectory.hdf5
```

## 3. 启动训练容器

```bash
docker start lerobot-walker-c1
```

## 4. 转换为 LeRobot 数据集

注意：下面的命令会覆盖同名的旧 LeRobot 数据集。

```bash
docker exec -it lerobot-walker-c1 bash -lc '
cd /ubt_IL && /lerobot/.venv/bin/python scripts/convert/convert_to_lerobot.py \
  --config scripts/convert/configs/Walker_C1_26_1RGB.json \
  --src_root /ubt_IL/dataset_source/walker_c1_ros \
  --tgt_path /ubt_IL/dataset \
  --repo_id Walker_C1_26_1RGB \
  --fps 30 \
  --robot_type walker_c1 \
  --task_name walker_c1_pick_place'
```

## 5. 正式训练 Diffusion Policy

训练会使用 GPU；建议先停止仿真释放显存：

```bash
docker stop walker-c1-ubt-sim
```

```bash
docker exec -it lerobot-walker-c1 bash -lc '
cd /ubt_IL && \
  /lerobot/.venv/bin/lerobot-train \
  --config_path=/ubt_IL/scripts/deploy/train_config_walker_c1_diffusion.json'
```

默认训练50000 step，每10000 step保存一次。在当前 RTX 5880 Ada 上预计约1.5小时。最终 checkpoint：

```text
ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model
```

训练未完成时，也可以把后续命令中的 `050000` 换成已经保存的 checkpoint，例如 `020000`。

## 6. 离线推理检查

加载正式 checkpoint，用数据集第0帧生成26维动作；该命令不会控制机器人。

```bash
docker exec -it lerobot-walker-c1 bash -lc '
HF_HUB_OFFLINE=1 /lerobot/.venv/bin/python \
  /ubt_IL/scripts/deploy/infer_walker_c1_diffusion.py \
  --checkpoint /ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model \
  --dataset /ubt_IL/dataset/Walker_C1_26_1RGB'
```

看到以下结果表示训练、保存和推理链路正常：

```text
policy=DiffusionPolicy
action_shape=(1, 26)
finite=True
```

## 7. C1仿真闭环推理

终端1启动仿真：

```bash
docker start walker-c1-ubt-sim
docker exec -it walker-c1-ubt-sim \
  bash /ubt_sim/scripts/start_c1_pick_place_sim.sh
```

终端2先手动执行一次 reset。推理数据从准备姿势开始，因此必须等 reset 完成后再启动策略：

```bash
docker exec -it walker-c1-ubt-sim bash -lc '
source /opt/ros/humble/setup.bash && \
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash && \
ROS_DOMAIN_ID=146 /usr/bin/python3 \
  /ubt_sim/teleoperation/control/walker_c1/reset.py'

docker start lerobot-walker-c1
```

单次闭环推理使用与 S2/Tienkung 相同的 LeRobot 原生 `base` strategy。下面显式设置最多
运行30秒；进程结束时会停止推理引擎并断开 Bridge2 和相机：

```bash
docker exec -it lerobot-walker-c1 bash -lc '
ROS_DOMAIN_ID=146 DURATION=30 \
POLICY_PATH=/ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model \
bash /ubt_IL/scripts/deploy/rollout_walker_c1.sh'
```

同一个脚本也支持多次推理，但采用外层进程循环，而不是常驻模型。每一轮都会先随机放置苹果，
然后重新加载 checkpoint、Bridge2 和相机，运行一次原生 `base` rollout，最后根据苹果与
固定盘心的距离统计成功率：

```bash
docker exec -it lerobot-walker-c1 bash -lc '
ROS_DOMAIN_ID=146 \
POLICY_PATH=/ubt_IL/model/Walker_C1_26_1RGB_diffusion/checkpoints/050000/pretrained_model \
bash /ubt_IL/scripts/deploy/rollout_walker_c1.sh \
  --episodes 10 --duration 30 --seed 1000'
```

不加 `--episodes` 时默认只运行1次。固定苹果在采集区域中心时增加 `--fixed-apple`。
多轮开始前只需手动执行一次 `reset.py`；每轮正常退出时，原生 rollout 会把机器人送回该轮
启动时的姿势。如果某轮异常退出或盘子被碰走，应停止批量运行，恢复机器人/场景后再继续。
当前脚本可以自动放回苹果，但盘子没有独立的自动归位接口。

如果使用其他 checkpoint，只需要替换 `POLICY_PATH`。reset 后头部和左臂保持固定，策略只实际驱动右臂及手部。
