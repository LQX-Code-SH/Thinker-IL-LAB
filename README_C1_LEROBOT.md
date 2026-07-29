# Walker C1（Astron）：仿真采集、训练与推理

以下命令均在仓库根目录执行。当前流程使用单个头部 RGB 相机、30 FPS、26维状态和26维动作。

## 项目概览

Walker C1 的目标是用**同一份控制代码**同时驱动 Isaac Sim 仿真和真机（切换
`ROS_DOMAIN_ID` 146 ↔ 0）。当前任务：机器人从 `reset.py` 准备姿势出发，在线读取桌上
苹果的真实位置，用 6D IK 现场规划抓取路径，抓起后放入盘子，再安全归位。

**这是在线感知 + 现场 IK 规划，不是逐帧回放录制轨迹**：每一局都重新读取物体位置、重新
解算关节角度。

整条链路分两层：

```text
Isaac Sim（仿真物理 + 传感器）
   │  ZMQ (127.0.0.1:5655-5658)
   ▼
ROS2-ZMQ Bridge（teleoperation/bridges/walker_c1/）—— 话题与真机 SDK 一致
   │  ROS2 DDS（domain 146=仿真 / 0=真机）
   ▼
控制代码（teleoperation/control/walker_c1/）：
   reset.py              安全分阶段回到准备姿势
   robot_controller.py   ikpy 6D IK、仿真步同步 wait_sim_steps()、安全 z 下限
   pick_place_controller.py  在线抓取主逻辑（读物体位置→IK→闭环对准→抓取→搬运→放盘→归位）
   │
   ▼
LeRobot（ubt_IL/）：HDF5 → LeRobot 数据集转换 → Diffusion Policy 训练 → checkpoint 推理
```


## 目录结构（Walker C1 相关部分）

```text
ubt_sim/
├── assets/robots/walker_c1/
│   └── Collected_walker_c1_v1_sensorKpkd/Collected_walker_astron_v1_sensorKpkd/
│       ├── walker_astron_v1_sensorKpkd_hands.usd  # 机器人 USD（身体+传感器+灵巧手）
│       └── SubUSDs/                               # 双目/鱼眼/下巴相机子文件；上面的 USD 通过
│                                                    # 相对路径引用它们，删除或挪动会导致相机加载失败
├── scripts/
│   ├── build_c1_apple_usd.py             # 重建任务苹果小型 USD
│   ├── start_c1_mentor_sensor_sim.sh     # 启动仿真 + 五相机 viewport（+ --control 开 ROS）
│   ├── start_c1_pick_place_sim.sh        # 启动仿真 + ROS bridge（无额外相机窗口）
│   ├── run_c1_pick_place_once.sh         # 单次/多次在线 IK 抓放 + 数据采集
│   └── run_c1_online_ik_batch.sh         # 批量运行，失败自动重启整个仿真栈
├── source/ubt_sim/devices/walker_c1/     # Isaac Lab 侧机器人配置/controller
└── teleoperation/
    ├── bridges/walker_c1/                # ROS2-ZMQ 桥接，话题与真机 SDK 一致
    └── control/walker_c1/
        ├── reset.py                     # 安全回零
        ├── robot_controller.py          # IK / 仿真步同步基础设施
        └── pick_place_controller.py     # 在线抓取主实现

ubt_IL/
├── scripts/convert/configs/Walker_C1_26_1RGB.json   # HDF5 -> LeRobot 转换配置
└── scripts/deploy/
    ├── train_config_walker_c1_diffusion.json        # Diffusion Policy 训练配置
    ├── infer_walker_c1_diffusion.py                  # 离线推理检查
    └── rollout_walker_c1.sh                          # 仿真/真机闭环推理入口
```

## 当前状态

完整链路已经跑通：HDF5采集 → LeRobot转换 → Diffusion Policy训练 → checkpoint离线推理 → C1仿真闭环控制。

初始 reset 和苹果落稳过程不录制，轨迹从准备姿势开始，并包含抓放结束后的右臂归位；左臂和头部在 reset 后保持固定。

当前 Diffusion Policy 使用 `horizon=64`、`n_action_steps=32`、`n_obs_steps=2`，训练默认 `batch_size=8`、`steps=50000`。

## 0. 首次构建容器

首次使用需要分别构建仿真容器和 LeRobot 训练/推理容器。已经创建过容器时可跳过本节，
直接从第 1 节开始。构建前请确认 Docker、NVIDIA 驱动和 NVIDIA Container Toolkit 可用。

### 0.1 构建 Isaac Sim 仿真容器

```bash
cd ubt_sim/docker
bash run.sh build
bash run.sh start
bash run.sh init
bash run.sh check
cd ../..
```

上述命令构建 `ubt-sim-isaac:latest` 镜像，创建并启动
`walker-c1-ubt-sim` 容器，随后安装 `ubt_sim`、编译 Walker ROS 2 消息和图像桥接。
`start` 是幂等操作：容器已存在时只会将其启动，不会重复创建。

### 0.2 构建 LeRobot 训练与推理容器

```bash
cd ubt_IL/docker
bash run.sh build
CONTAINER_NAME=lerobot-walker-c1 DOMAIN_ID=146 bash run.sh start
CONTAINER_NAME=lerobot-walker-c1 DOMAIN_ID=146 bash run.sh check
cd ../..
```

必须显式设置 `CONTAINER_NAME=lerobot-walker-c1`，因为 `ubt_IL/docker/env.sh`
的默认容器名是 `lerobot-tienkung`。`DOMAIN_ID=146` 表示连接 C1 仿真；
后续连接真机时使用 `DOMAIN_ID=0`。首次启动会在容器入口中安装 LeRobot、
Walker 插件并编译 ROS 2 消息，完成前不要中断。

## 1. 启动已创建的仿真容器

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

## 3. 启动已创建的训练容器

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



## 6. C1仿真闭环推理

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

不加 `--episodes` 时默认只运行1次。默认苹果固定放在采集区域中心，需要随机化时加
`--randomize-apple`（5cm×5cm 范围内采样）。
多轮开始前只需手动执行一次 `reset.py`；每轮正常退出时，原生 rollout 会把机器人送回该轮
启动时的姿势。如果某轮异常退出或盘子被碰走，应停止批量运行，恢复机器人/场景后再继续。
当前脚本可以自动放回苹果，但盘子没有独立的自动归位接口。

如果使用其他 checkpoint，只需要替换 `POLICY_PATH`。reset 后头部和左臂保持固定，策略只实际驱动右臂及手部。

## 已知限制

1. 当前 CPU 仿真约为实时的 0.25 倍，控制器里的等待都是按仿真步数计时
   （`wait_sim_steps()`），不是墙钟秒数；日志中看到的短暂停顿是正常的物理步等待。
2. 盘子没有独立的自动归位接口；盘子被碰走后需要人工恢复场景再继续批量运行。
3. 仿真进程长时间运行或经历剧烈失败后，PhysX 接触状态可能退化导致连续抓空；出现这种情况
   应完整重启仿真栈，而不是在同一进程里继续调参数。
4. 当前成果是 Isaac Sim + ROS2 控制链在仿真中的验证，尚未在真机上跑通，上真机前必须先完成
   关节命名核对。

## 参考文档

- [`C1_joint_map.md`](C1_joint_map.md) — 关节分组、限位表、与真机 SDK 文档的逐项核对结果。
- [`ubt_sim/README.md`](ubt_sim/README.md) — 仿真平台整体架构、多机器人支持、容器与扩展方式。
