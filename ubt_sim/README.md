# UBT-EDU-Sim-Lab 

UBTECH EDU 机器人仿真平台，基于 NVIDIA Isaac Sim 5.0 + Isaac Lab 2.2 支持多款机器人ROS2仿真控制，用于真机部署前的验证测试，模仿学习仿真数据采集等功能。

当前支持三款机器人（天工 Pro / Walker S2 / Walker C1）、遥操作仿真、数据采集（HDF5/LeRobot 格式）、轨迹回放、零件随机化。容器化部署，内部桥接ROS2 Humble（Py 3.10）与 Isaac Sim（Py 3.11）通信。

## Git Clone 注意事项

本项目使用 Git LFS 管理 3D 模型等大文件（`*.usd`、`*.urdf`、`*.exr`、`*.png`）。未安装 Git LFS 或下载不完整时，这些文件仅为指针文件，仿真无法启动。

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone <仓库地址>  # 先克隆代码，跳过 LFS
git lfs install                          # 安装 LFS（仅需一次）
git clone <仓库地址>                      # 克隆（LFS 文件自动下载）
git lfs pull                             # 补全未下载的 LFS 文件
```


## 支持的机器人

| 机器人 | DOF | 夹爪 | 手部 | USD 路径 |
|--------|-----|------|------|----------|
| TienKung Pro | 50 | — | 12×2 (五指灵巧手) | `assets/robots/tienkung_pro/tienkung_pro_v2.usd` |
| Walker S2 | 34 | 2×2 (PGC 两指) | — | `assets/robots/walker_s2/s2_v1.usd` |
| Walker C1 (Astron) | 53 | — | 11×2 (五指灵巧手) | `assets/robots/walker_c1/Collected_walker_c1_v1_sensorKpkd/Collected_walker_astron_v1_sensorKpkd/walker_astron_v1_sensorKpkd_hands.usd` |

## 当前支持的任务

| 任务 ID | 机器人 | 场景 | 说明 |
|---------|--------|------|------|
| `UBTSim-TienkungPro-Parlor-v0` | 天工 Pro | 客厅 | 基础遥操作，RGB + Depth 相机 |
| `UBTSim-WalkerS2-PartSorting-v0` | Walker S2 | 仓库 | 零件分拣，含零件随机化 |
| `UBTSim-WalkerC1-Parlor-v0` | Walker C1 | 客厅 | 在线 IK 苹果抓放，实时读物体位置现场规划，非轨迹回放 |

任务通过 YAML 配置文件定义（`config/*.yaml`），Python 任务类从 YAML 自动加载场景、机器人、相机和仿真参数。

## 快速开始

```bash
# 1. 构建并启动容器
cd docker
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check

# 2. 进入容器
bash run.sh bash

# 3. 启动仿真（根据 UBT_SIM_TASK 自动检测机器人并启动对应桥接）
# 天工 Pro（默认）：
bash scripts/start_sim.sh
# Walker S2：
UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh

# 4. 天工 Pro 数据采集
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung_pro/reset.py     # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/tienkung_pro/pick_place_save_data.py  # 单次采集
bash /ubt_sim/teleoperation/control/tienkung_pro/save_data.sh              # 批量采集
# （其他）测试天工 Pro 相机
python3 /ubt_sim/teleoperation/image/image_client.py
/usr/bin/python3 /ubt_sim/teleoperation/image/test_ros_image.py            # 测试相机ROS话题

# 5. Walker S2 数据采集
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_reset.py  # 重置场景，或R键重置
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_controller.py --init  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/pick_part_save_data.py  # 单次采集
bash /ubt_sim/teleoperation/control/walker_s2/save_data.sh                     # 批量采集
# （其他）测试walker S2 相机
/usr/bin/python3 /ubt_sim/teleoperation/control/walker_s2/walker_s2_camera.py --save --count 

# 6. Walker C1 数据采集与推理
docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/start_c1_pick_place_sim.sh   # 启动仿真+桥接
docker exec -it walker-c1-ubt-sim bash -lc \
  'ROS_DOMAIN_ID=146 /usr/bin/python3 /ubt_sim/teleoperation/control/walker_c1/reset.py'  # 回准备姿势
docker exec -it walker-c1-ubt-sim bash /ubt_sim/scripts/run_c1_pick_place_once.sh    # 单次在线IK抓放+采集
```
完整的采集/训练/推理流程、架构说明和安全须知见 [`README_C1_LEROBOT.md`](../README_C1_LEROBOT.md)。

## 项目结构

```
ubt_sim/
├── config/                # YAML 任务/场景配置
│   ├── tienkung_pro/
│   └── walker_s2/
├── source/ubt_sim/        # Python pip 包（Py 3.11，Isaac Sim 侧）
│   ├── devices/           # 机器人配置 + 遥操作设备
│   ├── env/               # 数字孪生环境 + MDP
│   ├── task/              # Gym 任务注册
│   └── utils/             # 工具函数
├── teleoperation/         # 遥操作（Py 3.10，ROS2 侧）
│   ├── bridges/           # ROS2-ZMQ 桥接
│   ├── control/           # 机器人控制 + 数据采集
│   ├── image/             # 图像传输
│   ├── msgs/              # ROS2 自定义消息
│   └── tools/             # 诊断工具
├── scripts/               # 仿真启动脚本
├── assets/                # 3D 模型（USD, URDF, 贴图）
│   ├── robots/
│   └── scenes/
├── docker/                # Docker 容器配置
├── shell/                 # Isaac Sim 运行时软链接
├── docs/                  # 文档
└── dataset/               # 采集数据（运行时生成）
```

## 扩展

### 添加新场景
1. 放置 3D 文件到 `assets/scenes/<name>/`
2. 创建 `config/<task>.yaml`（含 scene、robot、cameras、simulation 字段）
3. 创建 `source/ubt_sim/task/<task>/`（继承 `ManagerBasedRLDigitalTwinEnvCfg`）
4. 在 `source/ubt_sim/__init__.py` 注册任务

### 添加新机器人
扩展新机器人需在仿真侧、场景侧、遥操作侧分别新建模块，并接入启动链路。

**仿真侧（Isaac Sim，Py 3.11）**
- 设备包 `source/ubt_sim/devices/<name>/`：`config.py`（`<NAME>_CFG` / 关节组常量 / `<NAME>_USD_PATH`）+ `controller.py`（`<Name>Controller(DeviceBase)`）+ `action_process.py` + `__init__.py`
- 在 `source/ubt_sim/devices/__init__.py` 导出 controller 与 config 常量
- 任务包 `source/ubt_sim/task/<task>/`：`__init__.py`（`gym.register("UBTSim-<Robot>-<Scene>-v0", ...)`）+ `<task>_env_cfg.py`（继承 `ManagerBasedRLDigitalTwinEnvCfg`，实现 `use_teleop_device()` / `preprocess_device_action()`）
- 在 `source/ubt_sim/__init__.py` 加 `from .task.<task> import *`

**场景侧**
- 机器人模型 `assets/robots/<name>/`（USD + URDF + textures）
- 场景资产 `assets/scenes/<scene>/`（USD）
- 任务/场景配置 `config/<name>/<task>.yaml`（scene、robot、cameras、simulation 字段）

**遥操作侧（ROS2，Py 3.10）**
- 桥接 `teleoperation/bridges/<name>/<name>_ros2_zmq_bridge.py` + `<name>_bridge_config.yaml`
- 控制脚本 `teleoperation/control/<name>/`（回零、抓取、数据采集、constants）
- 自定义 ROS2 消息（按需）：源码放 `teleoperation/msgs/`，容器内构建到 `/opt/ubt_sim/`

**启动链路（编辑现有文件）**
- `scripts/sim_runner.py`：`_detect_robot()` 识别新任务名；`main()` 按机器人加 controller 实例化 / physics device 分支
- `scripts/start_sim.sh`：任务名→ROBOT 检测、bridge 启动分支

### 添加新控制任务
- 继承 `DeviceBase`（[source/ubt_sim/devices/device_base.py](source/ubt_sim/devices/device_base.py)），实现 `reset()` / `advance()` / `add_callback()`
- 在对应任务 EnvCfg 中实现 `preprocess_device_action()` 方法完成动作映射


## Walker S2 特殊说明

### GPU 渲染 + CPU PhysX

Walker S2 默认采用 **GPU 渲染 + CPU PhysX** 架构：`start_sim.sh` / `sim_runner.py` 将渲染/AppLauncher 设备设为 `cuda:0`，但 Isaac Lab physics/env 设备设为 `cpu`。这是为了规避 Isaac Sim 5.0 + Isaac Lab 2.2 在 Walker S2 articulation 初始化时 `get_dof_velocities()` 触发的 PhysX GPU tensor device mismatch（`getVelocities: expected device 0, received device -1`）。不要默认改回 GPU physics；如需实验可显式设置 `UBT_SIM_WALKER_S2_PHYSICS_DEVICE=cuda:0`。

### 头部材质修复

`sim_runner.py` 在环境初始化后自动修复 Walker S2 头部材质绑定（通过 `source/ubt_sim/utils/head_material.py`），支持多种材质模式通过 `UBT_SIM_WALKER_S2_HEAD_MATERIAL_MODE` 控制：
- `all`（默认）：每个 GeomSubset 使用对应材质
- `stable`：全部使用稳定灰调材质
- 也可指定单一材质（如 `paint_matte`、`steel_blued`、`glass`）




## License

Apache-2.0
