# Walker C1（Astron）仿真使用说明

## 概述

Walker C1 为全机 53 DOF 人形（双臂 14 + 双手 11×2 + 头 2 + 腰 3 + 双腿 12），上体可控 41 关节。
本仓库实现其 Isaac Sim 仿真、ROS2-ZMQ 桥接、在线 IK 抓放与数据采集。

![Walker C1 仿真界面预览](../assets/walker_c1.png)



## 1. 环境准备

```bash
cd ubt_sim/docker
bash run.sh build          # 首次构建镜像
bash run.sh start          # 创建/启动容器
bash run.sh init           # 编译各机型消息包（含 walker_c1_msgs，与 walker_sdk 隔离）
bash run.sh check          # 校验环境（GPU / ROS2 / 消息包）

# 进入容器：
UBT_SIM_ROBOT=walker_c1 bash run.sh bash
```

> `bash run.sh bash` 会自动 source ROS2 与 Walker 消息包；C1 与 S2 的消息包同名但互相隔离，
> 靠 `UBT_SIM_ROBOT=walker_c1` 区分。


## 2. 启动仿真

```bash
# 方式一：C1 专用脚本（100 Hz 物理步进）
bash scripts/start_c1_pick_place_sim.sh

# 方式二：统一启动器
UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0 bash scripts/start_sim.sh

```

启动后会依次打印 ZMQ 端口、action 关节顺序、手部关节限位与增益摘要。

常用环境变量（可叠加）：

| 变量 | 作用 | 默认 |
|------|------|------|
| `UBT_SIM_TASK` | 任务名（决定机器人类型） | `UBTSim-TienkungPro-Parlor-v0` |
| `UBT_SIM_WALKER_C1_USD_PATH` | 覆盖机器人 USD | `usd/walker_c1_merged.usd` |
| `UBT_SIM_LOAD_ONLY=1` | 只加载场景、不起桥接 | 关 |
| `--headless` | 无窗口运行 | 无显示器场景 |
| `--device cpu\|cuda` | 运行设备 | 视机器而定 |



## 3. 复位

```bash
# 任务预备位（ROS 话题，分三段：抬肩 → 收肘 → 预备位）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode task

# 场景复位（等价于在仿真窗口按 R）
/usr/bin/python3 teleoperation/control/walker_c1/reset.py --mode sim-scene
```

仿真窗口内 `R` = 场景复位，`Space` = 请求保存当前 episode（与 ROS 侧录制器配合）。

## 4. 数据采集

```bash
# 默认：直接采一条（复位场景 → 回预备位 → 抓放一次 → 落盘）
bash teleoperation/control/walker_c1/save_data.sh

# 批量：采 20 条，每集之间自动复位，成功才落盘
bash teleoperation/control/walker_c1/save_data.sh --batch 20
```

落盘位置：`dataset/walker_c1/<epoch秒>/trajectory.hdf5`（目录 0777、文件 0666）。


## 5. 数据回放

```bash
# 数据预览：
/usr/bin/python3 teleoperation/tools/playback_walker_c1.py --mode preview

# 控制回放（向仿真/真机同款话题发布控制机器人回放动作）
/usr/bin/python3 teleoperation/tools/playback_walker_c1.py \
    --episode dataset/walker_c1/<epoch秒>/trajectory.hdf5 --mode control
```


![Walker C1 数据集预览](../assets/Walker_C1数据集预览.png)

## 6. 常见问题

| 现象 | 排查 |
|------|------|
| `gym.make` 找不到任务 ID | `source/ubt_sim/__init__.py` 是否包含 `from .task.walker_c1_parlor import *` |
| 桥接启动即退出，报 `RMW implementation not installed` | RMW 与镜像不匹配：本仓库镜像只装了 `rmw_cyclonedds_cpp`，用 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；若镜像装了 FastRTPS 则用 `rmw_fastrtps_cpp`（`start_sim.sh` 已自动探测） |
| 桥接报 `Address already in use (tcp://*:5656)` | 旧桥接未退干净：先 `kill -9` 残留 `sim_runner`/`ros2_zmq_bridge`，确认 5655-5658 全 free 再启动 |
| 控制器报 `no /mc/sdk/robot_state` | 桥接没起来（端口冲突最常见）；用 `ros2 topic hz /mc/sdk/robot_state` 确认约 48 Hz |
| 指令发出去机器人不动 | 检查 `/mc/sdk/robot_command` 是否用了 `elbow_pitch`（`elbow_roll` 也可，桥接有别名） |
| 图像话题无数据 | 未加 `--enable_cameras`；或 C++ 图像桥未编译（`bash teleoperation/bridges/build_cpp_bridge.sh`） |
| 采集 HDF5 帧数不一致 | 录制节点在图像未到齐时会跳拍，属预期；若整集 0 帧检查相机话题 |
| 苹果抓不住 / 弹飞 | 手部增益必须保持 `10/1`、`effort_limit_sim=2`（力控）；不要按老调试值放大 |
| 与 Walker S2 同时启动后指令串流 | 二者端口相同（5655-5658），同一时间只能跑一个 |
| 仿真窗口消失但进程还在 | 上一次批量用了 `--headless`；见「在线 IK 抓放」下方提示 |
