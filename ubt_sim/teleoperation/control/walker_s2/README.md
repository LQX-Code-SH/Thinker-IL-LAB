# Walker S2 EDU 探索者 control

Walker S2 EDU 探索者 ROS2 control helpers copied and adapted from `/home/qingxiangliu/work/ubt_IL/walker/walker_sdk_ros2/robot_control`.

Run these scripts with system Python and the Walker SDK ROS2 messages sourced:

```bash
source /opt/ros/humble/setup.bash
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash
```

Typical simulation flow:

```bash
# Terminal 1: start Walker S2 EDU 探索者 sim and bridge
cd /home/qingxiangliu/work/Thinker-IL-LAB/ubt_sim
UBT_SIM_TASK=UBTSim-WalkerS2-PartSorting-v0 bash scripts/start_sim.sh

# Terminal 2: query state / run small tests（统一入口 walker_s2_controller.py + 子命令）
cd /home/qingxiangliu/work/Thinker-IL-LAB/ubt_sim/teleoperation/control/walker_s2
/usr/bin/python3 walker_s2_controller.py state --print-state      # 原 walker_s2_controller.py --print-state
/usr/bin/python3 walker_s2_controller.py joint --print            # 原 walker_s2_joint_test.py --print
/usr/bin/python3 walker_s2_controller.py camera --msg-type sensor_msgs/Image
/usr/bin/python3 walker_s2_controller.py home                     # 分段回 home 全零位 + 张开双手
/usr/bin/python3 walker_s2_controller.py endpoint                 # 末端/TCP 位姿
/usr/bin/python3 walker_s2_controller.py analyze --help           # 关节阶跃/正弦响应分析
# 无子命令默认走 state：python3 walker_s2_controller.py --print-state
```

## 目录结构

顶层 = 直接运行的入口脚本；`utils/` = 父类 + 工具类（被引用，不直接运行）。

顶层入口：

- `walker_s2_controller.py` - 统一 CLI 入口（子命令 `state`/`joint`/`endpoint`/`home`/`analyze`/`camera`，合并自原 6 个 controller 家族脚本）。
- `pick_part.py` - 抓取任务编排 + CLI（`--save` 控制 HDF5 录制，合并原 `pick_part_save_data.py`）。
- `carry_box.py` - 双臂搬箱任务 + CLI。

`utils/` 子包：

- `controller.py` - `WalkerS2Controller`（核心控制器：身体轨迹/手/夹爪/IK/EE-delta/位姿 + 回零 + 单关节调试 + 末端位姿调试）+ `RobotController` 别名 + state/joint/endpoint/home CLI。
- `constants.py` - 单一常量来源（关节名/限位/topic/位姿/夹爪参数）。
- `ik.py` - `WalkerS2IK`（pinocchio IK 包装）。
- `camera.py` - `Camera`（sensor_msgs/Image + shm_msgs）+ camera CLI。
- `joint_analyzer.py` - `JointAnalyzer`（关节阶跃/正弦响应分析）。
- `recorder.py` - `WalkerS2DataRecorder`（HDF5 录制节点，`pick_part.py --save` 使用）。

消费方（`carry_box` / `pick_part` 等）应 `from utils.controller import ...`；`utils/` 内部用相对导入。
