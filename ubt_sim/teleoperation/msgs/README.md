# ROS2 消息包（按机型分目录）

控制脚本与桥接发的都是真机同款话题，消息定义按**机型**分目录存放：每台机器人只
依赖自己目录里的包，编译产物也落在各自的独立工作空间，互不覆盖、互不 source。

| 机型 | 目录 | 包 | 编译产物 / 安装位置 |
|------|------|----|---------------------|
| Walker S2 | [walker_s2/](walker_s2/) | `rosa_msgs` `shm_msgs` `mc_task_msgs` `mc_state_msgs` `ecat_task_msgs` | `/opt/ubt_sim/walker_sdk_ros2_msgs/install` |
| Walker C1 | [walker_c1/](walker_c1/) | `rosa_msgs` `shm_msgs` `mc_task_msgs` `mc_state_msgs` `ecat_task_msgs` | `/opt/ubt_sim/walker_c1_msgs/install` |
| 天工行者 Pro | [tienkung_pro/](tienkung_pro/) | `bodyctrl_msgs` | deb 装入 `/opt/ros/humble`（ROS2 自动发现） |
| 天工行者 Dex | [tienkung_dex/](tienkung_dex/) | `ros2_bridge_msgs` `inspire_hand_msgs` `brainco_hand_msgs` | `/opt/ubt_sim/tienkung_dex_msgs/install` |

## 编译 / 安装

容器内（`/ubt_sim` 即仓库的 `ubt_sim/`）：

```bash
# Walker S2
bash teleoperation/msgs/walker_s2/build.sh
source /opt/ubt_sim/walker_sdk_ros2_msgs/install/setup.bash

# Walker C1（独立源码与工作空间）
bash teleoperation/msgs/walker_c1/build.sh
source /opt/ubt_sim/walker_c1_msgs/install/setup.bash

# TienKung Dex（因时 + 强脑双手）
bash teleoperation/msgs/tienkung_dex/build.sh
source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash

# TienKung Pro
bash teleoperation/msgs/tienkung_pro/install.sh    # dpkg 安装 bodyctrl_msgs
```

`docker/run.sh init` 会依次调用上面四个脚本；任何一个失败都不影响其它继续编译，
最后汇总报错。只想给某一台机器人补消息包时，单独跑它自己的脚本即可。

输出工作空间可用环境变量改：`WALKER_MSGS_OUT`、`WALKER_C1_MSGS_OUT`、
`TIENKUNG_DEX_MSGS_OUT`。

## 目录说明

```
walker_s2/
└── walker_sdk_ros2_msgs_src/src/{rosa_msgs,shm_msgs,mc_task_msgs,mc_state_msgs,ecat_task_msgs}
walker_c1/
├── build.sh
└── walker_sdk_ros2_msgs_src/src/{rosa_msgs,shm_msgs,mc_task_msgs,mc_state_msgs,ecat_task_msgs}
tienkung_pro/
├── ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb   # x86_64 预编译，含原生 type-support
├── install.sh
└── bodyctrl_msgs_src/                           # 同版本源码：arm64 编译与重新出包用
    ├── release.sh                               # 打 deb
    └── src/bodyctrl_msgs
tienkung_dex/
├── build.sh
├── ros2_bridge_msgs_src/src/ros2_bridge_msgs     # 臂/头/腰/腿/状态，与真机一致
└── dex_hand_msgs_src/src/{inspire_hand_msgs,brainco_hand_msgs}
```

## 注意

- **Walker S2 与 Walker C1 含同名包**（`mc_task_msgs` 等），但源码目录与工作空间
  完全独立：可以分别按机型演进而互不影响。同一 shell **不要同时 source** 两个
  workspace —— 后 source 的会覆盖前者的同名包。切换机型时建议：
  - `UBT_SIM_ROBOT=walker_c1 bash run.sh bash`（交互 shell 按机型只加载一份）
  - 或新开一个 shell 再 source 目标机型
- 两份 SDK 消息源码目前接口一致（仅 `package.xml` 的维护者邮箱在 S2 那份做过脱敏）；
  之后任何一侧改字段，都要同步检查另一侧是否需要跟进，或明确只在单机型演进。
- `tienkung_dex` 的消息包是**按厂商 SDK 用法重建**的，字段布局未与厂商包逐字段
  核对；上真机时以厂商自带消息包为准（桥接启动会打 WARN）。
- `bodyctrl_msgs` 走 deb 装入 `/opt/ros`，是唯一装到系统 ROS 空间的包；它只被
  `tienkung_pro` 使用，其余机型不导入，因此不影响隔离。
- 改消息定义后重新编译即可；重新出包在 `tienkung_pro/bodyctrl_msgs_src/` 里执行
  `bash release.sh`（脚本用相对路径，必须先 cd 进去），产物是新的 deb。
