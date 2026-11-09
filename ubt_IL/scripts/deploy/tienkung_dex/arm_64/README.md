# 天工行者 Dex（Walker TienKung Dex）真机部署（Thor / Jazzy，去 Docker）

本目录是 **天工行者 Dex 真机** 的自包含部署包，镜像 `scripts/deploy/tienkung_pro/arm_64` 的做法：
不 sudo、不建软链、不改原脚本，用 `PROJECT_ROOT` + 环境变量覆盖原脚本里的容器路径
（`/ubt_IL`、`/lerobot/.venv`、`/opt/ros/humble`）。

- 目标机器：NVIDIA **Thor**，Ubuntu 24.04，**ROS 2 Jazzy**，python 3.12，torch 2.9.1+cu130（`sm_110`）
- 实物手型：**强脑 BrainCo Revo2（6 电机，SDK 1–1000 闭合度）** → `JOINT_CONFIG=tienkung_dex_brainco_13`
- 因时 RH56 手（r13 数据集）→ `JOINT_CONFIG=tienkung_dex_inspire_13`：策略侧也是
  **13D = 右臂 7 + 右手 6 维归一化 0~1**，由 Bridge2 展开成 13 个 0.1° 角码
  （详见 `../README.md`「手部量纲」）。
- 动作空间：**13D = 右臂 7 + 右手 6**；左臂/腰/头不进模型（`robot_ready` 摆位后保持）

## 1 架构（双 Python 栈，ZMQ 解耦）

```
层 A  $VENV (3.12)：lerobot-rollout / lerobot-train
        │ 5559 动作 ▼        ▲ 5560 状态       ▲ 5558 JPEG(640×360)
层 B  系统 python3 (3.12) + Jazzy：ros2_deploy_bridge.py
        │ /arm/cmd、/right_hand/set_motor_multi          ▲ /robot_state(≈200Hz)
        └ /ob_camera_head/color/image_raw (1280×720 → 缩放 640×360 → JPEG)
层 C  系统 python3 + Jazzy：robot_ready.py（预备位，与层 B 互斥）
```

**为什么不 docker / conda**

| 方案 | 结论 |
|---|---|
| docker | 仓库 `Dockerfile.arm64` 是 JetPack6 / 22.04 / Humble / Orin `sm_87`，与 Thor（JetPack7 / 24.04 / Jazzy / `sm_110`）不兼容；且当前用户不在 docker 组、无免密 sudo |
| conda | 宿主 torch 来自已删除的本地 wheel，PyPI 的 aarch64 torch 无 CUDA，jetson-ai-lab `jp7/cu130` 索引查不到 cp312 轮子 → 拿不到可用 torch |
| **venv(`--system-site-packages`)** | 直接继承宿主 `sm_110` torch；实测 numpy 2.2.6 下 torch/torchvision/cv2 均正常（Orin 上必须钉 `numpy<2` 的坑在这里不存在） |

## 2 文件

| 文件 | 作用 |
|---|---|
| `env.sh` | 公共常量（`PROJECT_ROOT`/`VENV`/`BRIDGE_SCRIPT`）+ `dex_source_ros`（显式 source Jazzy） |
| `setup_env.sh` | 建 venv、装 LeRobot/插件（`--no-deps` 保 torch）、自检 13D 契约 |
| `robot_ready_host.sh` | 预备位（系统 python + Jazzy） |
| `rollout_host.sh` | 策略推理（venv），带维度预检 |
| `bridge_smoke.sh` | 只读冒烟：验证 状态/图像/急停 三条链路，不动机器人 |
| `camera_preview.sh` | 头相机预览：实时窗口 / `--snapshot` 存图 / `--size raw` 看 720p |
| `hold_test.py` | 控制通路零位移验收（目标 = 当前实测角度，不产生动作） |
| `staged_test.sh` | **分步微动**：头 / 手 / 臂各 ±0.05 rad（手 80 码）偏出再回位，带实测核对 |
| `robot_home_host.sh` | **回站立位**：按预备位四段航点倒序走（手全程在桌面以上），目标为 `HOME_POSE` + 真机微偏移 |

## 3 使用顺序

```bash
cd ubt_IL/scripts/deploy/tienkung_dex/arm_64

bash setup_env.sh                     # ① 建环境（一次性）
bash bridge_smoke.sh                  # ② 只读体检：status/image 计数
bash camera_preview.sh --snapshot     # ③ 看一眼策略视角画面
source ./env.sh && dex_source_ros
/usr/bin/python3 hold_test.py --groups arm   # ④ 零位移通路验收（层 B：系统 python + Jazzy）
bash staged_test.sh --step all        # ⑤ 分步微动（头/手/臂），确认方向与回位
bash robot_ready_host.sh --dry-run    # ⑥ 先看四段目标
bash robot_ready_host.sh --real       # ⑦ 实发预备位（人盯急停）
POLICY_PATH=/abs/pretrained_model \
  DURATION=10 MAX_RELATIVE_TARGET=0.01 \
  bash rollout_host.sh                # ⑧ 首轮短时小限幅
```

## 4 关键参数

| 变量 | 默认 | 说明 |
|---|---|---|
| `POLICY_PATH` | 无（必填） | checkpoint 的 `pretrained_model` 目录 |
| `JOINT_CONFIG` | `tienkung_dex_brainco_13` | 强脑 6 电机；因时 20D 用 `tienkung_dex_20`；因时 r13 数据集用 `tienkung_dex_inspire_13` |
| `HAND_ACTION_MODE` | 空（自动） | 留空由 `JOINT_CONFIG` 推导；`tienkung_dex_inspire_13` → `pico6` |
| `FPS` | `15` | 必须 = 数据集帧率 |
| `CAMERA_PORT` / `IMAGE_ENABLED` | `5558` / `1` | 真机走 Bridge2 转码 |
| `CAMERA_WIDTH/HEIGHT` | `640×360` | 同时用于 Bridge2 缩放与 `dex_camera` 裁剪窗口 |
| `MAX_RELATIVE_TARGET` | 空 | 首轮 `0.01`（rad/步） |
| `DISPLAY_CAM` | `true` | 本机无 X（SSH）时设 `false` |

## 5 与 Orin 版（tienkung_pro/arm_64）的差异

1. **不必钉 `numpy<2`**：Thor 的 torch 2.9.1 支持 numpy 2，LeRobot 要求 `numpy>=2` 正好满足。
2. **不需要 `image_server`**：Dex 的头相机由 Bridge2 直接用 `--enable-image` 转码，少一个常驻进程。
3. **ROS 发行版是 Jazzy**：原脚本硬编码 `/opt/ros/humble` 会直接退出，host 版统一走 `dex_source_ros`。
4. **`bash -lc` 不 source `.bashrc`**（实测）：Bridge2 靠父进程 env 继承 ROS，所以启动脚本必须先 source。

## 6 已知坑

- **相机尺寸不匹配不会报错**：720p 原样送入时 `dex_camera` 只按 `offset_x+width` 切列，会拿到"左半幅"，
  策略照样输出、只是画面分布全错。已由 Bridge2 的 `image_width/image_height` 在编码前缩放修正，
  首次务必用 `DISPLAY_CAM=true` 或抓帧确认画面是完整视野。
- **`/arm/cmd` 无人消费**：机器人必须处于话题控制模式（遥控器切半身/全身策略并上拨），
  否则"策略在跑但机器人不动"。
- **`robot_ready` 与 Bridge2 互斥**：两者都写 `/arm/cmd`，顺序永远是先 ready 再 rollout。
- **急停**：Bridge2 监听 `/power/board/key_status` 的 `is_estop`，为 true 时停发；已下发目标仍在驱动器里，
  必要时要按物理急停。
- **`wrist_roll_l` 的 40 Hz 摩擦极限环**（2026-11-03 实测）：目标停在 ≈0 时位置只抖 0.15°，
  但速度峰峰 1.36 rad/s、电流峰峰 2.4，听感是持续高频嗡嗡；偏到 +0.03 rad 后速度峰峰 0.002、
  电流 0，往复 2 轮 100% 复现。已由 `constants.DEPLOY_HOME_OFFSETS` 在**真机** home/预备位上
  叠加 +0.03（仿真常量与训练姿态不动）。若换手/换机后右腕也出现，同样往该表里加一条即可。
