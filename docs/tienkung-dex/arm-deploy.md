# 天工行者 Dex：机器人本体部署（Thor / Jazzy，无容器）

本页说明如何把策略直接跑在机器人本体的 ARM 板（NVIDIA **Thor**，Ubuntu 24.04 + ROS 2 **Jazzy** + Python 3.12）上，不需要 Docker。数据转换与训练仍建议在工作站/容器内完成，本体上只做部署。

脚本位置：`ubt_IL/scripts/deploy/tienkung_dex/arm_64/`。

## 1. 运行架构（双 Python 栈，ZMQ 解耦）

```
层 A  $VENV (3.12)：lerobot-rollout / lerobot-train
        │ 5559 动作 ▼        ▲ 5560 状态       ▲ 5558 JPEG(640×360)
层 B  系统 python3 (3.12) + Jazzy：ros2_deploy_bridge.py
        │ /arm/cmd、/right_hand/set_motor_multi          ▲ /robot_state(≈200Hz)
        └ /ob_camera_head/color/image_raw (1280×720 → 缩放 640×360 → JPEG)
层 C  系统 python3 + Jazzy：robot_ready.py（预备位，与层 B 互斥）
```

| 层 | 解释器 | 跑什么 | 端口 / 话题 |
|---|---|---|---|
| A | `$PROJECT_ROOT/.venv-dex`（3.12） | `lerobot-rollout` / `lerobot-train` | 5559 发动作 / 5560 收状态 |
| B | `/usr/bin/python3` + Jazzy | `ros2_deploy_bridge.py` | 5558 JPEG、`/arm/cmd`、手指令 |
| C | `/usr/bin/python3` + Jazzy | `robot_ready.py`（与层 B 互斥） | `/arm/cmd` |

## 2. 为什么不用 Docker / conda

| 方案 | 结论 |
|---|---|
| docker | 仓库 `Dockerfile.arm64` 是 JetPack6 / 22.04 / Humble / Orin `sm_87`，与 Thor（JetPack7 / 24.04 / Jazzy / `sm_110`）不兼容；且当前用户不在 docker 组、无免密 sudo |
| conda | 宿主 torch 来自已删除的本地 wheel，PyPI 的 aarch64 torch 无 CUDA，jetson-ai-lab `jp7/cu130` 索引查不到 cp312 轮子 → 拿不到可用 torch |
| **venv(`--system-site-packages`)** | 直接继承宿主 `sm_110` torch；实测 numpy 2.2.6 下 torch/torchvision/cv2 均正常（Orin 上必须钉 `numpy<2` 的坑在这里不存在） |

## 3. 文件清单

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

## 4. 环境构建（首次）

仓库（含 `lerobot` 子模块）需同步到本体——`PROJECT_ROOT` 由脚本位置推导为仓库内的 `ubt_IL/`。LFS 大文件（`ubt_sim` 仿真资源）不需要。

**前置检查**：

```bash
python3 -V                                   # 3.12
ls /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c "import rclpy, ros2_bridge_msgs, bodyctrl_msgs, inspire_hand_msgs, brainco_hand_msgs"
/usr/bin/python3 -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_arch_list())"
```

厂商消息包默认装在系统里；若装在自定义工作空间，设 `TIENKUNG_DEX_MSGS_WS=<ws>`（默认 `/opt/tienkung_dex_msgs_ws`）。`ROS_SETUP` 可显式指定 ROS 发行版，默认依次找 `/opt/ros/jazzy`、`/opt/ros/humble`。

**建环境**：

```bash
cd ubt_IL/scripts/deploy/tienkung_dex/arm_64
bash setup_env.sh
```

`setup_env.sh` 幂等（重复跑会跳过建 venv、重装依赖），做的事：

| 步骤 | 说明 |
|---|---|
| 建 venv | `python3 -m venv --system-site-packages $PROJECT_ROOT/.venv-dex`；**不装 torch**，继承宿主 `sm_110` |
| numpy | 装 `numpy==2.2.6`（LeRobot 要求 `>=2.0,<2.3`） |
| 运行时依赖 | draccus / gymnasium / einops / safetensors / huggingface-hub / datasets / av / pyzmq 等；**不装 opencv**（用宿主带 GUI 的 cv2） |
| numpy2-ABI 包 | pandas / pyarrow / matplotlib / scikit-learn 用 `--ignore-installed`，否则宿主 `~/.local` 里按 numpy1 编译的包会报 `numpy.dtype size changed` |
| editable 安装 | `--no-deps -e lerobot` + `-e tienkung_dex`（`--no-deps` 防止 pip 把 torch 换成 PyPI 的 CPU 版） |
| 自检 | torch CUDA / `sm_110`、numpy 版本、三种 `JOINT_CONFIG` 的维度与手型契约、`lerobot-rollout --help` |

跑完应看到 `cuda=True`、`archs` 含 `sm_110`、`numpy 2.2.6`，以及：

```text
joint_config=tienkung_dex_brainco_13  dim=13 hand=brainco mode=rad13 ...
joint_config=tienkung_dex_20          dim=20 hand=inspire  mode=rad13 ...
joint_config=tienkung_dex_inspire_13  dim=13 hand=inspire  mode=pico6 ...
dex_camera 已注册 | 13D 契约: ...
[setup] lerobot-rollout OK
[setup] 完成。
```

失败点集中在两处：`cuda=False` 说明 venv 没继承到宿主 torch；`numpy.dtype size changed` 说明 ABI 包没装进 venv（重跑 `setup_env.sh`）。

## 5. 首轮联调（不接策略）

```bash
cd ubt_IL/scripts/deploy/tienkung_dex/arm_64

bash bridge_smoke.sh                     # 只读体检：status ≥30 Hz、image ≥10 fps、estop=false
bash camera_preview.sh --snapshot        # 抓一张策略视角（/tmp/dex_camera_preview.png）
source ./env.sh && dex_source_ros        # 层 B 用系统 python + Jazzy
/usr/bin/python3 hold_test.py --groups arm   # 零位移通路验收
bash staged_test.sh --step all           # 头/手/臂微动再回位，确认方向与回位
bash robot_ready_host.sh --dry-run       # 先看四段目标
bash robot_ready_host.sh --real          # 实发预备位（人盯急停）
```

因时 r13 链路：`HAND_VARIANT=inspire HAND_ACTION_MODE=pico6 bash bridge_smoke.sh`（状态里的 `right_hand` 应为 6 维）。`camera_preview.sh` 会自己起一个只订阅的 Bridge2，`--size raw` 看原始 1280×720，`--port 5558` 连已有桥接。

## 6. 部署推理

```bash
POLICY_PATH=/abs/path/pretrained_model \
JOINT_CONFIG=tienkung_dex_brainco_13 \
CAMERA_PORT=5558 IMAGE_ENABLED=1 FPS=30 \
DURATION=10 MAX_RELATIVE_TARGET=0.01 \
bash rollout_host.sh
```

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

## 7. 与容器版对照

| 容器版 | 本体版 | 说明 |
|---|---|---|
| `robot_ready.sh --real` | `robot_ready_host.sh --real` | 系统 python3 + Jazzy |
| `rollout.sh` | `rollout_host.sh` | venv 内跑，自动拉起 Bridge2 |
| `preview_camera.py` | `camera_preview.sh` | 本体上由 Bridge2 直接转码 |
| `inference_server.sh` | 暂无 | 常驻推理服务仍需容器版 |

## 8. 注意事项与排错

- **相机尺寸不匹配不会报错**：720p 原样送入时 `dex_camera` 只按 `offset_x+width` 切列，会拿到「左半幅」，策略照样输出、只是画面分布全错。已由 Bridge2 的 `image_width/image_height` 在编码前缩放修正，首次务必 `camera_preview.sh --snapshot` 确认是完整视野。
- **`bash -lc` 不 source `.bashrc`**：Bridge2 靠父进程继承 ROS 环境，脚本里用 `dex_source_ros` 显式 source Jazzy；手工起桥接时同样要先 source。
- **`/arm/cmd` 无人消费**：机器人必须处于话题控制模式（遥控器切半身策略并上拨），否则「策略在跑但机器人不动」。
- **`robot_ready` 与 Bridge2 互斥**：两者都写 `/arm/cmd`，顺序永远是先 ready 再 rollout。
- **急停**：Bridge2 监听 `/power/board/key_status` 的 `is_estop`；已下发目标仍在驱动器里，必要时要按物理急停。
- **`wrist_roll_l` 40 Hz 摩擦极限环**：真机 home/预备位已由 `constants.DEPLOY_HOME_OFFSETS` 叠加 +0.03 rad；换机后若右腕同样嗡嗡，往该表加一条。
- **`cuda=False`**：venv 没继承到宿主 torch，确认 `python3 -m venv --system-site-packages` 创建的是 `$PROJECT_ROOT/.venv-dex`。
- **`numpy.dtype size changed`**：ABI 包没装进 venv，重跑 `setup_env.sh`。

完整文件清单与实测记录见仓库 `ubt_IL/scripts/deploy/tienkung_dex/arm_64/README.md`。
