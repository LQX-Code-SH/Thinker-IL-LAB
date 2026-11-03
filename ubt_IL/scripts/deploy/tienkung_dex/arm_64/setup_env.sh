#!/bin/bash
# 层 A 环境构建：venv(--system-site-packages) + LeRobot + tienkung_dex 插件。
# 目标机器：NVIDIA Thor，Ubuntu 24.04 / ROS 2 Jazzy / python 3.12 / torch 2.9.1+cu130(sm_110)
#
# 为什么用 venv 而不是 conda / docker（2026-11-03 实测）：
#   * 宿主 torch 是本地编译的 aarch64 wheel（已删源），PyPI 的 aarch64 torch 无 CUDA，
#     jetson-ai-lab 的 jp7/cu130 索引查不到 cp312+aarch64 轮子 → conda 拿不到可用 torch；
#   * venv --system-site-packages 直接继承宿主的 sm_110 torch，且实测 numpy 2.2.6 下
#     torch / torchvision / cv2 全部正常（Orin 上必须钉 numpy<2 的坑在 Thor 上不存在）；
#   * docker 的 arm64 镜像面向 JetPack6/Orin sm_87，与本机不兼容（详见 arm_64/README.md）。
set -e

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

PIP_MIRROR="${PIP_MIRROR:-https://pypi.tuna.tsinghua.edu.cn/simple}"
IDX="-i $PIP_MIRROR --extra-index-url https://pypi.org/simple"
PIP="$VENV/bin/pip"

echo "[setup] PROJECT_ROOT = $PROJECT_ROOT"
echo "[setup] VENV         = $VENV"

# ── 1. venv（幂等；--system-site-packages 继承 Jetson torch）────────────────
if [ -x "$VENV/bin/python" ]; then
    echo "[setup] venv 已存在，跳过创建"
else
    echo "[setup] 创建 venv（--system-site-packages）..."
    python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/python" -V

# ── 2. numpy 2：LeRobot 0.5.x 要求 numpy>=2,<2.3 ────────────────────────────
# 宿主 numpy 是 1.26.4（其他栈在用）；venv 内装 2.x 只影响层 A。
echo "[setup] 安装 numpy 2.2.6 ..."
env -u PYTHONPATH $PIP install --no-cache-dir $IDX "numpy==2.2.6"

# ── 3. LeRobot 运行时依赖 ──────────────────────────────────────────────────
# 逐条列明而不是 `pip install -e lerobot`：后者会把 torch 换回 PyPI 的 CPU 版。
# 不装 opencv（用宿主 GUI 版 cv2；opencv-python-headless 会覆盖掉 imshow）。
echo "[setup] 安装 LeRobot 运行时依赖 ..."
env -u PYTHONPATH $PIP install --no-cache-dir $IDX \
    "draccus==0.10.0" "gymnasium>=1.1.1,<2" einops safetensors \
    "huggingface-hub>=1.0,<2" "requests>=2.32" termcolor tqdm \
    "packaging>=24.2" "Pillow>=10" \
    datasets "av>=15.0.0,<16.0.0" pyzmq "cmake>=3.29.0.1"

# 3.5 含 C 扩展、且宿主 ~/.local 里已有一份**按 numpy 1.x 编译**的包：
#     pip 在 --system-site-packages 下会认为"已满足"而跳过，导致
#     "numpy.dtype size changed, Expected 96 ... got 88" 这种 ABI 崩溃。
#     import 链：lerobot.processor → transformers → sklearn/pandas。
#     必须 --ignore-installed 在 venv 内装 numpy 2 兼容版本。
echo "[setup] 安装 numpy2-ABI 依赖（pandas/pyarrow/matplotlib/sklearn）..."
env -u PYTHONPATH $PIP install --no-cache-dir --ignore-installed $IDX \
    pandas pyarrow "matplotlib>=3.7" scikit-learn

# 3.6 上一步可能把 numpy 顶到 2.4+（LeRobot 约束是 >=2.0,<2.3），最后钉回来。
echo "[setup] 回落 numpy 到 2.2.6（LeRobot 上限 <2.3）..."
env -u PYTHONPATH $PIP install --no-cache-dir $IDX "numpy==2.2.6"

# ── 4. editable 安装（--no-deps：绝不让 pip 碰 torch/numpy）────────────────
echo "[setup] editable 安装 LeRobot + tienkung_dex 插件 ..."
$PIP install --no-cache-dir --no-deps -e "$PROJECT_ROOT/lerobot"
$PIP install --no-cache-dir --no-deps -e "$DEX_DIR/lerobot_robot_tienkung_dex"

# ── 5. 自检 ────────────────────────────────────────────────────────────────
echo "[setup] 自检 ..."
env -u PYTHONPATH "$VENV/bin/python" - <<'PY'
import numpy, torch, torchvision, cv2

print(f"torch      {torch.__version__}  cuda={torch.cuda.is_available()}"
      f"  archs={torch.cuda.get_arch_list() if torch.cuda.is_available() else '-'}")
print(f"torchvision {torchvision.__version__} | numpy {numpy.__version__} | cv2 {cv2.__version__}")

_np = tuple(int(x) for x in numpy.__version__.split(".")[:2])
assert (2, 0) <= _np < (2, 3), f"LeRobot 要求 numpy>=2.0,<2.3，当前 {numpy.__version__}"
assert torch.cuda.is_available(), "CUDA 不可用：venv 没继承到宿主 Jetson torch"

import lerobot
from lerobot_robot_tienkung_dex import TienKungDexRobotConfig, DexCamera  # noqa: F401

cfgs = {}
for name in ("tienkung_dex_brainco_13", "tienkung_dex_20", "tienkung_dex_inspire_13"):
    cfg = TienKungDexRobotConfig(joint_config=name)
    cfgs[name] = cfg
    print(f"joint_config={name:26s} dim={len(cfg.all_joints):2d} hand={cfg.hand_variant} "
          f"mode={cfg.hand_action_mode} action_hand={len(cfg.right_hand_action_joints)}")

cfg = cfgs["tienkung_dex_brainco_13"]
assert len(cfg.all_joints) == 13 and cfg.hand_variant == "brainco"
assert cfg.topic_right_hand_cmd == "/right_hand/set_motor_multi"
insp = cfgs["tienkung_dex_inspire_13"]
assert (len(insp.all_joints) == 13 and insp.hand_variant == "inspire"
        and insp.hand_action_mode == "pico6"
        and len(insp.right_hand_action_joints) == 6
        and len(insp.right_hand_joints) == 13
        and len(insp.hand_open_position) == 6)
print("dex_camera 已注册 | 13D 契约: 右臂7 + 强脑6电机；因时 pico6: 6 维 0~1 -> 13 通道码")

# Bridge2 需要的关节序（插件传给桥接、桥接逐字断言）
print("right_hand_joints:", [j.removesuffix(".pos") for j in cfg.right_hand_joints])
PY

echo "[setup] CLI 入口检查（会真实 import lerobot.rollout 全链路）..."
if env -u PYTHONPATH "$VENV/bin/lerobot-rollout" --help >/dev/null 2>/tmp/dex_rollout_help.err; then
    echo "[setup] lerobot-rollout OK"
else
    echo "[setup] ERROR: lerobot-rollout 导入失败：" >&2
    tail -25 /tmp/dex_rollout_help.err >&2
    exit 1
fi

echo ""
echo "[setup] 完成。"
echo "        python : $VENV/bin/python"
echo "        rollout: bash $SCRIPT_DIR/rollout_host.sh"
echo "        无需 activate 也可；交互使用请: source $VENV/bin/activate"
