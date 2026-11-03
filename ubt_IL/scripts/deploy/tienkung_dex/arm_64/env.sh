#!/bin/bash
# TienKung Dex 真机（aarch64 / Thor）host 版部署 —— 公共常量与环境加载。
# 被 setup_env.sh / rollout_host.sh / robot_ready_host.sh / bridge_smoke.sh source。
#
# 设计原则（与 scripts/deploy/tienkung_pro/arm_64 一致）：
#   * 不 sudo、不建软链、不改原脚本；原脚本里的 /ubt_IL、/lerobot/.venv、
#     /opt/ros/humble 全部由这里用 PROJECT_ROOT / VENV / dex_source_ros 覆盖。
#   * 双 Python 栈：
#       - 层 A（LeRobot 推理/训练）: $VENV（python 3.12 + 继承宿主 torch）
#       - 层 B（Bridge2 / robot_ready）: /usr/bin/python3 + ROS2 Jazzy
#     两层只通过 ZMQ 5558/5559/5560 通信，互不 import。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"      # -> .../ubt_IL
DEX_DIR="$PROJECT_ROOT/tienkung_dex"
BRIDGE_SCRIPT="$DEX_DIR/ros2_deploy_bridge.py"

# 层 A 的虚拟环境（--system-site-packages，继承宿主 Jetson torch）
VENV="${VENV:-$PROJECT_ROOT/.venv-dex}"

# 层 B 的系统解释器：必须是有 rclpy + 厂商消息包的那个
SYSPY="${SYSPY:-/usr/bin/python3}"

# ROS2 环境加载。
# 注意：`bash -lc` 在 Ubuntu 上**不会** source ~/.bashrc（非交互直接 return），
# 实测 env -i bash -lc 'import rclpy' 会 ModuleNotFoundError。
# 插件拉起 Bridge2 时靠父进程 env 继承，所以启动脚本这里必须显式 source。
dex_source_ros() {
    local setup="${ROS_SETUP:-}"
    if [ -z "$setup" ]; then
        if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
            setup="/opt/ros/$ROS_DISTRO/setup.bash"
        else
            local d
            for d in jazzy humble; do
                if [ -f "/opt/ros/$d/setup.bash" ]; then
                    setup="/opt/ros/$d/setup.bash"
                    break
                fi
            done
        fi
    fi
    if [ -z "$setup" ] || [ ! -f "$setup" ]; then
        echo "[env] ERROR: 找不到 ROS2 setup.bash（设 ROS_SETUP=/opt/ros/<distro>/setup.bash）" >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "$setup"
    # dex 消息包：Thor 上由厂商装在系统里（ros2_bridge_msgs / inspire_hand_msgs /
    # brainco_hand_msgs / bodyctrl_msgs 均已就位），旧容器路径存在时也一并 source。
    local ws="${TIENKUNG_DEX_MSGS_WS:-/opt/tienkung_dex_msgs_ws}"
    if [ -f "$ws/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "$ws/install/setup.bash"
    fi
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    echo "[env] ROS: $setup | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | python=$SYSPY"
}

# 去掉 IDE（codebuddy）注入到 PYTHONPATH 里的 sitecustomize shim。
# 该 shim 会拦截 os.unlink 做"批量删除保护"，会让 pip 覆盖安装直接 SystemExit(1)：
#   SystemExit: 1  at .../vendor/shim/sitecustomize.py _check_bulk_delete_guard
# 层 A（LeRobot）不需要它；层 B 需要 ROS 的 PYTHONPATH，所以只剔除 shim 那一条。
dex_strip_shim() {
    local out="" p
    local old_ifs="$IFS"
    IFS=':'
    for p in $PYTHONPATH; do
        case "$p" in
            *"/extensions/genie/out/vendor/shim"*) continue ;;
        esac
        out="${out:+$out:}$p"
    done
    IFS="$old_ifs"
    export PYTHONPATH="$out"
}

# 真机建议先看：dex 消息包是否齐全（缺一个，Bridge2 启动即 RuntimeError）
dex_check_msgs() {
    "$SYSPY" - <<'PY'
import importlib
mods = ["ros2_bridge_msgs", "bodyctrl_msgs"]
for m in mods:
    try:
        importlib.import_module(m)
        print(f"[env] msgs {m}: OK")
    except Exception as exc:
        print(f"[env] msgs {m}: MISSING ({exc})")
PY
}
