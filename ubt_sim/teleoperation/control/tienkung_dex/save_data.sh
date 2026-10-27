#!/bin/bash
# TienKung Dex 数据采集启动脚本
#
# 两种模式：
#   手动录制  —— 只录当前机器人在做什么，Enter 开始/停止，s 保存，q 退出
#   批量采集  —— 每集「复位场景 → 回预备位 → 抓取 → 放到盘上」，自动落盘，跑完即走
#
# 用法：
#   bash teleoperation/control/tienkung_dex/save_data.sh                       # 手动 Enter 控制
#   bash teleoperation/control/tienkung_dex/save_data.sh --auto-start --duration 30 --episodes 5
#   bash teleoperation/control/tienkung_dex/save_data.sh --batch 20            # 批量采 20 集
#   bash teleoperation/control/tienkung_dex/save_data.sh --batch 20 --variant brainco
#   bash teleoperation/control/tienkung_dex/save_data.sh --batch 20 --randomize-apple
#                              # 每集把苹果随机挪 x=0~10 / y=0~-10 cm（世界系）
#
# 批量模式每集产出一个时间戳目录：dataset/tienkung_dex/<epoch秒>/trajectory.hdf5
#
# 不用 set -e（与 tienkung_pro 一致）：单集失败不能中断整批，改用退出码跟踪 +
# 连续失败熔断。pick_apple.py --save 只在这一集通过任务判定（苹果↔盘心距离
# < --task-dist-tol，默认 0.15 m）并落盘时返回 0，抓空、放偏、零帧都算失败。
set -uo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[ERROR] 未找到 ROS2 Humble 环境"
    exit 1
fi

# 真机同款消息包（仿真机上由 run.sh init 编译；真机上用厂商自带的）
if [ -f /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash ]; then
    set +u
    source /opt/ubt_sim/tienkung_dex_msgs/install/setup.bash
    set -u
fi

# 仿真 146 / 真机 0：采集代码本身完全一样
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-146}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

OUTPUT_DIR="${UBT_SIM_DEX_OUTPUT_DIR:-$PROJECT_DIR/dataset/tienkung_dex}"
mkdir -p "$OUTPUT_DIR"

# ------------------------------------------------------------------ 批量采集
# 一集 = 复位场景 + pick_apple.py --save（同一进程里录，边界精确到"张手放到盘上"）
if [ "${1:-}" = "--batch" ]; then
    COUNT="${2:?用法: save_data.sh --batch <集数> [pick_apple 参数...]}"
    shift 2
    # 复位后等场景（苹果/盘子/机器人）稳定下来再回预备位
    RESET_WAIT="${RESET_WAIT:-3}"
    # 单集失败不中断整批，连续失败这么多次才熔断（疑似桥接/仿真没起来）；
    # 0 表示不熔断。默认值与 tienkung_pro 的 save_data.sh 相同。
    MAX_CONSECUTIVE_FAIL="${MAX_CONSECUTIVE_FAIL:-10}"

    echo "[INFO] 批量采集 $COUNT 集，输出目录: $OUTPUT_DIR"
    success=0
    fail=0
    consecutive_fail=0

    for ((i = 1; i <= COUNT; i++)); do
        echo "=================================="
        echo "第 $i / $COUNT 集"
        echo "=================================="
        # 仿真里才有 /sim/cmd_reset；真机没人订阅，发了无害
        /usr/bin/python3 "$SCRIPT_DIR/reset.py" >/dev/null 2>&1 || true
        sleep "$RESET_WAIT"

        /usr/bin/python3 "$SCRIPT_DIR/pick_apple.py" \
            --save --save-dir "$OUTPUT_DIR" "$@"
        rc=$?

        if [ $rc -eq 0 ]; then
            success=$((success + 1))
            consecutive_fail=0
        else
            fail=$((fail + 1))
            consecutive_fail=$((consecutive_fail + 1))
            echo "[WARN] 第 $i 集退出码 $rc（抓取或保存失败）"
            if [ "$MAX_CONSECUTIVE_FAIL" -gt 0 ] && [ $consecutive_fail -ge "$MAX_CONSECUTIVE_FAIL" ]; then
                echo "[ERROR] 连续 $MAX_CONSECUTIVE_FAIL 集失败，中止整批（疑似桥接/仿真未就绪）。"
                echo "        设 MAX_CONSECUTIVE_FAIL=0 可关闭熔断。"
                break
            fi
        fi
        sleep 2
    done

    echo "=================================="
    echo "采集结束：成功 $success / 失败 $fail（共 $((success + fail)) 集），落盘在 $OUTPUT_DIR/<epoch秒>/trajectory.hdf5"
    echo "=================================="
    # 全部失败 -> 1（提示调用方/CI）；至少成功一次 -> 0
    if [ $success -eq 0 ] && [ $fail -gt 0 ]; then
        exit 1
    fi
    exit 0
fi

# ------------------------------------------------------------------ 手动录制
echo "[INFO] 输出目录: $OUTPUT_DIR"
exec /usr/bin/python3 "$SCRIPT_DIR/save_data.py" --output-dir "$OUTPUT_DIR" "$@"
