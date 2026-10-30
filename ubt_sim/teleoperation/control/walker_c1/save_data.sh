#!/bin/bash
# Walker C1 数据采集启动脚本
#
# 用法（默认直接执行，无交互按键）：
#   bash teleoperation/control/walker_c1/save_data.sh                  # 采 1 条
#   bash teleoperation/control/walker_c1/save_data.sh --batch 20       # 采 20 条
#   bash teleoperation/control/walker_c1/save_data.sh --manual         # 交互录制（Enter/s/q）
#
# 每集流程：复位场景 → 回任务预备位 → 在线 IK 抓放一次 → 成功才落盘
#
# 批量模式每集产出一个时间戳目录：dataset/walker_c1/<epoch秒>/trajectory.hdf5
#
# 不用 set -e（与其它机型一致）：单集失败不能中断整批，改用退出码跟踪 +
# 连续失败熔断。pick_place_controller.py --save 只在这一集通过任务判定
# （苹果↔盘心距离 < --task-dist-tol）并落盘时返回 0。
set -uo pipefail

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[ERROR] 未找到 ROS2 Humble 环境"
    exit 1
fi

# Walker SDK 消息包（Walker S2 与 C1 共用同一个工作空间）
if [ -f /opt/ubt_sim/walker_c1_msgs/install/setup.bash ]; then
    set +u
    source /opt/ubt_sim/walker_c1_msgs/install/setup.bash
    set -u
else
    echo "[WARN] 未找到 /opt/ubt_sim/walker_c1_msgs，先执行: cd docker && bash run.sh init"
fi

# 默认与真机同域（0）；需要隔离时显式传 ROS_DOMAIN_ID=146
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# RMW 与镜像保持一致：本仓库镜像只装了 cyclonedds
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
    if [ -f /opt/ros/humble/lib/librmw_fastrtps_cpp.so ]; then
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    else
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

OUTPUT_DIR="${UBT_SIM_WALKER_C1_OUTPUT_DIR:-$PROJECT_DIR/dataset/walker_c1}"
mkdir -p "$OUTPUT_DIR"

# ------------------------------------------------------------------ 采集（默认 1 条）
# 无参数即执行：复位场景 → 回预备位 → 抓放一次 → 成功才落盘
# --batch N 采 N 条；--manual 才是交互录制（Enter/s/q）
COUNT=1
MODE="auto"
if [ "${1:-}" = "--batch" ]; then
    COUNT="${2:?用法: save_data.sh --batch <集数> [pick_place 参数...]}"
    shift 2
elif [ "${1:-}" = "--manual" ]; then
    MODE="manual"
    shift
fi

if [ "$MODE" = "auto" ]; then
    RESET_WAIT="${RESET_WAIT:-2}"
    MAX_CONSECUTIVE_FAIL="${MAX_CONSECUTIVE_FAIL:-10}"

    echo "[INFO] 采集 $COUNT 集，输出目录: $OUTPUT_DIR"
    success=0
    fail=0
    consecutive_fail=0

    for ((i = 1; i <= COUNT; i++)); do
        echo "=================================="
        echo "第 $i / $COUNT 集"
        echo "=================================="
        # 场景复位走 ROS /sim/cmd_reset（桥接转发），再回任务预备位
        /usr/bin/python3 "$SCRIPT_DIR/reset.py" --mode sim-scene >/dev/null 2>&1 || true
        /usr/bin/python3 "$SCRIPT_DIR/reset.py" --mode task >/dev/null 2>&1 || true
        sleep "$RESET_WAIT"

        /usr/bin/python3 "$SCRIPT_DIR/pick_place_controller.py" \
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
    if [ $success -eq 0 ] && [ $fail -gt 0 ]; then
        exit 1
    fi
    exit 0
fi

# ------------------------------------------------------------------ 交互录制（--manual）
echo "[INFO] 手动录制（Enter 开始/停止，s 保存，q 退出）；输出目录: $OUTPUT_DIR"
# 仿真相机 data_types 只有 rgb，不开深度录制，否则等不到深度图会 0 帧
exec /usr/bin/python3 "$SCRIPT_DIR/save_data.py" --output-dir "$OUTPUT_DIR" --no-depth "$@"
