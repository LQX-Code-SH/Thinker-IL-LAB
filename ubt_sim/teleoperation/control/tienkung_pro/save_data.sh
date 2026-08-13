#!/bin/bash

# 检测 ROS2 发行版并 source 对应环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: ROS2 environment not found!"
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# bodyctrl_msgs 已通过 deb 包安装在 /opt/ros/<distro>/share/bodyctrl_msgs
# 无需额外 source，ROS2 会自动发现

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 不用 set -e：单集失败不应中断整批。改用退出码跟踪 + 连续失败熔断。
# 注意：pick_place_save_data.py 在任务未完成时丢弃数据但（待 M10 修复前）退出码 0，
# 故此处的 fail 计数目前仅捕获崩溃；M10 修复后亦可捕获"零产出"。
MAX_CONSECUTIVE_FAIL="${MAX_CONSECUTIVE_FAIL:-10}"
success=0
fail=0
consecutive_fail=0

for i in {1..400}
do
    echo "=================================="
    echo "Starting iteration $i / 400"
    echo "=================================="
    python3 "$SCRIPT_DIR/pick_place_save_data.py"
    rc=$?
    if [ $rc -eq 0 ]; then
        success=$((success + 1))
        consecutive_fail=0
    else
        fail=$((fail + 1))
        consecutive_fail=$((consecutive_fail + 1))
        echo "[WARN] Iteration $i 退出码 $rc"
        if [ "$MAX_CONSECUTIVE_FAIL" -gt 0 ] && [ $consecutive_fail -ge $MAX_CONSECUTIVE_FAIL ]; then
            echo "[ERROR] 连续 $MAX_CONSECUTIVE_FAIL 次迭代失败，中止整批（疑似 bridge/sim 未就绪）。"
            echo "        设 MAX_CONSECUTIVE_FAIL=0 可关闭熔断。"
            break
        fi
    fi
    sleep 2
done

echo "=================================="
echo "采集结束：成功 $success / 失败 $fail（共 $((success + fail)) 次迭代）"
echo "=================================="
# 退出码：全部失败 -> 1；至少有一次成功 -> 0
if [ $success -eq 0 ] && [ $fail -gt 0 ]; then
    exit 1
fi
