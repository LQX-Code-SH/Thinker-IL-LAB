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

# PARTS=("part_a_ori" "part_a_red" "part_b_blue" "part_b_ori")
PARTS=("part_a_red")

# 不用 set -e：单集抓取失败不应中断整批。改用退出码跟踪 + 连续失败熔断。
MAX_CONSECUTIVE_FAIL="${MAX_CONSECUTIVE_FAIL:-10}"
success=0
fail=0
consecutive_fail=0

for i in {1..400}
do
    first=true
    iter_ok=true
    for part in "${PARTS[@]}"
    do
        if $first; then
            extra_flags="--reset-scene --robot-init"
            first=false
        else
            extra_flags=""
        fi

        echo "=================================="
        echo "Iteration $i / 400 - Part: $part"
        echo "=================================="
        python3 "$SCRIPT_DIR/pick_part.py" \
            --part "$part" \
            --save \
            --capture-hz 15 \
            --zmq-image-port 5657 \
            --no-randomize-parts \
            --no-unlock-waist \
            $extra_flags
        rc=$?
        if [ $rc -ne 0 ]; then
            echo "[WARN] Iteration $i part $part 退出码 $rc（抓取/保存失败或崩溃）"
            iter_ok=false
        fi
        sleep 2
    done

    if $iter_ok; then
        success=$((success + 1))
        consecutive_fail=0
    else
        fail=$((fail + 1))
        consecutive_fail=$((consecutive_fail + 1))
        if [ "$MAX_CONSECUTIVE_FAIL" -gt 0 ] && [ $consecutive_fail -ge $MAX_CONSECUTIVE_FAIL ]; then
            echo "[ERROR] 连续 $MAX_CONSECUTIVE_FAIL 次迭代失败，中止整批（疑似 bridge/sim 未就绪）。"
            echo "        设 MAX_CONSECUTIVE_FAIL=0 可关闭熔断。"
            break
        fi
    fi
done

echo "=================================="
echo "采集结束：成功 $success / 失败 $fail（共 $((success + fail)) 次迭代）"
echo "=================================="
# 退出码：全部失败 -> 1（提示调用方/CI）；至少有一次成功 -> 0
if [ $success -eq 0 ] && [ $fail -gt 0 ]; then
    exit 1
fi
