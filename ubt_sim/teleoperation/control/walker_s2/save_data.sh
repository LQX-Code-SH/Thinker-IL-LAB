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

for i in {1..400}
do
    first=true
    for part in "${PARTS[@]}"
    do
        if $first; then
            extra_flags="--reset-scene --robot-init"
            first=false
        else
            extra_flags=""
        fi

        echo "=================================="
        echo "Iteration $i / 400 — Part: $part"
        echo "=================================="
        python3 "$SCRIPT_DIR/pick_part.py" \
            --part "$part" \
            --save \
            --no-randomize-parts \
            --no-unlock-waist \
            $extra_flags
        sleep 2
    done
done
