#!/bin/bash
# Walker S2 开发者模式切换
# 用法:
#   bash dev_mode.sh on       # 进入开发者模式
#   bash dev_mode.sh off      # 退出开发者模式
set -e

SSH_HOST="192.168.11.2"
SSH_PORT="2222"
SSH_USER="ubt"

MODE="${1:-on}"
case "$MODE" in
    on)  DATA="true"  ;;
    off) DATA="false" ;;
    *)
        echo "用法: $0 [on|off]   默认 on（进入开发者模式）"
        exit 1
        ;;
esac

echo ">>> 开发者模式 -> $DATA"

# heredoc → 登录 shell（.profile 补全 DDS 环境）+ 显式 source ROS2
# 用尾巴取最后一行的 grep 输出作为状态值，过滤掉 MOTD 和 service call 输出
STATE=$(ssh -p "$SSH_PORT" "$SSH_USER@$SSH_HOST" 2>/dev/null << EOF | tail -1
    source /opt/ros/humble/setup.bash
    ros2 service call /sys/task/developer_mode std_srvs/srv/SetBool "{data: $DATA}" 2>&1
    ros2 topic echo /sys/state/walker_mode --once --qos-reliability best_effort 2>/dev/null | grep -oP 'data:\s*\K\w+'
EOF
)

if [ "$STATE" = "$DATA" ]; then
    echo "✓ 成功（walker_mode: $STATE）"
else
    echo "✗ 失败（期望 $DATA，实际 $STATE）"
    exit 1
fi
