#!/bin/bash
# Walker S2 推理服务客户端薄包装。
# 用法: bash inference_client.sh <launch|load|start|stop|home|status|shutdown|watch> [args...]
# 示例:
#   bash inference_client.sh launch --policy-path /ubt_IL/model/.../pretrained_model --robot-model walker_s2_10d --inference-hz 2 --fps 15
#   bash inference_client.sh status
#   bash inference_client.sh start
#   bash inference_client.sh stop
#   bash inference_client.sh home
#   bash inference_client.sh load --policy-path /ubt_IL/model/<other>/.../pretrained_model --robot-model walker_s2_10d
#   bash inference_client.sh shutdown
#   bash inference_client.sh watch
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 优先用 lerobot venv 的 python（容器内自带 pyzmq）；否则回退系统 python3。
if [ -x /lerobot/.venv/bin/python ]; then
    PY=/lerobot/.venv/bin/python
else
    PY=python3
fi

exec "$PY" "$SCRIPT_DIR/inference_client.py" "$@"
