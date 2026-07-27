#!/bin/bash
# Run the mentor C1 body/sensors merged with the proven dexterous hands.
# Default is a safe load-only preview; pass --control for physics + ROS control.

set -euo pipefail

ASSET_DIR=/ubt_sim/assets/robots/walker_c1/Collected_walker_c1_v1_sensorKpkd/Collected_walker_astron_v1_sensorKpkd
MERGED_USD="$ASSET_DIR/walker_astron_v1_sensorKpkd_hands.usd"
CONTROL_MODE=0
C1_STEP_HZ="${UBT_SIM_C1_STEP_HZ:-100}"

if [ "${1:-}" = "--control" ]; then
    CONTROL_MODE=1
    shift
fi

if [ ! -f "$MERGED_USD" ]; then
    echo "[ERROR] Merged C1 USD not found: $MERGED_USD" >&2
    exit 1
fi

export UBT_SIM_WALKER_C1_USD_PATH="$MERGED_USD"
export UBT_SIM_C1_SENSOR_VIEWPORTS=1
export UBT_SIM_TASK=UBTSim-WalkerC1-Parlor-v0
export ROS_DOMAIN_ID=146

if [ "$CONTROL_MODE" -eq 1 ]; then
    unset UBT_SIM_LOAD_ONLY
    unset UBT_SIM_NO_BRIDGE
    echo "[INFO] Starting merged C1 with physics, ROS control, and five sensor viewports."
else
    export UBT_SIM_LOAD_ONLY=1
    export UBT_SIM_NO_BRIDGE=1
    echo "[INFO] Starting merged C1 in load-only sensor preview mode."
fi

cd /ubt_sim
exec bash scripts/start_sim.sh --device cpu --step_hz "$C1_STEP_HZ" "$@"
