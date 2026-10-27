#!/usr/bin/env bash
# Identical field entry on both Orins; role is resolved from hostname.
set -euo pipefail
if [[ -f "$HOME/.config/sany/workspace.bash" ]]; then
  source "$HOME/.config/sany/workspace.bash"
fi
SANY_WS="${SANY_WS:-$HOME/Sany}"
LIGHTNING_LM_INSTALL_SETUP="${LIGHTNING_LM_INSTALL_SETUP:-$SANY_WS/lightning_lm_ws/install/setup.bash}"
set +u
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 run livox_ros_driver2_core run_lidar_bundle.py --role auto \
  --output-root "${SANY_WS}/runs" \
  --vehicle-config "${SANY_VEHICLE_CONFIG:-/opt/conf/vehicle_config.yaml}"
