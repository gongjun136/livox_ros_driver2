#!/usr/bin/env bash

set -euo pipefail

ROS_DISTRO_NAME="${1:-humble}"
case "$ROS_DISTRO_NAME" in
  humble|jazzy)
    ;;
  *)
    echo "Unsupported ROS distribution: $ROS_DISTRO_NAME (expected humble or jazzy)" >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
cd "$WORKSPACE_ROOT"

export MAKEFLAGS="-j4"
export COLCON_EXTENSION_BLOCKLIST="colcon_core.event_handler.desktop_notification${COLCON_EXTENSION_BLOCKLIST:+:$COLCON_EXTENSION_BLOCKLIST}"
colcon build \
  --executor sequential \
  --packages-up-to livox_ros_driver2_core \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release
