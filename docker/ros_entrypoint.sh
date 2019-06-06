#!/bin/bash
set -e

# setup ros2 environment
source "$ROSBAGV2_WS/install/setup.bash"
source "$CONFBOT_WS/install/setup.bash"
exec "$@"
