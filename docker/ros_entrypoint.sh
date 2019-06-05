#!/bin/bash
set -e

# setup ros2 environment
source "$ROS2_UNDERLAY_WS/install/setup.bash"
exec "$@"
