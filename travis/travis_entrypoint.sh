#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/crystal/setup.bash"
cd $ROS2_WS
colcon build --symlink-install
colcon test
colcon test-result
exec "$@"

