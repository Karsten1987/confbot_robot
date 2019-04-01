#!/bin/bash
set -e

cd $ROS2_WS

# install dependencies
rosdep update && apt-get -qq update && rosdep install -y \
  --from-paths src \
  --ignore-src

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# build
colcon build \
    --symlink-install \
    --cmake-args -DSECURITY=ON --no-warn-unused-cli
# test
colcon test \
    --executor sequential \
    --event-handlers console_direct+
colcon test-result
