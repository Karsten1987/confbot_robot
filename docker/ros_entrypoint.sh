#!/bin/bash
set -e

# setup ros2 environment
source "$CONFBOT_WS/install/setup.bash"
exec "$@"
