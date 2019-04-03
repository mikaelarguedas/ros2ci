#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_UNDERLAY_WS/install/setup.bash"
exec "$@"
