#!/bin/bash
set -e

# Source ROS 2 setup
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"