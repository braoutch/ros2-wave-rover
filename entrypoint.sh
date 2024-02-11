#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/ros2-wave-rover/install/setup.bash" --

# Welcome information
echo "Wave rover from Wave share docker image"
echo "---------------------"
echo 'ROS distro: ' Humble
echo "---"  
exec "$@"