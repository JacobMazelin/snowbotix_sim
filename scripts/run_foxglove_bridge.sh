#!/bin/bash
# Launch foxglove_bridge on port 8765

source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy

ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
