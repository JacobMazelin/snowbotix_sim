#!/bin/bash
# Bridge /clock from Gazebo to ROS 2

source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy

ros2 run ros_gz_bridge parameter_bridge /clock
