#!/bin/bash
# Quick command to send cmd_vel to vehicle

source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy

VEHICLE="${1:-vehicle_blue}"
LINEAR_X="${2:-1.0}"
ANGULAR_Z="${3:-0.5}"

echo "Sending cmd_vel to $VEHICLE: linear.x=$LINEAR_X, angular.z=$ANGULAR_Z"

ros2 topic pub --once /model/$VEHICLE/cmd_vel geometry_msgs/Twist "{
  linear: {x: $LINEAR_X, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}
}" --qos-reliability reliable
