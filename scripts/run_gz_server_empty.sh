#!/bin/bash
# Run Gazebo server only with empty world (headless)

source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/ros_gz_sim_demos/models

ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r -s empty.sdf"
