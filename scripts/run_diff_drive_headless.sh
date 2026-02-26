#!/bin/bash
# Headless diff drive simulation with bridges

source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/ros_gz_sim_demos/models

# Kill any existing gz processes
pkill -f "gz sim" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
sleep 2

# Launch Gazebo server-only with diff_drive world
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r -s diff_drive.sdf" gz_sim_server:=true gz_sim_gui:=false &

# Wait for Gazebo to start
sleep 3

# Bridge /clock
ros2 run ros_gz_bridge parameter_bridge /clock &

# Bridge vehicle_blue cmd_vel and odometry
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &

# Bridge vehicle_green cmd_vel and odometry
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_green/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &

echo "Diff drive headless simulation started!"
echo "Topics available:"
echo "  /clock"
echo "  /model/vehicle_blue/cmd_vel"
echo "  /model/vehicle_blue/odometry"
echo "  /model/vehicle_green/cmd_vel"
echo "  /model/vehicle_green/odometry"
