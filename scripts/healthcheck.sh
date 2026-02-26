#!/bin/bash
# Health check script

source /opt/ros/jazzy/setup.bash 2>/dev/null || echo "Warning: ROS not sourced"
export ROS_DISTRO=jazzy

echo "=========================================="
echo "  Snowbotix Sim Health Check"
echo "=========================================="
echo ""

echo "=== System Info ==="
echo "Date: $(date)"
echo "ROS_DISTRO: $ROS_DISTRO"
which ros2 && ros2 --version 2>/dev/null || echo "ROS2 not found in PATH"
echo ""

echo "=== Network Ports ==="
if command -v ss &> /dev/null; then
    ss -lntp 2>/dev/null | grep -E "(8765| gazebo)" || echo "No gazebo/foxglove ports found"
elif command -v netstat &> /dev/null; then
    netstat -lntp 2>/dev/null | grep -E "(8765| gazebo)" || echo "No gazebo/foxglove ports found"
fi
echo ""

echo "=== Process Status ==="
echo "Gazebo processes:"
pgrep -a "gz sim" 2>/dev/null | head -5 || echo "  No gz sim running"

echo "Bridge processes:"
pgrep -a "parameter_bridge" 2>/dev/null | head -10 || echo "  No bridges running"

echo "Foxglove bridge:"
pgrep -a "foxglove_bridge" 2>/dev/null | head -3 || echo "  No foxglove_bridge running"
echo ""

echo "=== ROS 2 Topics (first 30) ==="
if command -v ros2 &> /dev/null; then
    ros2 topic list 2>/dev/null | head -30 || echo "No topics available"
else
    echo "ROS2 not available"
fi
echo ""

echo "=== Key Topics Check ==="
if command -v ros2 &> /dev/null; then
    for topic in "/clock" "/model/vehicle_blue/odometry" "/model/vehicle_green/odometry"; do
        if ros2 topic list 2>/dev/null | grep -q "$topic"; then
            echo "✓ $topic exists"
        else
            echo "✗ $topic not found"
        fi
    done
fi
echo ""

echo "=== Deployment Status ==="
if [ -d ~/snowbotix_sim ]; then
    echo "✓ snowbotix_sim directory exists"
    if [ -d ~/snowbotix_sim/.git ]; then
        echo "✓ Git repository initialized"
        cd ~/snowbotix_sim && git log --oneline -1 2>/dev/null || echo "  No commits yet"
    else
        echo "✗ Git repository not initialized"
    fi
else
    echo "✗ snowbotix_sim directory not found"
fi
echo ""

echo "=========================================="
echo "  Health Check Complete"
echo "=========================================="
