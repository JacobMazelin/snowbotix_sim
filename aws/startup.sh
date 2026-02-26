#!/bin/bash
# Startup script - runs when the AWS instance boots
# This automatically resumes all services after a restart

LOG_FILE="/tmp/startup.log"

echo "$(date): ==========================================" >> $LOG_FILE
echo "$(date): Snowbotix Sim Startup Script" >> $LOG_FILE
echo "$(date): ==========================================" >> $LOG_FILE

# Function to log and print
log() {
    echo "$(date): $1" >> $LOG_FILE
    echo "$1"
}

# Source ROS
source /opt/ros/jazzy/setup.bash
export ROS_DISTRO=jazzy

# Ensure we're in the snowbotix_sim directory
cd ~/snowbotix_sim || exit 1

log "Starting up snowbotix_sim services..."

# 1. Start auto-shutdown timer (4 hour limit)
log "→ Starting auto-shutdown service (4h limit)..."
systemctl --user start auto-shutdown.service 2>/dev/null || log "  Auto-shutdown already running or needs setup"

# 2. Start foxglove bridge
log "→ Starting Foxglove bridge on port 8765..."
pkill -f "foxglove_bridge" 2>/dev/null || true
sleep 1
nohup bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765' > /tmp/foxglove_bridge.log 2>&1 &
sleep 2

# 3. Sync latest code from GitHub
log "→ Syncing with GitHub..."
if [ -d .git ]; then
    git fetch origin main >> $LOG_FILE 2>&1
    git reset --hard origin/main >> $LOG_FILE 2>&1
    log "  Synced to latest commit: $(git log --oneline -1)"
else
    log "  ⚠️  Git not initialized, skipping sync"
fi

# 4. Build any custom packages
log "→ Building custom packages..."
if [ -d src ]; then
    cd ~/snowbotix_sim
    colcon build --symlink-install 2>&1 | tail -5 >> $LOG_FILE || log "  No packages to build"
fi

# 5. Start clock bridge
log "→ Starting /clock bridge..."
nohup bash -c 'source /opt/ros/jazzy/setup.bash && source ~/snowbotix_sim/install/setup.bash 2>/dev/null; ros2 run ros_gz_bridge parameter_bridge /clock' > /tmp/clock_bridge.log 2>&1 &
sleep 1

# 6. Start auto-update service
log "→ Starting auto-update service..."
systemctl --user start snowbotix-auto-update.service 2>/dev/null || log "  Auto-update already running or needs setup"

# 7. Start simulation
log "→ Starting headless simulation..."
nohup bash -c 'source /opt/ros/jazzy/setup.bash && source ~/snowbotix_sim/install/setup.bash 2>/dev/null; cd ~/snowbotix_sim && ./scripts/run_diff_drive_headless.sh' > /tmp/simulation.log 2>&1 &

log ""
log "✅ Startup complete! Services running:"
log "  - Auto-shutdown timer (4h limit)"
log "  - Foxglove bridge (port 8765)"
log "  - Clock bridge (/clock)"
log "  - Auto-update service (polls GitHub)"
log "  - Gazebo simulation (diff_drive)"
log ""
log "⏱️  Time until auto-shutdown: 4 hours"
log "📊 Check status: ~/snowbotix_sim/scripts/healthcheck.sh"
log "📊 View logs: tail -f /tmp/startup.log"
