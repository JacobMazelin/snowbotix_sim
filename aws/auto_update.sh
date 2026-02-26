#!/bin/bash
# Auto-update script - runs continuously on AWS to pull latest changes

LOG_FILE="/tmp/auto_update.log"
REPO_DIR="$HOME/snowbotix_sim"

echo "$(date): Starting auto-update service..." >> $LOG_FILE

cd $REPO_DIR || exit 1

while true; do
    # Check if git repo exists
    if [ -d .git ]; then
        # Fetch latest changes
        git fetch origin main >> $LOG_FILE 2>&1
        
        # Check if there are new commits
        LOCAL=$(git rev-parse HEAD)
        REMOTE=$(git rev-parse origin/main)
        
        if [ "$LOCAL" != "$REMOTE" ]; then
            echo "$(date): New changes detected, pulling..." >> $LOG_FILE
            
            # Pull changes
            git pull origin main >> $LOG_FILE 2>&1
            
            # Restart simulation
            echo "$(date): Restarting simulation with new code..." >> $LOG_FILE
            
            # Stop existing processes
            pkill -f "gz sim" 2>/dev/null || true
            pkill -f "parameter_bridge" 2>/dev/null || true
            sleep 2
            
            # Source ROS
            source /opt/ros/jazzy/setup.bash
            
            # Restart clock bridge
            nohup bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /clock' > /tmp/clock_bridge.log 2>&1 &
            
            # Restart simulation
            nohup ./scripts/run_diff_drive_headless.sh > /tmp/simulation.log 2>&1 &
            
            echo "$(date): Simulation restarted successfully" >> $LOG_FILE
        fi
    fi
    
    # Check every 30 seconds
    sleep 30
done
