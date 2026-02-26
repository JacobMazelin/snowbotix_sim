#!/bin/bash
# Auto-shutdown script - shuts down instance after 4 hours of runtime
# This prevents accidentally leaving the instance running and racking up costs

SHUTDOWN_DELAY=$((4 * 60 * 60))  # 4 hours in seconds
WARNING_TIMES=(10800 7200 3600 1800 600 300 60)  # Warnings at: 3h, 2h, 1h, 30m, 10m, 5m, 1m before shutdown
LOG_FILE="/tmp/auto_shutdown.log"

log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S'): $1" | tee -a $LOG_FILE
}

log_message "=========================================="
log_message "Auto-shutdown service started"
log_message "Instance will shutdown in 4 hours"
log_message "=========================================="

# Function to broadcast warning to all logged-in users
broadcast_warning() {
    local minutes=$1
    local message="WARNING: System will shutdown in $minutes minutes. Save your work!"
    echo "$message" | wall
    log_message "Broadcasted: $message"
}

# Calculate shutdown time
START_TIME=$(date +%s)
SHUTDOWN_TIME=$((START_TIME + SHUTDOWN_DELAY))

log_message "Start time: $(date -d @$START_TIME '+%H:%M:%S')"
log_message "Shutdown scheduled for: $(date -d @$SHUTDOWN_TIME '+%H:%M:%S')"

# Main loop - check every minute
while true; do
    CURRENT_TIME=$(date +%s)
    TIME_REMAINING=$((SHUTDOWN_TIME - CURRENT_TIME))
    
    # Check if it's time to shutdown
    if [ $TIME_REMAINING -le 0 ]; then
        log_message "Shutdown time reached! Initiating shutdown..."
        
        # Final warning
        echo "SYSTEM SHUTTING DOWN NOW!" | wall
        sleep 5
        
        # Graceful shutdown
        log_message "Executing shutdown command"
        sudo shutdown -h now "Auto-shutdown: 4 hour limit reached"
        exit 0
    fi
    
    # Check for warning times
    for warning_time in "${WARNING_TIMES[@]}"; do
        if [ $TIME_REMAINING -le $warning_time ] && [ $TIME_REMAINING -gt $((warning_time - 60)) ]; then
            minutes=$((warning_time / 60))
            broadcast_warning $minutes
            log_message "Shutdown warning: $minutes minutes remaining"
            break
        fi
    done
    
    # Log status every 10 minutes
    if [ $((TIME_REMAINING % 600)) -eq 0 ]; then
        log_message "Time remaining: $((TIME_REMAINING / 60)) minutes"
    fi
    
    sleep 60
done
