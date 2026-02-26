#!/bin/bash
# Start the AWS instance and SSH into it
# This script is run from your Mac

INSTANCE_IP="18.223.190.83"
PEM_FILE="$HOME/Downloads/SnowbotixSim.pem"

echo "🚀 Starting Snowbotix Sim Session"
echo "=================================="
echo ""

# Check if instance is reachable
echo "→ Checking if instance is running..."
if ! ping -c 1 -W 2 $INSTANCE_IP &> /dev/null; then
    echo "  ⚠️  Instance appears to be offline"
    echo ""
    echo "Please start the instance from AWS Console:"
    echo "  1. Go to https://console.aws.amazon.com/ec2/"
    echo "  2. Select 'Instances' from the left menu"
    echo "  3. Find your instance (ip: $INSTANCE_IP)"
    echo "  4. Right-click → 'Start instance'"
    echo ""
    echo "⏱️  Wait 1-2 minutes for it to boot, then run this script again"
    exit 1
fi

echo "  ✅ Instance is online!"
echo ""

# SSH into instance and check services
echo "→ Connecting to instance..."
echo "  Running startup checks..."
echo ""

ssh -i "$PEM_FILE" ubuntu@$INSTANCE_IP << 'EOF'
    echo "=== Instance Status ==="
    uptime
    echo ""
    
    echo "=== Services Status ==="
    systemctl --user is-active auto-shutdown.service 2>/dev/null && echo "✓ Auto-shutdown: RUNNING" || echo "✗ Auto-shutdown: NOT RUNNING"
    systemctl --user is-active foxglove-bridge.service 2>/dev/null && echo "✓ Foxglove bridge: RUNNING" || echo "✗ Foxglove bridge: NOT RUNNING"
    
    if pgrep -f "gz sim" > /dev/null; then
        echo "✓ Gazebo simulation: RUNNING"
    else
        echo "✗ Gazebo simulation: NOT RUNNING"
    fi
    
    echo ""
    echo "=== Time Until Auto-Shutdown ==="
    if [ -f /tmp/auto_shutdown.log ]; then
        tail -5 /tmp/auto_shutdown.log 2>/dev/null | grep "Time remaining" | tail -1 || echo "  Check logs: tail -f /tmp/auto_shutdown.log"
    else
        echo "  Auto-shutdown not started yet"
    fi
    echo ""
    
    echo "=== Quick Commands ==="
    echo "  Check health:     cd ~/snowbotix_sim && ./scripts/healthcheck.sh"
    echo "  View logs:        tail -f /tmp/simulation.log"
    echo "  Move vehicle:     ./scripts/move_vehicle.sh vehicle_blue 1.0 0.5"
    echo "  Manual shutdown:  sudo shutdown -h now"
    echo "  Cancel shutdown:  systemctl --user stop auto-shutdown.service"
    echo ""
EOF

echo ""
echo "✅ Instance is ready!"
echo ""
echo "Next steps:"
echo "  1. Setup SSH tunnel: ./scripts/setup_tunnel.sh"
echo "  2. Open Foxglove → Connect to ws://localhost:8765"
echo "  3. Start coding and pushing changes!"
echo ""
echo "⚠️  Remember: Instance will auto-shutdown in 4 hours"
echo "   To keep running longer: ssh in and stop auto-shutdown service"
