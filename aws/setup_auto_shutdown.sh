#!/bin/bash
# Setup systemd service for auto-shutdown

SERVICE_NAME="auto-shutdown"
SERVICE_FILE="$HOME/.config/systemd/user/$SERVICE_NAME.service"

mkdir -p ~/.config/systemd/user

cat > $SERVICE_FILE << 'EOF'
[Unit]
Description=Auto-shutdown after 4 hours
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'cd ~/snowbotix_sim && ./aws/auto_shutdown.sh'
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable $SERVICE_NAME.service

echo "✅ Auto-shutdown service installed!"
echo ""
echo "🕐 The instance will automatically shutdown after 4 hours"
echo ""
echo "Commands:"
echo "  Start:   systemctl --user start $SERVICE_NAME.service"
echo "  Stop:    systemctl --user stop $SERVICE_NAME.service"
echo "  Status:  systemctl --user status $SERVICE_NAME.service"
echo "  Logs:    tail -f /tmp/auto_shutdown.log"
echo ""
echo "⚠️  To cancel shutdown:"
echo "  systemctl --user stop auto-shutdown.service"
echo "  sudo shutdown -c"
