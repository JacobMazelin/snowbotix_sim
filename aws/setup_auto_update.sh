#!/bin/bash
# Setup systemd service for auto-update

SERVICE_NAME="snowbotix-auto-update"
SERVICE_FILE="$HOME/.config/systemd/user/$SERVICE_NAME.service"

cat > $SERVICE_FILE << EOF
[Unit]
Description=Snowbotix Sim Auto-Update Service
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/snowbotix_sim && ./aws/auto_update.sh'
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable $SERVICE_NAME.service

echo "Auto-update service installed!"
echo "Start with: systemctl --user start $SERVICE_NAME.service"
echo "Check status: systemctl --user status $SERVICE_NAME.service"
