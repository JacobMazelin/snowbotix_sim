#!/bin/bash
# Setup SSH tunnel and open Foxglove

EC2_IP="18.223.190.83"
PEM_FILE="$HOME/Downloads/SnowbotixSim.pem"

echo "Starting SSH tunnel to $EC2_IP on port 8765..."
echo "Keep this terminal open!"
echo ""
echo "In Foxglove, connect to: ws://localhost:8765"
echo ""

ssh -i "$PEM_FILE" -L 8765:localhost:8765 -N ubuntu@$EC2_IP
