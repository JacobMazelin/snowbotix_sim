# Snowbotix Simulation

Headless ROS 2 Jazzy + Gazebo Sim simulation on AWS with Foxglove Studio integration.

## 🎉 Quick Start

1. **Start the AWS instance** (if stopped):
   ```bash
   ./scripts/check_instance.sh
   # If stopped, start from AWS Console (link in output)
   ```

2. **Wait 1-2 minutes** for boot

3. **Setup SSH tunnel** (keep terminal open):
   ```bash
   ./scripts/setup_tunnel.sh
   ```

4. **Open Foxglove** → Add Connection → WebSocket → `ws://localhost:8765`

5. **Edit code in VS Code** → Push to GitHub → See updates in 30-60 seconds!

## 📚 Documentation

- **[WORKFLOW.md](WORKFLOW.md)** - Complete development workflow guide
- **[INSTANCE_MANAGEMENT.md](INSTANCE_MANAGEMENT.md)** - AWS instance management & cost control

## 🔄 Development Workflow

```
Mac (VS Code) → GitHub → GitHub Actions → AWS EC2 → Foxglove
```

1. **Edit** files in `src/`, `launch/`, `scripts/`
2. **Save** (auto-saves after 1 second)
3. **Commit** (Git panel: `Cmd+Shift+G` → `Cmd+Enter`)
4. **Push** (`Cmd+Shift+P` → "Git: Push")
5. **Wait 30-60 seconds** for GitHub Actions to deploy
6. **See updates** in Foxglove automatically!

## 💰 Cost-Saving Features

✅ **4-Hour Auto-Shutdown**: Instance automatically stops after 4 hours
✅ **Auto-Restart on Boot**: All services start automatically when instance starts
✅ **Safe to Stop**: All code in GitHub, nothing lost on shutdown

**Typical cost**: $0.10-0.30 per week (with auto-shutdown)

## 📁 Project Structure

```
snowbotix_sim/
├── .github/workflows/       # Auto-deployment to AWS
├── .vscode/                # VS Code settings (auto-save, etc.)
├── src/                    # ROS 2 packages (your code here!)
├── launch/                 # Launch files
├── scripts/                # Utility scripts
├── aws/                    # AWS deployment & auto-shutdown scripts
├── config/                 # Configuration files
├── README.md              # This file
├── WORKFLOW.md            # Detailed workflow guide
└── INSTANCE_MANAGEMENT.md # Instance management & cost control
```

## 🛠️ Common Commands

```bash
# From Mac
./scripts/check_instance.sh      # Check instance status
./scripts/setup_tunnel.sh        # Start SSH tunnel
./scripts/quick_push.sh          # Quick commit & push

# From AWS (SSH in)
./scripts/healthcheck.sh         # Check simulation status
./scripts/move_vehicle.sh vehicle_blue 1.0 0.5  # Move robot
systemctl --user stop auto-shutdown.service  # Cancel 4h shutdown
sudo shutdown -h now             # Stop instance immediately

# View logs
tail -f /tmp/auto_shutdown.log    # Shutdown timer
tail -f /tmp/simulation.log       # Simulation output
```

## 🎯 Key Features

- ✅ **Fully Automated**: Git push → AWS deploy → Foxglove update (30-60 sec)
- ✅ **Headless**: No GUI on AWS, everything in Foxglove
- ✅ **Cost-Controlled**: 4-hour auto-shutdown prevents runaway costs
- ✅ **Auto-Recovery**: Services auto-start when instance boots
- ✅ **GitHub-Native**: All code safe in GitHub, syncs automatically

## 🚀 Links

- **GitHub Repo**: https://github.com/JacobMazelin/snowbotix_sim
- **GitHub Actions**: https://github.com/JacobMazelin/snowbotix_sim/actions
- **AWS Console**: https://console.aws.amazon.com/ec2/

## 📞 Need Help?

See the detailed guides:
- [WORKFLOW.md](WORKFLOW.md) - Development workflow
- [INSTANCE_MANAGEMENT.md](INSTANCE_MANAGEMENT.md) - Cost control & instance management

Happy coding! 🤖
