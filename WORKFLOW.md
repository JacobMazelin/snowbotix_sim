# Snowbotix Sim - Autonomous DevOps Workflow

🎉 **Your fully autonomous system is ready!**

## ✅ What's Set Up

1. **Local Repository**: `~/snowbotix_sim` with full project structure
2. **GitHub Repository**: https://github.com/JacobMazelin/snowbotix_sim
3. **GitHub Actions**: Auto-deployment workflow configured
4. **AWS EC2**: Simulation running with auto-update service
5. **VS Code Workspace**: Ready to edit and push
6. **Auto-Update**: AWS pulls changes every 30 seconds

## 🔄 Workflow

### Edit → Push → Deploy (Fully Automatic!)

```
┌─────────────────────────────────────────────────────────────┐
│  YOUR MAC                                                   │
│  ┌─────────────────┐                                         │
│  │ VS Code         │                                         │
│  │ Edit files      │                                         │
│  │ Cmd+S (Save)    │                                         │
│  └────────┬────────┘                                         │
│           │                                                   │
│  ┌────────▼────────┐                                         │
│  │ Git Panel       │  ← Auto-stages changes                  │
│  │ Cmd+Enter       │  ← Commit                               │
│  │ Cmd+Shift+P →   │                                         │
│  │ "Git: Push"     │                                         │
│  └────────┬────────┘                                         │
└───────────┼─────────────────────────────────────────────────┘
            │
            ▼ Push triggers GitHub Actions
┌─────────────────────────────────────────────────────────────┐
│  GITHUB                                                     │
│  ┌─────────────────┐                                         │
│  │ GitHub Actions  │                                         │
│  │ Build & Deploy  │  → SSH to AWS                          │
│  └─────────────────┘                                         │
└───────────┬─────────────────────────────────────────────────┘
            │
            ▼ Auto-deployment
┌─────────────────────────────────────────────────────────────┐
│  AWS EC2 (18.223.190.83)                                    │
│  ┌─────────────────┐                                         │
│  │ Simulation      │  ← Running headless                    │
│  │ Gazebo + ROS 2  │                                         │
│  └─────────────────┘                                         │
│                                                             │
│  ┌─────────────────┐                                         │
│  │ Auto-Update     │  ← Checks GitHub every 30 sec          │
│  │ Service         │     Restarts simulation with new code  │
│  └─────────────────┘                                         │
└───────────┬─────────────────────────────────────────────────┘
            │
            ▼ SSH Tunnel
┌─────────────────────────────────────────────────────────────┐
│  FOXGLOVE STUDIO                                            │
│  Connect to: ws://localhost:8765                           │
└─────────────────────────────────────────────────────────────┘
```

## 🚀 How to Use (One-Time Setup)

### 1. Open VS Code
Already done! VS Code should be open with your workspace.

### 2. Start SSH Tunnel (in terminal)
```bash
cd ~/snowbotix_sim
./scripts/setup_tunnel.sh
```

Keep this terminal open! It creates the tunnel to AWS.

### 3. Open Foxglove
1. Open Foxglove Studio
2. Add Connection → WebSocket
3. URL: `ws://localhost:8765`
4. Click "Open"

## 📝 Daily Workflow

### Making Changes

1. **Edit Code** in VS Code
   - Modify files in `src/`, `launch/`, `scripts/`, etc.
   - VS Code auto-saves after 1 second of idle

2. **Commit Changes**
   - Open Git panel: `Cmd+Shift+G`
   - Review changes
   - Type commit message
   - Press `Cmd+Enter` to commit

3. **Push to GitHub**
   - `Cmd+Shift+P` → type "Git: Push" → Enter
   - Or click the sync icon in the bottom left

4. **Wait 30-60 seconds**
   - GitHub Actions automatically deploys
   - AWS auto-update pulls and restarts
   - Foxglove shows new simulation state

### Quick Commands (in VS Code terminal)

```bash
# View simulation logs
ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83 'tail -f /tmp/simulation.log'

# Move a vehicle
./scripts/move_vehicle.sh vehicle_blue 1.0 0.5

# Health check
ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83 'cd ~/snowbotix_sim && ./scripts/healthcheck.sh'
```

## 📊 GitHub Actions Status

Check deployment status at:
https://github.com/JacobMazelin/snowbotix_sim/actions

## 🛠️ Troubleshooting

### GitHub Actions Fails
1. Check SSH key is in GitHub secrets: Settings → Secrets → `AWS_SSH_KEY`
2. Check the Actions logs for errors

### Simulation Not Updating
1. Check AWS auto-update logs:
   ```bash
   ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83 'tail -f /tmp/auto_update.log'
   ```

### Can't Connect to Foxglove
1. Make sure SSH tunnel is running
2. Check foxglove_bridge is listening:
   ```bash
   ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83 'ss -lntp | grep 8765'
   ```

### Port Already in Use
```bash
# On AWS, kill existing processes
ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83 'pkill -f "gz sim"; pkill -f "foxglove"'
```

## 🎯 Project Structure

```
snowbotix_sim/
├── .github/workflows/     # Auto-deployment config
│   └── deploy.yml          # GitHub Actions workflow
├── .vscode/                # VS Code settings
│   ├── settings.json       # Editor settings (auto-save, etc.)
│   ├── tasks.json          # VS Code tasks
│   └── extensions.json     # Recommended extensions
├── aws/                    # AWS deployment scripts
│   ├── auto_update.sh      # Auto-update daemon
│   └── setup_auto_update.sh
├── launch/                 # ROS 2 launch files
│   └── diff_drive_headless.launch.py
├── scripts/                # Utility scripts
│   ├── run_diff_drive_headless.sh
│   ├── healthcheck.sh
│   ├── move_vehicle.sh
│   └── setup_tunnel.sh
├── src/                    # Your ROS 2 nodes (empty, ready for you)
├── config/                 # Config files
├── README.md              # Project documentation
└── .gitignore             # Git ignore patterns
```

## 🔑 Key Features

- ✅ **Auto-save**: Changes saved after 1 second of idle
- ✅ **Auto-stage**: Git stages changes automatically
- ✅ **One-command push**: `Cmd+Shift+P` → "Git: Push"
- ✅ **Auto-deploy**: GitHub Actions deploys to AWS
- ✅ **Auto-update**: AWS pulls changes every 30 seconds
- ✅ **Headless**: No GUI on AWS, everything in Foxglove

## 📞 Quick Reference

| Action | Command |
|--------|---------|
| Save file | `Cmd+S` |
| Open Git panel | `Cmd+Shift+G` |
| Commit | `Cmd+Enter` (with message) |
| Push to GitHub | `Cmd+Shift+P` → "Git: Push" |
| Setup SSH tunnel | `./scripts/setup_tunnel.sh` |
| Check health | `./scripts/healthcheck.sh` (on AWS) |
| Move vehicle | `./scripts/move_vehicle.sh vehicle_blue 1.0 0.5` |

## 🎉 You're All Set!

Your autonomous DevOps system is running. Just:
1. Edit files in VS Code
2. Save (auto-saves anyway)
3. Push to GitHub
4. Wait 30-60 seconds
5. See updates in Foxglove!

**No terminal commands needed** - everything is automated! 🚀
