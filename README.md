# Snowbotix Simulation

Headless ROS 2 Jazzy + Gazebo Sim simulation on AWS with Foxglove Studio integration.

## Quick Start

1. **On Mac**: Open this folder in VS Code
2. **Edit code** in `src/` and `launch/` directories
3. **Push to GitHub** (`Cmd+Shift+P` → "Git: Push")
4. **Changes automatically deploy** to AWS and appear in Foxglove

## Architecture

```
Mac (VS Code) → GitHub → GitHub Actions → AWS EC2 → Foxglove
```

## Directory Structure

```
snowbotix_sim/
├── src/                    # ROS 2 nodes and code
├── launch/                 # Launch files
├── config/                 # Configuration files
├── scripts/                # Utility scripts
├── aws/                    # AWS deployment scripts
├── .github/workflows/      # GitHub Actions
└── README.md
```

## Development Workflow

1. Edit files in VS Code
2. Changes are tracked automatically
3. Push to GitHub (or VS Code auto-pushes on save)
4. GitHub Actions deploys to AWS
5. Simulation restarts with new code
6. Foxglove updates automatically

## Git Commands

```bash
# Stage all changes
git add .

# Commit with message
git commit -m "Your commit message"

# Push to GitHub (triggers deployment)
git push origin main
```

## VS Code Shortcuts

- `Cmd+Shift+G` - Open Git panel
- `Cmd+Enter` - Commit
- `Cmd+Shift+P` → "Git: Push" - Push to GitHub

## Connecting to Foxglove

Once deployed:
1. Open Foxglove Studio
2. Add Connection → WebSocket
3. URL: `ws://localhost:8765`
4. Create SSH tunnel first: `ssh -i ~/Downloads/SnowbotixSim.pem -L 8765:localhost:8765 ubuntu@18.223.190.83 -N`
