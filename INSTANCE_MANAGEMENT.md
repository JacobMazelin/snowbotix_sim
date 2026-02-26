# Instance Management & Cost Control

This guide explains how to manage your AWS EC2 instance to minimize costs while maintaining a smooth development workflow.

## 💰 Cost-Saving Features

### ✅ 4-Hour Auto-Shutdown (Active Now!)

Your instance is configured to automatically shutdown after 4 hours of runtime. This prevents accidentally leaving it running overnight.

**Current Status:**
- ⏱️ Shutdown scheduled for: ~4 hours from boot time
- 📊 Check remaining time: `tail -f /tmp/auto_shutdown.log`
- 🔔 Warnings at: 3h, 2h, 1h, 30min, 10min, 5min, 1min before shutdown

**To cancel auto-shutdown (if you need more time):**
```bash
# SSH into the instance, then run:
systemctl --user stop auto-shutdown.service
sudo shutdown -c
```

## 🚀 Quick Start Guide

### Starting Your Session

When you want to work:

1. **Start the AWS Instance** (if it's stopped):
   ```bash
   # Check if instance is running
   ./scripts/check_instance.sh
   
   # If stopped, start from AWS Console:
   # https://console.aws.amazon.com/ec2/
   # → Instances → Select instance → Start
   ```

2. **Wait 1-2 minutes** for instance to boot

3. **Setup SSH Tunnel**:
   ```bash
   ./scripts/setup_tunnel.sh
   ```
   Keep this terminal open!

4. **Open Foxglove** → Connect to `ws://localhost:8765`

5. **Start coding!** Edit files, push to GitHub, see updates in 30-60 seconds

### Ending Your Session

When you're done:

**Option 1: Let auto-shutdown handle it (Recommended)**
- Just close your SSH tunnel terminal
- Instance will auto-shutdown in 4 hours (or less if already running)
- All code is safely in GitHub

**Option 2: Manual shutdown (immediate)**
```bash
# SSH into instance
ssh -i ~/Downloads/SnowbotixSim.pem ubuntu@18.223.190.83

# Then run:
sudo shutdown -h now
```

**Option 3: Stop from AWS Console**
- Go to https://console.aws.amazon.com/ec2/
- Select your instance
- Actions → Instance State → Stop

## 📊 Monitoring

### Check Instance Status (from Mac)
```bash
./scripts/check_instance.sh
```

This shows:
- ✅ Whether instance is online
- ✅ Service status (auto-shutdown, simulation, etc.)
- ⏱️ Time remaining until auto-shutdown
- 📋 Quick commands reference

### Monitor on AWS
```bash
# SSH in and check:
tail -f /tmp/auto_shutdown.log    # Shutdown timer
tail -f /tmp/simulation.log       # Simulation output
tail -f /tmp/startup.log          # Startup logs

# Health check
./scripts/healthcheck.sh
```

## 🔄 When You Stop/Start the Instance

### What Happens on Stop:
- ✅ All code safely stored in GitHub
- ⚠️ Simulation stops (will restart on boot)
- ⚠️ Services stop (will restart on boot)
- ✅ Auto-shutdown timer resets (4 hours from next boot)

### What Happens on Start:
- 🔄 Startup script runs automatically
- 🔄 Services start automatically:
  - Auto-shutdown timer (4 hours)
  - Foxglove bridge (port 8765)
  - Clock bridge (/clock)
  - Auto-update service
  - Gazebo simulation
- 🔄 Code syncs from GitHub (latest main branch)
- 🔄 Custom packages rebuild automatically

**You don't need to do anything!** Just:
1. Start the instance
2. Wait 2-3 minutes
3. Setup SSH tunnel
4. Open Foxglove
5. Everything is ready!

## 💵 Cost Estimates

**t3.medium instance (2 vCPU, 4 GB RAM)**
- Running: ~$0.0416/hour = ~$1/day if left running 24/7
- With 4-hour limit: ~$0.17 per session max
- **Your setup (stop when done)**: Probably $0.10-0.30 per week

**Ways you save money:**
- ✅ Auto-shutdown prevents accidental 24/7 running
- ✅ You can stop/start whenever you want
- ✅ Only charged while instance is "Running" state
- ✅ No data transfer charges (within AWS limits)

## 🛠️ Troubleshooting

### "Instance appears to be offline"
The instance is stopped. You need to start it from AWS Console:
1. https://console.aws.amazon.com/ec2/
2. Instances → Select your instance (ip: 18.223.190.83)
3. Actions → Instance State → Start
4. Wait 1-2 minutes, then run `./scripts/check_instance.sh`

### "Shutdown in X minutes" warnings
These are normal! The auto-shutdown sends warnings so you know it's coming.

**To extend your session:**
```bash
# SSH into instance, then:
systemctl --user stop auto-shutdown.service
# Now you can work as long as you want (remember to stop manually!)
```

### Services didn't start automatically
If the instance starts but simulation isn't running:
```bash
# SSH in and run startup manually:
cd ~/snowbotix_sim && ./aws/startup.sh
```

### Can't connect to Foxglove
1. Check SSH tunnel is running: `lsof -i :8765` on Mac
2. Check foxglove_bridge on AWS: `ss -lntp | grep 8765`
3. Restart tunnel: `./scripts/setup_tunnel.sh`

## 📅 Recommended Workflow

### For Daily Development:

1. **Morning**: Start instance, setup tunnel, open Foxglove
2. **Work**: Code → Push → See updates in Foxglove
3. **Breaks**: Leave running, auto-shutdown will warn you
4. **End of day**: Either:
   - Let auto-shutdown handle it (4 hours)
   - Stop manually if you want to save every cent

### For Long Sessions (>4 hours):

1. Cancel auto-shutdown: `systemctl --user stop auto-shutdown.service`
2. Work as long as needed
3. **Remember to stop manually!**

### For Cost-Conscious Mode:

1. When done working: Stop instance immediately
2. Next time: Start instance, wait 2 min, everything auto-starts

## 🎯 Summary Commands

```bash
# On Mac
./scripts/check_instance.sh      # Check if ready
./scripts/setup_tunnel.sh        # Start SSH tunnel

# On AWS (SSH in)
systemctl --user stop auto-shutdown.service    # Cancel shutdown
sudo shutdown -h now                           # Stop now
./scripts/healthcheck.sh                       # Check status
./aws/startup.sh                               # Manual restart

# AWS Console
# https://console.aws.amazon.com/ec2/
# → Start/Stop instance manually
```

## ✅ You're All Set!

Your instance will:
- ✅ Auto-shutdown after 4 hours (saves money)
- ✅ Auto-restart all services on boot (saves time)
- ✅ Auto-sync code from GitHub (keeps updated)
- ✅ Auto-build custom packages (stays current)

**Just remember**: Start the instance when you need it, let auto-shutdown handle the rest! 🎉
