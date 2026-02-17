#!/bin/bash

# 1. Clean up any previous "ghost" processes or hardware locks
echo "[SYSTEM] Clearing serial locks and old bridge instances..."
sudo fuser -k /dev/ttyUSB0 2>/dev/null
pkill -9 serial_bridge 2>/dev/null

# 2. Source the environment
source /opt/ros/jazzy/setup.bash
source ~/mecca_ws/install/setup.bash

# 3. Launch the ROS nodes in the background
echo "[SYSTEM] Launching Mecca nodes..."
ros2 launch mecca_launch mecca_bringup.launch.py &
LAUNCH_PID=$!

# 4. Wait for the nodes to initialize
sleep 3

# 5. Lifecycle "Handshake"
echo "[SYSTEM] Configuring serial_bridge..."
ros2 lifecycle set /serial_bridge configure

sleep 2

echo "[SYSTEM] Activating serial_bridge..."
ros2 lifecycle set /serial_bridge activate

echo "[SYSTEM] Mecca is LIVE. Press Ctrl+C to shutdown."

# 6. Keep the script running so the background nodes stay alive
# and handle cleanup on exit
trap "echo '[SYSTEM] Shutting down...'; kill $LAUNCH_PID; exit" SIGINT SIGTERM
wait