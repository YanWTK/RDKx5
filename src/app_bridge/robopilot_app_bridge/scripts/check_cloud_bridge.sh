#!/usr/bin/env bash
# Check script for MQTT cloud bridge readiness.
# Does NOT print any passwords.

set -eo pipefail

CONFIG_FILE="/opt/xiaogua/ros2_ws/src/robopilot_app_bridge/config/mqtt_bridge.env"
LOG_FILE="/tmp/robopilot_app_logs/robot_cloud_bridge.log"

echo "=== RoboPilot MQTT Cloud Bridge Check ==="
echo ""

# 1. Config file
echo "[1] Config file: $CONFIG_FILE"
if [ -f "$CONFIG_FILE" ]; then
  echo "    OK: file exists"
  # Check if placeholder values remain
  if grep -q "请填写" "$CONFIG_FILE"; then
    echo "    WARN: contains placeholder values — edit before starting"
  else
    echo "    OK: no placeholder values found"
  fi
  # Show keys only (no values)
  echo "    Keys configured:"
  grep -E "^[A-Z_]+=" "$CONFIG_FILE" | cut -d= -f1 | sed 's/^/      /'
else
  echo "    MISSING: run cp mqtt_bridge.env.example mqtt_bridge.env and edit"
fi
echo ""

# 2. Python paho-mqtt
echo "[2] Python paho-mqtt"
if python3 -c "import paho.mqtt; print('    OK: paho-mqtt', paho.mqtt.__version__)" 2>/dev/null; then
  :
else
  echo "    MISSING: pip install --user paho-mqtt"
fi
echo ""

# 3. Python dotenv
echo "[3] Python python-dotenv"
if python3 -c "import dotenv; print('    OK: python-dotenv')" 2>/dev/null; then
  :
else
  echo "    MISSING: pip install --user python-dotenv"
fi
echo ""

# 4. ROS2
echo "[4] ROS2"
if source /opt/ros/humble/setup.bash 2>/dev/null && python3 -c "import rclpy; print('    OK: rclpy available')" 2>/dev/null; then
  :
else
  echo "    WARN: rclpy not available — ROS2 may not be sourced"
fi
echo ""

# 5. Process running
echo "[5] robot_cloud_bridge process"
if pgrep -f "robot_cloud_bridge" >/dev/null 2>&1; then
  echo "    OK: process running (PID=$(pgrep -f robot_cloud_bridge))"
else
  echo "    NOT running"
fi
echo ""

# 6. Log file
echo "[6] Log file: $LOG_FILE"
if [ -f "$LOG_FILE" ]; then
  echo "    OK: exists ($(wc -l < "$LOG_FILE") lines)"
  echo "    Last 5 lines:"
  tail -5 "$LOG_FILE" | sed 's/^/      /'
else
  echo "    NOT found (bridge may not have started yet)"
fi
echo ""

# 7. MQTT connectivity (test without printing password)
echo "[7] MQTT connectivity test"
if [ -f "$CONFIG_FILE" ] && ! grep -q "请填写" "$CONFIG_FILE"; then
  MQTT_HOST=$(grep "^MQTT_HOST=" "$CONFIG_FILE" | cut -d= -f2)
  MQTT_PORT=$(grep "^MQTT_PORT=" "$CONFIG_FILE" | cut -d= -f2)
  MQTT_PORT="${MQTT_PORT:-1883}"
  echo "    Testing TCP connection to $MQTT_HOST:$MQTT_PORT ..."
  if timeout 5 bash -c "echo >/dev/tcp/$MQTT_HOST/$MQTT_PORT" 2>/dev/null; then
    echo "    OK: TCP port reachable"
  else
    echo "    FAIL: cannot reach $MQTT_HOST:$MQTT_PORT (check firewall/security group)"
  fi
else
  echo "    SKIPPED: config not ready"
fi
echo ""

# 8. ROS topics/services (only if ROS is running)
echo "[8] ROS topics/services"
if source /opt/ros/humble/setup.bash 2>/dev/null && source /opt/xiaogua/ros2_ws/install/setup.bash 2>/dev/null; then
  for topic in /cmd_vel /voice_cmd /robot_status /mode/status /voltage /robot_pose /odom /scan; do
    if ros2 topic info "$topic" >/dev/null 2>&1; then
      echo "    OK: $topic"
    else
      echo "    --: $topic (not available)"
    fi
  done
  for svc in /mode/switch_to_mapping /mode/switch_to_navigation /mode/switch_to_patrol /mode/get_status /mapping/start /mapping/save /mapping/stop; do
    if ros2 service type "$svc" >/dev/null 2>&1; then
      echo "    OK: $svc"
    else
      echo "    --: $svc (not available)"
    fi
  done
else
  echo "    SKIPPED: ROS2 not available"
fi
echo ""

echo "=== Check complete ==="
