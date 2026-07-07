#!/usr/bin/env bash
set -eo pipefail

cd /opt/xiaogua/ros2_ws
source /opt/ros/humble/setup.bash
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
fi
source install/setup.bash

python3 /opt/xiaogua/ros2_ws/src/robopilot_app_bridge/scripts/capture_calib_images.py "$@"
