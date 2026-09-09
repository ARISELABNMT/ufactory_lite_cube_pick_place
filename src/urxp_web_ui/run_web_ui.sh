#!/usr/bin/env bash
# URXP Web UI — one-terminal launcher.
# Usage: ~/URXP_ws/src/urxp_web_ui/run_web_ui.sh
set -e
source /opt/ros/jazzy/setup.bash
source ~/URXP_ws/install/setup.bash
export URXP_ROBOT_IP="${URXP_ROBOT_IP:-192.168.1.165}"
export URXP_WEB_UI_PORT="${URXP_WEB_UI_PORT:-8080}"
echo "Robot IP:  $URXP_ROBOT_IP  (override with URXP_ROBOT_IP=... before running)"
echo "Open:      http://localhost:$URXP_WEB_UI_PORT"
exec ros2 run urxp_web_ui web_ui_server
