#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WITH_NAV_BRIDGE="true"

if [[ "${1:-}" == "--with-nav" ]]; then
  WITH_NAV_BRIDGE="true"
fi

cd "$ROOT_DIR"

# ROS setup scripts may read AMENT_TRACE_SETUP_FILES without default expansion.
# Temporarily disable nounset while sourcing to avoid false unbound-variable exits.
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/unitree_ros2/example/src/install/setup.bash
set -u

# Ensure only one simulator/controller stack owns DDS channels.
pkill -f unitree_mujoco || true
pkill -f g1_ctrl || true
pkill -f "ros2 launch mujuco_sim" || true

# Use auto_start_velocity=false to keep robot in stable FixStand state (equivalent to reset)
exec ros2 launch mujuco_sim g1_nav_loco.launch.py auto_start_velocity:=false
