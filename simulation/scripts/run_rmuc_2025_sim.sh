#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_PATH="$WS_DIR/src/omni_bringup/gazebo/worlds/rmuc_2025.sdf"
NAV2_PARAMS_FILE="$WS_DIR/src/omni_bringup/config/nav2_isaac_sim.yaml"
NVBLOX_PARAMS_FILE="$WS_DIR/src/omni_bringup/config/nvblox_isaac_sim.yaml"
CLIFF_DETECTOR_PARAMS_FILE="$WS_DIR/src/oakd_perception/config/cliff_detector_gazebo.yaml"
TRAVERSABLE_DEPTH_FILTER_PARAMS_FILE="$WS_DIR/src/oakd_perception/config/traversable_depth_filter_gazebo.yaml"
SIM_LOCK_FILE="/tmp/ground_robot_nav_gazebo_harmonic_${UID:-user}.lock"
LAUNCH_PATTERN='[r]os2 launch omni_bringup gazebo_harmonic_nav.launch.py'
GAZEBO_PATTERN='[g]z sim.*ground_robot_nav_ws.*\.sdf'

for required_file in \
  "$WORLD_PATH" \
  "$NAV2_PARAMS_FILE" \
  "$NVBLOX_PARAMS_FILE" \
  "$CLIFF_DETECTOR_PARAMS_FILE" \
  "$TRAVERSABLE_DEPTH_FILTER_PARAMS_FILE"; do
  if [ ! -f "$required_file" ]; then
    echo "Missing RMUC 2025 simulation input: $required_file" >&2
    exit 1
  fi
done

old_simulation_running() {
  pgrep -f "$LAUNCH_PATTERN" >/dev/null || pgrep -f "$GAZEBO_PATTERN" >/dev/null
}

simulation_lock_is_free() {
  flock -n "$SIM_LOCK_FILE" -c true
}

signal_old_simulation() {
  local signal_name="$1"
  pkill "-$signal_name" -f "$LAUNCH_PATTERN" 2>/dev/null || true
  pkill "-$signal_name" -f "$GAZEBO_PATTERN" 2>/dev/null || true
}

wait_for_old_simulation() {
  local attempts="$1"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    if ! old_simulation_running && simulation_lock_is_free; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

if old_simulation_running || ! simulation_lock_is_free; then
  echo "[rmuc_2025_sim] Found an old simulation; sending SIGINT..."
  signal_old_simulation INT

  if ! wait_for_old_simulation 20; then
    echo "[rmuc_2025_sim] Graceful shutdown timed out; sending SIGTERM..."
    signal_old_simulation TERM
  fi

  if ! wait_for_old_simulation 10; then
    echo "[rmuc_2025_sim] Old simulation did not stop; refusing duplicate launch." >&2
    exit 1
  fi

  echo "[rmuc_2025_sim] Old simulation stopped."
else
  echo "[rmuc_2025_sim] No old simulation found."
fi

echo "[rmuc_2025_sim] Starting one RMUC 2025 simulation..."

# Defaults start one Gazebo window, RViz, the simulated sensor bridges, nvblox,
# and Nav2, then wait for a manually selected navigation goal. Automatic RMUC
# 2025 patrol remains available as an explicit launch override.
# Append launch arguments to override any default, for example:
#   ./simulation/scripts/run_rmuc_2025_sim.sh launch_navigation:=false
#   ./simulation/scripts/run_rmuc_2025_sim.sh launch_mid360:=false
#   ./simulation/scripts/run_rmuc_2025_sim.sh launch_oakd:=false
#   ./simulation/scripts/run_rmuc_2025_sim.sh launch_auto_goals:=true
exec "$WS_DIR/simulation/scripts/run_gazebo_harmonic_nav.sh" \
  arena:=rmuc_2025 \
  world:="$WORLD_PATH" \
  launch_gazebo:=true \
  launch_bridge:=true \
  launch_mid360:=false \
  launch_oakd:=true \
  launch_odometry_velocity_estimator:=true \
  launch_navigation:=true \
  launch_nvblox:=true \
  launch_traversable_depth_filter:=true \
  nvblox_depth_image_topic:=/rgbd_camera/depth_obstacles \
  traversable_depth_filter_params_file:="$TRAVERSABLE_DEPTH_FILTER_PARAMS_FILE" \
  launch_cliff_detector:=true \
  launch_nav2:=true \
  nav2_params_file:="$NAV2_PARAMS_FILE" \
  nvblox_params_file:="$NVBLOX_PARAMS_FILE" \
  cliff_detector_params_file:="$CLIFF_DETECTOR_PARAMS_FILE" \
  launch_rviz:=true \
  rviz_config:="$WS_DIR/src/omni_bringup/rviz/gazebo_sensor_check.rviz" \
  auto_goals_json:='[{"x":-1.77,"y":0.03,"yaw":3.14},{"x":1.93,"y":-0.22,"yaw":0.0},{"x":1.38,"y":-2.42,"yaw":-1.57},{"x":-0.92,"y":-2.77,"yaw":3.14}]' \
  launch_auto_goals:=false \
  "$@"
