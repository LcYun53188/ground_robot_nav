#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WITH_VENV="$WS_DIR/scripts/with_venv.sh"
NAV2_CONFIG="$WS_DIR/src/omni_bringup/config/nav2_3d_nav.yaml"
NVBLOX_CONFIG="$WS_DIR/src/omni_bringup/config/nvblox_3d_nav.yaml"
VSLAM_CONFIG="$WS_DIR/src/omni_bringup/config/isaac_visual_slam_oakd.yaml"

TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-6}"
TF_TIMEOUT_SEC="${TF_TIMEOUT_SEC:-6}"
STATIC_ONLY="${STATIC_ONLY:-false}"

fail() {
  echo "FAIL $*" >&2
  exit 1
}

ok() {
  echo "OK $*"
}

require_grep() {
  local pattern="$1"
  local file="$2"
  local label="$3"

  if ! grep -qE "$pattern" "$file"; then
    fail "$label not found in $file"
  fi
  ok "$label"
}

check_pkg() {
  local package="$1"

  if "$WITH_VENV" ros2 pkg prefix "$package" >/dev/null 2>&1; then
    ok "package available: $package"
  else
    fail "package missing or workspace not sourced: $package"
  fi
}

check_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local actual_type

  actual_type="$("$WITH_VENV" ros2 topic type "$topic" 2>/dev/null || true)"
  if [[ "$actual_type" != "$expected_type" ]]; then
    fail "topic type: $topic expected $expected_type, got ${actual_type:-<missing>}"
  fi
  ok "topic type: $topic -> $actual_type"
}

check_topic_hz() {
  local topic="$1"
  local output
  local status

  set +e
  output="$(timeout "$TOPIC_TIMEOUT_SEC" "$WITH_VENV" ros2 topic hz "$topic" 2>&1)"
  status="$?"
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "$output" >&2
    fail "topic hz command failed: $topic"
  fi
  if ! grep -q "average rate" <<<"$output"; then
    echo "$output" >&2
    fail "topic hz did not report an average rate: $topic"
  fi

  ok "topic hz: $topic"
  grep "average rate" <<<"$output" | tail -n 1
}

check_tf() {
  local parent_frame="$1"
  local child_frame="$2"
  local output
  local status

  set +e
  output="$(timeout "$TF_TIMEOUT_SEC" "$WITH_VENV" ros2 run tf2_ros tf2_echo \
    "$parent_frame" "$child_frame" 2>&1)"
  status="$?"
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "$output" >&2
    fail "tf command failed: $parent_frame -> $child_frame"
  fi
  if ! grep -Eq "Transform|Translation|Rotation" <<<"$output"; then
    echo "$output" >&2
    fail "tf did not produce a transform: $parent_frame -> $child_frame"
  fi
  ok "tf: $parent_frame -> $child_frame"
}

check_static_config() {
  require_grep 'tracking_mode: 1' "$VSLAM_CONFIG" "Visual SLAM VIO tracking mode"
  require_grep 'publish_odom_to_base_tf: true' "$VSLAM_CONFIG" "Visual SLAM odom->base TF"
  require_grep 'global_frame: odom' "$NVBLOX_CONFIG" "nvblox odom global frame"
  require_grep 'publish_map_slice: true' "$NVBLOX_CONFIG" "nvblox map slice publishing"
  require_grep 'NvbloxCostmapLayer' "$NAV2_CONFIG" "Nav2 nvblox costmap layer"
  require_grep 'nav2_mppi_controller::MPPIController' "$NAV2_CONFIG" "Nav2 MPPI controller"
  require_grep 'motion_model: "Omni"' "$NAV2_CONFIG" "MPPI omni motion model"
  require_grep 'odom_topic: /visual_slam/tracking/odometry' "$NAV2_CONFIG" "Nav2 visual slam odom topic"
}

check_package_availability() {
  check_pkg isaac_ros_visual_slam
  check_pkg nvblox_ros
  check_pkg nvblox_nav2
  check_pkg nav2_bringup
  check_pkg nav2_controller
  check_pkg nav2_mppi_controller
}

check_runtime_graph() {
  check_topic_type "/visual_slam/tracking/odometry" "nav_msgs/msg/Odometry"
  check_topic_type "/nvblox_node/static_map_slice" "nvblox_msgs/msg/DistanceMapSlice"
  check_topic_type "/cmd_vel" "geometry_msgs/msg/Twist"

  check_topic_hz "/visual_slam/tracking/odometry"
  check_topic_hz "/nvblox_node/static_map_slice"
  check_topic_hz "/cmd_vel"

  check_tf "map" "odom"
  check_tf "odom" "base_link"
}

main() {
  check_static_config
  check_package_availability

  if [[ "$STATIC_ONLY" == "true" ]]; then
    exit 0
  fi

  check_runtime_graph
}

main "$@"
