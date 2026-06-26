#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$WS_DIR/scripts/nav_launch.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.env"
fi
if [[ -f "$WS_DIR/scripts/nav_launch.local.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.local.env"
fi

TIMEOUT_SEC="${DEBUG_TIMEOUT_SEC:-4}"
CHECK_MID360="${CHECK_NAV_MID360:-false}"

usage() {
  cat <<'EOF'
Usage:
  scripts/check_nav_stack.sh [options]

Defaults are read from scripts/nav_launch.env and optional
scripts/nav_launch.local.env. Command-line options override variables.

Options:
  --mid360
      Also check MID360 and LIO topics.

  --timeout <sec>
      Per-check timeout. Default: 4.

  -h, --help
      Show this help.
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    echo "$option requires a value" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mid360)
      CHECK_MID360="true"
      shift
      ;;
    --timeout)
      require_value "$1" "${2:-}"
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --timeout=*)
      TIMEOUT_SEC="${1#*=}"
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source "$WS_DIR/scripts/nav_debug_lib.sh"

print_header "Core Topics"
require_topic /tf
require_topic /tf_static
require_topic /odometry/local
require_topic /local_map/occupancy
require_topic /nav/cmd_vel
require_topic /nav/emergency
require_topic /nav/safety_status
optional_topic /vio/odometry
optional_topic /lio/odometry

print_header "Rates"
check_topic_hz /odometry/local odometry
check_topic_hz /local_map/occupancy local_map
check_topic_hz /nav/cmd_vel cmd_vel

print_header "Frames"
check_frame_id /local_map/occupancy map
check_tf map base_link
optional_topic /oakd/points_filtered
if topic_exists /oakd/points_filtered; then
  check_frame_id /oakd/points_filtered oakd_camera_optical_frame
fi

if [[ "$CHECK_MID360" == "true" ]]; then
  print_header "MID360 / LIO"
  require_topic /livox/lidar
  require_topic /livox/imu
  require_topic /mid360/points
  optional_topic /lio/odometry
  check_topic_hz /mid360/points mid360_points
  check_tf base_link mid360_link
fi

finish_report
