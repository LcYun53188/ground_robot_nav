#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$WS_DIR/scripts/nav_launch.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.env"
fi
if [[ -f "$WS_DIR/scripts/nav_launch.local.env" ]]; then
  source "$WS_DIR/scripts/nav_launch.local.env"
fi

OUTPUT="${BAG_OUTPUT:-}"
INCLUDE_OFFLINE_MAP="${BAG_INCLUDE_OFFLINE_MAP:-false}"
INCLUDE_OMNI="${BAG_INCLUDE_OMNI:-false}"
EXTRA_TOPICS=()
if declare -p BAG_EXTRA_TOPICS >/dev/null 2>&1; then
  EXTRA_TOPICS=("${BAG_EXTRA_TOPICS[@]}")
fi

usage() {
  cat <<'EOF'
Usage:
  scripts/record_nav_debug_bag.sh [options] [extra_topic ...]

Defaults are read from scripts/nav_launch.env and optional
scripts/nav_launch.local.env. Command-line options override variables.

Options:
  -o, --output <bag_dir>
      Output bag directory. Default: nav_debug_<timestamp>.

  --offline-map
      Include offline map fusion topics.

  --omni
      Include ground omni-wheel bridge topics.

  -h, --help
      Show this help.

Examples:
  scripts/record_nav_debug_bag.sh
  scripts/record_nav_debug_bag.sh --offline-map --omni
  scripts/record_nav_debug_bag.sh -o test_run /base/state
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
    -o|--output)
      require_value "$1" "${2:-}"
      OUTPUT="$2"
      shift 2
      ;;
    --output=*)
      OUTPUT="${1#*=}"
      shift
      ;;
    --offline-map)
      INCLUDE_OFFLINE_MAP="true"
      shift
      ;;
    --omni)
      INCLUDE_OMNI="true"
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_TOPICS+=("$@")
      break
      ;;
    *)
      EXTRA_TOPICS+=("$1")
      shift
      ;;
  esac
done

if [[ -z "$OUTPUT" ]]; then
  OUTPUT="nav_debug_$(date +%Y%m%d_%H%M%S)"
fi

TOPICS=(
  /tf
  /tf_static
  /vio/odometry
  /lio/odometry
  /odometry/local
  /oakd/points
  /oakd/points_filtered
  /livox/lidar
  /livox/imu
  /mid360/points
  /perception/obstacle_points
  /local_map/occupancy
  /nav/cmd_vel
  /nav/emergency
  /nav/safety_status
)

if [[ "$INCLUDE_OFFLINE_MAP" == "true" ]]; then
  TOPICS+=(
    /static_map/occupancy
    /local_map/sensor_occupancy
  )
fi

if [[ "$INCLUDE_OMNI" == "true" ]]; then
  TOPICS+=(
    /base/state
    /base/status
    /base/diagnostics
  )
fi

TOPICS+=("${EXTRA_TOPICS[@]}")

echo "Recording nav debug bag:"
echo "  output: $OUTPUT"
echo "  topics: ${#TOPICS[@]}"
printf '    %s\n' "${TOPICS[@]}"

exec "$WS_DIR/scripts/with_venv.sh" ros2 bag record -o "$OUTPUT" "${TOPICS[@]}"
