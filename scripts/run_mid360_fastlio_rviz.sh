#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
LIVOX_LAUNCH="${MID360_LIVOX_LAUNCH:-msg_MID360_launch.py}"
FASTLIO_CONFIG_FILE="${FASTLIO_CONFIG_FILE:-mid360.yaml}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-true}"
START_LIVOX_DRIVER="${START_LIVOX_DRIVER:-true}"
PRELOAD_SYSTEM_LIBUSB="${PRELOAD_SYSTEM_LIBUSB:-true}"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_mid360_fastlio_rviz.sh [options] [fast_lio_launch_arg:=value ...]

Starts the MID360 Livox driver, FastLIO2 with the MID360 config, and RViz.
The script uses scripts/with_venv.sh, so the workspace .venv and install/setup
are loaded consistently with the rest of this repo.

Options:
  --no-rviz
      Start FastLIO2 without RViz.

  --skip-driver
      Do not start livox_ros_driver2. Useful when /livox/lidar and /livox/imu
      are already being published.

  --config-file <yaml>
      FastLIO2 config file under fast_lio/config. Default: mid360.yaml.

  --livox-launch <launch.py>
      livox_ros_driver2 launch file. Default: msg_MID360_launch.py.

  --no-libusb-preload
      Do not preload the system libusb. By default this avoids /opt/MVS libusb
      shadowing the system libusb required by PCL.

  --dry-run
      Print the resolved commands without executing them.

Examples:
  scripts/run_mid360_fastlio_rviz.sh
  scripts/run_mid360_fastlio_rviz.sh --no-rviz
  scripts/run_mid360_fastlio_rviz.sh --skip-driver rviz:=true
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

FASTLIO_EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help|help)
      usage
      exit 0
      ;;
    --no-rviz)
      FASTLIO_RVIZ="false"
      shift
      ;;
    --skip-driver)
      START_LIVOX_DRIVER="false"
      shift
      ;;
    --config-file)
      require_value "$1" "${2:-}"
      FASTLIO_CONFIG_FILE="$2"
      shift 2
      ;;
    --config-file=*)
      FASTLIO_CONFIG_FILE="${1#*=}"
      shift
      ;;
    --livox-launch)
      require_value "$1" "${2:-}"
      LIVOX_LAUNCH="$2"
      shift 2
      ;;
    --livox-launch=*)
      LIVOX_LAUNCH="${1#*=}"
      shift
      ;;
    --no-libusb-preload)
      PRELOAD_SYSTEM_LIBUSB="false"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    --)
      shift
      FASTLIO_EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      if [[ "$1" != *:=* ]]; then
        echo "Unknown option or launch argument: $1" >&2
        echo "Launch arguments must use name:=value syntax." >&2
        exit 2
      fi
      FASTLIO_EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -x "$WS_DIR/scripts/with_venv.sh" ]]; then
  echo "Missing executable $WS_DIR/scripts/with_venv.sh" >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

if [[ "$PRELOAD_SYSTEM_LIBUSB" == "true" && -f /lib/x86_64-linux-gnu/libusb-1.0.so.0 ]]; then
  export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
fi

LIVOX_CMD=(
  "$WS_DIR/scripts/with_venv.sh"
  ros2
  launch
  livox_ros_driver2
  "$LIVOX_LAUNCH"
)

FASTLIO_CMD=(
  "$WS_DIR/scripts/with_venv.sh"
  ros2
  launch
  fast_lio
  mapping.launch.py
  config_file:="$FASTLIO_CONFIG_FILE"
  rviz:="$FASTLIO_RVIZ"
  "${FASTLIO_EXTRA_ARGS[@]}"
)

cat <<EOF
MID360 + FastLIO2 launch:
  workspace       : $WS_DIR
  ROS_LOG_DIR     : $ROS_LOG_DIR
  Livox driver    : $START_LIVOX_DRIVER ($LIVOX_LAUNCH)
  FastLIO2 config : $FASTLIO_CONFIG_FILE
  RViz            : $FASTLIO_RVIZ
  LD_PRELOAD      : ${LD_PRELOAD:-<empty>}

Expected MID360 topics:
  /livox/lidar    livox_ros_driver2/msg/CustomMsg
  /livox/imu      sensor_msgs/msg/Imu

Expected FastLIO2 outputs after sensor data is flowing:
  /Odometry
  /path
  /cloud_registered
EOF

if command -v ip >/dev/null 2>&1; then
  echo
  if NETWORK_INFO="$(ip -br addr 2>/dev/null)"; then
    echo "Current network interfaces:"
    echo "$NETWORK_INFO"
  else
    echo "Current network interfaces: unavailable in this shell"
  fi
fi

if [[ "$DRY_RUN" == "true" ]]; then
  echo
  if [[ "$START_LIVOX_DRIVER" == "true" ]]; then
    printf 'Livox command: '
    printf '%q ' "${LIVOX_CMD[@]}"
    echo
  fi
  printf 'FastLIO2 command: '
  printf '%q ' "${FASTLIO_CMD[@]}"
  echo
  exit 0
fi

LIVOX_PID=""
cleanup() {
  if [[ -n "$LIVOX_PID" ]] && kill -0 "$LIVOX_PID" >/dev/null 2>&1; then
    kill "$LIVOX_PID" >/dev/null 2>&1 || true
    wait "$LIVOX_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$START_LIVOX_DRIVER" == "true" ]]; then
  echo
  echo "Starting livox_ros_driver2..."
  "${LIVOX_CMD[@]}" &
  LIVOX_PID="$!"
  sleep 2
fi

echo
echo "Starting FastLIO2 and RViz..."
exec "${FASTLIO_CMD[@]}"
