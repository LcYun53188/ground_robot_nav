#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONFIG_FILE="${ISAAC_SIM_NAV_CONFIG:-$WS_DIR/simulation/config/isaac_sim_nav.yaml}"
SCENE_SCRIPT="$WS_DIR/simulation/scripts/ground_nav_scene.py"
ROS_BRIDGE_SCRIPT="$WS_DIR/simulation/scripts/ros_nav_bridge.py"

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Missing Isaac Sim config: $CONFIG_FILE" >&2
  exit 1
fi

if [ ! -f "$SCENE_SCRIPT" ]; then
  echo "Missing Isaac Sim scene script: $SCENE_SCRIPT" >&2
  exit 1
fi

CONFIG_VALUES="$(
  "$WS_DIR/scripts/with_venv.sh" python3 - "$CONFIG_FILE" <<'PY'
import shlex
import sys

config_path = sys.argv[1]
python_executable = ""
domain_id = "0"
start_external_ros_bridge = "true"

section = []
with open(config_path, "r", encoding="utf-8") as stream:
    for raw_line in stream:
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        stripped = line.strip()
        if stripped.endswith(":"):
            key = stripped[:-1].strip()
            level = indent // 2
            section = section[:level] + [key]
            continue
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        path = section + [key]
        if path == ["isaac_sim", "python_executable"]:
            python_executable = value
        elif path == ["ros", "domain_id"]:
            domain_id = value or "0"
        elif path == ["ros", "start_external_ros_bridge"]:
            start_external_ros_bridge = value or "true"

print(f"PYTHON_EXECUTABLE={shlex.quote(python_executable)}")
print(f"ROS_DOMAIN_ID={shlex.quote(domain_id)}")
print(f"START_EXTERNAL_ROS_BRIDGE={shlex.quote(start_external_ros_bridge)}")
PY
)"

eval "$CONFIG_VALUES"

ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-$PYTHON_EXECUTABLE}"
if [ -z "$ISAAC_SIM_PYTHON" ]; then
  echo "Set ISAAC_SIM_PYTHON or isaac_sim.python_executable in $CONFIG_FILE" >&2
  echo "Example: export ISAAC_SIM_PYTHON=/home/nuc/isaacsim/python.sh" >&2
  exit 1
fi

if [ ! -x "$ISAAC_SIM_PYTHON" ]; then
  echo "Isaac Sim Python executable is not executable: $ISAAC_SIM_PYTHON" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "$ROS_LOG_DIR"

BRIDGE_PID=""
cleanup() {
  if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

case "${START_EXTERNAL_ROS_BRIDGE,,}" in
  1|true|yes|on)
    "$WS_DIR/scripts/with_venv.sh" python3 "$ROS_BRIDGE_SCRIPT" --config "$CONFIG_FILE" &
    BRIDGE_PID="$!"
    ;;
esac

# Keep ROS Jazzy's Python 3.12 path out of Isaac Sim's Python 3.11 process.
env -u PYTHONPATH "$ISAAC_SIM_PYTHON" "$SCENE_SCRIPT" --config "$CONFIG_FILE" "$@"
