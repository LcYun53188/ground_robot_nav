#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

VENV_DIR="$WS_DIR/.venv"

if [ ! -x "$VENV_DIR/bin/python" ]; then
  echo "Missing .venv. Create it with: uv venv .venv" >&2
  exit 1
fi

# Do not source bin/activate here: activation scripts contain the absolute path
# from the directory where the environment was created and can become stale if
# a workspace is copied or renamed. Derive the environment from this workspace.
export VIRTUAL_ENV="$VENV_DIR"
export PATH="$VIRTUAL_ENV/bin:$PATH"
hash -r

# Make the virtualenv site-packages visible to ROS helper scripts that may run
# under the system Python during code generation.
VENV_SITE_PACKAGES="$VIRTUAL_ENV/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages"
LOCAL_DEPS="$WS_DIR/.deps"
export PYTHONPATH="$LOCAL_DEPS:$VENV_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"
export LIVOX_SDK2_ROOT="${LIVOX_SDK2_ROOT:-$LOCAL_DEPS/livox_sdk2}"
export LD_LIBRARY_PATH="$LIVOX_SDK2_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
  source /opt/ros/iron/setup.bash
else
  echo "No supported ROS distro setup found under /opt/ros" >&2
  exit 1
fi

if [ "${SKIP_WS_SETUP:-false}" != "true" ] && [ -f install/setup.bash ]; then
  source install/setup.bash
fi

if [ "$#" -eq 0 ]; then
  exec bash
fi

# Force colcon to run under this virtualenv Python.
if [ "$1" = "colcon" ]; then
  shift
  exec "$VIRTUAL_ENV/bin/python" -m colcon "$@"
fi

exec "$@"
