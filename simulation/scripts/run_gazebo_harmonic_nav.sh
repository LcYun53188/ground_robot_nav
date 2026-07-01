#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "$WS_DIR/scripts/with_venv.sh" ros2 launch omni_bringup gazebo_harmonic_nav.launch.py "$@"
