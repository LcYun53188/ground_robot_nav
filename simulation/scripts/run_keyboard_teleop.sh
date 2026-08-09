#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

exec "$WS_DIR/scripts/with_venv.sh" \
  python3 "$WS_DIR/src/omni_bringup/scripts/keyboard_teleop.py" "$@"
