#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROOT="${ISAAC_SIM_45_ROOT:-$WS_DIR/.venv-isaac-sim-4.5}"
APP="${ISAAC_SIM_APP:-$ROOT/bin/isaacsim}"
EXPERIENCE="${ISAAC_SIM_EXPERIENCE:-$ROOT/lib/python3.10/site-packages/isaacsim/apps/isaacsim.exp.base.kit}"

if [[ ! -x "$APP" ]]; then
  cat >&2 <<EOF
Isaac Sim 4.5 launcher not found or not executable:
  $APP

Install Isaac Sim 4.5 under:
  $ROOT

Or set:
  export ISAAC_SIM_45_ROOT=/path/to/.venv-isaac-sim-4.5
  export ISAAC_SIM_APP=/path/to/isaacsim
EOF
  exit 1
fi

if [[ ! -f "$EXPERIENCE" ]]; then
  cat >&2 <<EOF
Isaac Sim 4.5 base experience not found:
  $EXPERIENCE

Set ISAAC_SIM_EXPERIENCE if your 4.5 installation uses a different app file.
EOF
  exit 1
fi

exec env -u PYTHONPATH -u PYTHONHOME \
  OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}" \
  __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}" \
  __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}" \
  __VK_LAYER_NV_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}" \
  "$APP" "$EXPERIENCE" \
  --reset-user \
  --/renderer/multiGpu/enabled=false \
  --/app/renderer/resolution/width="${ISAAC_SIM_RENDER_WIDTH:-640}" \
  --/app/renderer/resolution/height="${ISAAC_SIM_RENDER_HEIGHT:-480}" \
  --/app/window/width="${ISAAC_SIM_WINDOW_WIDTH:-960}" \
  --/app/window/height="${ISAAC_SIM_WINDOW_HEIGHT:-540}" \
  "$@"
