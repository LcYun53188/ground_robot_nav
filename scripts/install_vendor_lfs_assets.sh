#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

require_git_lfs() {
  if command -v git-lfs >/dev/null 2>&1 || git lfs version >/dev/null 2>&1; then
    return
  fi

  sudo apt-get update
  sudo apt-get install -y git-lfs
}

pull_lfs_assets() {
  git lfs install

  for repo in \
    "$WS_DIR/src/isaac_ros_nitros" \
    "$WS_DIR/src/isaac_ros_visual_slam"; do
    if [ -d "$repo/.git" ] || git -C "$repo" rev-parse --git-dir >/dev/null 2>&1; then
      git -C "$repo" lfs pull
    fi
  done
}

verify_gxf_core() {
  local lib="$WS_DIR/src/isaac_ros_nitros/isaac_ros_gxf/gxf/core/lib/gxf_x86_64_cuda_13_0/core/libgxf_core.so"
  if file "$lib" | grep -q "ASCII text"; then
    echo "Git LFS asset was not downloaded correctly: $lib" >&2
    exit 1
  fi
  file "$lib"
}

main() {
  require_git_lfs
  pull_lfs_assets
  verify_gxf_core
}

main "$@"
