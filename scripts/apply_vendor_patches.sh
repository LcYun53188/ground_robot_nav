#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

apply_patch_if_needed() {
  local repo_dir="$1"
  local patch_file="$2"

  if [ ! -d "$repo_dir" ]; then
    echo "Missing $repo_dir. Run: git submodule update --init --recursive" >&2
    exit 1
  fi

  if (cd "$repo_dir" && git apply --check "$WS_DIR/$patch_file" >/dev/null 2>&1); then
    (cd "$repo_dir" && git apply "$WS_DIR/$patch_file")
    echo "Applied $patch_file"
  elif (cd "$repo_dir" && git apply --reverse --check "$WS_DIR/$patch_file" >/dev/null 2>&1); then
    echo "Already applied $patch_file"
  else
    echo "Cannot apply $patch_file cleanly in $repo_dir" >&2
    exit 1
  fi
}

apply_patch_if_needed "src/livox_ros_driver2" "patches/vendor/livox_ros_driver2.patch"
apply_patch_if_needed "src/FAST_LIO_ROS2" "patches/vendor/fast_lio_ros2.patch"
apply_patch_if_needed "third_party/Livox-SDK2" "patches/vendor/livox_sdk2.patch"
apply_patch_if_needed "src/isaac_ros_nvblox" "patches/vendor/isaac_ros_nvblox.patch"
apply_patch_if_needed "src/navigation2" "patches/vendor/navigation2.patch"
apply_patch_if_needed "src/magic_enum" "patches/vendor/magic_enum.patch"
apply_patch_if_needed "src/isaac_ros_nitros" "patches/vendor/isaac_ros_nitros.patch"
apply_patch_if_needed "src/negotiated" "patches/vendor/negotiated.patch"
apply_patch_if_needed "src/isaac_ros_image_pipeline" "patches/vendor/isaac_ros_image_pipeline.patch"
if [ -f "src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/thirdparty/stdgpu/stdgpu_fix_cuda13_2_proxy_reference.patch" ] &&
  rg -q "::memset\\(block_hash.begin\\(\\) \\+ idx" \
    "src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/gpu_hash/internal/cuda/impl/gpu_hash_interface_impl.cuh"; then
  echo "Already applied patches/vendor/nvblox_core.patch"
else
  apply_patch_if_needed "src/isaac_ros_nvblox/nvblox_ros/nvblox_core" "patches/vendor/nvblox_core.patch"
fi
