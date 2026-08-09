#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

MODE="apply"
case "${1:-}" in
  "" | --apply)
    ;;
  --reverse)
    MODE="reverse"
    ;;
  *)
    echo "Usage: $0 [--apply|--reverse]" >&2
    exit 2
    ;;
esac

apply_patch_if_needed() {
  local repo_dir="$1"
  local patch_file="$2"
  local expected_commit="$3"

  if [ ! -d "$repo_dir" ]; then
    echo "Missing $repo_dir. Run: git submodule update --init --recursive" >&2
    exit 1
  fi

  local actual_commit
  actual_commit="$(git -C "$repo_dir" rev-parse HEAD)"
  if [[ "$actual_commit" != "$expected_commit" ]]; then
    echo "Unexpected commit in $repo_dir" >&2
    echo "  expected: $expected_commit" >&2
    echo "  actual:   $actual_commit" >&2
    echo "Run: git submodule update --init --recursive --force" >&2
    exit 1
  fi

  if [[ "$MODE" == "reverse" ]]; then
    if (cd "$repo_dir" && git apply --reverse --check --whitespace=nowarn "$WS_DIR/$patch_file" >/dev/null 2>&1); then
      (cd "$repo_dir" && git apply --reverse --whitespace=nowarn "$WS_DIR/$patch_file")
      echo "Reverted $patch_file"
    elif (cd "$repo_dir" && git apply --check --whitespace=nowarn "$WS_DIR/$patch_file" >/dev/null 2>&1); then
      echo "Already reverted $patch_file"
    else
      echo "Cannot reverse $patch_file cleanly in $repo_dir" >&2
      exit 1
    fi
    return
  fi

  if (cd "$repo_dir" && git apply --check --whitespace=nowarn "$WS_DIR/$patch_file" >/dev/null 2>&1); then
    (cd "$repo_dir" && git apply --whitespace=nowarn "$WS_DIR/$patch_file")
    echo "Applied $patch_file"
  elif (cd "$repo_dir" && git apply --reverse --check --whitespace=nowarn "$WS_DIR/$patch_file" >/dev/null 2>&1); then
    echo "Already applied $patch_file"
  else
    echo "Cannot apply $patch_file cleanly in $repo_dir" >&2
    exit 1
  fi
}

apply_patch_if_needed \
  "src/livox_ros_driver2" \
  "patches/vendor/livox_ros_driver2.patch" \
  "13eb05e4e6dd7a765b934d0c5fd6236676a57b49"
apply_patch_if_needed \
  "src/FAST_LIO_ROS2" \
  "patches/vendor/fast_lio_ros2.patch" \
  "2fffc570a25d0df172720bac034fbdb6a13d2162"
apply_patch_if_needed \
  "third_party/Livox-SDK2" \
  "patches/vendor/livox_sdk2.patch" \
  "f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
apply_patch_if_needed \
  "src/isaac_ros_nvblox" \
  "patches/vendor/isaac_ros_nvblox.patch" \
  "6362295e581ef243773c8a348ac46711e4a1fca4"
apply_patch_if_needed \
  "src/navigation2" \
  "patches/vendor/navigation2.patch" \
  "f3f5d1f64b4905e31ddab3dc5b861f701aa3771c"
apply_patch_if_needed \
  "src/magic_enum" \
  "patches/vendor/magic_enum.patch" \
  "9f19f78a7d726af84761ecd6d8414613507a95e6"
apply_patch_if_needed \
  "src/isaac_ros_nitros" \
  "patches/vendor/isaac_ros_nitros.patch" \
  "a22f10d4918662c485b0a1323e2fe1d8c21407a9"
apply_patch_if_needed \
  "src/negotiated" \
  "patches/vendor/negotiated.patch" \
  "eac198b55dcd052af5988f0f174902913c5f20e7"
apply_patch_if_needed \
  "src/isaac_ros_image_pipeline" \
  "patches/vendor/isaac_ros_image_pipeline.patch" \
  "ab21ed0818e50bd4524a442bc186acbde8de8a56"
apply_patch_if_needed \
  "src/isaac_ros_nvblox/nvblox_ros/nvblox_core" \
  "patches/vendor/nvblox_core.patch" \
  "3f42b210df9ad7a2099f00fcf324049d97342cb0"
