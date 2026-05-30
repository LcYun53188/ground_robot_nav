#!/usr/bin/env bash
set -euo pipefail

colcon build --symlink-install \
  --packages-skip \
    fast_lio \
    livox_ros_driver2 \
    livox_sdk2 \
    px4_msgs
