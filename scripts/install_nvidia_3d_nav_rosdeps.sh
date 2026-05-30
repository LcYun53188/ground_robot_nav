#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

require_sudo() {
  if ! sudo -v; then
    echo "sudo authentication failed" >&2
    exit 1
  fi
}

install_known_apt_deps() {
  sudo apt-get update
  sudo apt-get install -y \
    ros-"$ROS_DISTRO"-bond \
    ros-"$ROS_DISTRO"-bondcpp \
    ros-"$ROS_DISTRO"-smclib
}

install_rosdep_deps() {
  if ! command -v rosdep >/dev/null 2>&1; then
    sudo apt-get install -y python3-rosdep
  fi

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
  fi

  rosdep update
  rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y \
    --rosdistro "$ROS_DISTRO" \
    --skip-keys "magic_enum nvsci"
}

main() {
  require_sudo
  install_known_apt_deps
  install_rosdep_deps
}

main "$@"
