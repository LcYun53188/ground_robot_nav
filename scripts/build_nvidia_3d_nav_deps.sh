#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-13.2}"
cd "$WS_DIR"

"$WS_DIR/scripts/apply_vendor_patches.sh"

for package in \
  "ros-$ROS_DISTRO-bond" \
  "ros-$ROS_DISTRO-bondcpp" \
  "ros-$ROS_DISTRO-smclib"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q "install ok installed"; then
    echo "Missing ROS system dependency: $package" >&2
    echo "Run: ./scripts/install_nvidia_3d_nav_rosdeps.sh" >&2
    exit 2
  fi
done

gxf_core_lib="$WS_DIR/src/isaac_ros_nitros/isaac_ros_gxf/gxf/core/lib/gxf_x86_64_cuda_13_0/core/libgxf_core.so"
if file "$gxf_core_lib" | grep -q "ASCII text"; then
  echo "Missing Git LFS binary asset: $gxf_core_lib" >&2
  echo "Run: ./scripts/install_vendor_lfs_assets.sh" >&2
  exit 2
fi

if [ ! -x "$CUDA_HOME/bin/nvcc" ]; then
  echo "Missing nvcc: $CUDA_HOME/bin/nvcc" >&2
  echo "Install CUDA Toolkit 13.2 or set CUDA_HOME before running this script." >&2
  exit 2
fi

exec env \
  SKIP_WS_SETUP=true \
  CC=/usr/bin/gcc \
  CXX=/usr/bin/g++ \
  CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
  CUDA_HOME="$CUDA_HOME" \
  CUDAToolkit_ROOT="$CUDA_HOME" \
  CUDACXX="$CUDA_HOME/bin/nvcc" \
  PATH="$CUDA_HOME/bin:$PATH" \
  LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$WS_DIR/scripts/with_venv.sh" colcon build --symlink-install \
  --parallel-workers "${COLCON_PARALLEL_WORKERS:-2}" \
  --cmake-clean-cache \
  --packages-up-to \
    isaac_ros_visual_slam \
    nvblox_ros \
    nvblox_nav2 \
    nav2_mppi_controller \
    omni_bringup \
  --packages-skip \
    fast_lio \
    livox_ros_driver2 \
    livox_sdk2 \
    opennav_docking \
    opennav_docking_bt \
    opennav_docking_core \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_CUDA_COMPILER="$CUDA_HOME/bin/nvcc" \
    -DCUDAToolkit_ROOT="$CUDA_HOME" \
    -DCMAKE_CUDA_ARCHITECTURES=89 \
    -DUSE_SYSTEM_EIGEN=ON
