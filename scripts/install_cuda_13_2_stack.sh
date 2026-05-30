#!/usr/bin/env bash
set -euo pipefail

CUDA_KEYRING_URL="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"
CUDA_KEYRING_DEB="/tmp/cuda-keyring_1.1-1_all.deb"
CUDA_REPO_LIST="/etc/apt/sources.list.d/cuda-ubuntu2404-x86_64.list"
CUDA_HOME_DIR="/usr/local/cuda-13.2"

require_sudo() {
  if ! sudo -v; then
    echo "sudo authentication failed" >&2
    exit 1
  fi
}

install_nvidia_modprobe() {
  if ! command -v nvidia-modprobe >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y nvidia-modprobe
  fi

  sudo nvidia-modprobe -u -c=0 || true
}

install_cuda_repo() {
  if [ ! -f "$CUDA_REPO_LIST" ]; then
    curl -L "$CUDA_KEYRING_URL" -o "$CUDA_KEYRING_DEB"
    sudo dpkg -i "$CUDA_KEYRING_DEB"
  fi
  sudo apt-get update
}

install_cuda_toolkit() {
  sudo apt-get install -y cuda-toolkit-13-2
}

print_shell_exports() {
  cat <<EOF

Add this to ~/.bashrc if it is not already present:

export CUDA_HOME=$CUDA_HOME_DIR
export CUDAToolkit_ROOT=$CUDA_HOME_DIR
export PATH=$CUDA_HOME_DIR/bin:\$PATH
export LD_LIBRARY_PATH=$CUDA_HOME_DIR/lib64\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}

Then run:

source ~/.bashrc
nvcc --version
nvidia-smi
./scripts/build_nvidia_3d_nav_deps.sh
EOF
}

main() {
  require_sudo

  echo "Installing NVIDIA device-node helper..."
  install_nvidia_modprobe

  echo "Installing CUDA Toolkit 13.2 apt repository..."
  install_cuda_repo

  echo "Installing CUDA Toolkit 13.2..."
  install_cuda_toolkit

  echo "Verifying driver and toolkit..."
  nvidia-smi || true
  "$CUDA_HOME_DIR/bin/nvcc" --version

  print_shell_exports
}

main "$@"
