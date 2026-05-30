#!/usr/bin/env bash
set -euo pipefail

VPI_REPO="deb https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main"
VPI_LIST="/etc/apt/sources.list.d/nvidia-vpi4-r38-4.list"
VPI_KEY_URL="https://repo.download.nvidia.com/jetson/jetson-ota-public.asc"

require_sudo() {
  if ! sudo -v; then
    echo "sudo authentication failed" >&2
    exit 1
  fi
}

install_vpi_repo() {
  sudo apt-get update
  sudo apt-get install -y gnupg software-properties-common
  sudo apt-key adv --fetch-key "$VPI_KEY_URL"
  if [ ! -f "$VPI_LIST" ] || ! grep -qF "$VPI_REPO" "$VPI_LIST"; then
    echo "$VPI_REPO" | sudo tee "$VPI_LIST" >/dev/null
  fi
  sudo apt-get update
}

install_vpi_packages() {
  sudo apt-get install -y libnvvpi4 vpi4-dev vpi4-samples
}

verify_vpi() {
  find /opt /usr -name 'vpiConfig.cmake' -o -name 'vpi-config.cmake' 2>/dev/null
  dpkg -l | grep -E 'libnvvpi|vpi4'
}

main() {
  require_sudo
  install_vpi_repo
  install_vpi_packages
  verify_vpi

  cat <<'EOF'

VPI installation finished. Retry:

export CUDA_HOME=/usr/local/cuda-13.2
export CUDAToolkit_ROOT=/usr/local/cuda-13.2
export PATH=/usr/local/cuda-13.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.2/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
./scripts/build_nvidia_3d_nav_deps.sh
EOF
}

main "$@"
