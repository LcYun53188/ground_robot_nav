# CUDA Toolkit 13.2 安装指南

本文档用于修复 Isaac ROS / nvblox 构建时报错：

```text
Failed to find nvcc.
Compiler requires the CUDA toolkit.
```

当前机器基线：

- OS: Ubuntu 24.04 noble, x86_64
- GPU: NVIDIA GeForce RTX 4070 Laptop GPU
- Driver: `595.71.05`
- `nvidia-smi` 显示 CUDA runtime capability: `13.2`
- 当前缺失项：`nvcc`

## 1. 版本匹配结论

使用 `CUDA Toolkit 13.2`，不要安装滚动到更新大版本的 `cuda-toolkit` 泛化包。

原因：

- NVIDIA CUDA Toolkit 13.x 需要 580+ Linux driver。
- CUDA 13.2 Update 1 的 Linux driver 下限是 `595.58.03`。
- 当前驱动是 `595.71.05`，满足 CUDA 13.2。
- CUDA 13.3 的 Linux driver 下限是 `610.43.02`，当前驱动不匹配。

因此本机推荐组合：

```text
NVIDIA driver 595.71.05 + CUDA Toolkit 13.2
```

## 2. 安装前确认

确认驱动正常：

```bash
nvidia-smi
```

确认当前没有 `nvcc`：

```bash
command -v nvcc || true
nvcc --version || true
```

确认当前驱动包：

```bash
dpkg -l | grep -E 'nvidia-driver|cuda-toolkit|nvidia-cuda'
```

当前项目机已安装：

```text
nvidia-driver-595-open 595.71.05-0ubuntu0.24.04.1
```

## 3. 添加 NVIDIA CUDA apt 源

推荐直接运行仓库脚本：

```bash
./scripts/install_cuda_13_2_stack.sh
```

该脚本会：

- 安装 `nvidia-modprobe`，用于创建 `/dev/nvidia*` 设备节点。
- 添加 Ubuntu 24.04 的 NVIDIA CUDA apt 源。
- 安装固定版本 `cuda-toolkit-13-2`。
- 不安装 `cuda` / `cuda-drivers` 元包。

如果需要手动执行，按下面步骤操作。

在 Ubuntu 24.04 上添加 NVIDIA 官方 CUDA repo keyring：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
```

## 4. 安装 CUDA Toolkit 13.2

只安装 toolkit，不安装 `cuda` / `cuda-drivers` 元包：

```bash
sudo apt-get install -y cuda-toolkit-13-2
```

不要使用：

```bash
sudo apt-get install cuda
sudo apt-get install cuda-toolkit
sudo apt-get install cuda-drivers
```

这些元包可能随 NVIDIA repo 滚动到 CUDA 13.3 或触发驱动升级，不适合作为当前 595.71.05 驱动的固定安装方式。

## 5. 环境变量

CUDA 13.2 默认安装到：

```text
/usr/local/cuda-13.2
```

临时生效：

```bash
export CUDA_HOME=/usr/local/cuda-13.2
export CUDAToolkit_ROOT=/usr/local/cuda-13.2
export PATH=/usr/local/cuda-13.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.2/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

写入当前用户 shell：

```bash
cat <<'EOF' >> ~/.bashrc

# CUDA Toolkit 13.2 for Isaac ROS builds
export CUDA_HOME=/usr/local/cuda-13.2
export CUDAToolkit_ROOT=/usr/local/cuda-13.2
export PATH=/usr/local/cuda-13.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.2/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
EOF
source ~/.bashrc
```

## 6. 验证 CUDA Toolkit

确认 `nvcc`：

```bash
nvcc --version
```

期望看到 CUDA release `13.2`。

确认驱动仍是 595.71.05：

```bash
nvidia-smi
```

确认 CMake 能找到 CUDA：

```bash
rm -rf /tmp/cuda_check
mkdir -p /tmp/cuda_check
cat >/tmp/cuda_check/CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.24)
project(cuda_check LANGUAGES CUDA)
find_package(CUDAToolkit REQUIRED)
message(STATUS "CUDAToolkit_VERSION=${CUDAToolkit_VERSION}")
message(STATUS "CMAKE_CUDA_COMPILER=${CMAKE_CUDA_COMPILER}")
EOF
cmake -S /tmp/cuda_check -B /tmp/cuda_check/build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.2/bin/nvcc \
  -DCUDAToolkit_ROOT=/usr/local/cuda-13.2
```

如果只需要验证 Isaac ROS 构建，直接运行：

```bash
./scripts/install_nvidia_3d_nav_rosdeps.sh
./scripts/install_vendor_lfs_assets.sh
./scripts/build_nvidia_3d_nav_deps.sh
```

当前目标机器是 RTX 4070 Laptop GPU，构建脚本会为 nvblox/Isaac ROS 设置：

```bash
-DCMAKE_CUDA_ARCHITECTURES=89
-DUSE_SYSTEM_EIGEN=ON
```

如果构建继续报错找不到 `vpiConfig.cmake`，安装 NVIDIA VPI 4：

```bash
./scripts/install_vpi_4_stack.sh
```

该脚本会添加 NVIDIA VPI Ubuntu 24.04 x86_64 repo，并安装：

- `libnvvpi4`
- `vpi4-dev`
- `vpi4-samples`

如果构建在 Nav2 包上报错找不到 `bond`、`bondcpp`、`smclib` 或其他 ROS 系统依赖，重新运行：

```bash
./scripts/install_nvidia_3d_nav_rosdeps.sh
```

如果构建在 `libgxf_core.so` 报 `file format not recognized`，说明 Isaac ROS 子仓库的 Git LFS 二进制库还只是 pointer 文本，运行：

```bash
./scripts/install_vendor_lfs_assets.sh
```

## 7. 驱动保护

如果需要避免 apt 后续自动把 595 驱动升级到不期望的版本，可以临时 hold：

```bash
sudo apt-mark hold nvidia-driver-595-open
```

取消 hold：

```bash
sudo apt-mark unhold nvidia-driver-595-open
```

只有在明确计划升级到 CUDA 13.3 或更高版本时，才应重新评估并升级驱动。

## 8. 常见问题

### `nvidia-smi` 报无法和 NVIDIA driver 通信，但模块已加载

先检查设备节点：

```bash
ls -l /dev/nvidia*
lsmod | grep '^nvidia'
```

如果内核模块已加载但没有 `/dev/nvidia*`，安装并运行：

```bash
sudo apt-get install -y nvidia-modprobe
sudo nvidia-modprobe -u -c=0
nvidia-smi
```

### `nvidia-smi` 显示 CUDA Version 13.2，为什么还缺 `nvcc`？

`nvidia-smi` 显示的是驱动支持的 CUDA runtime capability，不代表系统已经安装 CUDA Toolkit。`nvcc` 属于 CUDA Toolkit，需要单独安装 `cuda-toolkit-13-2`。

### `apt` 计划安装或升级驱动怎么办？

停止安装，确认命令是否使用了 `cuda`、`cuda-toolkit` 或 `cuda-drivers` 元包。当前推荐只安装：

```bash
sudo apt-get install -y cuda-toolkit-13-2
```

### Isaac ROS 仍找不到 CUDA

确认环境变量：

```bash
echo $CUDA_HOME
echo $CUDAToolkit_ROOT
command -v nvcc
```

必要时清理失败构建缓存后重试：

```bash
rm -rf build/isaac_ros_common build/isaac_ros_nitros build/isaac_ros_visual_slam build/isaac_ros_nvblox
SKIP_WS_SETUP=true ./scripts/with_venv.sh colcon build --symlink-install \
  --packages-up-to isaac_ros_visual_slam nvblox_ros nvblox_nav2 nav2_mppi_controller omni_bringup
```

## 9. 参考

- NVIDIA CUDA Toolkit Release Notes: https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
- NVIDIA CUDA Installation Guide for Linux: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/
- NVIDIA CUDA downloads for Ubuntu 24.04: https://developer.nvidia.com/cuda-downloads
