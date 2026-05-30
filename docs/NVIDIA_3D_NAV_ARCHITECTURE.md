# NVIDIA 3D Navigation Architecture

This is the replacement architecture for the ground robot when the target is
Isaac ROS 3D perception and Nav2 control without wheel odometry or MID360/OAK-D
cross-device timestamp alignment.

## Core data path

```text
OAK-D stereo images + OAK-D IMU
    -> isaac_ros_visual_slam
    -> map->odom and odom->base_link TF
    -> /visual_slam/tracking/odometry

OAK-D depth image
    -> nvblox_ros
    -> /nvblox_node/static_map_slice

/nvblox_node/static_map_slice
    -> nvblox_nav2 costmap layer
    -> Nav2 planner/controller
    -> /cmd_vel
    -> ground_serial_bridge
```

The first version intentionally uses only OAK-D as the core perception and
localization sensor. This keeps stereo, IMU, depth, and camera info in one sensor
clock domain and avoids OAK-D/MID360 synchronization work.

## Phase 1-2 bringup

For the Phase 1-2 skeleton and OAK-D data-integrity check, start only the OAK-D
and static TF part of the NVIDIA entry point:

```bash
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_visual_slam:=false \
  launch_nvblox:=false \
  launch_nav2:=false \
  launch_ground_bridge:=false
```

In another terminal, verify the required stereo, IMU, depth, and TF outputs:

```bash
./scripts/check_nvidia_3d_nav.sh
```

Expected OAK-D topics for the first NVIDIA path:

- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/left/camera_info`
- `/oakd/right/camera_info`
- `/oakd/imu/raw`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`

The current `/oakd/left/image_raw` and `/oakd/right/image_raw` names are kept for
compatibility with existing launch files, but the images are published from the
DepthAI `StereoDepth` rectified stereo outputs.

## Phase 3-6 dependencies

The NVIDIA path uses repo-local vendor sources for the packages that are not
expected to be present in a minimal ROS installation:

- `src/isaac_ros_visual_slam`: Isaac ROS Visual SLAM / cuVSLAM.
- `src/isaac_ros_nvblox`: ROS nvblox nodes and `nvblox_nav2`.
- `src/isaac_ros_common`: shared Isaac ROS CMake and launch utilities.
- `src/isaac_ros_nitros`: Isaac ROS NITROS/GXF transport packages.
- `src/navigation2`: Nav2 Jazzy source, including `nav2_mppi_controller`.
- `src/negotiated`: REP-2009 type negotiation support required by NITROS.
- `src/isaac_ros_image_pipeline`: only `isaac_ros_vpi_utils` is built for the
  NITROS image type dependency; image processing nodes are ignored by patch.

Initialize and patch the vendor sources with:

```bash
git submodule update --init --recursive
./scripts/apply_vendor_patches.sh
```

Install ROS system dependencies that are still expected from apt/rosdep:

```bash
./scripts/install_nvidia_3d_nav_rosdeps.sh
```

This covers Nav2 system packages such as `bond`, `bondcpp`, and `smclib`.

Download Git LFS vendor binaries used by Isaac ROS:

```bash
./scripts/install_vendor_lfs_assets.sh
```

Build the NVIDIA 3D navigation dependency set with:

```bash
./scripts/build_nvidia_3d_nav_deps.sh
```

The vendor patches under `patches/vendor/` keep upstream demo/test packages out
of the default workspace build where they are not needed for this robot path.

## Phase 3-6 validation

Static configuration and package availability check:

```bash
STATIC_ONLY=true ./scripts/check_nvidia_3d_nav_mvp.sh
```

Runtime check after starting the NVIDIA path:

```bash
./scripts/with_venv.sh ros2 launch omni_bringup nvidia_3d_nav.launch.py
./scripts/check_nvidia_3d_nav_mvp.sh
```

The runtime check expects:

- `/visual_slam/tracking/odometry`
- `map -> odom`
- `odom -> base_link`
- `/nvblox_node/static_map_slice`
- `/cmd_vel`

`nav2_3d_nav.yaml` uses `nav2_mppi_controller::MPPIController` with the `Omni`
motion model and conservative omni-wheel velocity limits.

## Optional ESS path

ESS is not enabled by default in `nvidia_3d_nav.launch.py` because it requires a
TensorRT engine and rectified stereo topics with left/right `CameraInfo`.

When the OAK-D image pipeline publishes rectified stereo and matching camera
info, enable it with:

```bash
ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_ess:=true \
  ess_engine_file:=/absolute/path/to/ess.engine \
  left_image_topic:=/oakd/left/image_rect \
  right_image_topic:=/oakd/right/image_rect \
  left_camera_info_topic:=/oakd/left/camera_info \
  right_camera_info_topic:=/oakd/right/camera_info
```

Then set `nvblox_depth_image_topic` and `nvblox_depth_camera_info_topic` to the
depth image produced by the disparity-to-depth stage.

## Launch

Default OAK-D + cuVSLAM + nvblox + Nav2:

```bash
ros2 launch omni_bringup nvidia_3d_nav.launch.py
```

Bring up mapping without driving the base:

```bash
ros2 launch omni_bringup nvidia_3d_nav.launch.py \
  launch_ground_bridge:=false
```

If Isaac ROS Visual SLAM is not installed yet, the launch file logs that it was
skipped. Install and source Isaac ROS before expecting `odom->base_link` to be
available.

## Packages kept

- `oakd_perception`: hardware input for OAK-D images, IMU, and fallback depth.
- `nvblox_ros`: 3D TSDF/ESDF mapping.
- `nvblox_nav2`: Nav2 costmap bridge.
- `nav2_*`: goal following, planning, local control, recovery behaviors.
- `ground_serial_bridge`: converts Nav2 velocity commands to the base.

## Packages removed from the core path

- `FAST_LIO_ROS2`: not used in the first NVIDIA architecture.
- `VINS-Fusion-ros2`: replaced by `isaac_ros_visual_slam`.
- `nav_mapping/local_map_builder`: replaced by nvblox.
- custom SE(2) DWA launch path: replaced by Nav2 controller server.

## Constraints

- No wheel odometry is required; `isaac_ros_visual_slam` owns `odom->base_link`.
- No MID360 is used in the core path.
- The system still needs correct OAK-D internal image/IMU timestamps.
- If Visual SLAM loses tracking in low texture or poor lighting, there is no
  wheel odometry fallback in this architecture.
