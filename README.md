# Ground Robot Navigation

This workspace targets a ground omni-wheel robot using OAK-D perception,
Isaac ROS cuVSLAM, Isaac ROS Nvblox, Nav2, and a serial bridge to the base
controller.

## Main Entry

Build the ground stack:

```bash
./scripts/build_ground_stack.sh
source install/setup.bash
```

Launch the ground navigation stack:

```bash
ros2 launch omni_bringup ground_nav.launch.py
```

## Architecture

The intended runtime chain is:

```text
OAK-D stereo/depth + IMU
        ↓
isaac_ros_visual_slam + robot_localization
        ↓
odom -> base_link
        ↓
isaac_ros_nvblox
        ↓
nvblox_nav2 costmap layer
        ↓
Nav2 controller
        ↓
/cmd_vel
        ↓
ground_serial_bridge
```

Current local packages provide:

- `omni_bringup`: launch and configuration for the ground stack.
- `oakd_perception`: OAK-D IMU, stereo images, depth image, camera info, and point cloud.
- `ground_serial_bridge`: `/cmd_vel` or `/nav/cmd_vel` to the base MCU.
- `robot_localization`: filtered odometry and standard TF output.
- `nav_mapping`, `nav_planning`, `nav_safety`: shared utilities kept while Nav2/nvblox migration is completed.

External Isaac ROS/Nav2 runtime dependencies are expected to provide:

- `isaac_ros_visual_slam`
- `isaac_ros_nvblox`
- `nvblox_nav2`
- `nav2_bringup`
- `nav2_mppi_controller` or another Nav2 controller

## Required Runtime Topics

OAK-D perception:

- `/oakd/imu/raw`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`
- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/points_filtered`

Localization output:

- `/odometry/filtered`
- `odom -> base_link`

Control:

- `/cmd_vel`
- `/nav/emergency`
- `/nav/safety_status`

## Frame Contract

```text
odom
└── base_link
    ├── base_footprint
    └── oakd_imu_link
        └── oakd_camera_optical_frame
```

Tune the OAK-D mounting transform at launch:

```bash
ros2 launch omni_bringup ground_nav.launch.py oakd_x:=0.12 oakd_z:=0.28
```

## Ground-Only Scope

The active project scope is ground navigation, cuVSLAM localization, nvblox
mapping, Nav2 planning, and the ground serial bridge.
