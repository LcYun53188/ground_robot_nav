# Isaac Sim Simulation

This workflow runs Isaac Sim as the simulated hardware side and reuses the
existing NVIDIA 3D navigation stack on the ROS2 side.

## Architecture

Isaac Sim publishes:

- `/clock`
- `/tf`
- `/visual_slam/tracking/odometry`
- `/oakd/left/image_raw`
- `/oakd/right/image_raw`
- `/oakd/left/camera_info`
- `/oakd/right/camera_info`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`

Isaac Sim subscribes:

- `/cmd_vel`

By default, `simulation/scripts/run_isaac_sim_nav.sh` starts a small external
ROS2 bridge process under the workspace Python. This avoids mixing Isaac Sim's
Python 3.11 runtime with ROS Jazzy's Python 3.12 `rclpy` extension.

ROS2 runs `omni_bringup/isaac_sim_nav.launch.py`, which wraps
`nvidia_3d_nav.launch.py` with:

- `use_sim_time:=true`
- `launch_oakd:=false`
- `launch_visual_slam:=false`
- `launch_ground_bridge:=false`
- Isaac Sim topic names for OAK-D depth/image inputs

The simulation odometry intentionally uses Isaac Sim ground truth on
`/visual_slam/tracking/odometry`. This validates the Nav2/nvblox closed loop; it
does not validate real cuVSLAM tracking quality.

## Configure Isaac Sim

Edit `simulation/config/isaac_sim_nav.yaml`.

Set one of:

```yaml
isaac_sim:
  python_executable: "/absolute/path/to/isaacsim/python.sh"
```

or export it before launch:

```bash
export ISAAC_SIM_PYTHON=/absolute/path/to/isaacsim/python.sh
```

Keep the same ROS domain for both terminals:

```yaml
ros:
  domain_id: 0
```

The runner exports this value as `ROS_DOMAIN_ID` for Isaac Sim. Use the same
value in the ROS2 terminal if you override it manually.

## STL and USD Assets

`simulation/config/isaac_sim_nav.yaml` reserves these paths:

```yaml
scene:
  robot_mesh_stl_path: ""
  scene_mesh_stl_path: ""
  world_usd_path: ""
```

Leave them empty to use the built-in primitive scene. Put local assets under
`simulation/assets/` when practical, but avoid committing large mesh files.

For the first implementation, USD references are loaded directly. STL paths are
accepted as reserved configuration and reported in the log if the current Isaac
Sim build cannot import them directly; convert STL to USD in Isaac Sim for a
stable workflow.

## Start Simulation

Terminal 1, start Isaac Sim:

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./simulation/scripts/run_isaac_sim_nav.sh
```

This also starts `simulation/scripts/ros_nav_bridge.py` unless
`ros.start_external_ros_bridge` is set to `false` in
`simulation/config/isaac_sim_nav.yaml`.

Terminal 2, start ROS2 navigation:

```bash
cd /home/nuc/Program/ground_robot_nav_ws
./scripts/with_venv.sh ros2 launch omni_bringup isaac_sim_nav.launch.py
```

Optional RViz:

```bash
cd /home/nuc/Program/ground_robot_nav_ws
RVIZ_FIXED_FRAME=odom ./scripts/run_rviz_nav.sh
```

## Acceptance Checks

Check topics:

```bash
./scripts/with_venv.sh ros2 topic list -t
```

Expected topics include:

- `/clock`
- `/visual_slam/tracking/odometry`
- `/oakd/depth/image`
- `/oakd/depth/camera_info`
- `/cmd_vel`

Check TF:

```bash
./scripts/with_venv.sh ros2 run tf2_ros tf2_echo odom base_link
```

Check depth frequency:

```bash
./scripts/with_venv.sh ros2 topic hz /oakd/depth/image
```

Check Nav2:

```bash
./scripts/with_venv.sh ros2 lifecycle nodes
./scripts/with_venv.sh ros2 topic echo /cmd_vel
```

In RViz, set `Fixed Frame` to `odom`, send a Nav2 goal, and verify:

- Nav2 becomes active.
- `/cmd_vel` is published.
- Isaac Sim robot moves.
- `/nvblox_node/static_map_slice` appears and Nav2 costmaps update.

## Troubleshooting

No Isaac Sim topics appear:

- Confirm Isaac Sim ROS2 Bridge is enabled in your Isaac Sim installation.
- Confirm both terminals use the same `ROS_DOMAIN_ID`.
- Confirm `RMW_IMPLEMENTATION` is compatible with the Isaac Sim ROS2 Bridge.

`/clock` is missing or not advancing:

- Start Isaac Sim first.
- Keep all ROS2 navigation nodes on `use_sim_time:=true`.

TF from `odom` to `base_link` is missing:

- Confirm `simulation/scripts/ground_nav_scene.py` is running.
- Confirm `/visual_slam/tracking/odometry` is being published.

Nav2 does not accept goals:

- Use `odom` as RViz Fixed Frame.
- Confirm Nav2 lifecycle nodes are active.
- Confirm nvblox publishes `/nvblox_node/static_map_slice`.

Depth is present but nvblox is empty:

- Confirm `/oakd/depth/camera_info` frame matches the depth image frame.
- Confirm `nvblox_isaac_sim.yaml` has `use_sim_time: true`.
