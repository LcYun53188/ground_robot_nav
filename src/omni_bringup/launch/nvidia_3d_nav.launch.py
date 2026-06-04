"""NVIDIA Isaac ROS 3D navigation bringup for the ground platform.

This launch file keeps the first NVIDIA architecture deliberately camera-first:
OAK-D provides stereo, IMU, and depth-derived data; Isaac ROS Visual SLAM feeds a
guarded odometry stream; robot_localization publishes the final odom->base_link;
nvblox owns the 3D map; Nav2 consumes nvblox through its costmap plugin. MID360
and wheel odometry are intentionally not part of this entry point.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _package_launch(package_name, launch_file):
    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None

    if os.path.dirname(launch_file):
        path = os.path.join(package_share, launch_file)
    else:
        path = os.path.join(package_share, "launch", launch_file)
    if not os.path.exists(path):
        return None
    return path


def _load_ros_parameters(params_file, node_name):
    if not params_file or not os.path.exists(params_file):
        return {}
    with open(params_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if node_name in data:
        return data[node_name].get("ros__parameters", {})
    if "ros__parameters" in data:
        return data["ros__parameters"]
    return {}


def _optional_include(context, enabled_arg, package_arg, launch_arg, label, *, remaps=None, args=None):
    if not _as_bool(LaunchConfiguration(enabled_arg).perform(context)):
        return []

    package_name = LaunchConfiguration(package_arg).perform(context)
    launch_file = LaunchConfiguration(launch_arg).perform(context)
    launch_path = _package_launch(package_name, launch_file)
    if launch_path is None:
        return [
            LogInfo(
                msg=(
                    f'{label} requested, but package "{package_name}" or launch '
                    f'file "{launch_file}" was not found; skipping it'
                )
            )
        ]

    actions = []
    for source, target in remaps or []:
        actions.append(SetRemap(src=source, dst=target))
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=(args or {}).items(),
        )
    )
    return [GroupAction(actions=actions)]


def launch_setup(context, *args, **kwargs):
    visual_slam_params = LaunchConfiguration("visual_slam_params_file").perform(context)
    visual_slam_node_params = _load_ros_parameters(visual_slam_params, "visual_slam")
    visual_slam_node_params["use_sim_time"] = LaunchConfiguration("use_sim_time")
    launch_odom_guard = _as_bool(LaunchConfiguration("launch_odom_guard").perform(context))
    launch_robot_localization = _as_bool(
        LaunchConfiguration("launch_robot_localization").perform(context)
    )
    odom_guard_publish_tf = _as_bool(
        LaunchConfiguration("odom_guard_publish_tf").perform(context)
    )
    if launch_robot_localization or (launch_odom_guard and odom_guard_publish_tf):
        visual_slam_node_params["publish_odom_to_base_tf"] = False
    ess_engine = LaunchConfiguration("ess_engine_file").perform(context)
    oakd_imu_axis_mode = LaunchConfiguration("oakd_imu_axis_mode").perform(context)

    nodes = []

    if (
        _as_bool(LaunchConfiguration("launch_visual_slam").perform(context))
        and oakd_imu_axis_mode != "raw"
    ):
        nodes.append(
            LogInfo(
                msg=(
                    "WARNING: Isaac ROS Visual SLAM should use "
                    "oakd_imu_axis_mode:=raw unless the OAK-D camera-to-IMU TF "
                    "has been recalibrated for the remapped IMU frame. "
                    f"Current oakd_imu_axis_mode={oakd_imu_axis_mode!r} can cause "
                    "violent VIO jumps."
                )
            )
        )

    if _as_bool(LaunchConfiguration("launch_oakd").perform(context)):
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("oakd_perception"),
                            "launch",
                            "oakd_unified.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "imu_frequency": "400",
                    "imu_axis_mode": LaunchConfiguration("oakd_imu_axis_mode"),
                    "imu_to_camera_tf_source": LaunchConfiguration(
                        "oakd_imu_to_camera_tf_source"
                    ),
                    "imu_to_camera_socket": LaunchConfiguration(
                        "oakd_imu_to_camera_socket"
                    ),
                    "image_frequency": LaunchConfiguration("oakd_image_frequency"),
                    "image_poll_frequency": LaunchConfiguration(
                        "oakd_image_poll_frequency"
                    ),
                    "image_queue_size": LaunchConfiguration("oakd_image_queue_size"),
                    "image_pair_max_dt_ms": LaunchConfiguration(
                        "oakd_image_pair_max_dt_ms"
                    ),
                    "image_output_mode": LaunchConfiguration(
                        "oakd_image_output_mode"
                    ),
                    "image_qos_depth": LaunchConfiguration("oakd_image_qos_depth"),
                    "image_publish_order": LaunchConfiguration(
                        "oakd_image_publish_order"
                    ),
                    "image_inter_publish_delay_ms": LaunchConfiguration(
                        "oakd_image_inter_publish_delay_ms"
                    ),
                    "pointcloud_frequency": LaunchConfiguration(
                        "oakd_pointcloud_frequency"
                    ),
                    "enable_pointcloud_publish": LaunchConfiguration(
                        "oakd_enable_pointcloud_publish"
                    ),
                    "enable_depth_publish": LaunchConfiguration(
                        "oakd_enable_depth_publish"
                    ),
                    "enable_passive_stereo": "true",
                    "enable_active_stereo": "false",
                    "stereo_quality_mode": LaunchConfiguration(
                        "oakd_stereo_quality_mode"
                    ),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "left_image_topic": LaunchConfiguration("left_image_topic"),
                    "right_image_topic": LaunchConfiguration("right_image_topic"),
                    "left_camera_info_topic": LaunchConfiguration(
                        "left_camera_info_topic"
                    ),
                    "right_camera_info_topic": LaunchConfiguration(
                        "right_camera_info_topic"
                    ),
                    "depth_image_topic": LaunchConfiguration("oakd_depth_image_topic"),
                    "depth_camera_info_topic": LaunchConfiguration(
                        "oakd_depth_camera_info_topic"
                    ),
                    "left_camera_frame_id": LaunchConfiguration(
                        "left_camera_frame_id"
                    ),
                    "right_camera_frame_id": LaunchConfiguration(
                        "right_camera_frame_id"
                    ),
                    "stereo_baseline_m": LaunchConfiguration("stereo_baseline_m"),
                    "left_camera_x": LaunchConfiguration("left_camera_x"),
                    "right_camera_x": LaunchConfiguration("right_camera_x"),
                    "pointcloud_frame_id": LaunchConfiguration("camera_optical_frame"),
                    "imu_frame_id": LaunchConfiguration("imu_frame"),
                }.items(),
            )
        )

    if _as_bool(LaunchConfiguration("launch_visual_slam").perform(context)):
        visual_slam_package = LaunchConfiguration("visual_slam_package").perform(context)
        try:
            get_package_share_directory(visual_slam_package)
            nodes.append(
                ComposableNodeContainer(
                    name="visual_slam_launch_container",
                    namespace="",
                    package="rclcpp_components",
                    executable="component_container",
                    composable_node_descriptions=[
                        ComposableNode(
                            name="visual_slam_node",
                            package=visual_slam_package,
                            plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                            parameters=[visual_slam_node_params],
                            remappings=[
                                ("visual_slam/image_0", LaunchConfiguration("left_image_topic")),
                                ("visual_slam/image_1", LaunchConfiguration("right_image_topic")),
                                (
                                    "visual_slam/camera_info_0",
                                    LaunchConfiguration("left_camera_info_topic"),
                                ),
                                (
                                    "visual_slam/camera_info_1",
                                    LaunchConfiguration("right_camera_info_topic"),
                                ),
                                ("visual_slam/imu", LaunchConfiguration("imu_topic")),
                            ],
                        )
                    ],
                    output="screen",
                )
            )
        except PackageNotFoundError:
            nodes.append(
                LogInfo(
                    msg=(
                        f'Isaac ROS Visual SLAM requested, but package "{visual_slam_package}" '
                        "was not found; skipping it"
                    )
                )
            )

    nodes.append(
        Node(
            package="nav_guard",
            executable="visual_odom_guard",
            name="visual_odom_guard",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "input_topic": LaunchConfiguration("odom_guard_input_topic"),
                    "output_topic": LaunchConfiguration("odom_guard_output_topic"),
                    "status_topic": LaunchConfiguration("odom_guard_status_topic"),
                    "path_topic": LaunchConfiguration("odom_guard_path_topic"),
                    "publish_path": LaunchConfiguration("odom_guard_publish_path"),
                    "path_max_poses": LaunchConfiguration("odom_guard_path_max_poses"),
                    "publish_tf": False
                    if launch_robot_localization
                    else LaunchConfiguration("odom_guard_publish_tf"),
                    "odom_frame": LaunchConfiguration("odom_guard_odom_frame"),
                    "base_frame": LaunchConfiguration("odom_guard_base_frame"),
                    "max_step_xy_m": LaunchConfiguration("odom_guard_max_step_xy_m"),
                    "max_step_z_m": LaunchConfiguration("odom_guard_max_step_z_m"),
                    "max_yaw_step_deg": LaunchConfiguration(
                        "odom_guard_max_yaw_step_deg"
                    ),
                    "max_speed_xy_mps": LaunchConfiguration(
                        "odom_guard_max_speed_xy_mps"
                    ),
                    "max_yaw_rate_dps": LaunchConfiguration(
                        "odom_guard_max_yaw_rate_dps"
                    ),
                    "publish_rejected_as_hold": LaunchConfiguration(
                        "odom_guard_publish_rejected_as_hold"
                    ),
                    "max_hold_sec_before_reseed": LaunchConfiguration(
                        "odom_guard_max_hold_sec_before_reseed"
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("launch_odom_guard")),
        )
    )

    nodes.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                LaunchConfiguration("ekf_params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("odometry/filtered", LaunchConfiguration("filtered_odom_topic")),
            ],
            condition=IfCondition(LaunchConfiguration("launch_robot_localization")),
        )
    )

    ess_args = {
        "engine_file_path": ess_engine,
        "threshold": LaunchConfiguration("ess_threshold"),
    }
    nodes.extend(
        _optional_include(
            context,
            "launch_ess",
            "ess_package",
            "ess_launch_file",
            "Isaac ROS ESS",
            remaps=[
                ("left/image_rect", LaunchConfiguration("left_image_topic")),
                ("right/image_rect", LaunchConfiguration("right_image_topic")),
                ("left/camera_info", LaunchConfiguration("left_camera_info_topic")),
                ("right/camera_info", LaunchConfiguration("right_camera_info_topic")),
            ],
            args=ess_args,
        )
    )

    nodes.append(
        Node(
            package="nvblox_ros",
            executable="nvblox_node",
            name="nvblox_node",
            output="screen",
            parameters=[
                LaunchConfiguration("nvblox_params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("camera_0/depth/image", LaunchConfiguration("nvblox_depth_image_topic")),
                ("camera_0/depth/camera_info", LaunchConfiguration("nvblox_depth_camera_info_topic")),
                ("camera_0/color/image", LaunchConfiguration("left_image_topic")),
                ("camera_0/color/camera_info", LaunchConfiguration("left_camera_info_topic")),
            ],
            condition=IfCondition(LaunchConfiguration("launch_nvblox")),
        )
    )

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": LaunchConfiguration("nav2_params_file"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_nav2")),
        )
    )

    nodes.append(
        Node(
            package="ground_serial_bridge",
            executable="ground_serial_bridge_node",
            name="ground_serial_bridge",
            output="screen",
            parameters=[LaunchConfiguration("ground_bridge_params_file")],
            condition=IfCondition(LaunchConfiguration("launch_ground_bridge")),
        )
    )

    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_oakd_tf",
            arguments=[
                LaunchConfiguration("oakd_x"),
                LaunchConfiguration("oakd_y"),
                LaunchConfiguration("oakd_z"),
                LaunchConfiguration("oakd_yaw"),
                LaunchConfiguration("oakd_pitch"),
                LaunchConfiguration("oakd_roll"),
                "base_link",
                LaunchConfiguration("imu_frame"),
            ],
            condition=IfCondition(LaunchConfiguration("publish_oakd_static_tf")),
        )
    )

    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_footprint_tf",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_oakd", default_value="true"),
            DeclareLaunchArgument("launch_visual_slam", default_value="true"),
            DeclareLaunchArgument("launch_ess", default_value="false"),
            DeclareLaunchArgument("launch_nvblox", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_ground_bridge", default_value="true"),
            DeclareLaunchArgument("launch_odom_guard", default_value="true"),
            DeclareLaunchArgument("launch_robot_localization", default_value="true"),
            DeclareLaunchArgument(
                "odom_guard_input_topic",
                default_value="/visual_slam/tracking/odometry",
            ),
            DeclareLaunchArgument(
                "odom_guard_output_topic",
                default_value="/visual_slam/guarded_odometry",
            ),
            DeclareLaunchArgument(
                "odom_guard_status_topic",
                default_value="/visual_slam/odom_guard/status",
            ),
            DeclareLaunchArgument(
                "odom_guard_path_topic",
                default_value="/visual_slam/guarded_path",
            ),
            DeclareLaunchArgument("odom_guard_publish_path", default_value="true"),
            DeclareLaunchArgument("odom_guard_path_max_poses", default_value="2000"),
            DeclareLaunchArgument("odom_guard_publish_tf", default_value="true"),
            DeclareLaunchArgument("odom_guard_odom_frame", default_value="odom"),
            DeclareLaunchArgument("odom_guard_base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_guard_max_step_xy_m", default_value="0.20"),
            DeclareLaunchArgument("odom_guard_max_step_z_m", default_value="0.15"),
            DeclareLaunchArgument("odom_guard_max_yaw_step_deg", default_value="20.0"),
            DeclareLaunchArgument("odom_guard_max_speed_xy_mps", default_value="1.2"),
            DeclareLaunchArgument("odom_guard_max_yaw_rate_dps", default_value="120.0"),
            DeclareLaunchArgument(
                "odom_guard_publish_rejected_as_hold", default_value="true"
            ),
            DeclareLaunchArgument(
                "odom_guard_max_hold_sec_before_reseed", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "filtered_odom_topic", default_value="/odometry/filtered"
            ),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "ekf_visual_slam.yaml"]
                ),
            ),
            DeclareLaunchArgument("publish_oakd_static_tf", default_value="true"),
            DeclareLaunchArgument("oakd_image_frequency", default_value="25"),
            DeclareLaunchArgument("oakd_imu_axis_mode", default_value="raw"),
            DeclareLaunchArgument("oakd_imu_to_camera_tf_source", default_value="manual"),
            DeclareLaunchArgument("oakd_imu_to_camera_socket", default_value="CAM_A"),
            DeclareLaunchArgument("oakd_image_poll_frequency", default_value="75"),
            DeclareLaunchArgument("oakd_image_queue_size", default_value="2"),
            DeclareLaunchArgument("oakd_image_pair_max_dt_ms", default_value="8.0"),
            DeclareLaunchArgument("oakd_image_output_mode", default_value="rectified"),
            DeclareLaunchArgument("oakd_image_qos_depth", default_value="4"),
            DeclareLaunchArgument("oakd_image_publish_order", default_value="left_first"),
            DeclareLaunchArgument(
                "oakd_image_inter_publish_delay_ms", default_value="1.0"
            ),
            DeclareLaunchArgument("oakd_pointcloud_frequency", default_value="15"),
            DeclareLaunchArgument("oakd_stereo_quality_mode", default_value="auto"),
            # Visual SLAM and nvblox consume stereo images, IMU, and depth image.
            # The host-generated PointCloud2 stream is expensive and not needed
            # in this NVIDIA path, so keep it opt-in.
            DeclareLaunchArgument("oakd_enable_pointcloud_publish", default_value="false"),
            DeclareLaunchArgument("oakd_enable_depth_publish", default_value="true"),
            DeclareLaunchArgument(
                "visual_slam_package", default_value="isaac_ros_visual_slam"
            ),
            DeclareLaunchArgument(
                "visual_slam_launch_file",
                default_value="isaac_ros_visual_slam.launch.py",
            ),
            DeclareLaunchArgument("ess_package", default_value="isaac_ros_ess"),
            DeclareLaunchArgument("ess_launch_file", default_value="isaac_ros_ess.launch.py"),
            DeclareLaunchArgument("ess_engine_file", default_value=""),
            DeclareLaunchArgument("ess_threshold", default_value="0.0"),
            DeclareLaunchArgument(
                "visual_slam_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "config",
                        "isaac_visual_slam_oakd.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nvblox_3d_nav.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nav2_3d_nav.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "ground_bridge_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "config",
                        "ground_serial_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("left_image_topic", default_value="/oakd/left/image_raw"),
            DeclareLaunchArgument("right_image_topic", default_value="/oakd/right/image_raw"),
            DeclareLaunchArgument(
                "left_camera_info_topic", default_value="/oakd/left/camera_info"
            ),
            DeclareLaunchArgument(
                "right_camera_info_topic", default_value="/oakd/right/camera_info"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/oakd/imu/raw"),
            DeclareLaunchArgument(
                "oakd_depth_image_topic", default_value="/oakd/depth/image"
            ),
            DeclareLaunchArgument(
                "oakd_depth_camera_info_topic", default_value="/oakd/depth/camera_info"
            ),
            DeclareLaunchArgument(
                "nvblox_depth_image_topic", default_value="/oakd/depth/image"
            ),
            DeclareLaunchArgument(
                "nvblox_depth_camera_info_topic", default_value="/oakd/depth/camera_info"
            ),
            DeclareLaunchArgument("imu_frame", default_value="oakd_imu_link"),
            DeclareLaunchArgument(
                "camera_optical_frame", default_value="oakd_camera_optical_frame"
            ),
            DeclareLaunchArgument(
                "left_camera_frame_id", default_value="oakd_left_camera_optical_frame"
            ),
            DeclareLaunchArgument(
                "right_camera_frame_id", default_value="oakd_right_camera_optical_frame"
            ),
            DeclareLaunchArgument("stereo_baseline_m", default_value="0.075"),
            DeclareLaunchArgument("left_camera_x", default_value="-0.0375"),
            DeclareLaunchArgument("right_camera_x", default_value="0.0375"),
            DeclareLaunchArgument("oakd_x", default_value="0.12"),
            DeclareLaunchArgument("oakd_y", default_value="0.0"),
            DeclareLaunchArgument("oakd_z", default_value="0.28"),
            DeclareLaunchArgument("oakd_yaw", default_value="0.0"),
            # OAK-D raw IMU axes are not ROS base_link axes. With the OAK-D
            # camera facing forward and level, these defaults align base_link
            # (X forward, Y left, Z up) with the OAK-D IMU frame while keeping
            # oakd_camera_optical_frame at the standard optical convention:
            # Z forward, X right, Y down.
            DeclareLaunchArgument("oakd_pitch", default_value="1.57079632679"),
            DeclareLaunchArgument("oakd_roll", default_value="3.14159265359"),
            OpaqueFunction(function=launch_setup),
        ]
    )
