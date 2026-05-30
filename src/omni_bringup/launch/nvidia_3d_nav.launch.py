"""NVIDIA Isaac ROS 3D navigation bringup for the ground platform.

This launch file keeps the first NVIDIA architecture deliberately single-sensor:
OAK-D provides stereo, IMU, and depth-derived data; Isaac ROS Visual SLAM owns
odom->base_link; nvblox owns the 3D map; Nav2 consumes nvblox through its costmap
plugin. MID360 and wheel odometry are intentionally not part of this entry point.
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
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


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
    ess_engine = LaunchConfiguration("ess_engine_file").perform(context)

    nodes = []

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
                    "image_frequency": "30",
                    "pointcloud_frequency": "15",
                    "enable_depth_publish": "true",
                    "enable_passive_stereo": "true",
                    "enable_active_stereo": "false",
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

    visual_slam_args = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    if visual_slam_params:
        visual_slam_args["params_file"] = visual_slam_params

    nodes.extend(
        _optional_include(
            context,
            "launch_visual_slam",
            "visual_slam_package",
            "visual_slam_launch_file",
            "Isaac ROS Visual SLAM",
            remaps=[
                ("visual_slam/image_0", LaunchConfiguration("left_image_topic")),
                ("visual_slam/image_1", LaunchConfiguration("right_image_topic")),
                ("visual_slam/camera_info_0", LaunchConfiguration("left_camera_info_topic")),
                ("visual_slam/camera_info_1", LaunchConfiguration("right_camera_info_topic")),
                ("visual_slam/imu", LaunchConfiguration("imu_topic")),
            ],
            args=visual_slam_args,
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
            DeclareLaunchArgument("oakd_pitch", default_value="0.0"),
            DeclareLaunchArgument("oakd_roll", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
