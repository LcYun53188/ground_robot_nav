"""
Ground omni-wheel navigation stack launch file.

This legacy ground launch reuses shared perception, mapping, safety, and SE(2)
planning packages, then optionally starts a ground serial bridge.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LAUNCH_DEFAULTS = {
    "enable_oakd_perception": "true",
    "enable_mid360": "false",
    "launch_se2_dwa": "true",
    "launch_dwb": "false",
    "enable_offline_map": "false",
    "offline_map_yaml": "",
    "offline_map_topic": "/static_map/occupancy",
    "sensor_map_topic": "/local_map/sensor_occupancy",
    "fused_map_topic": "/local_map/occupancy",
    "enable_ground_serial_bridge": "false",
    "obstacle_pointcloud_source": "oakd",
    "oakd_pointcloud_topic": "/oakd/points_filtered",
    "mid360_pointcloud_topic": "/mid360/points",
    "mid360_custom_topic": "/livox/lidar",
    "mid360_frame_id": "mid360_link",
    "combined_pointcloud_topic": "/perception/obstacle_points",
    "combined_pointcloud_frame": "base_link",
    "combined_pointcloud_publish_rate": "10.0",
    "combined_pointcloud_max_age_sec": "0.5",
    "mid360_driver_package": "livox_ros_driver2",
    "mid360_driver_launch": "launch_ROS2/msg_MID360_launch.py",
    "ground_serial_bridge_package": "ground_serial_bridge",
    "ground_serial_bridge_executable": "ground_serial_bridge_node",
}


def _launch_arg(name, description=""):
    return DeclareLaunchArgument(
        name,
        default_value=LAUNCH_DEFAULTS[name],
        description=description,
    )


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _selected_obstacle_topic(context):
    source = LaunchConfiguration("obstacle_pointcloud_source").perform(context).lower()
    if source == "mid360":
        return LaunchConfiguration("mid360_pointcloud_topic").perform(context)
    if source == "both":
        return LaunchConfiguration("combined_pointcloud_topic").perform(context)
    return LaunchConfiguration("oakd_pointcloud_topic").perform(context)


def _optional_launch(context, enabled, package_arg, launch_arg, label):
    if not enabled:
        return []

    package_name = LaunchConfiguration(package_arg).perform(context)
    launch_file = LaunchConfiguration(launch_arg).perform(context)
    if not package_name or not launch_file:
        return [LogInfo(msg=f"{label} enabled, but package or launch file is empty")]

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'{label} enabled, but package "{package_name}" was not found; '
                    "skipping optional launch"
                )
            )
        ]

    if os.path.dirname(launch_file):
        launch_path = os.path.join(package_share, launch_file)
    else:
        launch_path = os.path.join(package_share, "launch", launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(
                msg=(
                    f'{label} enabled, but launch file "{launch_path}" was not found; '
                    "skipping optional launch"
                )
            )
        ]

    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))]


def launch_setup(context, *args, **kwargs):
    config_file = PathJoinSubstitution(
        [FindPackageShare("omni_bringup"), "config", "omni_nav_stack.yaml"]
    )
    ground_bridge_config = PathJoinSubstitution(
        [FindPackageShare("omni_bringup"), "config", "ground_serial_bridge.yaml"]
    )

    source = LaunchConfiguration("obstacle_pointcloud_source").perform(context).lower()
    selected_topic = _selected_obstacle_topic(context)
    enable_mid360 = _as_bool(LaunchConfiguration("enable_mid360").perform(context))
    enable_ground_serial_bridge = _as_bool(
        LaunchConfiguration("enable_ground_serial_bridge").perform(context)
    )
    enable_offline_map = _as_bool(
        LaunchConfiguration("enable_offline_map").perform(context)
    )
    offline_map_yaml = LaunchConfiguration("offline_map_yaml").perform(context)
    should_launch_map_fusion = enable_offline_map and bool(offline_map_yaml) and os.path.exists(
        offline_map_yaml
    )
    local_map_output_topic = (
        LaunchConfiguration("sensor_map_topic")
        if should_launch_map_fusion
        else LaunchConfiguration("fused_map_topic")
    )

    nodes = []
    if enable_offline_map:
        if not offline_map_yaml:
            nodes.append(
                LogInfo(
                    msg=(
                        "offline map enabled, but offline_map_yaml is empty; "
                        "skipping map_server"
                    )
                )
            )
        elif not os.path.exists(offline_map_yaml):
            nodes.append(
                LogInfo(
                    msg=(
                        f'offline map enabled, but "{offline_map_yaml}" was not '
                        "found; skipping map_server"
                    )
                )
            )
        else:
            nodes.extend(
                [
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="offline_map_server",
                        output="screen",
                        parameters=[
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("omni_bringup"),
                                    "config",
                                    "offline_map.yaml",
                                ]
                            ),
                            {"yaml_filename": offline_map_yaml},
                        ],
                        remappings=[
                            ("map", LaunchConfiguration("offline_map_topic")),
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="offline_map_lifecycle_manager",
                        output="screen",
                        parameters=[
                            {
                                "autostart": True,
                                "node_names": ["offline_map_server"],
                            }
                        ],
                    ),
                    LogInfo(
                        msg=(
                            "offline map is published and fused into "
                            "/local_map/occupancy for local planning"
                        )
                    ),
                ]
            )

    nodes.extend(
        _optional_launch(
            context,
            enable_mid360,
            "mid360_driver_package",
            "mid360_driver_launch",
            "MID360 driver",
        )
    )

    if enable_mid360 and source in ("mid360", "both"):
        nodes.append(
            Node(
                package="nav_mapping",
                executable="livox_custom_to_pointcloud2",
                name="mid360_custom_to_pointcloud2",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("mid360_custom_topic"),
                        "output_topic": LaunchConfiguration("mid360_pointcloud_topic"),
                        "frame_id": LaunchConfiguration("mid360_frame_id"),
                    }
                ],
            )
        )

    if source == "both":
        nodes.append(
            Node(
                package="nav_mapping",
                executable="pointcloud_combiner",
                name="obstacle_pointcloud_combiner",
                output="screen",
                parameters=[
                    {
                        "primary_topic": LaunchConfiguration("oakd_pointcloud_topic"),
                        "secondary_topic": LaunchConfiguration("mid360_pointcloud_topic"),
                        "output_topic": LaunchConfiguration("combined_pointcloud_topic"),
                        "output_frame": LaunchConfiguration("combined_pointcloud_frame"),
                        "publish_rate": LaunchConfiguration(
                            "combined_pointcloud_publish_rate"
                        ),
                        "max_source_age_sec": LaunchConfiguration(
                            "combined_pointcloud_max_age_sec"
                        ),
                    }
                ],
            )
        )

    nodes.extend(
        [
            Node(
                package="nav_mapping",
                executable="local_map_builder",
                name="local_map_builder",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "pointcloud_topic": selected_topic,
                        "output_topic": local_map_output_topic,
                    },
                ],
            ),
            Node(
                package="nav_mapping",
                executable="occupancy_grid_fusion",
                name="occupancy_grid_fusion",
                output="screen",
                condition=IfCondition(str(should_launch_map_fusion).lower()),
                parameters=[
                    config_file,
                    {
                        "dynamic_map_topic": LaunchConfiguration("sensor_map_topic"),
                        "static_map_topic": LaunchConfiguration("offline_map_topic"),
                        "output_map_topic": LaunchConfiguration("fused_map_topic"),
                    },
                ],
            ),
            Node(
                package="nav_planning",
                executable="se2_dwa_local_planner",
                name="se2_dwa_local_planner",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_se2_dwa")),
                parameters=[config_file],
            ),
            Node(
                package="nav_planning",
                executable="dwb_bridge",
                name="dwb_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_dwb")),
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nav_planning"),
                            "config",
                            "dwb_local_planner.yaml",
                        ]
                    )
                ],
                remappings=[
                    ("/oakd/points", selected_topic),
                    ("/nav/cmd_vel", "/nav/cmd_vel"),
                    ("/nav/goal_pose", "/nav/goal_pose"),
                ],
            ),
            Node(
                package="nav_safety",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[config_file, {"pointcloud_topic": selected_topic}],
            ),
        ]
    )

    if enable_ground_serial_bridge:
        nodes.append(
            Node(
                package=LaunchConfiguration("ground_serial_bridge_package"),
                executable=LaunchConfiguration("ground_serial_bridge_executable"),
                name="ground_serial_bridge",
                output="screen",
                parameters=[ground_bridge_config],
            )
        )
    else:
        nodes.append(
            LogInfo(
                msg=(
                    "ground_serial_bridge disabled; /nav/cmd_vel will be published "
                    "but not sent to the base"
                )
            )
        )

    return nodes


def generate_launch_description():
    oakd_perception_launch = IncludeLaunchDescription(
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
            "imu_frequency": "200",
            "pointcloud_frequency": "20",
            "enable_passive_stereo": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_oakd_perception")),
    )

    return LaunchDescription(
        [
            _launch_arg(
                "enable_oakd_perception",
                "Start the OAK-D perception launch for obstacle point clouds.",
            ),
            _launch_arg("enable_mid360", "Start MID360 driver and conversion nodes."),
            _launch_arg("launch_se2_dwa", "Start the SE(2) DWA local planner."),
            _launch_arg("launch_dwb", "Start the optional DWB bridge planner."),
            _launch_arg(
                "enable_offline_map",
                "Publish an offline occupancy map with nav2_map_server.",
            ),
            _launch_arg(
                "offline_map_yaml",
                "Absolute path to a nav2 map YAML file.",
            ),
            _launch_arg(
                "offline_map_topic",
                "Topic used by the offline occupancy map.",
            ),
            _launch_arg(
                "sensor_map_topic",
                "Intermediate real-time occupancy map before static map fusion.",
            ),
            _launch_arg(
                "fused_map_topic",
                "Occupancy map consumed by local planners.",
            ),
            _launch_arg(
                "enable_ground_serial_bridge",
                "Start the ground serial bridge if the package is available.",
            ),
            _launch_arg(
                "obstacle_pointcloud_source",
                "Obstacle source: oakd, mid360, or both.",
            ),
            _launch_arg("oakd_pointcloud_topic"),
            _launch_arg("mid360_pointcloud_topic"),
            _launch_arg("mid360_custom_topic"),
            _launch_arg("mid360_frame_id"),
            _launch_arg("combined_pointcloud_topic"),
            _launch_arg("combined_pointcloud_frame"),
            _launch_arg("combined_pointcloud_publish_rate"),
            _launch_arg("combined_pointcloud_max_age_sec"),
            _launch_arg("mid360_driver_package"),
            _launch_arg("mid360_driver_launch"),
            _launch_arg("ground_serial_bridge_package"),
            _launch_arg("ground_serial_bridge_executable"),
            oakd_perception_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
