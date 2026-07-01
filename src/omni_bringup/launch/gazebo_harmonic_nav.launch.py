"""Gazebo Harmonic + ros_gz simulation for the ground omni-wheel stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("gz_args"),
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_gazebo")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="omni_gazebo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
    )

    color_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="omni_gazebo_color_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/image"],
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
    )

    depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="omni_gazebo_depth_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/depth_image"],
        condition=IfCondition(LaunchConfiguration("launch_bridge")),
    )

    gazebo_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_camera_static_tf",
        output="screen",
        arguments=[
            "0.18",
            "0.0",
            "0.22",
            "-1.57079632679",
            "0.0",
            "-1.57079632679",
            "base_link",
            "oakd_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("launch_navigation")),
    )

    nvidia_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("omni_bringup"), "launch", "nvidia_3d_nav.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "launch_oakd": "false",
            "launch_visual_slam": "false",
            "launch_ess": "false",
            "launch_nvblox": LaunchConfiguration("launch_nvblox"),
            "launch_nav2": LaunchConfiguration("launch_nav2"),
            "launch_ground_bridge": "false",
            "launch_odom_guard": "false",
            "launch_robot_localization": "false",
            "publish_oakd_static_tf": "false",
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "nvblox_params_file": LaunchConfiguration("nvblox_params_file"),
            "left_image_topic": "/rgbd_camera/image",
            "left_camera_info_topic": "/rgbd_camera/camera_info",
            "right_image_topic": "/rgbd_camera/image",
            "right_camera_info_topic": "/rgbd_camera/camera_info",
            "oakd_depth_image_topic": "/rgbd_camera/depth_image",
            "oakd_depth_camera_info_topic": "/rgbd_camera/camera_info",
            "nvblox_depth_image_topic": "/rgbd_camera/depth_image",
            "nvblox_depth_camera_info_topic": "/rgbd_camera/camera_info",
            "camera_optical_frame": "oakd_camera_optical_frame",
            "imu_frame": "oakd_imu_link",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_navigation")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="gazebo_nav_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    auto_goals = Node(
        package="omni_bringup",
        executable="gazebo_auto_goals.py",
        name="gazebo_auto_goals",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goals_json": ParameterValue(
                    LaunchConfiguration("auto_goals_json"), value_type=str
                ),
                "start_delay_sec": ParameterValue(
                    LaunchConfiguration("auto_goals_start_delay_sec"), value_type=float
                ),
                "pause_between_goals_sec": ParameterValue(
                    LaunchConfiguration("auto_goals_pause_between_goals_sec"),
                    value_type=float,
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_auto_goals")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "worlds",
                        "omni_harmonic_demo.sdf",
                    ]
                ),
            ),
            DeclareLaunchArgument("gz_args", default_value=["-r -v 3 ", world]),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "config",
                        "omni_gazebo_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("launch_bridge", default_value="true"),
            DeclareLaunchArgument("launch_navigation", default_value="true"),
            DeclareLaunchArgument("launch_nvblox", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_auto_goals", default_value="true"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nav2_isaac_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nvblox_isaac_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "rviz", "nvblox_map_check.rviz"]
                ),
            ),
            DeclareLaunchArgument(
                "auto_goals_json",
                default_value=(
                    '[{"x": 1.35, "y": -1.15, "yaw": 0.0}, '
                    '{"x": -1.20, "y": 1.20, "yaw": 1.57}, '
                    '{"x": 1.15, "y": 1.25, "yaw": 3.14}, '
                    '{"x": -1.35, "y": -1.05, "yaw": -1.57}]'
                ),
            ),
            DeclareLaunchArgument("auto_goals_start_delay_sec", default_value="10.0"),
            DeclareLaunchArgument(
                "auto_goals_pause_between_goals_sec", default_value="3.0"
            ),
            gazebo,
            bridge,
            color_image_bridge,
            depth_image_bridge,
            gazebo_camera_tf,
            nvidia_nav,
            rviz,
            auto_goals,
        ]
    )
