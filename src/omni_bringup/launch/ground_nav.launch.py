"""Ground omni-wheel navigation stack with Isaac ROS Nvblox and Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_oakd = LaunchConfiguration("launch_oakd")
    launch_ekf = LaunchConfiguration("launch_ekf")
    launch_nvblox = LaunchConfiguration("launch_nvblox")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_ground_bridge = LaunchConfiguration("launch_ground_bridge")

    nav2_params = LaunchConfiguration("nav2_params_file")
    nvblox_params = LaunchConfiguration("nvblox_params_file")
    ekf_params = LaunchConfiguration("ekf_params_file")
    bridge_params = LaunchConfiguration("ground_bridge_params_file")

    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("oakd_perception"), "launch", "oakd_unified.launch.py"]
            )
        ),
        launch_arguments={
            "imu_frequency": "400",
            "pointcloud_frequency": "15",
            "enable_depth_publish": "true",
            "depth_image_topic": "/oakd/depth/image",
            "depth_camera_info_topic": "/oakd/depth/camera_info",
            "pointcloud_frame_id": "oakd_camera_optical_frame",
            "imu_frame_id": "oakd_imu_link",
        }.items(),
        condition=IfCondition(launch_oakd),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_ekf),
    )

    nvblox_node = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[nvblox_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("camera_0/depth/image", "/oakd/depth/image"),
            ("camera_0/depth/camera_info", "/oakd/depth/camera_info"),
            ("camera_0/color/image", "/oakd/left/image_raw"),
            ("camera_0/color/camera_info", "/oakd/depth/camera_info"),
            ("pointcloud", "/oakd/points_filtered"),
        ],
        condition=IfCondition(launch_nvblox),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    ground_bridge_node = Node(
        package="ground_serial_bridge",
        executable="ground_serial_bridge_node",
        name="ground_serial_bridge",
        output="screen",
        parameters=[bridge_params],
        condition=IfCondition(launch_ground_bridge),
    )

    base_to_oakd_tf = Node(
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
            "oakd_imu_link",
        ],
    )

    base_to_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_footprint_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_oakd", default_value="true"),
            DeclareLaunchArgument("launch_ekf", default_value="true"),
            DeclareLaunchArgument("launch_nvblox", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_ground_bridge", default_value="true"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nav2_nvblox.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "nvblox.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("omni_bringup"), "config", "ekf_omni.yaml"]
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
            DeclareLaunchArgument("oakd_x", default_value="0.12"),
            DeclareLaunchArgument("oakd_y", default_value="0.0"),
            DeclareLaunchArgument("oakd_z", default_value="0.28"),
            DeclareLaunchArgument("oakd_yaw", default_value="0.0"),
            DeclareLaunchArgument("oakd_pitch", default_value="0.0"),
            DeclareLaunchArgument("oakd_roll", default_value="0.0"),
            base_to_footprint_tf,
            base_to_oakd_tf,
            oakd_launch,
            ekf_node,
            nvblox_node,
            nav2_launch,
            ground_bridge_node,
        ]
    )
