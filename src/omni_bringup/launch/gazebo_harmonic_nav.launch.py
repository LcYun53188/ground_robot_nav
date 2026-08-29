"""Gazebo Harmonic + ros_gz simulation for the ground omni-wheel stack."""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arena = LaunchConfiguration("arena")
    world = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")
    oakd_bridge_config = LaunchConfiguration("oakd_bridge_config")
    mid360_bridge_config = LaunchConfiguration("mid360_bridge_config")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_oakd = LaunchConfiguration("launch_oakd")
    launch_mid360 = LaunchConfiguration("launch_mid360")

    gazebo_models_path = PathJoinSubstitution(
        [FindPackageShare("omni_bringup"), "gazebo", "models"]
    )
    register_gazebo_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", gazebo_models_path
    )

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
        condition=IfCondition(launch_bridge),
    )

    oakd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="omni_gazebo_oakd_bridge",
        output="screen",
        parameters=[
            {
                "config_file": oakd_bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    mid360_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="omni_gazebo_mid360_bridge",
        output="screen",
        parameters=[
            {
                "config_file": mid360_bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_mid360)),
    )

    color_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="omni_gazebo_color_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/image"],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="omni_gazebo_depth_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/depth_image"],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_odometry_velocity = Node(
        package="omni_bringup",
        executable="gazebo_odometry_velocity.py",
        name="gazebo_odometry_velocity",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
            AndSubstitution(
                launch_bridge,
                LaunchConfiguration("launch_odometry_velocity_estimator"),
            )
        ),
    )

    traversable_depth_filter = Node(
        package="oakd_perception",
        executable="traversable_depth_filter",
        name="traversable_depth_filter",
        output="screen",
        parameters=[LaunchConfiguration("traversable_depth_filter_params_file")],
        condition=IfCondition(
            LaunchConfiguration("launch_traversable_depth_filter")
        ),
    )

    gazebo_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_camera_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0.18",
            "--y",
            "0.0",
            "--z",
            "0.16",
            "--yaw",
            "0.0",
            "--pitch",
            "0.31415926536",
            "--roll",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "oakd_camera_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_optical_static_tf",
        output="screen",
        arguments=[
            "--yaw",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--frame-id",
            "oakd_camera_link",
            "--child-frame-id",
            "oakd_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_oakd_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_imu_static_tf",
        output="screen",
        arguments=[
            "--frame-id",
            "oakd_camera_link",
            "--child-frame-id",
            "oakd_imu_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_mid360_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_mid360_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0.113137085",
            "--y",
            "-0.113137085",
            "--z",
            "0.18",
            "--yaw",
            "0.7853981634",
            "--pitch",
            "0.0",
            "--roll",
            "0.5235987756",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "mid360_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_mid360)),
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
            "launch_cliff_detector": LaunchConfiguration(
                "launch_cliff_detector"
            ),
            "launch_nav2": LaunchConfiguration("launch_nav2"),
            "launch_ground_bridge": "false",
            "launch_odom_guard": "false",
            "launch_robot_localization": "false",
            "publish_oakd_static_tf": "false",
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "nvblox_params_file": LaunchConfiguration("nvblox_params_file"),
            "cliff_detector_params_file": LaunchConfiguration(
                "cliff_detector_params_file"
            ),
            "left_image_topic": "/rgbd_camera/image",
            "left_camera_info_topic": "/rgbd_camera/camera_info",
            "right_image_topic": "/rgbd_camera/image",
            "right_camera_info_topic": "/rgbd_camera/camera_info",
            "oakd_depth_image_topic": "/rgbd_camera/depth_image",
            "oakd_depth_camera_info_topic": "/rgbd_camera/camera_info",
            "nvblox_depth_image_topic": LaunchConfiguration(
                "nvblox_depth_image_topic"
            ),
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
                "arena",
                default_value="omni_harmonic_demo",
                description=(
                    "Bundled arena name: omni_harmonic_demo, rmuc_2024, "
                    "rmuc_2025, rmul_2024, or rmul_2025"
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "worlds",
                        PythonExpression(["'", arena, ".sdf'"]),
                    ]
                ),
                description="World SDF path; overrides the bundled arena selection",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value=[
                    "-r -v 3 --physics-engine gz-physics-dartsim-plugin ",
                    world,
                ],
                description=(
                    "Gazebo arguments. DART is required by the mecanum wheel model's "
                    "directional friction and supports the bundled static arena meshes."
                ),
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "config",
                        "omni_gazebo_core_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "oakd_bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "config",
                        "omni_gazebo_oakd_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "mid360_bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("omni_bringup"),
                        "gazebo",
                        "config",
                        "omni_gazebo_mid360_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("launch_bridge", default_value="true"),
            DeclareLaunchArgument(
                "launch_oakd",
                default_value="true",
                description="Bridge the simulated OAK-D RGB-D camera and IMU.",
            ),
            DeclareLaunchArgument(
                "launch_mid360",
                default_value="true",
                description="Bridge the simulated MID360 point cloud and IMU.",
            ),
            DeclareLaunchArgument("launch_navigation", default_value="true"),
            DeclareLaunchArgument(
                "launch_odometry_velocity_estimator",
                default_value="true",
                description=(
                    "Populate twist omitted by Gazebo's 3-D odometry publisher."
                ),
            ),
            DeclareLaunchArgument("launch_nvblox", default_value="true"),
            DeclareLaunchArgument(
                "launch_traversable_depth_filter", default_value="true"
            ),
            DeclareLaunchArgument(
                "nvblox_depth_image_topic",
                default_value="/rgbd_camera/depth_obstacles",
            ),
            DeclareLaunchArgument(
                "traversable_depth_filter_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("oakd_perception"),
                        "config",
                        "traversable_depth_filter_gazebo.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("launch_cliff_detector", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_auto_goals", default_value="false"),
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
                "cliff_detector_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("oakd_perception"),
                        "config",
                        "cliff_detector_gazebo.yaml",
                    ]
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
            register_gazebo_models,
            gazebo,
            bridge,
            oakd_bridge,
            mid360_bridge,
            color_image_bridge,
            depth_image_bridge,
            gazebo_odometry_velocity,
            traversable_depth_filter,
            gazebo_camera_tf,
            gazebo_camera_optical_tf,
            gazebo_oakd_imu_tf,
            gazebo_mid360_tf,
            nvidia_nav,
            rviz,
            auto_goals,
        ]
    )
