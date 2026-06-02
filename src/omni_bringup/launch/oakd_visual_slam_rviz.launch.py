"""OAK-D Visual SLAM validation with RViz.

This launch file is intentionally a narrow hardware validation entry point. It
starts only the OAK-D unified driver, Isaac ROS Visual SLAM in VIO-only mode,
the static OAK-D mounting TF, and RViz. It does not start nvblox, Nav2, ESS, or
the ground serial bridge, so odometry and TF problems are easier to isolate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("omni_bringup")

    nvidia_3d_nav_launch = PathJoinSubstitution(
        [bringup_share, "launch", "nvidia_3d_nav.launch.py"]
    )
    visual_slam_params_file = PathJoinSubstitution(
        [bringup_share, "config", "isaac_visual_slam_oakd_vio_only.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [ThisLaunchFileDir(), "..", "rviz", "visual_slam_check.rviz"]
    )

    # The OAK-D is assumed to be mounted facing forward and level. These
    # defaults keep base_link as the ROS robot frame (X forward, Y left, Z up)
    # while preserving the standard optical camera frame convention.
    oakd_roll = LaunchConfiguration("oakd_roll")
    oakd_pitch = LaunchConfiguration("oakd_pitch")
    oakd_yaw = LaunchConfiguration("oakd_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_odom_guard", default_value="true"),
            DeclareLaunchArgument("launch_robot_localization", default_value="true"),
            DeclareLaunchArgument(
                "odom_guard_output_topic",
                default_value="/visual_slam/guarded_odometry",
            ),
            DeclareLaunchArgument("odom_guard_publish_tf", default_value="false"),
            DeclareLaunchArgument(
                "filtered_odom_topic", default_value="/odometry/filtered"
            ),
            DeclareLaunchArgument(
                "ekf_params_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "ekf_visual_slam_3d.yaml"]
                ),
            ),
            DeclareLaunchArgument("odom_guard_max_step_xy_m", default_value="0.20"),
            DeclareLaunchArgument("odom_guard_max_step_z_m", default_value="0.15"),
            DeclareLaunchArgument("odom_guard_max_yaw_step_deg", default_value="20.0"),
            DeclareLaunchArgument("odom_guard_max_speed_xy_mps", default_value="1.2"),
            DeclareLaunchArgument("odom_guard_max_yaw_rate_dps", default_value="120.0"),
            DeclareLaunchArgument(
                "odom_guard_max_hold_sec_before_reseed", default_value="2.0"
            ),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("rviz_delay", default_value="3.0"),
            DeclareLaunchArgument("oakd_x", default_value="0.0"),
            DeclareLaunchArgument("oakd_y", default_value="0.0"),
            DeclareLaunchArgument("oakd_z", default_value="0.0"),
            DeclareLaunchArgument("oakd_yaw", default_value="0.0"),
            DeclareLaunchArgument("oakd_pitch", default_value="1.57079632679"),
            DeclareLaunchArgument("oakd_roll", default_value="3.14159265359"),
            DeclareLaunchArgument("oakd_image_frequency", default_value="25"),
            DeclareLaunchArgument("oakd_image_poll_frequency", default_value="75"),
            DeclareLaunchArgument("oakd_image_queue_size", default_value="2"),
            DeclareLaunchArgument("oakd_image_pair_max_dt_ms", default_value="8.0"),
            DeclareLaunchArgument("oakd_image_output_mode", default_value="rectified"),
            DeclareLaunchArgument("oakd_image_qos_depth", default_value="4"),
            DeclareLaunchArgument("oakd_image_publish_order", default_value="left_first"),
            DeclareLaunchArgument(
                "oakd_image_inter_publish_delay_ms", default_value="1.0"
            ),
            DeclareLaunchArgument("oakd_stereo_quality_mode", default_value="low_latency"),
            DeclareLaunchArgument("oakd_imu_axis_mode", default_value="raw"),
            DeclareLaunchArgument(
                "oakd_imu_to_camera_tf_source",
                default_value="manual",
                description=(
                    "Use 'manual' for the verified validation TF, or 'eeprom' "
                    "to try OAK-D factory IMU-camera extrinsics."
                ),
            ),
            DeclareLaunchArgument("oakd_imu_to_camera_socket", default_value="CAM_A"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nvidia_3d_nav_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "launch_oakd": "true",
                    "launch_visual_slam": "true",
                    "launch_ess": "false",
                    "launch_nvblox": "false",
                    "launch_nav2": "false",
                    "launch_ground_bridge": "false",
                    "launch_odom_guard": LaunchConfiguration("launch_odom_guard"),
                    "launch_robot_localization": LaunchConfiguration(
                        "launch_robot_localization"
                    ),
                    "odom_guard_output_topic": LaunchConfiguration(
                        "odom_guard_output_topic"
                    ),
                    "odom_guard_publish_tf": LaunchConfiguration(
                        "odom_guard_publish_tf"
                    ),
                    "odom_guard_max_step_xy_m": LaunchConfiguration(
                        "odom_guard_max_step_xy_m"
                    ),
                    "odom_guard_max_step_z_m": LaunchConfiguration(
                        "odom_guard_max_step_z_m"
                    ),
                    "odom_guard_max_yaw_step_deg": LaunchConfiguration(
                        "odom_guard_max_yaw_step_deg"
                    ),
                    "odom_guard_max_speed_xy_mps": LaunchConfiguration(
                        "odom_guard_max_speed_xy_mps"
                    ),
                    "odom_guard_max_yaw_rate_dps": LaunchConfiguration(
                        "odom_guard_max_yaw_rate_dps"
                    ),
                    "odom_guard_max_hold_sec_before_reseed": LaunchConfiguration(
                        "odom_guard_max_hold_sec_before_reseed"
                    ),
                    "filtered_odom_topic": LaunchConfiguration("filtered_odom_topic"),
                    "ekf_params_file": LaunchConfiguration("ekf_params_file"),
                    "publish_oakd_static_tf": "true",
                    "visual_slam_params_file": visual_slam_params_file,
                    "oakd_enable_depth_publish": "false",
                    "oakd_enable_pointcloud_publish": "false",
                    "oakd_imu_axis_mode": LaunchConfiguration("oakd_imu_axis_mode"),
                    "oakd_imu_to_camera_tf_source": LaunchConfiguration(
                        "oakd_imu_to_camera_tf_source"
                    ),
                    "oakd_imu_to_camera_socket": LaunchConfiguration(
                        "oakd_imu_to_camera_socket"
                    ),
                    "oakd_image_frequency": LaunchConfiguration("oakd_image_frequency"),
                    "oakd_image_poll_frequency": LaunchConfiguration(
                        "oakd_image_poll_frequency"
                    ),
                    "oakd_image_queue_size": LaunchConfiguration("oakd_image_queue_size"),
                    "oakd_image_pair_max_dt_ms": LaunchConfiguration(
                        "oakd_image_pair_max_dt_ms"
                    ),
                    "oakd_image_output_mode": LaunchConfiguration(
                        "oakd_image_output_mode"
                    ),
                    "oakd_image_qos_depth": LaunchConfiguration("oakd_image_qos_depth"),
                    "oakd_image_publish_order": LaunchConfiguration(
                        "oakd_image_publish_order"
                    ),
                    "oakd_image_inter_publish_delay_ms": LaunchConfiguration(
                        "oakd_image_inter_publish_delay_ms"
                    ),
                    "oakd_stereo_quality_mode": LaunchConfiguration(
                        "oakd_stereo_quality_mode"
                    ),
                    "oakd_x": LaunchConfiguration("oakd_x"),
                    "oakd_y": LaunchConfiguration("oakd_y"),
                    "oakd_z": LaunchConfiguration("oakd_z"),
                    "oakd_yaw": oakd_yaw,
                    "oakd_pitch": oakd_pitch,
                    "oakd_roll": oakd_roll,
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("rviz_delay"),
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", LaunchConfiguration("rviz_config")],
                        condition=IfCondition(LaunchConfiguration("launch_rviz")),
                    )
                ],
            ),
        ]
    )
