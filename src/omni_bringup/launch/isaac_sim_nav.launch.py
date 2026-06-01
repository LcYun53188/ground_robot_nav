"""ROS2 side of the Isaac Sim navigation closed loop.

Isaac Sim owns the simulated sensors, /clock, odom->base_link TF, and
/visual_slam/tracking/odometry. This launch file reuses the NVIDIA 3D Nav2 /
nvblox stack while keeping real OAK-D, real cuVSLAM, and the serial bridge off.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nvidia_launch = IncludeLaunchDescription(
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
            "publish_oakd_static_tf": "false",
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "nvblox_params_file": LaunchConfiguration("nvblox_params_file"),
            "left_image_topic": LaunchConfiguration("left_image_topic"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "left_camera_info_topic": LaunchConfiguration("left_camera_info_topic"),
            "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic"),
            "oakd_depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "oakd_depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic"),
            "nvblox_depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "nvblox_depth_camera_info_topic": LaunchConfiguration("depth_camera_info_topic"),
            "imu_frame": LaunchConfiguration("imu_frame"),
            "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
            "left_camera_frame_id": LaunchConfiguration("left_camera_frame_id"),
            "right_camera_frame_id": LaunchConfiguration("right_camera_frame_id"),
            "oakd_x": LaunchConfiguration("oakd_x"),
            "oakd_y": LaunchConfiguration("oakd_y"),
            "oakd_z": LaunchConfiguration("oakd_z"),
            "oakd_yaw": LaunchConfiguration("oakd_yaw"),
            "oakd_pitch": LaunchConfiguration("oakd_pitch"),
            "oakd_roll": LaunchConfiguration("oakd_roll"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_nvblox", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
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
            DeclareLaunchArgument("left_image_topic", default_value="/oakd/left/image_raw"),
            DeclareLaunchArgument("right_image_topic", default_value="/oakd/right/image_raw"),
            DeclareLaunchArgument(
                "left_camera_info_topic", default_value="/oakd/left/camera_info"
            ),
            DeclareLaunchArgument(
                "right_camera_info_topic", default_value="/oakd/right/camera_info"
            ),
            DeclareLaunchArgument("depth_image_topic", default_value="/oakd/depth/image"),
            DeclareLaunchArgument(
                "depth_camera_info_topic", default_value="/oakd/depth/camera_info"
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
            DeclareLaunchArgument("oakd_x", default_value="0.12"),
            DeclareLaunchArgument("oakd_y", default_value="0.0"),
            DeclareLaunchArgument("oakd_z", default_value="0.28"),
            DeclareLaunchArgument("oakd_yaw", default_value="0.0"),
            DeclareLaunchArgument("oakd_pitch", default_value="0.0"),
            DeclareLaunchArgument("oakd_roll", default_value="0.0"),
            nvidia_launch,
        ]
    )
