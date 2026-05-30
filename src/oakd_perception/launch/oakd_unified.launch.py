import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)

    # 基础参数字典
    base_params = {
        "imu_frequency": int(LaunchConfiguration("imu_frequency").perform(context)),
        "pointcloud_frequency": int(
            LaunchConfiguration("pointcloud_frequency").perform(context)
        ),
        "enable_passive_stereo": LaunchConfiguration("enable_passive_stereo").perform(
            context
        )
        == "true",
        "enable_active_stereo": LaunchConfiguration("enable_active_stereo").perform(
            context
        )
        == "true",
        "ir_intensity": int(LaunchConfiguration("ir_intensity").perform(context)),
        "sampling_step": int(LaunchConfiguration("sampling_step").perform(context)),
        "min_depth": int(LaunchConfiguration("min_depth").perform(context)),
        "max_depth": int(LaunchConfiguration("max_depth").perform(context)),
        "depth_border_crop_px": int(
            LaunchConfiguration("depth_border_crop_px").perform(context)
        ),
        "max_depth_jump_mm": int(
            LaunchConfiguration("max_depth_jump_mm").perform(context)
        ),
        "enable_fov_boundary_filter": LaunchConfiguration(
            "enable_fov_boundary_filter"
        ).perform(context)
        == "true",
        "enable_depth_publish": LaunchConfiguration("enable_depth_publish").perform(
            context
        )
        == "true",
        "auto_estimate_fov": LaunchConfiguration("auto_estimate_fov").perform(
            context
        )
        == "true",
        "fov_h_deg": float(LaunchConfiguration("fov_h_deg").perform(context)),
        "fov_v_deg": float(LaunchConfiguration("fov_v_deg").perform(context)),
        "fov_boundary_margin_m": float(
            LaunchConfiguration("fov_boundary_margin_m").perform(context)
        ),
        "imu_topic_name": LaunchConfiguration("imu_topic").perform(context),
        "pointcloud_topic": LaunchConfiguration("pointcloud_topic").perform(context),
        "filtered_pointcloud_topic": LaunchConfiguration(
            "filtered_pointcloud_topic"
        ).perform(context),
        "left_image_topic": LaunchConfiguration("left_image_topic").perform(context),
        "right_image_topic": LaunchConfiguration("right_image_topic").perform(context),
        "left_camera_info_topic": LaunchConfiguration(
            "left_camera_info_topic"
        ).perform(context),
        "right_camera_info_topic": LaunchConfiguration(
            "right_camera_info_topic"
        ).perform(context),
        "depth_image_topic": LaunchConfiguration("depth_image_topic").perform(context),
        "depth_camera_info_topic": LaunchConfiguration(
            "depth_camera_info_topic"
        ).perform(context),
        "left_camera_frame_id": LaunchConfiguration("left_camera_frame_id").perform(
            context
        ),
        "right_camera_frame_id": LaunchConfiguration("right_camera_frame_id").perform(
            context
        ),
        "stereo_baseline_m": float(
            LaunchConfiguration("stereo_baseline_m").perform(context)
        ),
        "imu_frame_id": LaunchConfiguration("imu_frame_id").perform(context),
        "pointcloud_frame_id": LaunchConfiguration("pointcloud_frame_id").perform(
            context
        ),
    }

    node_params = [base_params]

    # 如果指定了 YAML 文件，则将其加入参数列表（它将覆盖之前的参数）
    if params_file and os.path.exists(params_file):
        node_params.append(params_file)

    oakd_unified_node = Node(
        package="oakd_perception",
        executable="oakd_unified_node",
        name="oakd_unified",
        output="screen",
        parameters=node_params,
    )

    # 静态变换：oakd_imu_link -> oakd_camera_optical_frame
    #
    # 这组参数描述 OAK-D 设备内部 IMU/机身坐标系到相机光学坐标系的固定关系。
    # 它不是 OAK-D 相对机器人底盘的安装位置；整台 OAK-D 的底盘安装外参在
    # omni_bringup/launch/ground_nav.launch.py 中通过 base_link -> oakd_imu_link 配置。
    #
    # TF 链路整体应为：
    #   base_link -> oakd_imu_link -> oakd_camera_optical_frame
    #
    # 坐标系含义：
    #   - oakd_imu_link：OAK-D 内置 IMU / 相机机身参考坐标系。
    #   - oakd_camera_optical_frame：相机光学坐标系，也是 /oakd/points 和
    #     /oakd/points_filtered 默认使用的 frame_id。
    #
    # static_transform_publisher 参数顺序：
    #   x y z yaw pitch roll parent_frame child_frame
    #
    # 当前平移为 0：
    #   - 暂不区分 IMU 原点和相机光学中心之间的物理偏移。
    #   - 如果后续从 OAK-D EEPROM 或标定结果获得 IMU->Camera 的真实平移，
    #     应填入这里的 x/y/z，单位为米。
    #
    # 当前旋转：
    #   - yaw = 1.57 rad
    #   - pitch = 0
    #   - roll = 3.14 rad
    #
    # 这组旋转用于把 OAK-D 机身/IMU 坐标轴转换到 ROS optical frame 约定：
    #   - optical +X：图像右方
    #   - optical +Y：图像下方
    #   - optical +Z：相机前方
    #
    # 注意：
    #   - 如果这里旋转方向写反，点云会出现轴向翻转。
    #   - 如果只是移动 OAK-D 在飞机上的安装位置，不应改这里，应改
    #     base_link -> oakd_imu_link。
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_camera_tf",
        arguments=[
            "0",
            "0",
            "0",
            "1.57",
            "0",
            "3.14",
            "oakd_imu_link",
            "oakd_camera_optical_frame",
        ],
    )

    left_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="oakd_center_to_left_camera_tf",
        arguments=[
            LaunchConfiguration("left_camera_x"),
            "0",
            "0",
            "0",
            "0",
            "0",
            "oakd_camera_optical_frame",
            LaunchConfiguration("left_camera_frame_id"),
        ],
    )

    right_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="oakd_center_to_right_camera_tf",
        arguments=[
            LaunchConfiguration("right_camera_x"),
            "0",
            "0",
            "0",
            "0",
            "0",
            "oakd_camera_optical_frame",
            LaunchConfiguration("right_camera_frame_id"),
        ],
    )

    return [oakd_unified_node, static_tf_node, left_camera_tf_node, right_camera_tf_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value="", description="YAML参数文件的路径"
            ),
            DeclareLaunchArgument("imu_frequency", default_value="400"),
            DeclareLaunchArgument("pointcloud_frequency", default_value="20"),
            DeclareLaunchArgument("enable_passive_stereo", default_value="true"),
            DeclareLaunchArgument("enable_active_stereo", default_value="false"),
            DeclareLaunchArgument("ir_intensity", default_value="1600"),
            DeclareLaunchArgument("sampling_step", default_value="2"),
            DeclareLaunchArgument("min_depth", default_value="200"),
            DeclareLaunchArgument("max_depth", default_value="5000"),
            DeclareLaunchArgument("depth_border_crop_px", default_value="8"),
            DeclareLaunchArgument("max_depth_jump_mm", default_value="350"),
            DeclareLaunchArgument(
                "enable_fov_boundary_filter", default_value="true"
            ),
            DeclareLaunchArgument("enable_depth_publish", default_value="true"),
            DeclareLaunchArgument("auto_estimate_fov", default_value="true"),
            DeclareLaunchArgument("fov_h_deg", default_value="72.0"),
            DeclareLaunchArgument("fov_v_deg", default_value="53.0"),
            DeclareLaunchArgument(
                "fov_boundary_margin_m", default_value="0.15"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/oakd/imu/raw"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/oakd/points"),
            DeclareLaunchArgument(
                "filtered_pointcloud_topic", default_value="/oakd/points_filtered"
            ),
            DeclareLaunchArgument("left_image_topic", default_value="/oakd/left/image_raw"),
            DeclareLaunchArgument(
                "right_image_topic", default_value="/oakd/right/image_raw"
            ),
            DeclareLaunchArgument(
                "left_camera_info_topic", default_value="/oakd/left/camera_info"
            ),
            DeclareLaunchArgument(
                "right_camera_info_topic", default_value="/oakd/right/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_image_topic", default_value="/oakd/depth/image"
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/oakd/depth/camera_info",
            ),
            DeclareLaunchArgument("imu_frame_id", default_value="oakd_imu_link"),
            DeclareLaunchArgument(
                "pointcloud_frame_id", default_value="oakd_camera_optical_frame"
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
            OpaqueFunction(function=launch_setup),
        ]
    )
