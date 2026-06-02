import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _quaternion_from_matrix(rotation):
    """Convert a 3x3 rotation matrix to x/y/z/w quaternion."""
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    return qx, qy, qz, qw


def _camera_socket_from_name(socket_name):
    """Resolve a launch camera socket name to DepthAI's CameraBoardSocket."""
    import depthai as dai

    socket_key = socket_name.upper()
    aliases = {
        "RGB": "CAM_A",
        "COLOR": "CAM_A",
        "CENTER": "CAM_A",
        "LEFT": "LEFT",
        "RIGHT": "RIGHT",
    }
    socket_key = aliases.get(socket_key, socket_key)
    return getattr(dai.CameraBoardSocket, socket_key)


def _load_eeprom_imu_to_camera_tf(socket_name):
    """Read OAK-D EEPROM IMU-to-camera extrinsics as static TF arguments."""
    import depthai as dai

    with dai.Device() as device:
        calib = device.readCalibration()
        socket = _camera_socket_from_name(socket_name)
        transform = calib.getImuToCameraExtrinsics(socket, False)

    # DepthAI EEPROM extrinsic translations are reported in centimeters.
    # ROS TF uses meters, so convert here before publishing the static transform.
    translation = [float(transform[row][3]) / 100.0 for row in range(3)]
    rotation = [[float(transform[row][col]) for col in range(3)] for row in range(3)]
    qx, qy, qz, qw = _quaternion_from_matrix(rotation)
    return [
        "--x",
        f"{translation[0]:.9f}",
        "--y",
        f"{translation[1]:.9f}",
        "--z",
        f"{translation[2]:.9f}",
        "--qx",
        f"{qx:.12f}",
        "--qy",
        f"{qy:.12f}",
        "--qz",
        f"{qz:.12f}",
        "--qw",
        f"{qw:.12f}",
    ]


def _manual_imu_to_camera_tf_args():
    """Fallback IMU-to-camera transform used before EEPROM extrinsics were read."""
    return [
        "--x",
        "0",
        "--y",
        "0",
        "--z",
        "0",
        "--roll",
        "3.14",
        "--pitch",
        "0",
        "--yaw",
        "1.57",
    ]


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    imu_to_camera_tf_source = LaunchConfiguration("imu_to_camera_tf_source").perform(
        context
    )
    imu_to_camera_socket = LaunchConfiguration("imu_to_camera_socket").perform(context)

    # 基础参数字典
    base_params = {
        "imu_frequency": int(LaunchConfiguration("imu_frequency").perform(context)),
        "imu_axis_mode": LaunchConfiguration("imu_axis_mode").perform(context),
        "pointcloud_frequency": int(
            LaunchConfiguration("pointcloud_frequency").perform(context)
        ),
        "enable_pointcloud_publish": LaunchConfiguration(
            "enable_pointcloud_publish"
        ).perform(context)
        == "true",
        "enable_passive_stereo": LaunchConfiguration("enable_passive_stereo").perform(
            context
        )
        == "true",
        "enable_active_stereo": LaunchConfiguration("enable_active_stereo").perform(
            context
        )
        == "true",
        "ir_intensity": int(LaunchConfiguration("ir_intensity").perform(context)),
        "stereo_quality_mode": LaunchConfiguration("stereo_quality_mode").perform(
            context
        ),
        "image_output_mode": LaunchConfiguration("image_output_mode").perform(context),
        "image_qos_depth": int(
            LaunchConfiguration("image_qos_depth").perform(context)
        ),
        "image_publish_order": LaunchConfiguration("image_publish_order").perform(
            context
        ),
        "image_inter_publish_delay_ms": float(
            LaunchConfiguration("image_inter_publish_delay_ms").perform(context)
        ),
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
        "image_frequency": int(LaunchConfiguration("image_frequency").perform(context)),
        "image_poll_frequency": int(
            LaunchConfiguration("image_poll_frequency").perform(context)
        ),
        "image_queue_size": int(
            LaunchConfiguration("image_queue_size").perform(context)
        ),
        "image_pair_max_dt_ms": float(
            LaunchConfiguration("image_pair_max_dt_ms").perform(context)
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
    # 默认优先读取 OAK-D EEPROM 的 factory calibration：
    #   calib.getImuToCameraExtrinsics(<socket>, False)
    # 这与 Luxonis/DepthAI 官方 IMU 文档一致，避免手写相机-IMU外参。
    #
    # 该 TF 与 oakd_unified_node.py 的 imu_axis_mode=raw 配套：
    #   - /oakd/imu/raw 保持 DepthAI/OAK-D 原始 IMU 轴。
    #   - 由这条静态 TF 描述原始 IMU frame 到 Luxonis RDF/ROS optical
    #     camera frame 的关系。
    #
    # 对 Isaac ROS Visual SLAM 而言，相机-IMU外参必须和 IMU 消息的 frame
    # 保持同一套轴约定。不要同时预转换 IMU 轴又继续使用原始 IMU 外参。
    #
    # 如果 EEPROM IMU extrinsics 不可用，则回退到历史手写旋转：
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
    imu_to_camera_args = _manual_imu_to_camera_tf_args()
    if imu_to_camera_tf_source == "eeprom":
        try:
            imu_to_camera_args = _load_eeprom_imu_to_camera_tf(
                imu_to_camera_socket
            )
            print(
                "Loaded OAK-D EEPROM IMU-to-camera extrinsics "
                f"for socket {imu_to_camera_socket}: {imu_to_camera_args}"
            )
        except Exception as exc:
            print(
                "WARNING: failed to read OAK-D EEPROM IMU-to-camera "
                f"extrinsics for socket {imu_to_camera_socket}; "
                f"falling back to manual TF: {exc}"
            )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_camera_tf",
        arguments=imu_to_camera_args
        + [
            "--frame-id",
            "oakd_imu_link",
            "--child-frame-id",
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
            DeclareLaunchArgument(
                "imu_to_camera_tf_source",
                default_value="eeprom",
                description="Use 'eeprom' OAK-D factory IMU-camera extrinsics or 'manual' fallback TF.",
            ),
            DeclareLaunchArgument(
                "imu_to_camera_socket",
                default_value="CAM_A",
                description="CameraBoardSocket used for EEPROM IMU-to-camera extrinsics: CAM_A, LEFT, or RIGHT.",
            ),
            DeclareLaunchArgument("imu_frequency", default_value="400"),
            DeclareLaunchArgument("imu_axis_mode", default_value="raw"),
            DeclareLaunchArgument("pointcloud_frequency", default_value="20"),
            DeclareLaunchArgument("enable_pointcloud_publish", default_value="true"),
            DeclareLaunchArgument("enable_passive_stereo", default_value="true"),
            DeclareLaunchArgument("enable_active_stereo", default_value="false"),
            DeclareLaunchArgument("ir_intensity", default_value="1600"),
            DeclareLaunchArgument(
                "stereo_quality_mode",
                default_value="auto",
                description=(
                    "StereoDepth processing profile: auto, quality, or low_latency. "
                    "auto uses low_latency when depth and pointcloud publishing are disabled."
                ),
            ),
            DeclareLaunchArgument(
                "image_output_mode",
                default_value="rectified",
                description=(
                    "Image source for left/right image_raw: rectified uses "
                    "StereoDepth rectified outputs; mono bypasses StereoDepth "
                    "rectification for latency/drop diagnostics."
                ),
            ),
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
            DeclareLaunchArgument("image_frequency", default_value="25"),
            DeclareLaunchArgument("image_poll_frequency", default_value="75"),
            DeclareLaunchArgument("image_queue_size", default_value="2"),
            DeclareLaunchArgument("image_pair_max_dt_ms", default_value="8.0"),
            DeclareLaunchArgument("image_qos_depth", default_value="4"),
            DeclareLaunchArgument("image_publish_order", default_value="left_first"),
            DeclareLaunchArgument("image_inter_publish_delay_ms", default_value="1.0"),
            DeclareLaunchArgument("left_camera_x", default_value="-0.0375"),
            DeclareLaunchArgument("right_camera_x", default_value="0.0375"),
            OpaqueFunction(function=launch_setup),
        ]
    )
