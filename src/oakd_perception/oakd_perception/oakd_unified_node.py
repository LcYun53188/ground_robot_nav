"""Unified OAK-D node for IMU and depth data."""

import depthai as dai
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Imu, Image, PointCloud2
from std_msgs.msg import Header

from oakd_perception.fov_boundary_filter import (
    FOVBoundaryFilter,
    build_depth_filter_mask,
    estimate_fov_from_intrinsics,
)


class OakDUnifiedNode(Node):
    """Unified OAK-D node for IMU and depth data."""

    def __init__(self):
        """Initialize the unified OAK-D node."""
        super().__init__("oakd_unified_node")

        # ============ IMU配置参数 ============
        self.declare_parameter("imu_frequency", 400)
        self.declare_parameter("gyro_full_scale", "gyroscope_2000_dps")
        self.declare_parameter("accel_full_scale", "accelerometer_4g")
        self.declare_parameter("imu_topic_name", "/oakd/imu/raw")
        self.declare_parameter("imu_frame_id", "oakd_imu_link")

        # ============ 深度模式开关配置 ============
        self.declare_parameter("enable_passive_stereo", True)
        self.declare_parameter("enable_active_stereo", False)
        self.declare_parameter("ir_intensity", 1600)

        # ============ 点云过滤参数配置 ============
        self.declare_parameter("pointcloud_frequency", 20)
        self.declare_parameter("pointcloud_topic", "/oakd/points")
        self.declare_parameter("filtered_pointcloud_topic", "/oakd/points_filtered")
        self.declare_parameter("pointcloud_frame_id", "oakd_imu_link")
        self.declare_parameter("sampling_step", 2)
        self.declare_parameter("min_depth", 200)
        self.declare_parameter("max_depth", 5000)
        self.declare_parameter("depth_border_crop_px", 8)
        self.declare_parameter("max_depth_jump_mm", 350)
        self.declare_parameter("enable_fov_boundary_filter", True)
        self.declare_parameter("auto_estimate_fov", True)
        self.declare_parameter("fov_h_deg", 72.0)
        self.declare_parameter("fov_v_deg", 53.0)
        self.declare_parameter("fov_boundary_margin_m", 0.15)

        # ============ 图像输出配置 ============
        self.declare_parameter("enable_image_publish", True)
        self.declare_parameter("enable_depth_publish", True)
        self.declare_parameter("left_image_topic", "/oakd/left/image_raw")
        self.declare_parameter("right_image_topic", "/oakd/right/image_raw")
        self.declare_parameter("left_camera_info_topic", "/oakd/left/camera_info")
        self.declare_parameter("right_camera_info_topic", "/oakd/right/camera_info")
        self.declare_parameter("depth_image_topic", "/oakd/depth/image")
        self.declare_parameter("depth_camera_info_topic", "/oakd/depth/camera_info")
        self.declare_parameter("left_camera_frame_id", "oakd_left_camera_optical_frame")
        self.declare_parameter("right_camera_frame_id", "oakd_right_camera_optical_frame")
        self.declare_parameter("stereo_baseline_m", 0.075)
        self.declare_parameter("image_frequency", 30)

        # 获取IMU参数
        self.imu_frequency = self.get_parameter("imu_frequency").value
        self.gyro_full_scale = self.get_parameter("gyro_full_scale").value
        self.accel_full_scale = self.get_parameter("accel_full_scale").value
        self.imu_topic_name = self.get_parameter("imu_topic_name").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value

        # 获取深度参数
        self.enable_passive_stereo = self.get_parameter("enable_passive_stereo").value
        self.enable_active_stereo = self.get_parameter("enable_active_stereo").value
        self.ir_intensity = self.get_parameter("ir_intensity").value

        # 获取点云参数
        self.pointcloud_frequency = self.get_parameter("pointcloud_frequency").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.filtered_pointcloud_topic = self.get_parameter(
            "filtered_pointcloud_topic"
        ).value
        self.pointcloud_frame_id = self.get_parameter("pointcloud_frame_id").value
        self.sampling_step = self.get_parameter("sampling_step").value
        self.min_depth = self.get_parameter("min_depth").value
        self.max_depth = self.get_parameter("max_depth").value
        self.depth_border_crop_px = self.get_parameter("depth_border_crop_px").value
        self.max_depth_jump_mm = self.get_parameter("max_depth_jump_mm").value
        self.enable_fov_boundary_filter = self.get_parameter(
            "enable_fov_boundary_filter"
        ).value
        self.auto_estimate_fov = self.get_parameter("auto_estimate_fov").value
        self.fov_h_deg = self.get_parameter("fov_h_deg").value
        self.fov_v_deg = self.get_parameter("fov_v_deg").value
        self.fov_boundary_margin_m = self.get_parameter(
            "fov_boundary_margin_m"
        ).value

        # 获取图像参数
        self.enable_image_publish = self.get_parameter("enable_image_publish").value
        self.enable_depth_publish = self.get_parameter("enable_depth_publish").value
        self.left_image_topic = self.get_parameter("left_image_topic").value
        self.right_image_topic = self.get_parameter("right_image_topic").value
        self.left_camera_info_topic = self.get_parameter(
            "left_camera_info_topic"
        ).value
        self.right_camera_info_topic = self.get_parameter(
            "right_camera_info_topic"
        ).value
        self.depth_image_topic = self.get_parameter("depth_image_topic").value
        self.depth_camera_info_topic = self.get_parameter(
            "depth_camera_info_topic"
        ).value
        self.left_camera_frame_id = self.get_parameter("left_camera_frame_id").value
        self.right_camera_frame_id = self.get_parameter("right_camera_frame_id").value
        self.stereo_baseline_m = float(self.get_parameter("stereo_baseline_m").value)
        self.image_frequency = self.get_parameter("image_frequency").value

        # 发布器
        self.imu_pub = self.create_publisher(Imu, self.imu_topic_name, 10)
        self.pc_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.filtered_pc_pub = self.create_publisher(
            PointCloud2, self.filtered_pointcloud_topic, 10
        )
        if self.enable_image_publish:
            self.left_pub = self.create_publisher(Image, self.left_image_topic, 10)
            self.right_pub = self.create_publisher(Image, self.right_image_topic, 10)
            self.left_info_pub = self.create_publisher(
                CameraInfo, self.left_camera_info_topic, 10
            )
            self.right_info_pub = self.create_publisher(
                CameraInfo, self.right_camera_info_topic, 10
            )
        if self.enable_depth_publish:
            self.depth_pub = self.create_publisher(Image, self.depth_image_topic, 10)
            self.depth_info_pub = self.create_publisher(
                CameraInfo, self.depth_camera_info_topic, 10
            )

        # 内部状态
        self.imu_queue = None
        self.depth_queue = None
        self.left_queue = None
        self.right_queue = None
        self.pipeline = dai.Pipeline()
        self.device_time_base = None

        # 设置管道
        try:
            self.setup_pipeline()
            self.pipeline.start()
            self.get_logger().info(
                f"OAK-D 统一节点启动成功 [IMU: {self.imu_frequency}Hz, "
                f"点云: {self.pointcloud_frequency}Hz]"
            )
        except Exception as e:
            self.get_logger().error(f"管道启动失败: {e}")
            raise

        # 获取相机标定信息
        self.setup_calibration()
        self.setup_fov_boundary_filter()

        # 日志信息
        self.get_logger().info(
            f"深度模式 - 被动立体: {self.enable_passive_stereo}, "
            f"主动立体: {self.enable_active_stereo}"
        )
        if self.enable_active_stereo:
            self.get_logger().info(f"IR强度: {self.ir_intensity}")

        # IMU定时器：高频 (400Hz -> 2.5ms)
        imu_period = 1.0 / self.imu_frequency
        self.imu_timer = self.create_timer(imu_period, self.publish_imu)

        # 点云定时器：低频 (20Hz -> 50ms)
        pc_period = 1.0 / self.pointcloud_frequency
        self.pc_timer = self.create_timer(pc_period, self.publish_pointcloud)

        # 图像定时器
        if self.enable_image_publish:
            image_period = 1.0 / self.image_frequency
            self.image_timer = self.create_timer(image_period, self.publish_images)

    def setup_fov_boundary_filter(self):
        """Configure the frustum boundary filter for point cloud publishing."""
        if self.auto_estimate_fov:
            self.fov_h_deg, self.fov_v_deg = estimate_fov_from_intrinsics(
                self.fx, self.fy, 640, 400, self.cx, self.cy
            )

        self.fov_filter = FOVBoundaryFilter(
            fov_h=float(self.fov_h_deg),
            fov_v=float(self.fov_v_deg),
            margin=float(self.fov_boundary_margin_m),
        )

        self.get_logger().info(
            "FOV边界过滤已配置: "
            f"enabled={self.enable_fov_boundary_filter}, "
            f"auto_estimate_fov={self.auto_estimate_fov}, "
            f"fov_h={self.fov_h_deg:.2f}deg, fov_v={self.fov_v_deg:.2f}deg, "
            f"margin={self.fov_boundary_margin_m:.3f}m"
        )

    def _to_seconds(self, timestamp):
        """Convert DepthAI timestamp objects to floating-point seconds."""
        if timestamp is None:
            return None
        if hasattr(timestamp, "total_seconds"):
            return timestamp.total_seconds()
        try:
            return float(timestamp)
        except (TypeError, ValueError):
            return None

    def _extract_device_time(self, *objects):
        """Return the first available DepthAI device timestamp in seconds."""
        for obj in objects:
            if obj is None:
                continue
            for method_name in ("getTimestampDevice", "getTimestamp"):
                method = getattr(obj, method_name, None)
                if method is None:
                    continue
                try:
                    seconds = self._to_seconds(method())
                except Exception:
                    seconds = None
                if seconds is not None:
                    return seconds
        return None

    def _stamp_from_device_time(self, device_seconds):
        """Map DepthAI monotonic device time onto the ROS clock domain."""
        now = self.get_clock().now().to_msg()
        now_seconds = float(now.sec) + float(now.nanosec) * 1e-9

        if device_seconds is None:
            return now

        if self.device_time_base is None:
            self.device_time_base = (device_seconds, now_seconds)

        base_device, base_ros = self.device_time_base
        stamp_seconds = base_ros + (device_seconds - base_device)
        stamp = Header().stamp
        stamp.sec = int(stamp_seconds)
        stamp.nanosec = int((stamp_seconds - stamp.sec) * 1e9)
        return stamp

    def setup_pipeline(self):
        """Configure the DAI pipeline for IMU and depth."""
        # ============ 配置IMU ============
        imu = self.pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            self.imu_frequency,
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        self.imu_queue = imu.out.createOutputQueue()
        self.get_logger().info(f"IMU管道配置完成: {self.imu_frequency}Hz")

        # ============ 配置深度 ============
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # 主动立体配置
        ir_enabled = False
        if self.enable_active_stereo and hasattr(dai.node, "IRIlluminator"):
            try:
                ir_illuminator = self.pipeline.create(dai.node.IRIlluminator)
                ir_illuminator.setIntensity(self.ir_intensity)
                ir_illuminator.setFrequencyCheckInterval(100)
                ir_enabled = True
                self.get_logger().info(f"IR投影仪已启用，强度: {self.ir_intensity}")
            except Exception as e:
                self.get_logger().warn(f"IR投影仪启用失败: {e}，使用被动立体")

        # 选择预设模式
        if ir_enabled or self.enable_active_stereo:
            if hasattr(dai.node.StereoDepth.PresetMode, "HIGH_DENSITY"):
                preset_mode = dai.node.StereoDepth.PresetMode.HIGH_DENSITY
            elif hasattr(dai.node.StereoDepth.PresetMode, "MEDIUM_DENSITY"):
                preset_mode = dai.node.StereoDepth.PresetMode.MEDIUM_DENSITY
            else:
                preset_mode = dai.node.StereoDepth.PresetMode.FAST_DENSITY
        else:
            if hasattr(dai.node.StereoDepth.PresetMode, "FAST_DENSITY"):
                preset_mode = dai.node.StereoDepth.PresetMode.FAST_DENSITY
            elif hasattr(dai.node.StereoDepth.PresetMode, "MEDIUM_DENSITY"):
                preset_mode = dai.node.StereoDepth.PresetMode.MEDIUM_DENSITY
            else:
                preset_mode = dai.node.StereoDepth.PresetMode.HIGH_DENSITY

        stereo.setDefaultProfilePreset(preset_mode)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        # 硬件滤镜配置
        if hasattr(stereo.initialConfig, "get"):
            config = stereo.initialConfig.get()
        else:
            config = stereo.initialConfig

        pp = getattr(config, "postProcessing", None)
        if pp is not None:
            if hasattr(pp, "medianFilter"):
                pp.medianFilter = (
                    dai.MedianFilter.KERNEL_7x7
                    if self.enable_passive_stereo
                    else dai.MedianFilter.KERNEL_5x5
                )
            spatial = getattr(pp, "spatialFilter", None)
            if spatial is not None and hasattr(spatial, "enable"):
                spatial.enable = self.enable_passive_stereo
                if hasattr(spatial, "holeFillingRadius"):
                    spatial.holeFillingRadius = 2 if self.enable_passive_stereo else 1
            temporal = getattr(pp, "temporalFilter", None)
            if temporal is not None and hasattr(temporal, "enable"):
                temporal.enable = True

        if hasattr(stereo.initialConfig, "set"):
            stereo.initialConfig.set(config)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        if self.enable_image_publish:
            # Publish StereoDepth's rectified outputs instead of the raw
            # wide-FOV mono streams.
            self.left_queue = stereo.rectifiedLeft.createOutputQueue()
            self.right_queue = stereo.rectifiedRight.createOutputQueue()

        self.depth_queue = stereo.depth.createOutputQueue()
        self.get_logger().info("深度管道配置完成")

    def setup_calibration(self):
        """Load camera calibration data."""
        default_intrinsics = {
            "fx": 400.0,
            "fy": 400.0,
            "cx": 320.0,
            "cy": 200.0,
        }
        self.left_intrinsics = default_intrinsics.copy()
        self.right_intrinsics = default_intrinsics.copy()
        self.depth_intrinsics = default_intrinsics.copy()
        self.fx = self.depth_intrinsics["fx"]
        self.fy = self.depth_intrinsics["fy"]
        self.cx = self.depth_intrinsics["cx"]
        self.cy = self.depth_intrinsics["cy"]

        try:
            device = self.pipeline.getDevice()
            if device is not None and hasattr(device, "readCalibration"):
                calibData = device.readCalibration()
                self.left_intrinsics = self._read_camera_intrinsics(
                    calibData, dai.CameraBoardSocket.LEFT, default_intrinsics
                )
                self.right_intrinsics = self._read_camera_intrinsics(
                    calibData, dai.CameraBoardSocket.RIGHT, default_intrinsics
                )
                self.depth_intrinsics = self.right_intrinsics.copy()
                self.fx = self.depth_intrinsics["fx"]
                self.fy = self.depth_intrinsics["fy"]
                self.cx = self.depth_intrinsics["cx"]
                self.cy = self.depth_intrinsics["cy"]
                self.get_logger().info(
                    "标定信息已加载: "
                    f"left_fx={self.left_intrinsics['fx']:.1f}, "
                    f"right_fx={self.right_intrinsics['fx']:.1f}, "
                    f"depth_fx={self.depth_intrinsics['fx']:.1f}"
                )
        except Exception as e:
            self.get_logger().warn(f"标定信息加载失败，使用默认值: {e}")

    def _read_camera_intrinsics(self, calib_data, socket, fallback):
        """Read one camera intrinsic matrix from DepthAI calibration data."""
        try:
            intrinsics = calib_data.getCameraIntrinsics(socket, 640, 400)
            return {
                "fx": float(intrinsics[0][0]),
                "fy": float(intrinsics[1][1]),
                "cx": float(intrinsics[0][2]),
                "cy": float(intrinsics[1][2]),
            }
        except Exception as exc:
            self.get_logger().warn(f"{socket} 标定信息读取失败，使用默认值: {exc}")
            return fallback.copy()

    def publish_imu(self):
        """Publish IMU data."""
        if self.imu_queue is None:
            return

        try:
            imu_data = self.imu_queue.tryGet()
            if imu_data is None:
                return

            for packet in imu_data.packets:
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.frame_id = self.imu_frame_id

                accel_data = getattr(packet, "acceleroMeter", None)
                if accel_data is not None:
                    imu_msg.linear_acceleration.x = float(getattr(accel_data, "x", 0.0))
                    imu_msg.linear_acceleration.y = float(getattr(accel_data, "y", 0.0))
                    imu_msg.linear_acceleration.z = float(getattr(accel_data, "z", 0.0))

                gyro_data = getattr(packet, "gyroscope", None)
                if gyro_data is not None:
                    imu_msg.angular_velocity.x = float(getattr(gyro_data, "x", 0.0))
                    imu_msg.angular_velocity.y = float(getattr(gyro_data, "y", 0.0))
                    imu_msg.angular_velocity.z = float(getattr(gyro_data, "z", 0.0))

                device_seconds = self._extract_device_time(
                    packet, accel_data, gyro_data
                )
                imu_msg.header.stamp = self._stamp_from_device_time(device_seconds)

                imu_msg.linear_acceleration_covariance = [
                    0.01,
                    0.0,
                    0.0,
                    0.0,
                    0.01,
                    0.0,
                    0.0,
                    0.0,
                    0.01,
                ]
                imu_msg.angular_velocity_covariance = [
                    0.001,
                    0.0,
                    0.0,
                    0.0,
                    0.001,
                    0.0,
                    0.0,
                    0.0,
                    0.001,
                ]
                imu_msg.orientation_covariance[0] = -1.0
                self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU数据处理失败: {e}")

    def publish_pointcloud(self):
        """Publish point cloud data."""
        if self.depth_queue is None:
            return

        try:
            inDepth = self.depth_queue.tryGet()
            if inDepth is None:
                return

            depth_frame = inDepth.getFrame()
            stamp = self._stamp_from_device_time(self._extract_device_time(inDepth))

            if self.enable_depth_publish:
                header = Header()
                header.stamp = stamp
                header.frame_id = self.pointcloud_frame_id
                self.depth_pub.publish(self.create_depth_image_msg(depth_frame, header))
                self.depth_info_pub.publish(
                    self.create_camera_info_msg(header, depth_frame.shape, self.depth_intrinsics)
                )

            step = max(int(self.sampling_step), 1)
            depth_down = depth_frame[::step, ::step]

            height, width = depth_down.shape
            u = np.arange(0, width * step, step)
            v = np.arange(0, height * step, step)
            uu, vv = np.meshgrid(u, v)

            border_px = int(np.ceil(max(self.depth_border_crop_px, 0) / step))
            valid_mask = build_depth_filter_mask(
                depth_down,
                self.min_depth,
                self.max_depth,
                border_px=border_px,
                max_depth_jump_mm=self.max_depth_jump_mm,
            )
            z = depth_down[valid_mask] / 1000.0
            x = (uu[valid_mask] - self.cx) * z / self.fx
            y = (vv[valid_mask] - self.cy) * z / self.fy

            points = np.stack((x, y, z), axis=-1).astype(np.float32)

            header = Header()
            header.stamp = stamp
            header.frame_id = self.pointcloud_frame_id

            raw_pc_msg = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(raw_pc_msg)

            filtered_points = points
            if self.enable_fov_boundary_filter and len(points) > 0:
                filtered_points = self.fov_filter.filter_frustum_boundary(
                    points, margin=self.fov_boundary_margin_m
                )

            filtered_pc_msg = pc2.create_cloud_xyz32(header, filtered_points)
            self.filtered_pc_pub.publish(filtered_pc_msg)
        except Exception as e:
            self.get_logger().warn(f"点云发布失败: {e}")

    def create_image_msg(self, cv_frame, frame_id, stamp):
        """Convert a cv2 image to a ROS 2 Image message using native numpy conversion."""
        msg = Image()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.height = cv_frame.shape[0]
        msg.width = cv_frame.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = cv_frame.tobytes()
        return msg

    def create_depth_image_msg(self, depth_frame, header):
        """Create an OpenNI-style uint16 depth image in millimeters."""
        msg = Image()
        msg.header = header
        msg.height = depth_frame.shape[0]
        msg.width = depth_frame.shape[1]
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = depth_frame.astype(np.uint16, copy=False).tobytes()
        return msg

    def create_camera_info_msg(self, header, image_shape, intrinsics, tx=0.0):
        """Publish camera intrinsics for stereo, depth, and RGB-D consumers."""
        msg = CameraInfo()
        msg.header = header
        msg.height = int(image_shape[0])
        msg.width = int(image_shape[1])
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(intrinsics["fx"])
        fy = float(intrinsics["fy"])
        cx = float(intrinsics["cx"])
        cy = float(intrinsics["cy"])
        msg.k = [
            fx,
            0.0,
            cx,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            1.0,
        ]
        msg.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        msg.p = [
            fx,
            0.0,
            cx,
            float(tx),
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return msg

    def publish_images(self):
        """Publish left and right stereo images."""
        if self.left_queue is None or self.right_queue is None:
            return

        try:
            inLeft = self.left_queue.tryGet()
            inRight = self.right_queue.tryGet()

            if inLeft is None or inRight is None:
                return

            device_seconds = self._extract_device_time(inLeft, inRight)
            stamp = self._stamp_from_device_time(device_seconds)

            left_msg = self.create_image_msg(
                inLeft.getCvFrame(), self.left_camera_frame_id, stamp
            )
            right_msg = self.create_image_msg(
                inRight.getCvFrame(), self.right_camera_frame_id, stamp
            )
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
            self.left_info_pub.publish(
                self.create_camera_info_msg(
                    left_msg.header,
                    (left_msg.height, left_msg.width),
                    self.left_intrinsics,
                )
            )
            right_tx = -float(self.right_intrinsics["fx"]) * self.stereo_baseline_m
            self.right_info_pub.publish(
                self.create_camera_info_msg(
                    right_msg.header,
                    (right_msg.height, right_msg.width),
                    self.right_intrinsics,
                    tx=right_tx,
                )
            )
        except Exception as e:
            self.get_logger().warn(f"图像发布失败: {e}")

    def destroy_node(self):
        """Clean up resources before destroying the node."""
        if hasattr(self, "pipeline") and hasattr(self.pipeline, "stop"):
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    """Run the unified OAK-D node."""
    rclpy.init(args=args)
    node = OakDUnifiedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
