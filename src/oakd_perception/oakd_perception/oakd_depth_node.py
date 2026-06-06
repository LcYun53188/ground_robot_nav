"""Standalone OAK-D point cloud node.

Quick RViz check for the filtered point cloud:

1. Source the workspace:
    source install/setup.bash
2. Start the standalone node:
    ros2 run oakd_perception oakd_depth_node
3. Open RViz:
    rviz2
4. Set Fixed Frame to `oakd_imu_link`.
5. Add a PointCloud2 display and set Topic to `/oakd/points_filtered`.
6. Optionally add another PointCloud2 display for `/oakd/points` to compare
    raw and filtered results.

The node publishes both raw and filtered clouds; the filtered topic is the
recommended one for visualization and downstream consumption.


source install/setup.bash
ros2 run oakd_perception oakd_depth_node
打开 rviz2
Fixed Frame 设为 oakd_imu_link
点云话题设为 /oakd/points_filtered
也可以额外看 /oakd/points 对比原始结果


"""

import depthai as dai
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from oakd_perception.fov_boundary_filter import (
    FOVBoundaryFilter,
    build_depth_filter_mask,
    estimate_fov_from_intrinsics,
)


class OakDPointCloudNode(Node):
    def __init__(self):
        super().__init__("oakd_pointcloud_node")

        # ============ 深度模式开关配置 ============
        self.declare_parameter("enable_passive_stereo", True)
        self.declare_parameter("enable_active_stereo", False)
        self.declare_parameter("ir_intensity", 1600)

        # ============ 点云过滤参数配置 ============
        self.declare_parameter("sampling_step", 2)  # 采样间隔
        self.declare_parameter("min_depth", 200)  # 最小深度(mm)
        self.declare_parameter("max_depth", 15000)  # 最大深度(mm)
        self.declare_parameter("depth_border_crop_px", 8)
        self.declare_parameter("max_depth_jump_mm", 350)
        self.declare_parameter("filtered_pointcloud_topic", "/oakd/points_filtered")
        self.declare_parameter("enable_fov_boundary_filter", True)
        self.declare_parameter("auto_estimate_fov", True)
        self.declare_parameter("fov_h_deg", 72.0)
        self.declare_parameter("fov_v_deg", 53.0)
        self.declare_parameter("fov_boundary_margin_m", 0.15)

        self.enable_passive_stereo = self.get_parameter("enable_passive_stereo").value
        self.enable_active_stereo = self.get_parameter("enable_active_stereo").value
        self.ir_intensity = self.get_parameter("ir_intensity").value
        self.sampling_step = self.get_parameter("sampling_step").value
        self.min_depth = self.get_parameter("min_depth").value
        self.max_depth = self.get_parameter("max_depth").value
        self.depth_border_crop_px = self.get_parameter("depth_border_crop_px").value
        self.max_depth_jump_mm = self.get_parameter("max_depth_jump_mm").value
        self.filtered_pointcloud_topic = self.get_parameter(
            "filtered_pointcloud_topic"
        ).value
        self.enable_fov_boundary_filter = self.get_parameter(
            "enable_fov_boundary_filter"
        ).value
        self.auto_estimate_fov = self.get_parameter("auto_estimate_fov").value
        self.fov_h_deg = self.get_parameter("fov_h_deg").value
        self.fov_v_deg = self.get_parameter("fov_v_deg").value
        self.fov_boundary_margin_m = self.get_parameter(
            "fov_boundary_margin_m"
        ).value

        self.get_logger().info(
            f"Passive Stereo: {'ON' if self.enable_passive_stereo else 'OFF'}"
        )
        self.get_logger().info(
            f"Active Stereo:  {'ON' if self.enable_active_stereo else 'OFF'}"
        )
        if self.enable_active_stereo:
            self.get_logger().info(f"IR Intensity: {self.ir_intensity}")

        self.pc_pub = self.create_publisher(PointCloud2, "/oakd/points", 10)
        self.filtered_pc_pub = self.create_publisher(
            PointCloud2, self.filtered_pointcloud_topic, 10
        )
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()
        self.pipeline.start()

        self.fx = 400.0
        self.fy = 400.0
        self.cx = 320.0
        self.cy = 200.0
        device = getattr(self.pipeline, "device", None)
        if device is not None and hasattr(device, "readCalibration"):
            try:
                calibData = device.readCalibration()
                self.intrinsics = calibData.getCameraIntrinsics(
                    dai.CameraBoardSocket.RIGHT, 640, 400
                )
                self.fx = self.intrinsics[0][0]
                self.fy = self.intrinsics[1][1]
                self.cx = self.intrinsics[0][2]
                self.cy = self.intrinsics[1][2]
            except Exception:
                self.get_logger().warn(
                    "Failed to get calibration, using fallback intrinsics."
                )
        else:
            self.get_logger().warn(
                "DepthAI calibration API unavailable, using fallback intrinsics."
            )

        self.setup_fov_boundary_filter()

        mode_info = []
        if self.enable_passive_stereo:
            mode_info.append("被动立体")
        if self.enable_active_stereo and hasattr(dai.node, "IRIlluminator"):
            mode_info.append(f"主动立体(IR={self.ir_intensity})")
        mode_str = " + ".join(mode_info) if mode_info else "被动立体"

        self.get_logger().info(
            f"OAK-D 点云驱动节点已启动 [深度模式: {mode_str}]，正在发布点云..."
        )
        self.timer = self.create_timer(0.05, self.publish_pointcloud)

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

    def setup_pipeline(self):
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # ============ 主动立体深度配置 ============
        # 尝试启用 IR 投影仪（如果支持）
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
        self.depth_queue = stereo.depth.createOutputQueue()

    def publish_pointcloud(self):
        inDepth = self.depth_queue.tryGet()
        if inDepth is None:
            return

        depth_frame = inDepth.getFrame()

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
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oakd_imu_link"

        raw_pc_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(raw_pc_msg)

        filtered_points = points
        if self.enable_fov_boundary_filter and len(points) > 0:
            filtered_points = self.fov_filter.filter_frustum_boundary(
                points, margin=self.fov_boundary_margin_m
            )

        filtered_pc_msg = pc2.create_cloud_xyz32(header, filtered_points)
        self.filtered_pc_pub.publish(filtered_pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakDPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "pipeline") and hasattr(node.pipeline, "stop"):
            node.pipeline.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
