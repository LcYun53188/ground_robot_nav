"""Detect non-traversable terrain edges from a registered depth image."""

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from sensor_msgs_py import point_cloud2

from std_msgs.msg import Header

from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CliffDetectionConfig:
    """Geometry thresholds for the local elevation-grid detector."""

    resolution: float = 0.05
    min_drop_height: float = 0.05
    max_traversable_slope_deg: float = 30.0
    height_tolerance: float = 0.01
    min_lower_neighbors: int = 2


def terrain_height_mask(
    points_xyz: np.ndarray,
    ground_z: float,
    max_slope_deg: float,
    upper_tolerance: float,
    max_drop: float,
    max_height_change: float,
) -> np.ndarray:
    """Select terrain around a slope-aware height envelope."""
    slope_tangent = math.tan(math.radians(max_slope_deg))
    forward_distance = np.maximum(points_xyz[:, 0], 0.0)
    slope_height = np.minimum(
        slope_tangent * forward_distance, max_height_change
    )
    return (
        (points_xyz[:, 2] <= ground_z + slope_height + upper_tolerance)
        & (points_xyz[:, 2] >= ground_z - slope_height - max_drop)
    )


def depth_discontinuity_mask(
    depth: np.ndarray,
    min_depth_jump: float,
    min_support_pixels: int,
    detect_missing_depth: bool,
    points_xyz: np.ndarray | None = None,
    max_traversable_slope_deg: float = 90.0,
    height_tolerance: float = 0.0,
) -> np.ndarray:
    """Mark supported depth edges that cannot be explained by terrain slope.

    A fixed optical-depth jump is not a terrain discontinuity: a continuous
    ramp viewed at a grazing angle may have a large jump between adjacent
    pixels. When base-frame points are supplied, retain a depth edge only if
    its local height change exceeds the configured traversable-slope model.
    """
    valid = np.isfinite(depth) & (depth > 0.0)
    raw_edges = np.zeros(depth.shape, dtype=bool)
    use_geometry = points_xyz is not None
    if use_geometry and points_xyz.shape != depth.shape + (3,):
        raise ValueError('points_xyz must have shape depth.shape + (3,)')
    slope_tangent = math.tan(math.radians(max_traversable_slope_deg))

    def add_pair(
        center_slice: Tuple[slice, slice],
        neighbor_slice: Tuple[slice, slice],
    ) -> None:
        center = depth[center_slice]
        neighbor = depth[neighbor_slice]
        center_valid = valid[center_slice]
        neighbor_valid = valid[neighbor_slice]
        farther = neighbor_valid & (
            neighbor - center >= min_depth_jump
        )
        if use_geometry:
            center_points = points_xyz[center_slice]
            neighbor_points = points_xyz[neighbor_slice]
            horizontal_distance = np.hypot(
                neighbor_points[..., 0] - center_points[..., 0],
                neighbor_points[..., 1] - center_points[..., 1],
            )
            height_change = np.abs(
                neighbor_points[..., 2] - center_points[..., 2]
            )
            farther &= height_change > (
                slope_tangent * horizontal_distance + height_tolerance
            )
        if detect_missing_depth:
            farther |= ~neighbor_valid
        raw_edges[center_slice] |= center_valid & farther

    add_pair((slice(None), slice(None, -1)),
             (slice(None), slice(1, None)))
    add_pair((slice(None), slice(1, None)),
             (slice(None), slice(None, -1)))
    add_pair((slice(None, -1), slice(None)),
             (slice(1, None), slice(None)))
    add_pair((slice(1, None), slice(None)),
             (slice(None, -1), slice(None)))

    if raw_edges.shape[0] > 2 and raw_edges.shape[1] > 2:
        raw_edges[[0, -1], :] = False
        raw_edges[:, [0, -1]] = False

    support = np.zeros(depth.shape, dtype=np.uint8)
    padded = np.pad(raw_edges, 1, mode='constant')
    for row_offset in range(3):
        for col_offset in range(3):
            support += padded[
                row_offset:row_offset + depth.shape[0],
                col_offset:col_offset + depth.shape[1],
            ]
    return raw_edges & (support >= max(1, min_support_pixels))


def elevation_grid_points(points_xyz: np.ndarray,
                          resolution: float) -> np.ndarray:
    """Return the highest observed terrain point in each horizontal cell."""
    if points_xyz.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    cells: Dict[Tuple[int, int], np.ndarray] = {}
    for point in points_xyz:
        key = (
            int(math.floor(float(point[0]) / resolution)),
            int(math.floor(float(point[1]) / resolution)),
        )
        previous = cells.get(key)
        if previous is None or point[2] > previous[2]:
            cells[key] = point
    return np.asarray(list(cells.values()), dtype=np.float32)


def detect_cliff_edges(points_xyz: np.ndarray,
                       config: CliffDetectionConfig) -> np.ndarray:
    """Return upper cells neighboring a non-traversable terrain drop."""
    if points_xyz.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    grid_points = elevation_grid_points(points_xyz, config.resolution)
    cells: Dict[Tuple[int, int], np.ndarray] = {}
    for point in grid_points:
        key = (
            int(math.floor(float(point[0]) / config.resolution)),
            int(math.floor(float(point[1]) / config.resolution)),
        )
        previous = cells.get(key)
        if previous is None or point[2] > previous[2]:
            cells[key] = point

    slope_tangent = math.tan(
        math.radians(config.max_traversable_slope_deg)
    )
    neighbor_offsets = [
        (dx, dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if dx != 0 or dy != 0
    ]
    edge_points = []
    comparison_tolerance_m = 1.0e-5
    for key, upper_point in cells.items():
        lower_neighbors = 0
        for dx, dy in neighbor_offsets:
            lower_point = cells.get((key[0] + dx, key[1] + dy))
            if lower_point is None:
                continue
            measured_distance = math.hypot(
                float(lower_point[0] - upper_point[0]),
                float(lower_point[1] - upper_point[1]),
            )
            if measured_distance <= 1e-4:
                continue
            # All eight cells touch the current cell. Use one grid interval
            # for the slope limit so a straight step edge receives support
            # from its two diagonal neighbors as well as the direct neighbor.
            horizontal_distance = min(measured_distance, config.resolution)
            drop_height = float(upper_point[2] - lower_point[2])
            allowed_height_change = max(
                config.min_drop_height,
                slope_tangent * horizontal_distance + config.height_tolerance,
            )
            if drop_height + comparison_tolerance_m >= allowed_height_change:
                lower_neighbors += 1
        if lower_neighbors >= config.min_lower_neighbors:
            edge_points.append(upper_point)

    if not edge_points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(edge_points, dtype=np.float32)


def _rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a normalized quaternion into a 3x3 rotation matrix."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float32,
    )


class CliffDetector(Node):
    """Convert depth into a conservative point cloud of descending edges."""

    def __init__(self) -> None:
        """Initialize subscriptions, TF lookup, and detector parameters."""
        super().__init__('oakd_cliff_detector')
        self.declare_parameter('depth_topic', '/oakd/depth/image')
        self.declare_parameter(
            'camera_info_topic', '/oakd/depth/camera_info'
        )
        self.declare_parameter('output_topic', '/perception/cliff_points')
        self.declare_parameter(
            'clear_output_topic', '/perception/cliff_clear_points'
        )
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('pixel_stride', 4)
        self.declare_parameter('enable_depth_edge_detection', True)
        self.declare_parameter('min_depth_jump_m', 0.06)
        self.declare_parameter('min_depth_edge_support_pixels', 3)
        self.declare_parameter('detect_missing_depth_edges', True)
        self.declare_parameter('min_range_m', 0.18)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('lateral_range_m', 2.0)
        self.declare_parameter('expected_ground_z_m', -0.11)
        self.declare_parameter('ground_search_tolerance_m', 0.18)
        self.declare_parameter('max_ground_height_adjustment_m', 0.03)
        self.declare_parameter('ground_estimation_max_range_m', 1.2)
        self.declare_parameter('ground_estimation_half_width_m', 0.6)
        self.declare_parameter('ground_upper_tolerance_m', 0.08)
        self.declare_parameter('max_detectable_drop_m', 0.50)
        self.declare_parameter('max_terrain_height_change_m', 0.75)
        self.declare_parameter('grid_resolution_m', 0.05)
        self.declare_parameter('min_drop_height_m', 0.05)
        self.declare_parameter('max_traversable_slope_deg', 30.0)
        self.declare_parameter('height_tolerance_m', 0.01)
        self.declare_parameter('min_lower_neighbors', 2)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self._camera_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self._clear_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('clear_output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self._last_tf_warning_ns = 0

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _decode_depth(self, msg: Image) -> np.ndarray:
        if msg.encoding in ('16UC1', 'mono16'):
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            scale = 0.001
        elif msg.encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            scale = 1.0
        else:
            raise ValueError(f'unsupported depth encoding: {msg.encoding}')
        row_width = msg.step // dtype.itemsize
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, row_width
        )
        return depth[:, : msg.width].astype(np.float32) * scale

    def _project_depth(
        self, depth: np.ndarray, info: CameraInfo
    ) -> np.ndarray:
        stride = max(1, int(self.get_parameter('pixel_stride').value))
        sampled = depth[::stride, ::stride]
        rows, cols = np.indices(sampled.shape, dtype=np.float32)
        u = cols * stride
        v = rows * stride
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError('camera intrinsics are not valid yet')
        valid = np.isfinite(sampled) & (sampled > 0.0)
        z = sampled[valid]
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy
        return np.column_stack((x, y, z)).astype(np.float32, copy=False)

    def _project_depth_edges(
        self, depth: np.ndarray, info: CameraInfo, transform
    ) -> np.ndarray:
        if not bool(
            self.get_parameter('enable_depth_edge_detection').value
        ):
            return np.empty((0, 3), dtype=np.float32)
        stride = max(1, int(self.get_parameter('pixel_stride').value))
        sampled = depth[::stride, ::stride]
        rows, cols = np.indices(sampled.shape, dtype=np.float32)
        u = cols * stride
        v = rows * stride
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError('camera intrinsics are not valid yet')
        valid = np.isfinite(sampled) & (sampled > 0.0)
        safe_depth = np.where(valid, sampled, 0.0)
        camera_points = np.stack((
            (u - cx) * safe_depth / fx,
            (v - cy) * safe_depth / fy,
            safe_depth,
        ), axis=-1)
        base_points = self._transform_points(
            camera_points.reshape((-1, 3)), transform
        ).reshape(camera_points.shape)
        edge_mask = depth_discontinuity_mask(
            sampled,
            float(self.get_parameter('min_depth_jump_m').value),
            int(
                self.get_parameter(
                    'min_depth_edge_support_pixels'
                ).value
            ),
            bool(
                self.get_parameter('detect_missing_depth_edges').value
            ),
            base_points,
            float(
                self.get_parameter('max_traversable_slope_deg').value
            ),
            float(self.get_parameter('height_tolerance_m').value),
        )
        return base_points[edge_mask]
        )

    def _transform_points(self, points: np.ndarray, transform) -> np.ndarray:
        q = transform.transform.rotation
        t = transform.transform.translation
        rotation = _rotation_matrix(q.x, q.y, q.z, q.w)
        translation = np.asarray([t.x, t.y, t.z], dtype=np.float32)
        return points @ rotation.T + translation

    def _terrain_candidates(self, points: np.ndarray) -> np.ndarray:
        min_range = float(self.get_parameter('min_range_m').value)
        max_range = float(self.get_parameter('max_range_m').value)
        lateral_range = float(self.get_parameter('lateral_range_m').value)
        spatial = (
            (points[:, 0] >= min_range)
            & (points[:, 0] <= max_range)
            & (np.abs(points[:, 1]) <= lateral_range)
        )
        points = points[spatial]
        if points.size == 0:
            return points

        expected_ground = float(
            self.get_parameter('expected_ground_z_m').value
        )
        search_tolerance = float(
            self.get_parameter('ground_search_tolerance_m').value
        )
        estimation_range = float(
            self.get_parameter('ground_estimation_max_range_m').value
        )
        near = points[:, 0] <= estimation_range
        near &= np.abs(points[:, 1]) <= float(
            self.get_parameter('ground_estimation_half_width_m').value
        )
        near &= np.abs(points[:, 2] - expected_ground) <= search_tolerance
        if np.any(near):
            estimated_ground = float(np.median(points[near, 2]))
            max_adjustment = float(
                self.get_parameter(
                    'max_ground_height_adjustment_m'
                ).value
            )
            ground_z = float(np.clip(
                estimated_ground,
                expected_ground - max_adjustment,
                expected_ground + max_adjustment,
            ))
        else:
            ground_z = expected_ground
        upper_tolerance = float(
            self.get_parameter('ground_upper_tolerance_m').value
        )
        max_drop = float(
            self.get_parameter('max_detectable_drop_m').value
        )
        terrain = terrain_height_mask(
            points,
            ground_z,
            float(
                self.get_parameter('max_traversable_slope_deg').value
            ),
            upper_tolerance,
            max_drop,
            float(
                self.get_parameter('max_terrain_height_change_m').value
            ),
        )
        return points[terrain]

    def _configuration(self) -> CliffDetectionConfig:
        return CliffDetectionConfig(
            resolution=float(
                self.get_parameter('grid_resolution_m').value
            ),
            min_drop_height=float(
                self.get_parameter('min_drop_height_m').value
            ),
            max_traversable_slope_deg=float(
                self.get_parameter('max_traversable_slope_deg').value
            ),
            height_tolerance=float(
                self.get_parameter('height_tolerance_m').value
            ),
            min_lower_neighbors=int(
                self.get_parameter('min_lower_neighbors').value
            ),
        )

    def _lookup_transform(self, target_frame, source_frame, stamp):
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(
                seconds=float(
                    self.get_parameter('tf_timeout_sec').value
                )
            ),
        )

    def _warn_skipped_frame(self, source: str, exc: Exception) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warning_ns > 2_000_000_000:
            self.get_logger().warning(
                f'cliff detector skipped {source} frame: {exc}'
            )
            self._last_tf_warning_ns = now_ns

    def _detect_and_publish(
        self,
        points: np.ndarray,
        stamp,
        target_frame: str,
        depth_edge_points: np.ndarray,
    ) -> None:
        terrain_points = self._terrain_candidates(points)
        config = self._configuration()
        surface_points = elevation_grid_points(
            terrain_points, config.resolution
        )
        edges = detect_cliff_edges(surface_points, config)
        edge_candidates = self._terrain_candidates(depth_edge_points)
        if edge_candidates.size:
            edges = elevation_grid_points(
                np.vstack((edges, edge_candidates)), config.resolution
            )

        header = Header()
        header.stamp = stamp
        header.frame_id = target_frame
        cloud = point_cloud2.create_cloud_xyz32(header, edges.tolist())
        self._publisher.publish(cloud)
        clear_cloud = point_cloud2.create_cloud_xyz32(
            header, surface_points.tolist()
        )
        self._clear_publisher.publish(clear_cloud)

    def _depth_callback(self, msg: Image) -> None:
        if self._camera_info is None:
            return
        target_frame = str(self.get_parameter('target_frame').value)
        source_frame = msg.header.frame_id or self._camera_info.header.frame_id
        try:
            transform = self._lookup_transform(
                target_frame, source_frame, msg.header.stamp
            )
            depth = self._decode_depth(msg)
            camera_points = self._project_depth(depth, self._camera_info)
            base_edge_points = self._project_depth_edges(
                depth, self._camera_info, transform
            )
            base_points = self._transform_points(camera_points, transform)
            self._detect_and_publish(
                base_points,
                msg.header.stamp,
                target_frame,
                base_edge_points,
            )
        except (TransformException, ValueError) as exc:
            self._warn_skipped_frame('depth', exc)


def main(args=None) -> None:
    """Run the cliff detector node."""
    rclpy.init(args=args)
    node = CliffDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
