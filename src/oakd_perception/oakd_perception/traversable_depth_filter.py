"""Remove locally traversable surfaces from depth before nvblox integration."""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def _rotation_matrix(x, y, z, w):
    """Return the 3x3 rotation matrix for a normalized quaternion."""
    scale = x * x + y * y + z * z + w * w
    if scale <= 1.0e-12:
        return np.eye(3, dtype=np.float32)
    scale = 2.0 / scale
    return np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def filter_traversable_depth(
    depth,
    ray_x,
    ray_y,
    rotation_to_base,
    translation_to_base,
    max_slope_deg=30.0,
    min_surface_height_m=-1.50,
    max_surface_height_m=0.50,
    remove_below_height_m=-0.08,
    max_neighbor_height_jump_m=0.04,
    mask_dilation_pixels=1,
):
    """Return depth with smooth traversable surfaces encoded as invalid zero."""
    output = np.array(depth, dtype=np.float32, copy=True)
    if output.ndim != 2 or min(output.shape) < 3:
        return output

    valid = np.isfinite(output) & (output > 0.0)
    # Invalid Gazebo returns can be +/-inf. Replace them only for intermediate
    # geometry so NumPy does not emit warnings; `valid` still prevents those
    # pixels and their neighborhoods from being classified as traversable.
    projected_depth = np.where(valid, output, 0.0)
    x = ray_x * projected_depth
    y = ray_y * projected_depth
    full_base_x = (
        rotation_to_base[0, 0] * x
        + rotation_to_base[0, 1] * y
        + rotation_to_base[0, 2] * projected_depth
        + translation_to_base[0]
    )
    full_base_y = (
        rotation_to_base[1, 0] * x
        + rotation_to_base[1, 1] * y
        + rotation_to_base[1, 2] * projected_depth
        + translation_to_base[1]
    )
    full_base_height = (
        rotation_to_base[2, 0] * x
        + rotation_to_base[2, 1] * y
        + rotation_to_base[2, 2] * projected_depth
        + translation_to_base[2]
    )

    du_x = x[1:-1, 2:] - x[1:-1, :-2]
    du_y = y[1:-1, 2:] - y[1:-1, :-2]
    du_z = projected_depth[1:-1, 2:] - projected_depth[1:-1, :-2]
    dv_x = x[2:, 1:-1] - x[:-2, 1:-1]
    dv_y = y[2:, 1:-1] - y[:-2, 1:-1]
    dv_z = projected_depth[2:, 1:-1] - projected_depth[:-2, 1:-1]

    normal_x = du_y * dv_z - du_z * dv_y
    normal_y = du_z * dv_x - du_x * dv_z
    normal_z = du_x * dv_y - du_y * dv_x
    normal_norm = np.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
    base_normal_z = (
        rotation_to_base[2, 0] * normal_x
        + rotation_to_base[2, 1] * normal_y
        + rotation_to_base[2, 2] * normal_z
    )

    base_height = full_base_height[1:-1, 1:-1]
    du_height = full_base_height[1:-1, 2:] - full_base_height[1:-1, :-2]
    dv_height = full_base_height[2:, 1:-1] - full_base_height[:-2, 1:-1]
    du_horizontal_distance = np.hypot(
        full_base_x[1:-1, 2:] - full_base_x[1:-1, :-2],
        full_base_y[1:-1, 2:] - full_base_y[1:-1, :-2],
    )
    dv_horizontal_distance = np.hypot(
        full_base_x[2:, 1:-1] - full_base_x[:-2, 1:-1],
        full_base_y[2:, 1:-1] - full_base_y[:-2, 1:-1],
    )
    neighborhood_valid = (
        valid[1:-1, 1:-1]
        & valid[1:-1, :-2]
        & valid[1:-1, 2:]
        & valid[:-2, 1:-1]
        & valid[2:, 1:-1]
    )
    # A fixed height difference incorrectly rejects a valid ramp as its
    # projected pixel spacing grows with range. Compare each pair against the
    # configured traversable slope instead. `max_neighbor_height_jump_m` is
    # only the residual for depth rasterization/TF precision, so a real step
    # still has to fit the same local slope model to be removed.
    max_height_change = (
        math.tan(math.radians(max_slope_deg)) * du_horizontal_distance
        + max_neighbor_height_jump_m
    )
    max_vertical_height_change = (
        math.tan(math.radians(max_slope_deg)) * dv_horizontal_distance
        + max_neighbor_height_jump_m
    )
    smooth = (
        (np.abs(du_height) <= max_height_change)
        & (np.abs(dv_height) <= max_vertical_height_change)
    )
    # Permit tiny floating-point error at the configured slope limit. Without
    # this, a mathematically 30-degree plane can alternate between accepted
    # and rejected pixels after single-precision projection.
    traversable = np.abs(base_normal_z) >= (
        (math.cos(math.radians(max_slope_deg)) - 1.0e-6) * normal_norm
    )
    low_surface = (base_height >= min_surface_height_m) & (
        base_height <= max_surface_height_m
    )
    traversable_candidate = (
        neighborhood_valid
        & smooth
        & traversable
        & low_surface
        & (normal_norm > 1.0e-8)
    )
    remove = traversable_candidate.copy()
    dilation_domain = (
        neighborhood_valid
        & smooth
        & low_surface
        & (traversable | (normal_norm <= 1.0e-8))
    )
    # Fill only holes that are themselves smooth traversable terrain. Restricting
    # dilation to the candidate mask prevents a nearby ramp or floor patch from
    # erasing a step edge or the silhouette of a low obstacle.
    for _ in range(max(0, int(mask_dilation_pixels))):
        padded = np.pad(remove, 1, mode="constant", constant_values=False)
        remove = np.logical_or.reduce(
            [
                padded[row : row + remove.shape[0], column : column + remove.shape[1]]
                for row in range(3)
                for column in range(3)
            ]
        ) & dilation_domain
    # Publish the canonical invalid-depth value expected by the nvblox input
    # contract. With invalid-depth TSDF decay enabled, repeated observations of
    # this pixel reduce the weight of a previously integrated ramp outlier.
    output[1:-1, 1:-1][remove] = 0.0
    # Remove only the one-voxel band immediately above the supporting ground.
    # The previous +0.11 m threshold could erase an entire 16-22 cm obstacle,
    # depending on the camera TF error, before nvblox ever observed it.
    output[
        valid
        & (full_base_height >= min_surface_height_m)
        & (full_base_height <= remove_below_height_m)
    ] = 0.0
    return output


class TraversableDepthFilter(Node):
    """Filter ground and climbable ramps while retaining walls and depth edges."""

    def __init__(self):
        super().__init__("traversable_depth_filter")
        self.declare_parameter("depth_topic", "/rgbd_camera/depth_image")
        self.declare_parameter("camera_info_topic", "/rgbd_camera/camera_info")
        self.declare_parameter("output_topic", "/rgbd_camera/depth_obstacles")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_slope_deg", 30.0)
        self.declare_parameter("min_surface_height_m", -1.50)
        self.declare_parameter("max_surface_height_m", 0.50)
        self.declare_parameter("remove_below_height_m", -0.08)
        self.declare_parameter("max_neighbor_height_jump_m", 0.04)
        self.declare_parameter("mask_dilation_pixels", 1)

        self._camera_info = None
        self._ray_x = None
        self._ray_y = None
        self._rotation = None
        self._translation = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth_callback,
            qos_profile_sensor_data,
        )

    def _camera_info_callback(self, message):
        self._camera_info = message
        columns = np.arange(message.width, dtype=np.float32)
        rows = np.arange(message.height, dtype=np.float32)
        self._ray_x = np.broadcast_to(
            ((columns - message.k[2]) / message.k[0])[None, :],
            (message.height, message.width),
        )
        self._ray_y = np.broadcast_to(
            ((rows - message.k[5]) / message.k[4])[:, None],
            (message.height, message.width),
        )

    def _lookup_transform(self, source_frame):
        if self._rotation is not None:
            return True
        try:
            transform = self._tf_buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            ).transform
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for depth transform: {error}", throttle_duration_sec=2.0
            )
            return False
        q = transform.rotation
        t = transform.translation
        self._rotation = _rotation_matrix(q.x, q.y, q.z, q.w)
        self._translation = np.asarray([t.x, t.y, t.z], dtype=np.float32)
        return True

    def _depth_callback(self, message):
        if message.encoding != "32FC1" or self._camera_info is None:
            return
        if message.width != self._camera_info.width or message.height != self._camera_info.height:
            return
        if not self._lookup_transform(message.header.frame_id):
            return
        depth = np.frombuffer(message.data, dtype=np.float32).reshape(
            message.height, message.width
        )
        filtered = filter_traversable_depth(
            depth,
            self._ray_x,
            self._ray_y,
            self._rotation,
            self._translation,
            self.get_parameter("max_slope_deg").value,
            self.get_parameter("min_surface_height_m").value,
            self.get_parameter("max_surface_height_m").value,
            self.get_parameter("remove_below_height_m").value,
            self.get_parameter("max_neighbor_height_jump_m").value,
            self.get_parameter("mask_dilation_pixels").value,
        )
        output = Image()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        output.encoding = message.encoding
        output.is_bigendian = message.is_bigendian
        output.step = message.width * 4
        output.data = filtered.tobytes()
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TraversableDepthFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
