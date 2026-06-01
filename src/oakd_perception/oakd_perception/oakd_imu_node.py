import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class OakDImuNode(Node):
    def __init__(self):
        super().__init__("oakd_imu_node")

        self.declare_parameter("imu_frequency", 400)
        self.declare_parameter("gyro_full_scale", "gyroscope_2000_dps")
        self.declare_parameter("accel_full_scale", "accelerometer_4g")
        self.declare_parameter("topic_name", "/oakd/imu/raw")
        self.declare_parameter("frame_id", "oakd_imu_link")
        self.declare_parameter("imu_axis_mode", "raw")

        self.imu_frequency = self.get_parameter("imu_frequency").value
        self.gyro_full_scale = self.get_parameter("gyro_full_scale").value
        self.accel_full_scale = self.get_parameter("accel_full_scale").value
        self.topic_name = self.get_parameter("topic_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.imu_axis_mode = self.get_parameter("imu_axis_mode").value
        self._warned_unknown_imu_axis_mode = False

        self.imu_pub = self.create_publisher(Imu, self.topic_name, 10)
        self.imu_queue = None
        self.imu_node = None
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()

        try:
            self.pipeline.start()
            self.get_logger().info(
                f"OAK-D IMU raw node started [frequency: {self.imu_frequency}Hz, "
                f"gyro: {self.gyro_full_scale}, accel: {self.accel_full_scale}]"
            )
            self.get_logger().info(f"IMU output topic: {self.topic_name}")
            self.get_logger().info(f"IMU frame_id: {self.frame_id}")
            self.get_logger().info(f"IMU axis mode: {self.imu_axis_mode}")
        except Exception as e:
            self.get_logger().error(f"Failed to start IMU pipeline: {e}")
            return

        self.timer = self.create_timer(0.01, self.publish_imu)

    def _convert_imu_vector_to_ros(self, x_raw, y_raw, z_raw):
        if self.imu_axis_mode == "raw":
            return x_raw, y_raw, z_raw
        if self.imu_axis_mode == "swap_yaw_roll_invert_pitch":
            # Diagnostic-only host-side remap for checking raw IMU fields.
            # Do not use this with VIO unless the camera-to-IMU TF is updated
            # to the same frame convention.
            return z_raw, -y_raw, x_raw
        if self.imu_axis_mode == "oakd_to_ros":
            return z_raw, -y_raw, -x_raw
        if not self._warned_unknown_imu_axis_mode:
            self.get_logger().warn(
                f"Unknown imu_axis_mode={self.imu_axis_mode!r}; falling back to raw"
            )
            self._warned_unknown_imu_axis_mode = True
        return x_raw, y_raw, z_raw

    def setup_pipeline(self):
        try:
            imu = self.pipeline.create(dai.node.IMU)
            self.imu_node = imu
            imu.enableIMUSensor(
                [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
                self.imu_frequency,
            )
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
            self.imu_queue = imu.out.createOutputQueue()
            self.get_logger().info(f"IMU pipeline configured: {self.imu_frequency}Hz")
        except Exception as e:
            self.get_logger().error(f"Failed to configure IMU pipeline: {e}")

    def publish_imu(self):
        if self.imu_queue is None:
            return

        try:
            imu_data = self.imu_queue.tryGet()
            if imu_data is None:
                return

            for packet in imu_data.packets:
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id

                accel_data = getattr(packet, "acceleroMeter", None)
                if accel_data is not None:
                    ax, ay, az = self._convert_imu_vector_to_ros(
                        float(getattr(accel_data, "x", 0.0)),
                        float(getattr(accel_data, "y", 0.0)),
                        float(getattr(accel_data, "z", 0.0)),
                    )
                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az

                gyro_data = getattr(packet, "gyroscope", None)
                if gyro_data is not None:
                    wx, wy, wz = self._convert_imu_vector_to_ros(
                        float(getattr(gyro_data, "x", 0.0)),
                        float(getattr(gyro_data, "y", 0.0)),
                        float(getattr(gyro_data, "z", 0.0)),
                    )
                    imu_msg.angular_velocity.x = wx
                    imu_msg.angular_velocity.y = wy
                    imu_msg.angular_velocity.z = wz

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
            self.get_logger().warn(f"Failed to process IMU data: {e}")

    def destroy_node(self):
        if hasattr(self, "pipeline") and hasattr(self.pipeline, "stop"):
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakDImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IMU raw node...")


if __name__ == "__main__":
    main()
