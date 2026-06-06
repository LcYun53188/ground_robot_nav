#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <depthai/depthai.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

using namespace std::chrono_literals;

struct Intrinsics
{
  double fx{400.0};
  double fy{400.0};
  double cx{320.0};
  double cy{200.0};
};

double timestampToSeconds(const std::chrono::steady_clock::time_point & stamp)
{
  return std::chrono::duration<double>(stamp.time_since_epoch()).count();
}

}  // namespace

class OakDUnifiedNode : public rclcpp::Node
{
public:
  OakDUnifiedNode()
  : Node("oakd_unified_node")
  {
    declareParameters();
    loadParameters();
    setupPublishers();
    setupPipeline();
    setupCalibration();
    setupFovFilter();

    imu_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, static_cast<double>(imu_frequency_))),
      std::bind(&OakDUnifiedNode::publishImu, this));

    if (enable_depth_publish_ || enable_pointcloud_publish_) {
      pointcloud_timer_ = create_wall_timer(
        std::chrono::duration<double>(
          1.0 / std::max(1.0, static_cast<double>(pointcloud_frequency_))),
        std::bind(&OakDUnifiedNode::publishDepthAndPointcloud, this));
    }

    if (enable_image_publish_) {
      image_timer_ = create_wall_timer(
        std::chrono::duration<double>(
          1.0 / std::max(1.0, static_cast<double>(image_poll_frequency_))),
        std::bind(&OakDUnifiedNode::publishImages, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "OAK-D C++ unified node started [IMU: %dHz, images: %dHz, depth: %s, pointcloud: %s]",
      imu_frequency_, image_frequency_, enable_depth_publish_ ? "true" : "false",
      enable_pointcloud_publish_ ? "true" : "false");
  }

private:
  void declareParameters()
  {
    declare_parameter("imu_frequency", 400);
    declare_parameter("gyro_full_scale", "gyroscope_2000_dps");
    declare_parameter("accel_full_scale", "accelerometer_4g");
    declare_parameter("imu_topic_name", "/oakd/imu/raw");
    declare_parameter("imu_frame_id", "oakd_imu_link");
    declare_parameter("imu_axis_mode", "raw");

    declare_parameter("enable_passive_stereo", true);
    declare_parameter("enable_active_stereo", false);
    declare_parameter("ir_intensity", 1600);
    declare_parameter("stereo_quality_mode", "auto");

    declare_parameter("pointcloud_frequency", 20);
    declare_parameter("enable_pointcloud_publish", true);
    declare_parameter("pointcloud_topic", "/oakd/points");
    declare_parameter("filtered_pointcloud_topic", "/oakd/points_filtered");
    declare_parameter("pointcloud_frame_id", "oakd_imu_link");
    declare_parameter("sampling_step", 2);
    declare_parameter("min_depth", 200);
    declare_parameter("max_depth", 15000);
    declare_parameter("depth_border_crop_px", 8);
    declare_parameter("max_depth_jump_mm", 350);
    declare_parameter("enable_fov_boundary_filter", true);
    declare_parameter("auto_estimate_fov", true);
    declare_parameter("fov_h_deg", 72.0);
    declare_parameter("fov_v_deg", 53.0);
    declare_parameter("fov_boundary_margin_m", 0.15);

    declare_parameter("enable_image_publish", true);
    declare_parameter("enable_depth_publish", true);
    declare_parameter("left_image_topic", "/oakd/left/image_raw");
    declare_parameter("right_image_topic", "/oakd/right/image_raw");
    declare_parameter("left_camera_info_topic", "/oakd/left/camera_info");
    declare_parameter("right_camera_info_topic", "/oakd/right/camera_info");
    declare_parameter("depth_image_topic", "/oakd/depth/image");
    declare_parameter("depth_camera_info_topic", "/oakd/depth/camera_info");
    declare_parameter("left_camera_frame_id", "oakd_left_camera_optical_frame");
    declare_parameter("right_camera_frame_id", "oakd_right_camera_optical_frame");
    declare_parameter("stereo_baseline_m", 0.075);
    declare_parameter("image_frequency", 25);
    declare_parameter("image_poll_frequency", 75);
    declare_parameter("image_queue_size", 2);
    declare_parameter("image_pair_max_dt_ms", 8.0);
    declare_parameter("image_output_mode", "rectified");
    declare_parameter("image_qos_depth", 4);
    declare_parameter("image_publish_order", "left_first");
    declare_parameter("image_inter_publish_delay_ms", 1.0);
  }

  void loadParameters()
  {
    imu_frequency_ = get_parameter("imu_frequency").as_int();
    imu_topic_name_ = get_parameter("imu_topic_name").as_string();
    imu_frame_id_ = get_parameter("imu_frame_id").as_string();
    imu_axis_mode_ = get_parameter("imu_axis_mode").as_string();

    enable_passive_stereo_ = get_parameter("enable_passive_stereo").as_bool();
    enable_active_stereo_ = get_parameter("enable_active_stereo").as_bool();
    ir_intensity_ = get_parameter("ir_intensity").as_int();
    stereo_quality_mode_ = get_parameter("stereo_quality_mode").as_string();
    std::transform(
      stereo_quality_mode_.begin(), stereo_quality_mode_.end(), stereo_quality_mode_.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    pointcloud_frequency_ = get_parameter("pointcloud_frequency").as_int();
    enable_pointcloud_publish_ = get_parameter("enable_pointcloud_publish").as_bool();
    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    filtered_pointcloud_topic_ = get_parameter("filtered_pointcloud_topic").as_string();
    pointcloud_frame_id_ = get_parameter("pointcloud_frame_id").as_string();
    sampling_step_ = std::max(1, static_cast<int>(get_parameter("sampling_step").as_int()));
    min_depth_ = get_parameter("min_depth").as_int();
    max_depth_ = get_parameter("max_depth").as_int();
    depth_border_crop_px_ = get_parameter("depth_border_crop_px").as_int();
    max_depth_jump_mm_ = get_parameter("max_depth_jump_mm").as_int();
    enable_fov_boundary_filter_ = get_parameter("enable_fov_boundary_filter").as_bool();
    auto_estimate_fov_ = get_parameter("auto_estimate_fov").as_bool();
    fov_h_deg_ = get_parameter("fov_h_deg").as_double();
    fov_v_deg_ = get_parameter("fov_v_deg").as_double();
    fov_boundary_margin_m_ = get_parameter("fov_boundary_margin_m").as_double();

    enable_image_publish_ = get_parameter("enable_image_publish").as_bool();
    enable_depth_publish_ = get_parameter("enable_depth_publish").as_bool();
    left_image_topic_ = get_parameter("left_image_topic").as_string();
    right_image_topic_ = get_parameter("right_image_topic").as_string();
    left_camera_info_topic_ = get_parameter("left_camera_info_topic").as_string();
    right_camera_info_topic_ = get_parameter("right_camera_info_topic").as_string();
    depth_image_topic_ = get_parameter("depth_image_topic").as_string();
    depth_camera_info_topic_ = get_parameter("depth_camera_info_topic").as_string();
    left_camera_frame_id_ = get_parameter("left_camera_frame_id").as_string();
    right_camera_frame_id_ = get_parameter("right_camera_frame_id").as_string();
    stereo_baseline_m_ = get_parameter("stereo_baseline_m").as_double();
    image_frequency_ = get_parameter("image_frequency").as_int();
    image_poll_frequency_ = get_parameter("image_poll_frequency").as_int();
    image_queue_size_ = std::max(1, static_cast<int>(get_parameter("image_queue_size").as_int()));
    image_pair_max_dt_ms_ = get_parameter("image_pair_max_dt_ms").as_double();
    image_output_mode_ = get_parameter("image_output_mode").as_string();
    image_qos_depth_ = std::max(1, static_cast<int>(get_parameter("image_qos_depth").as_int()));
    image_publish_order_ = get_parameter("image_publish_order").as_string();
    image_inter_publish_delay_ms_ = std::max(0.0, get_parameter("image_inter_publish_delay_ms").as_double());

    if (image_output_mode_ != "rectified" && image_output_mode_ != "mono") {
      RCLCPP_WARN(get_logger(), "Unknown image_output_mode=%s, using rectified", image_output_mode_.c_str());
      image_output_mode_ = "rectified";
    }
    if (image_publish_order_ != "left_first" && image_publish_order_ != "right_first") {
      RCLCPP_WARN(get_logger(), "Unknown image_publish_order=%s, using left_first", image_publish_order_.c_str());
      image_publish_order_ = "left_first";
    }
  }

  void setupPublishers()
  {
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    auto image_qos = rclcpp::SensorDataQoS().keep_last(image_qos_depth_).best_effort();
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(20).best_effort();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, imu_qos);
    if (enable_pointcloud_publish_) {
      pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, sensor_qos);
      filtered_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_pointcloud_topic_, sensor_qos);
    }
    if (enable_image_publish_) {
      left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_image_topic_, image_qos);
      right_pub_ = create_publisher<sensor_msgs::msg::Image>(right_image_topic_, image_qos);
      left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(left_camera_info_topic_, image_qos);
      right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(right_camera_info_topic_, image_qos);
    }
    if (enable_depth_publish_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic_, sensor_qos);
      depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depth_camera_info_topic_, sensor_qos);
    }
  }

  void setupPipeline()
  {
    auto imu = pipeline_.create<dai::node::IMU>();
    imu->enableIMUSensor(
      {dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},
      static_cast<int>(imu_frequency_));
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imu_xout = pipeline_.create<dai::node::XLinkOut>();
    imu_xout->setStreamName("imu");
    imu->out.link(imu_xout->input);

    auto mono_left = pipeline_.create<dai::node::MonoCamera>();
    auto mono_right = pipeline_.create<dai::node::MonoCamera>();
    auto stereo = pipeline_.create<dai::node::StereoDepth>();

    mono_left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    mono_left->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    mono_left->setFps(static_cast<float>(image_frequency_));
    mono_right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    mono_right->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    mono_right->setFps(static_cast<float>(image_frequency_));

    const bool low_latency_stereo =
      stereo_quality_mode_ == "low_latency" ||
      (stereo_quality_mode_ == "auto" && !enable_depth_publish_ && !enable_pointcloud_publish_);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
    stereo->setLeftRightCheck(!low_latency_stereo);
    stereo->setSubpixel(!low_latency_stereo);

    auto config = stereo->initialConfig.get();
    if (low_latency_stereo) {
      config.postProcessing.median = dai::MedianFilter::MEDIAN_OFF;
      config.postProcessing.spatialFilter.enable = false;
      config.postProcessing.temporalFilter.enable = false;
    } else {
      config.postProcessing.median = enable_passive_stereo_ ?
        dai::MedianFilter::KERNEL_7x7 : dai::MedianFilter::KERNEL_5x5;
      config.postProcessing.spatialFilter.enable = enable_passive_stereo_;
      config.postProcessing.temporalFilter.enable = true;
    }
    stereo->initialConfig.set(config);

    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);

    if (enable_image_publish_) {
      if (image_output_mode_ == "mono") {
        auto left_xout = pipeline_.create<dai::node::XLinkOut>();
        auto right_xout = pipeline_.create<dai::node::XLinkOut>();
        left_xout->setStreamName("left");
        right_xout->setStreamName("right");
        mono_left->out.link(left_xout->input);
        mono_right->out.link(right_xout->input);
      } else {
        auto left_xout = pipeline_.create<dai::node::XLinkOut>();
        auto right_xout = pipeline_.create<dai::node::XLinkOut>();
        left_xout->setStreamName("left");
        right_xout->setStreamName("right");
        stereo->rectifiedLeft.link(left_xout->input);
        stereo->rectifiedRight.link(right_xout->input);
      }
    }

    if (enable_depth_publish_ || enable_pointcloud_publish_) {
      auto depth_xout = pipeline_.create<dai::node::XLinkOut>();
      depth_xout->setStreamName("depth");
      stereo->depth.link(depth_xout->input);
    } else if (image_output_mode_ == "mono") {
      auto depth_diag_xout = pipeline_.create<dai::node::XLinkOut>();
      depth_diag_xout->setStreamName("depth_diag");
      stereo->depth.link(depth_diag_xout->input);
    }

    device_ = std::make_unique<dai::Device>(pipeline_);
    imu_queue_ = device_->getOutputQueue("imu", 20, false);
    if (enable_image_publish_) {
      left_queue_ = device_->getOutputQueue("left", image_queue_size_, false);
      right_queue_ = device_->getOutputQueue("right", image_queue_size_, false);
    }
    if (enable_depth_publish_ || enable_pointcloud_publish_) {
      depth_queue_ = device_->getOutputQueue("depth", 4, false);
    } else if (image_output_mode_ == "mono") {
      mono_diagnostic_depth_queue_ = device_->getOutputQueue("depth_diag", 1, false);
    }
    RCLCPP_INFO(
      get_logger(),
      "DepthAI pipeline configured: low_latency_stereo=%s, left_right_check=%s, subpixel=%s",
      low_latency_stereo ? "true" : "false", !low_latency_stereo ? "true" : "false",
      !low_latency_stereo ? "true" : "false");
  }

  void setupCalibration()
  {
    try {
      auto calib = device_ ? device_->readCalibration() : pipeline_.getCalibrationData();
      left_intrinsics_ = readIntrinsics(calib, dai::CameraBoardSocket::CAM_B, left_intrinsics_);
      right_intrinsics_ = readIntrinsics(calib, dai::CameraBoardSocket::CAM_C, right_intrinsics_);
      depth_intrinsics_ = right_intrinsics_;
      try {
        const double baseline_cm = calib.getBaselineDistance(
          dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C);
        if (baseline_cm > 0.0) {
          stereo_baseline_m_ = baseline_cm / 100.0;
        }
      } catch (const std::exception & exc) {
        RCLCPP_WARN(get_logger(), "Failed to read stereo baseline, using parameter value: %s", exc.what());
      }
      RCLCPP_INFO(
        get_logger(), "Calibration loaded: left_fx=%.1f right_fx=%.1f baseline=%.4fm",
        left_intrinsics_.fx, right_intrinsics_.fx, stereo_baseline_m_);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to load calibration, using defaults: %s", exc.what());
    }
  }

  Intrinsics readIntrinsics(
    dai::CalibrationHandler & calib, dai::CameraBoardSocket socket, const Intrinsics & fallback)
  {
    try {
      const auto matrix = calib.getCameraIntrinsics(socket, 640, 400);
      return Intrinsics{
        static_cast<double>(matrix[0][0]),
        static_cast<double>(matrix[1][1]),
        static_cast<double>(matrix[0][2]),
        static_cast<double>(matrix[1][2])};
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Failed to read camera intrinsics, using fallback: %s", exc.what());
      return fallback;
    }
  }

  void setupFovFilter()
  {
    if (auto_estimate_fov_) {
      fov_h_deg_ =
        radToDeg(std::atan(depth_intrinsics_.cx / depth_intrinsics_.fx) +
        std::atan((640.0 - depth_intrinsics_.cx) / depth_intrinsics_.fx));
      fov_v_deg_ =
        radToDeg(std::atan(depth_intrinsics_.cy / depth_intrinsics_.fy) +
        std::atan((400.0 - depth_intrinsics_.cy) / depth_intrinsics_.fy));
    }
    half_fov_h_rad_ = degToRad(fov_h_deg_ / 2.0);
    half_fov_v_rad_ = degToRad(fov_v_deg_ / 2.0);
    RCLCPP_INFO(
      get_logger(), "FOV filter configured: enabled=%s fov_h=%.2fdeg fov_v=%.2fdeg margin=%.3fm",
      enable_fov_boundary_filter_ ? "true" : "false", fov_h_deg_, fov_v_deg_,
      fov_boundary_margin_m_);
  }

  std_msgs::msg::Header makeHeader(const std::string & frame_id, double device_seconds)
  {
    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = stampFromDeviceTime(device_seconds);
    return header;
  }

  rclcpp::Time stampFromDeviceTime(double device_seconds)
  {
    const auto now = get_clock()->now();
    if (!std::isfinite(device_seconds)) {
      return now;
    }
    if (!device_time_base_) {
      device_time_base_ = std::make_pair(device_seconds, now.seconds());
    }
    const auto [base_device, base_ros] = *device_time_base_;
    const double stamp_seconds = base_ros + (device_seconds - base_device);
    const auto sec = static_cast<int64_t>(std::floor(stamp_seconds));
    const auto nsec = static_cast<uint32_t>((stamp_seconds - static_cast<double>(sec)) * 1e9);
    return rclcpp::Time(sec, nsec, get_clock()->get_clock_type());
  }

  std::array<double, 3> convertImuVector(double x, double y, double z)
  {
    if (imu_axis_mode_ == "raw") {
      return {x, y, z};
    }
    if (imu_axis_mode_ == "swap_yaw_roll_invert_pitch") {
      return {z, -y, x};
    }
    if (imu_axis_mode_ == "oakd_to_ros") {
      return {z, -y, -x};
    }
    if (!warned_unknown_imu_axis_mode_) {
      RCLCPP_WARN(get_logger(), "Unknown imu_axis_mode=%s, using raw", imu_axis_mode_.c_str());
      warned_unknown_imu_axis_mode_ = true;
    }
    return {x, y, z};
  }

  void publishImu()
  {
    if (!imu_queue_) {
      return;
    }
    auto imu_data = imu_queue_->tryGet<dai::IMUData>();
    if (!imu_data) {
      return;
    }

    for (const auto & packet : imu_data->packets) {
      sensor_msgs::msg::Imu msg;
      msg.header = makeHeader(imu_frame_id_, timestampToSeconds(packet.acceleroMeter.getTimestampDevice()));

      const auto accel = convertImuVector(
        packet.acceleroMeter.x, packet.acceleroMeter.y, packet.acceleroMeter.z);
      msg.linear_acceleration.x = accel[0];
      msg.linear_acceleration.y = accel[1];
      msg.linear_acceleration.z = accel[2];

      const auto gyro = convertImuVector(
        packet.gyroscope.x, packet.gyroscope.y, packet.gyroscope.z);
      msg.angular_velocity.x = gyro[0];
      msg.angular_velocity.y = gyro[1];
      msg.angular_velocity.z = gyro[2];

      msg.orientation_covariance[0] = -1.0;
      msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
      msg.angular_velocity_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};
      imu_pub_->publish(msg);
    }
  }

  sensor_msgs::msg::Image createImageMsg(
    const dai::ImgFrame & frame, const std::string & frame_id, const rclcpp::Time & stamp,
    const std::string & encoding, size_t bytes_per_pixel)
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = static_cast<uint32_t>(frame.getHeight());
    msg.width = static_cast<uint32_t>(frame.getWidth());
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(msg.width * bytes_per_pixel);
    const auto & data = frame.getData();
    msg.data.assign(data.begin(), data.end());
    return msg;
  }

  sensor_msgs::msg::CameraInfo createCameraInfoMsg(
    const std_msgs::msg::Header & header, int height, int width,
    const Intrinsics & intrinsics, double tx = 0.0)
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header = header;
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    msg.k = {
      intrinsics.fx, 0.0, intrinsics.cx,
      0.0, intrinsics.fy, intrinsics.cy,
      0.0, 0.0, 1.0};
    msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
    msg.p = {
      intrinsics.fx, 0.0, intrinsics.cx, tx,
      0.0, intrinsics.fy, intrinsics.cy, 0.0,
      0.0, 0.0, 1.0, 0.0};
    return msg;
  }

  template<typename T>
  std::shared_ptr<T> latestQueueFrame(const std::shared_ptr<dai::DataOutputQueue> & queue)
  {
    std::shared_ptr<T> latest;
    while (true) {
      auto frame = queue->tryGet<T>();
      if (!frame) {
        return latest;
      }
      latest = frame;
    }
  }

  void publishImages()
  {
    if (!left_queue_ || !right_queue_) {
      return;
    }

    auto left = latestQueueFrame<dai::ImgFrame>(left_queue_);
    auto right = latestQueueFrame<dai::ImgFrame>(right_queue_);
    if (!left || !right) {
      return;
    }

    const double left_seconds = timestampToSeconds(left->getTimestampDevice());
    const double right_seconds = timestampToSeconds(right->getTimestampDevice());
    const double pair_dt_ms = std::abs(left_seconds - right_seconds) * 1000.0;
    if (pair_dt_ms > image_pair_max_dt_ms_) {
      ++image_drop_count_;
      if (image_drop_count_ <= 5 || image_drop_count_ % 30 == 0) {
        RCLCPP_WARN(
          get_logger(), "Dropping stereo pair: dt=%.2fms limit=%.2fms drops=%zu",
          pair_dt_ms, image_pair_max_dt_ms_, image_drop_count_);
      }
      return;
    }

    const double device_seconds = 0.5 * (left_seconds + right_seconds);
    const auto stamp = stampFromDeviceTime(device_seconds);

    auto left_msg = createImageMsg(*left, left_camera_frame_id_, stamp, "mono8", 1);
    auto right_msg = createImageMsg(*right, right_camera_frame_id_, stamp, "mono8", 1);

    const auto left_info = createCameraInfoMsg(
      left_msg.header, static_cast<int>(left_msg.height), static_cast<int>(left_msg.width),
      left_intrinsics_);
    const auto right_info = createCameraInfoMsg(
      right_msg.header, static_cast<int>(right_msg.height), static_cast<int>(right_msg.width),
      right_intrinsics_, -right_intrinsics_.fx * stereo_baseline_m_);

    if (image_publish_order_ == "right_first") {
      right_pub_->publish(right_msg);
      sleepBetweenStereoPublishes();
      left_pub_->publish(left_msg);
      right_info_pub_->publish(right_info);
      left_info_pub_->publish(left_info);
    } else {
      left_pub_->publish(left_msg);
      sleepBetweenStereoPublishes();
      right_pub_->publish(right_msg);
      left_info_pub_->publish(left_info);
      right_info_pub_->publish(right_info);
    }

    if (last_image_stamp_seconds_ && (device_seconds - *last_image_stamp_seconds_) * 1000.0 > 80.0) {
      RCLCPP_WARN(
        get_logger(), "OAK-D stereo image interval is high: %.2fms",
        (device_seconds - *last_image_stamp_seconds_) * 1000.0);
    }
    last_image_stamp_seconds_ = device_seconds;
  }

  void sleepBetweenStereoPublishes()
  {
    if (image_inter_publish_delay_ms_ <= 0.0) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(image_inter_publish_delay_ms_));
  }

  void publishDepthAndPointcloud()
  {
    if (!depth_queue_) {
      return;
    }

    auto depth = depth_queue_->tryGet<dai::ImgFrame>();
    if (!depth) {
      return;
    }

    const auto & depth_data = depth->getData();
    const int depth_width = static_cast<int>(depth->getWidth());
    const int depth_height = static_cast<int>(depth->getHeight());
    const auto header = makeHeader(pointcloud_frame_id_, timestampToSeconds(depth->getTimestampDevice()));

    if (enable_depth_publish_) {
      depth_pub_->publish(createImageMsg(*depth, header.frame_id, header.stamp, "16UC1", 2));
      depth_info_pub_->publish(createCameraInfoMsg(
        header, depth_height, depth_width, depth_intrinsics_));
    }

    if (!enable_pointcloud_publish_) {
      return;
    }

    auto raw_points = projectDepth(depth_data, depth_width, depth_height);
    pc_pub_->publish(createPointCloudMsg(header, raw_points));

    if (enable_fov_boundary_filter_) {
      raw_points.erase(
        std::remove_if(raw_points.begin(), raw_points.end(), [this](const auto & p) {
          return !pointInsideFrustumCore(p);
        }),
        raw_points.end());
    }
    filtered_pc_pub_->publish(createPointCloudMsg(header, raw_points));
  }

  std::vector<std::array<float, 3>> projectDepth(
    const std::vector<std::uint8_t> & depth_data, int width, int height)
  {
    std::vector<std::array<float, 3>> points;
    points.reserve(static_cast<size_t>(width * height / (sampling_step_ * sampling_step_)));

    const int border = std::max(0, depth_border_crop_px_);
    const auto * depth_u16 = reinterpret_cast<const uint16_t *>(depth_data.data());
    for (int v = border; v < height - border; v += sampling_step_) {
      for (int u = border; u < width - border; u += sampling_step_) {
        const uint16_t depth_mm = depth_u16[v * width + u];
        if (!depthValid(depth_u16, width, height, u, v, depth_mm)) {
          continue;
        }
        const float z = static_cast<float>(depth_mm) * 0.001f;
        const float x = static_cast<float>((u - depth_intrinsics_.cx) * z / depth_intrinsics_.fx);
        const float y = static_cast<float>((v - depth_intrinsics_.cy) * z / depth_intrinsics_.fy);
        points.push_back({x, y, z});
      }
    }
    return points;
  }

  bool depthValid(
    const uint16_t * depth_data, int width, int height, int u, int v, uint16_t depth_mm) const
  {
    if (depth_mm <= min_depth_ || depth_mm >= max_depth_) {
      return false;
    }
    if (max_depth_jump_mm_ <= 0) {
      return true;
    }
    const int right = std::min(u + 1, width - 1);
    const int down = std::min(v + 1, height - 1);
    const int jump_x = std::abs(static_cast<int>(depth_mm) - static_cast<int>(depth_data[v * width + right]));
    const int jump_y = std::abs(static_cast<int>(depth_mm) - static_cast<int>(depth_data[down * width + u]));
    return jump_x <= max_depth_jump_mm_ && jump_y <= max_depth_jump_mm_;
  }

  bool pointInsideFrustumCore(const std::array<float, 3> & p) const
  {
    const double x = p[0];
    const double y = p[1];
    const double z = p[2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 1e-6) {
      return false;
    }

    const double tan_h = std::tan(half_fov_h_rad_);
    const double tan_v = std::tan(half_fov_v_rad_);
    const double cos_h = std::cos(half_fov_h_rad_);
    const double cos_v = std::cos(half_fov_v_rad_);
    const double margin = std::max(0.0, fov_boundary_margin_m_);

    const std::array<double, 4> distances = {
      (x - z * tan_h) * cos_h,
      (-x - z * tan_h) * cos_h,
      (y - z * tan_v) * cos_v,
      (-y - z * tan_v) * cos_v};

    return std::all_of(distances.begin(), distances.end(), [margin](double d) {
      return d <= -margin;
    });
  }

  sensor_msgs::msg::PointCloud2 createPointCloudMsg(
    const std_msgs::msg::Header & header, const std::vector<std::array<float, 3>> & points)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = header;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.is_dense = false;
    msg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    for (const auto & point : points) {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    return msg;
  }

  static double degToRad(double deg) {return deg * M_PI / 180.0;}
  static double radToDeg(double rad) {return rad * 180.0 / M_PI;}

  int imu_frequency_{400};
  std::string imu_topic_name_;
  std::string imu_frame_id_;
  std::string imu_axis_mode_;
  bool warned_unknown_imu_axis_mode_{false};

  bool enable_passive_stereo_{true};
  bool enable_active_stereo_{false};
  int ir_intensity_{1600};
  std::string stereo_quality_mode_{"auto"};

  int pointcloud_frequency_{20};
  bool enable_pointcloud_publish_{true};
  std::string pointcloud_topic_;
  std::string filtered_pointcloud_topic_;
  std::string pointcloud_frame_id_;
  int sampling_step_{2};
  int min_depth_{200};
  int max_depth_{15000};
  int depth_border_crop_px_{8};
  int max_depth_jump_mm_{350};
  bool enable_fov_boundary_filter_{true};
  bool auto_estimate_fov_{true};
  double fov_h_deg_{72.0};
  double fov_v_deg_{53.0};
  double fov_boundary_margin_m_{0.15};
  double half_fov_h_rad_{0.0};
  double half_fov_v_rad_{0.0};

  bool enable_image_publish_{true};
  bool enable_depth_publish_{true};
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string left_camera_info_topic_;
  std::string right_camera_info_topic_;
  std::string depth_image_topic_;
  std::string depth_camera_info_topic_;
  std::string left_camera_frame_id_;
  std::string right_camera_frame_id_;
  double stereo_baseline_m_{0.075};
  int image_frequency_{25};
  int image_poll_frequency_{75};
  int image_queue_size_{2};
  double image_pair_max_dt_ms_{8.0};
  std::string image_output_mode_{"rectified"};
  int image_qos_depth_{4};
  std::string image_publish_order_{"left_first"};
  double image_inter_publish_delay_ms_{1.0};
  size_t image_drop_count_{0};

  Intrinsics left_intrinsics_;
  Intrinsics right_intrinsics_;
  Intrinsics depth_intrinsics_;

  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;
  std::shared_ptr<dai::DataOutputQueue> imu_queue_;
  std::shared_ptr<dai::DataOutputQueue> depth_queue_;
  std::shared_ptr<dai::DataOutputQueue> left_queue_;
  std::shared_ptr<dai::DataOutputQueue> right_queue_;
  std::shared_ptr<dai::DataOutputQueue> mono_diagnostic_depth_queue_;

  std::optional<std::pair<double, double>> device_time_base_;
  std::optional<double> last_image_stamp_seconds_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr pointcloud_timer_;
  rclcpp::TimerBase::SharedPtr image_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OakDUnifiedNode>());
  rclcpp::shutdown();
  return 0;
}
