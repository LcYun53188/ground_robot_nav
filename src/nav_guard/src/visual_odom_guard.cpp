#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double angleDelta(double current, double previous)
{
  return std::atan2(std::sin(current - previous), std::cos(current - previous));
}

double radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double degrees(double rad)
{
  return rad * 180.0 / M_PI;
}

}  // namespace

class VisualOdomGuard : public rclcpp::Node
{
public:
  VisualOdomGuard()
  : Node("visual_odom_guard")
  {
    declare_parameter("input_topic", "/visual_slam/tracking/odometry");
    declare_parameter("output_topic", "/visual_slam/guarded_odometry");
    declare_parameter("status_topic", "/visual_slam/odom_guard/status");
    declare_parameter("path_topic", "/visual_slam/guarded_path");
    declare_parameter("publish_path", true);
    declare_parameter("path_max_poses", 2000);
    declare_parameter("publish_tf", false);
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("max_step_xy_m", 0.20);
    declare_parameter("max_step_z_m", 0.15);
    declare_parameter("max_yaw_step_deg", 20.0);
    declare_parameter("max_speed_xy_mps", 1.2);
    declare_parameter("max_yaw_rate_dps", 120.0);
    declare_parameter("min_dt_sec", 0.001);
    declare_parameter("publish_rejected_as_hold", true);
    declare_parameter("zero_twist_on_hold", true);
    declare_parameter("log_rejects", true);
    declare_parameter("log_reject_every_n", 25);
    declare_parameter("max_hold_sec_before_reseed", 2.0);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    publish_path_ = get_parameter("publish_path").as_bool();
    path_max_poses_ = static_cast<size_t>(std::max<int64_t>(0, get_parameter("path_max_poses").as_int()));
    publish_tf_ = get_parameter("publish_tf").as_bool();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    max_step_xy_m_ = get_parameter("max_step_xy_m").as_double();
    max_step_z_m_ = get_parameter("max_step_z_m").as_double();
    max_yaw_step_ = radians(get_parameter("max_yaw_step_deg").as_double());
    max_speed_xy_mps_ = get_parameter("max_speed_xy_mps").as_double();
    max_yaw_rate_ = radians(get_parameter("max_yaw_rate_dps").as_double());
    min_dt_sec_ = get_parameter("min_dt_sec").as_double();
    publish_rejected_as_hold_ = get_parameter("publish_rejected_as_hold").as_bool();
    zero_twist_on_hold_ = get_parameter("zero_twist_on_hold").as_bool();
    log_rejects_ = get_parameter("log_rejects").as_bool();
    log_reject_every_n_ = static_cast<size_t>(std::max<int64_t>(1, get_parameter("log_reject_every_n").as_int()));
    max_hold_sec_before_reseed_ = get_parameter("max_hold_sec_before_reseed").as_double();

    path_msg_.header.frame_id = odom_frame_;
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    if (publish_path_) {
      path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 50, std::bind(&VisualOdomGuard::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Visual odom guard started: %s -> %s, publish_tf=%s, publish_path=%s",
      input_topic_.c_str(), output_topic_.c_str(), publish_tf_ ? "true" : "false",
      publish_path_ ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!last_accepted_) {
      acceptSample(*msg, "initial");
      return;
    }

    std::string reason;
    if (shouldAccept(*msg, *last_accepted_, reason)) {
      acceptSample(*msg, "accepted");
      return;
    }

    ++rejected_count_;
    if (shouldReseed(*msg)) {
      RCLCPP_WARN(
        get_logger(), "reseed guarded odom after sustained rejects: held_for=%.3fs, rejected=%zu",
        rejectedDuration(*msg), rejected_count_);
      acceptSample(*msg, "reseed");
      return;
    }

    if (log_rejects_ && shouldLogReject()) {
      RCLCPP_WARN(get_logger(), "%s", reason.c_str());
    }
    publishStatus(reason);

    if (publish_rejected_as_hold_) {
      publishHold(msg->header.stamp);
    }
  }

  bool shouldAccept(
    const nav_msgs::msg::Odometry & current,
    const nav_msgs::msg::Odometry & previous,
    std::string & reason) const
  {
    const double dt = stampSeconds(current.header.stamp) - stampSeconds(previous.header.stamp);
    if (dt < min_dt_sec_) {
      std::ostringstream out;
      out << "reject non-increasing odom stamp: dt=" << dt << "s";
      reason = out.str();
      return false;
    }

    const auto & cp = current.pose.pose.position;
    const auto & pp = previous.pose.pose.position;
    const double dx = cp.x - pp.x;
    const double dy = cp.y - pp.y;
    const double dz = cp.z - pp.z;
    const double dyaw = angleDelta(
      yawFromQuaternion(current.pose.pose.orientation),
      yawFromQuaternion(previous.pose.pose.orientation));

    const double step_xy = std::hypot(dx, dy);
    const double abs_dz = std::abs(dz);
    const double abs_dyaw = std::abs(dyaw);
    const double speed_xy = step_xy / dt;
    const double yaw_rate = abs_dyaw / dt;

    std::ostringstream failures;
    bool failed = false;
    appendFailure(failures, failed, step_xy > max_step_xy_m_, "step_xy", step_xy, "m", max_step_xy_m_, "m");
    appendFailure(failures, failed, abs_dz > max_step_z_m_, "step_z", abs_dz, "m", max_step_z_m_, "m");
    appendFailure(failures, failed, abs_dyaw > max_yaw_step_, "step_yaw", degrees(abs_dyaw), "deg", degrees(max_yaw_step_), "deg");
    appendFailure(failures, failed, speed_xy > max_speed_xy_mps_, "speed_xy", speed_xy, "m/s", max_speed_xy_mps_, "m/s");
    appendFailure(failures, failed, yaw_rate > max_yaw_rate_, "yaw_rate", degrees(yaw_rate), "deg/s", degrees(max_yaw_rate_), "deg/s");

    if (!failed) {
      reason = "accepted";
      return true;
    }

    failures << ", dt=" << format(dt, 3) << "s, rejected=" << (rejected_count_ + 1);
    reason = "reject visual odom jump: " + failures.str();
    return false;
  }

  static void appendFailure(
    std::ostringstream & out, bool & failed, bool condition, const char * name,
    double value, const char * value_unit, double limit, const char * limit_unit)
  {
    if (!condition) {
      return;
    }
    if (failed) {
      out << ", ";
    }
    failed = true;
    out << name << "=" << format(value, 3) << value_unit << ">" << format(limit, 3) << limit_unit;
  }

  static std::string format(double value, int precision)
  {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(precision);
    out << value;
    return out.str();
  }

  void acceptSample(const nav_msgs::msg::Odometry & msg, const std::string & status)
  {
    auto guarded = msg;
    guarded.header.frame_id = odom_frame_;
    guarded.child_frame_id = base_frame_;
    last_accepted_ = guarded;
    first_rejected_stamp_.reset();
    ++accepted_count_;
    odom_pub_->publish(guarded);
    publishTf(guarded);
    publishPath(guarded);

    std::ostringstream text;
    text << status << ": accepted=" << accepted_count_ << ", rejected=" << rejected_count_
         << ", held=" << held_count_;
    publishStatus(text.str());
  }

  void publishHold(const builtin_interfaces::msg::Time & stamp)
  {
    if (!last_accepted_) {
      return;
    }
    auto held = *last_accepted_;
    held.header.stamp = stamp;
    if (zero_twist_on_hold_) {
      held.twist.twist.linear.x = 0.0;
      held.twist.twist.linear.y = 0.0;
      held.twist.twist.linear.z = 0.0;
      held.twist.twist.angular.x = 0.0;
      held.twist.twist.angular.y = 0.0;
      held.twist.twist.angular.z = 0.0;
    }
    ++held_count_;
    odom_pub_->publish(held);
    publishTf(held);
    publishPath(held);
  }

  bool shouldReseed(const nav_msgs::msg::Odometry & msg)
  {
    if (max_hold_sec_before_reseed_ <= 0.0) {
      return false;
    }
    if (!first_rejected_stamp_) {
      first_rejected_stamp_ = msg.header.stamp;
      return false;
    }
    return rejectedDuration(msg) >= max_hold_sec_before_reseed_;
  }

  double rejectedDuration(const nav_msgs::msg::Odometry & msg) const
  {
    if (!first_rejected_stamp_) {
      return 0.0;
    }
    return stampSeconds(msg.header.stamp) - stampSeconds(*first_rejected_stamp_);
  }

  bool shouldLogReject() const
  {
    return rejected_count_ == 1 || rejected_count_ % log_reject_every_n_ == 0;
  }

  void publishTf(const nav_msgs::msg::Odometry & odom)
  {
    if (!tf_broadcaster_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publishPath(const nav_msgs::msg::Odometry & odom)
  {
    if (!path_pub_) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_msg_.header.stamp = odom.header.stamp;
    path_msg_.header.frame_id = odom_frame_;
    path_msg_.poses.push_back(pose);
    if (path_max_poses_ > 0 && path_msg_.poses.size() > path_max_poses_) {
      path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.end() - path_max_poses_);
    }
    path_pub_->publish(path_msg_);
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string path_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_path_{true};
  bool publish_tf_{false};
  bool publish_rejected_as_hold_{true};
  bool zero_twist_on_hold_{true};
  bool log_rejects_{true};
  size_t path_max_poses_{2000};
  size_t log_reject_every_n_{25};

  double max_step_xy_m_{0.20};
  double max_step_z_m_{0.15};
  double max_yaw_step_{radians(20.0)};
  double max_speed_xy_mps_{1.2};
  double max_yaw_rate_{radians(120.0)};
  double min_dt_sec_{0.001};
  double max_hold_sec_before_reseed_{2.0};

  std::optional<nav_msgs::msg::Odometry> last_accepted_;
  std::optional<builtin_interfaces::msg::Time> first_rejected_stamp_;
  size_t accepted_count_{0};
  size_t rejected_count_{0};
  size_t held_count_{0};
  nav_msgs::msg::Path path_msg_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualOdomGuard>());
  rclcpp::shutdown();
  return 0;
}
