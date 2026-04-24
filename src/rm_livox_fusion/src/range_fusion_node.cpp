#include "range_fusion_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace rm_livox_fusion
{
RangeFusionNode::RangeFusionNode()
: Node("range_fusion_node")
{
  camera_optical_frame_ =
    this->declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
  accum_cloud_frame_ =
    this->declare_parameter<std::string>("accum_cloud_frame", "odom");
  angle_unit_ = this->declare_parameter<std::string>("angle_unit", "deg");
  gate_yaw_ = this->declare_parameter<double>("gate_yaw", 1.0);
  roi_scale_ = this->declare_parameter<double>("roi_scale", 1.5);
  use_z_as_range_ = this->declare_parameter<bool>("use_z_as_range", false);
  valid_range_min_ = this->declare_parameter<double>("valid_range_min", 0.0);
  valid_range_max_ = this->declare_parameter<double>("valid_range_max", 0.0);
  if (valid_range_min_ < 0.0) {
    valid_range_min_ = 0.0;
  }
  if (valid_range_max_ > 0.0 && valid_range_max_ < valid_range_min_) {
    RCLCPP_WARN(
      get_logger(), "valid_range_max < valid_range_min, disabling max range gate");
    valid_range_max_ = 0.0;
  }
  min_points_ = static_cast<size_t>(this->declare_parameter<int64_t>("min_points", 30));
  mad_thresh_ = this->declare_parameter<double>("mad_thresh", 0.3);
  fallback_to_pnp_ = this->declare_parameter<bool>("fallback_to_pnp", true);
  output_stability_logic_ =
    this->declare_parameter<std::string>("output_stability_logic", "and");
  executor_threads_ =
    static_cast<size_t>(std::max<int64_t>(
      1, this->declare_parameter<int64_t>("executor_threads", 3)));

  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  send_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  camera_info_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  send_pub_ = create_publisher<auto_aim_interfaces::msg::Send>(
    "send_out", rclcpp::SensorDataQoS());
  rclcpp::SubscriptionOptions cloud_sub_options;
  cloud_sub_options.callback_group = cloud_callback_group_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::cloudCallback, this, std::placeholders::_1),
    cloud_sub_options);
  rclcpp::SubscriptionOptions camera_info_sub_options;
  camera_info_sub_options.callback_group = camera_info_callback_group_;
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::cameraInfoCallback, this, std::placeholders::_1),
    camera_info_sub_options);
  rclcpp::SubscriptionOptions send_sub_options;
  send_sub_options.callback_group = send_callback_group_;
  send_sub_ = create_subscription<auto_aim_interfaces::msg::Send>(
    "send_in", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::sendCallback, this, std::placeholders::_1),
    send_sub_options);
}

void RangeFusionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  has_camera_info_ = (fx_ > 0.0 && fy_ > 0.0);
  if (has_camera_info_) {
    camera_info_sub_.reset();
  }
}

void RangeFusionNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud_ = msg;
}

void RangeFusionNode::sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  auto out_msg = auto_aim_interfaces::msg::Send();
  out_msg.header = msg->header;
  out_msg.angle = 1234.0f;
  out_msg.pixel_angle = msg->pixel_angle;
  out_msg.longitudinal_distance = 0.0f;
  out_msg.lateral_distance = 0.0f;
  out_msg.u = msg->u;
  out_msg.v = msg->v;
  out_msg.roi_radius = msg->roi_radius;

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = last_cloud_;
  }

  if (!cloud) {
    handleNoCloud(*msg, out_msg);
    send_pub_->publish(out_msg);
    return;
  }

  rclcpp::Time lookup_time = getLookupTime(*msg, *cloud);
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      camera_optical_frame_, accum_cloud_frame_, lookup_time,
      rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    handleNoCloud(*msg, out_msg);
    send_pub_->publish(out_msg);
    return;
  }

  const auto &t = transform.transform.translation;
  const auto &q_msg = transform.transform.rotation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 rotation(q);
  const double r00 = rotation[0][0];
  const double r01 = rotation[0][1];
  const double r02 = rotation[0][2];
  const double r10 = rotation[1][0];
  const double r11 = rotation[1][1];
  const double r12 = rotation[1][2];
  const double r20 = rotation[2][0];
  const double r21 = rotation[2][1];
  const double r22 = rotation[2][2];
  const double tx = t.x;
  const double ty = t.y;
  const double tz = t.z;

  double theta = toRadians(msg->pixel_angle);
  double gate = toRadians(gate_yaw_);
  bool use_roi = has_camera_info_ && (msg->roi_radius > 0.0f);
  double roi_u = static_cast<double>(msg->u);
  double roi_v = static_cast<double>(msg->v);
  double roi_r = static_cast<double>(msg->roi_radius);
  double roi_scale = roi_scale_ > 0.0 ? roi_scale_ : 1.0;
  roi_r *= roi_scale;
  double roi_r2 = roi_r * roi_r;

  std::vector<double> ranges;
  std::vector<double> lateral_values;
  std::vector<double> longitudinal_values;
  const size_t point_count =
    static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
  ranges.reserve(point_count);
  lateral_values.reserve(point_count);
  longitudinal_values.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double source_x = static_cast<double>(*iter_x);
    double source_y = static_cast<double>(*iter_y);
    double source_z = static_cast<double>(*iter_z);
    if (!std::isfinite(source_x) || !std::isfinite(source_y) || !std::isfinite(source_z)) {
      continue;
    }
    double x = r00 * source_x + r01 * source_y + r02 * source_z + tx;
    double y = r10 * source_x + r11 * source_y + r12 * source_z + ty;
    double z = r20 * source_x + r21 * source_y + r22 * source_z + tz;
    if (z <= 0.0) {
      continue;
    }
    if (use_roi) {
      double u = fx_ * x / z + cx_;
      double v = fy_ * y / z + cy_;
      double du = u - roi_u;
      double dv = v - roi_v;
      if (du * du + dv * dv > roi_r2) {
        continue;
      }
    } else {
      double bearing = std::atan2(x, z);
      if (std::abs(bearing - theta) > gate) {
        continue;
      }
    }
    double range = 0.0;
    if (use_z_as_range_) {
      range = z;
    } else {
      range = std::sqrt(x * x + z * z);
    }
    if (range < valid_range_min_) {
      continue;
    }
    if (valid_range_max_ > 0.0 && range > valid_range_max_) {
      continue;
    }
    ranges.push_back(range);
    lateral_values.push_back(x);
    longitudinal_values.push_back(z);
  }

  bool roi_ok = false;
  double lidar_distance = 0.0;
  double lateral_distance = 0.0;
  double longitudinal_distance = 0.0;
  double actual_angle = 0.0;
  double mad_value = std::numeric_limits<double>::quiet_NaN();
  if (ranges.size() >= min_points_) {
    double median = computeMedian(ranges);
    std::vector<double> deviations;
    deviations.reserve(ranges.size());
    for (double value : ranges) {
      deviations.push_back(std::abs(value - median));
    }
    mad_value = computeMedian(deviations);
    if (mad_value <= mad_thresh_) {
      roi_ok = true;
      lidar_distance = median;
      lateral_distance = computeMedian(lateral_values);
      longitudinal_distance = computeMedian(longitudinal_values);
      actual_angle = std::atan2(lateral_distance, longitudinal_distance);
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "ROI %s | points=%zu mad=%.4f dist=%.3f",
    roi_ok ? "OK" : "INVALID",
    ranges.size(),
    mad_value,
    roi_ok ? lidar_distance : 0.0);

  if (roi_ok) {
    out_msg.distance = static_cast<float>(lidar_distance);
    out_msg.lateral_distance = static_cast<float>(lateral_distance);
    out_msg.longitudinal_distance = static_cast<float>(longitudinal_distance);
    if (angle_unit_ == "rad") {
      out_msg.angle = static_cast<float>(actual_angle);
    } else {
      out_msg.angle = static_cast<float>(actual_angle * 180.0 / kPi);
    }
  } else if (fallback_to_pnp_) {
    out_msg.distance = msg->distance;
  } else {
    out_msg.distance = 0.0f;
  }

  out_msg.stability = computeStability(msg->stability, roi_ok);
  send_pub_->publish(out_msg);
}

void RangeFusionNode::handleNoCloud(
  const auto_aim_interfaces::msg::Send &in,
  auto_aim_interfaces::msg::Send &out)
{
  out.angle = 1234.0f;
  out.pixel_angle = in.pixel_angle;
  out.longitudinal_distance = 0.0f;
  out.lateral_distance = 0.0f;
  if (fallback_to_pnp_) {
    out.distance = in.distance;
    out.stability = in.stability;
    return;
  }
  out.distance = 0.0f;
  out.stability = 0;
}

rclcpp::Time RangeFusionNode::getLookupTime(
  const auto_aim_interfaces::msg::Send &in,
  const sensor_msgs::msg::PointCloud2 &cloud) const
{
  bool has_send_stamp = (in.header.stamp.sec != 0 || in.header.stamp.nanosec != 0);
  if (has_send_stamp) {
    return rclcpp::Time(in.header.stamp);
  }
  return rclcpp::Time(cloud.header.stamp);
}

double RangeFusionNode::toRadians(double angle) const
{
  if (angle_unit_ == "rad") {
    return angle;
  }
  return angle * kPi / 180.0;
}

double RangeFusionNode::computeMedian(std::vector<double> values) const
{
  if (values.empty()) {
    return 0.0;
  }
  size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double median = values[mid];
  if (values.size() % 2 == 0) {
    auto max_it = std::max_element(values.begin(), values.begin() + mid);
    median = (*max_it + median) * 0.5;
  }
  return median;
}

uint8_t RangeFusionNode::computeStability(uint8_t input, bool roi_ok) const
{
  if (output_stability_logic_ == "or") {
    return static_cast<uint8_t>((input != 0) || roi_ok);
  }
  return static_cast<uint8_t>((input != 0) && roi_ok);
}

size_t RangeFusionNode::executorThreads() const
{
  return executor_threads_;
}
}  // namespace rm_livox_fusion

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_livox_fusion::RangeFusionNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), node->executorThreads());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
