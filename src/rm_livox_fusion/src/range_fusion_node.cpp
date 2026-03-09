#include "range_fusion_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
  min_points_ = static_cast<size_t>(this->declare_parameter<int64_t>("min_points", 30));
  mad_thresh_ = this->declare_parameter<double>("mad_thresh", 0.3);
  fallback_to_pnp_ = this->declare_parameter<bool>("fallback_to_pnp", true);
  output_stability_logic_ =
    this->declare_parameter<std::string>("output_stability_logic", "and");

  window_min_sec_ = this->declare_parameter<double>("window_min_sec", 0.25);
  window_max_sec_ = this->declare_parameter<double>("window_max_sec", 1.5);
  window_step_sec_ = this->declare_parameter<double>("window_step_sec", 0.15);
  target_points_ =
    static_cast<size_t>(this->declare_parameter<int64_t>("target_points", 80));
  age_tau_sec_ = this->declare_parameter<double>("age_tau_sec", 0.6);
  mad_k_ = this->declare_parameter<double>("mad_k", 2.5);
  cloud_sync_tolerance_sec_ =
    this->declare_parameter<double>("cloud_sync_tolerance_ms", 60.0) / 1000.0;
  range_min_ = this->declare_parameter<double>("range_min", 2.0);
  range_max_ = this->declare_parameter<double>("range_max", 40.0);
  use_pnp_prior_gate_ = this->declare_parameter<bool>("use_pnp_prior_gate", true);
  pnp_prior_rel_tol_ = this->declare_parameter<double>("pnp_prior_rel_tol", 0.35);
  pnp_prior_abs_tol_ = this->declare_parameter<double>("pnp_prior_abs_tol", 1.0);
  hold_last_lidar_on_failure_ =
    this->declare_parameter<bool>("hold_last_lidar_on_failure", true);

  if (window_min_sec_ <= 0.0) {
    window_min_sec_ = 0.1;
  }
  if (window_max_sec_ < window_min_sec_) {
    window_max_sec_ = window_min_sec_;
  }
  if (window_step_sec_ <= 0.0) {
    window_step_sec_ = 0.05;
  }
  if (target_points_ < min_points_) {
    target_points_ = min_points_;
  }
  if (age_tau_sec_ <= 0.0) {
    age_tau_sec_ = 0.1;
  }
  if (mad_k_ <= 0.0) {
    mad_k_ = 2.5;
  }
  if (cloud_sync_tolerance_sec_ < 0.0) {
    cloud_sync_tolerance_sec_ = 0.0;
  }
  if (range_min_ < 0.0) {
    range_min_ = 0.0;
  }
  if (range_max_ > 0.0 && range_max_ < range_min_) {
    range_max_ = range_min_;
  }
  if (pnp_prior_rel_tol_ < 0.0) {
    pnp_prior_rel_tol_ = 0.0;
  }
  if (pnp_prior_abs_tol_ < 0.0) {
    pnp_prior_abs_tol_ = 0.0;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  send_pub_ = create_publisher<auto_aim_interfaces::msg::Send>(
    "send_out", rclcpp::SensorDataQoS());
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::cloudCallback, this, std::placeholders::_1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::cameraInfoCallback, this, std::placeholders::_1));
  send_sub_ = create_subscription<auto_aim_interfaces::msg::Send>(
    "send_in", rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::sendCallback, this, std::placeholders::_1));
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
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Input cloud frame_id is empty");
    return;
  }

  rclcpp::Time lookup_time = isZeroStamp(msg->header.stamp)
    ? rclcpp::Time(0, 0, get_clock()->get_clock_type())
    : rclcpp::Time(msg->header.stamp);

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      accum_cloud_frame_, msg->header.frame_id, lookup_time,
      rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Cloud TF lookup failed: %s", ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_tf;
  try {
    tf2::doTransform(*msg, cloud_tf, transform);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Cloud transform failed: %s", ex.what());
    return;
  }

  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(cloud_tf, *pcl_cloud);
  if (pcl_cloud->empty()) {
    return;
  }

  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->points.reserve(pcl_cloud->points.size());
  for (const auto &pt : pcl_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    filtered->points.push_back(pt);
  }
  filtered->width = static_cast<uint32_t>(filtered->points.size());
  filtered->height = 1;
  filtered->is_dense = true;
  if (filtered->empty()) {
    return;
  }

  rclcpp::Time stamp = isZeroStamp(cloud_tf.header.stamp)
    ? now()
    : rclcpp::Time(cloud_tf.header.stamp);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  cloud_buffer_.push_back(CloudFrame{stamp, filtered});
  pruneCloudBufferLocked(stamp);
}

void RangeFusionNode::sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  auto out_msg = auto_aim_interfaces::msg::Send();
  out_msg.header = msg->header;
  out_msg.angle = msg->angle;
  out_msg.u = msg->u;
  out_msg.v = msg->v;
  out_msg.roi_radius = msg->roi_radius;

  const bool has_send_stamp = !isZeroStamp(msg->header.stamp);
  rclcpp::Time reference_time = has_send_stamp ? rclcpp::Time(msg->header.stamp) : now();
  rclcpp::Time latest_cloud_stamp;
  std::vector<CloudFrame> cloud_frames;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_buffer_.empty()) {
      handleNoCloud(*msg, out_msg);
      send_pub_->publish(out_msg);
      return;
    }

    latest_cloud_stamp = cloud_buffer_.back().stamp;
    if (!has_send_stamp) {
      reference_time = latest_cloud_stamp;
    }
    cloud_frames.assign(cloud_buffer_.begin(), cloud_buffer_.end());
  }

  if (has_send_stamp) {
    const double newest_delta = (reference_time - latest_cloud_stamp).seconds();
    if (newest_delta > window_max_sec_ + cloud_sync_tolerance_sec_) {
      handleNoCloud(*msg, out_msg);
      send_pub_->publish(out_msg);
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      camera_optical_frame_, accum_cloud_frame_, reference_time,
      rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    handleNoCloud(*msg, out_msg);
    send_pub_->publish(out_msg);
    return;
  }

  tf2::Transform tf_cam_from_accum;
  const auto &t = transform.transform.translation;
  const auto &q = transform.transform.rotation;
  tf_cam_from_accum.setOrigin(tf2::Vector3(t.x, t.y, t.z));
  tf_cam_from_accum.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));

  const double theta = toRadians(msg->angle);
  const double gate = toRadians(gate_yaw_);
  const bool use_roi = has_camera_info_ && (msg->roi_radius > 0.0f);
  const double roi_u = static_cast<double>(msg->u);
  const double roi_v = static_cast<double>(msg->v);
  const double roi_scale = roi_scale_ > 0.0 ? roi_scale_ : 1.0;
  const double roi_r = static_cast<double>(msg->roi_radius) * roi_scale;
  const double roi_r2 = roi_r * roi_r;

  size_t selected_points = 0;
  size_t selected_inliers = 0;
  double selected_mad = std::numeric_limits<double>::quiet_NaN();
  double selected_window = window_min_sec_;
  double selected_oldest_age = 0.0;
  bool has_robust_result = false;
  double lidar_distance = 0.0;

  for (double window_sec = window_min_sec_;
    window_sec <= window_max_sec_ + 1e-6;
    window_sec += window_step_sec_)
  {
    std::vector<double> ranges;
    std::vector<double> ages_sec;
    double oldest_age = 0.0;

    for (const auto &frame : cloud_frames) {
      const double age_sec = (reference_time - frame.stamp).seconds();
      if (age_sec < -cloud_sync_tolerance_sec_ || age_sec > window_sec) {
        continue;
      }
      const double non_negative_age = std::max(0.0, age_sec);
      oldest_age = std::max(oldest_age, non_negative_age);

      for (const auto &pt_accum : frame.cloud->points) {
        tf2::Vector3 pt_in(
          static_cast<double>(pt_accum.x),
          static_cast<double>(pt_accum.y),
          static_cast<double>(pt_accum.z));
        tf2::Vector3 pt = tf_cam_from_accum * pt_in;
        const double x = pt.x();
        const double y = pt.y();
        const double z = pt.z();

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z <= 0.0) {
          continue;
        }

        if (use_roi) {
          const double u = fx_ * x / z + cx_;
          const double v = fy_ * y / z + cy_;
          const double du = u - roi_u;
          const double dv = v - roi_v;
          if (du * du + dv * dv > roi_r2) {
            continue;
          }
        } else {
          const double bearing = std::atan2(x, z);
          if (std::abs(bearing - theta) > gate) {
            continue;
          }
        }

        const double range = use_z_as_range_ ? z : std::sqrt(x * x + y * y + z * z);
        if (!std::isfinite(range) || range <= 0.0) {
          continue;
        }
        if (range < range_min_) {
          continue;
        }
        if (range_max_ > 0.0 && range > range_max_) {
          continue;
        }
        if (use_pnp_prior_gate_ && msg->distance > 0.0f) {
          const double prior = static_cast<double>(msg->distance);
          const double tol = pnp_prior_abs_tol_ + pnp_prior_rel_tol_ * prior;
          if (std::abs(range - prior) > tol) {
            continue;
          }
        }
        ranges.push_back(range);
        ages_sec.push_back(non_negative_age);
      }
    }

    if (ranges.size() < min_points_) {
      selected_points = ranges.size();
      selected_window = window_sec;
      selected_oldest_age = oldest_age;
      continue;
    }

    double candidate_distance = 0.0;
    double candidate_mad = std::numeric_limits<double>::quiet_NaN();
    size_t candidate_inliers = 0;
    if (!estimateDistanceRobust(
        ranges, ages_sec, candidate_distance, candidate_mad, candidate_inliers))
    {
      selected_points = ranges.size();
      selected_window = window_sec;
      selected_oldest_age = oldest_age;
      selected_mad = candidate_mad;
      continue;
    }

    has_robust_result = true;
    lidar_distance = candidate_distance;
    selected_mad = candidate_mad;
    selected_points = ranges.size();
    selected_inliers = candidate_inliers;
    selected_window = window_sec;
    selected_oldest_age = oldest_age;
    if (selected_points >= target_points_ || window_sec + 1e-6 >= window_max_sec_) {
      break;
    }
  }

  const bool roi_ok = has_robust_result;
  if (roi_ok) {
    out_msg.distance = static_cast<float>(lidar_distance);
    last_lidar_distance_ = out_msg.distance;
    has_last_lidar_distance_ = true;
  } else if (hold_last_lidar_on_failure_ && has_last_lidar_distance_) {
    out_msg.distance = last_lidar_distance_;
  } else if (fallback_to_pnp_) {
    out_msg.distance = msg->distance;
  } else {
    out_msg.distance = 0.0f;
  }

  const double data_age_ms = (now() - latest_cloud_stamp).seconds() * 1000.0;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "ROI %s | points=%zu inliers=%zu mad=%.4f dist=%.3f win=%.2fs oldest=%.1fms cloud_age=%.1fms",
    roi_ok ? "OK" : "INVALID",
    selected_points,
    selected_inliers,
    selected_mad,
    roi_ok ? lidar_distance : 0.0,
    selected_window,
    selected_oldest_age * 1000.0,
    data_age_ms);

  out_msg.stability = computeStability(msg->stability, roi_ok);
  send_pub_->publish(out_msg);
}

void RangeFusionNode::handleNoCloud(
  const auto_aim_interfaces::msg::Send &in,
  auto_aim_interfaces::msg::Send &out)
{
  if (hold_last_lidar_on_failure_ && has_last_lidar_distance_) {
    out.distance = last_lidar_distance_;
    out.stability = 0;
    return;
  }
  if (fallback_to_pnp_) {
    out.distance = in.distance;
    out.stability = in.stability;
    return;
  }
  out.distance = 0.0f;
  out.stability = 0;
}

void RangeFusionNode::pruneCloudBufferLocked(const rclcpp::Time &newest_stamp)
{
  const double keep_sec =
    window_max_sec_ + std::max(window_step_sec_, 0.0) + cloud_sync_tolerance_sec_ + 0.2;
  while (!cloud_buffer_.empty()) {
    const double age_sec = (newest_stamp - cloud_buffer_.front().stamp).seconds();
    if (age_sec <= keep_sec) {
      break;
    }
    cloud_buffer_.pop_front();
  }
}

bool RangeFusionNode::isZeroStamp(const builtin_interfaces::msg::Time &stamp) const
{
  return stamp.sec == 0 && stamp.nanosec == 0;
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
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double median = values[mid];
  if (values.size() % 2 == 0) {
    const auto max_it = std::max_element(values.begin(), values.begin() + mid);
    median = (*max_it + median) * 0.5;
  }
  return median;
}

double RangeFusionNode::computeWeightedMedian(
  const std::vector<double> &values,
  const std::vector<double> &weights) const
{
  if (values.empty()) {
    return 0.0;
  }

  std::vector<std::pair<double, double>> pairs;
  pairs.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    const double value = values[i];
    const double weight = i < weights.size() ? weights[i] : 1.0;
    if (!std::isfinite(value) || !std::isfinite(weight) || weight <= 0.0) {
      continue;
    }
    pairs.emplace_back(value, weight);
  }

  if (pairs.empty()) {
    return computeMedian(values);
  }

  std::sort(
    pairs.begin(), pairs.end(),
    [](const std::pair<double, double> &a, const std::pair<double, double> &b) {
      return a.first < b.first;
    });

  double total_weight = 0.0;
  for (const auto &item : pairs) {
    total_weight += item.second;
  }
  if (total_weight <= 0.0) {
    return computeMedian(values);
  }

  const double half_weight = total_weight * 0.5;
  double cumulative = 0.0;
  for (const auto &item : pairs) {
    cumulative += item.second;
    if (cumulative >= half_weight) {
      return item.first;
    }
  }
  return pairs.back().first;
}

bool RangeFusionNode::estimateDistanceRobust(
  const std::vector<double> &ranges,
  const std::vector<double> &ages_sec,
  double &distance,
  double &mad_value,
  size_t &inlier_count) const
{
  inlier_count = 0;
  if (ranges.size() < min_points_) {
    return false;
  }

  const double median = computeMedian(ranges);
  std::vector<double> deviations;
  deviations.reserve(ranges.size());
  for (const double value : ranges) {
    deviations.push_back(std::abs(value - median));
  }
  mad_value = computeMedian(deviations);
  if (!std::isfinite(mad_value)) {
    return false;
  }

  const double robust_scale = std::max(mad_value, 1e-3);
  const double gate = mad_k_ * robust_scale;
  if (!std::isfinite(gate) || gate <= 0.0) {
    return false;
  }

  const double tau = std::max(age_tau_sec_, 1e-3);
  std::vector<double> inliers;
  std::vector<double> weights;
  inliers.reserve(ranges.size());
  weights.reserve(ranges.size());

  for (size_t i = 0; i < ranges.size(); ++i) {
    const double value = ranges[i];
    if (!std::isfinite(value) || std::abs(value - median) > gate) {
      continue;
    }

    const double age = i < ages_sec.size() ? std::max(0.0, ages_sec[i]) : 0.0;
    const double weight = std::exp(-age / tau);
    if (!std::isfinite(weight) || weight <= 0.0) {
      continue;
    }

    inliers.push_back(value);
    weights.push_back(weight);
  }

  inlier_count = inliers.size();
  if (inlier_count < min_points_) {
    return false;
  }

  distance = computeWeightedMedian(inliers, weights);
  if (!std::isfinite(distance) || distance <= 0.0) {
    return false;
  }

  if (mad_thresh_ > 0.0 && mad_value > mad_thresh_) {
    return false;
  }

  return true;
}

uint8_t RangeFusionNode::computeStability(uint8_t input, bool roi_ok) const
{
  if (output_stability_logic_ == "or") {
    return static_cast<uint8_t>((input != 0) || roi_ok);
  }
  return static_cast<uint8_t>((input != 0) && roi_ok);
}
}  // namespace rm_livox_fusion

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_livox_fusion::RangeFusionNode>());
  rclcpp::shutdown();
  return 0;
}
