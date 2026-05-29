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
namespace
{
constexpr float kNoTargetDistance = -1.0f;
constexpr float kNoTargetAngle = 666.0f;
constexpr uint8_t kLightNotDetected = 0;
constexpr uint8_t kLightVisible = 1;
constexpr uint8_t kDoorOpenLightOccluded = 2;
constexpr uint8_t kDoorBlocked = 3;

bool hasValidTarget(const auto_aim_interfaces::msg::Send &msg)
{
  return msg.light_detected == kLightVisible &&
    std::isfinite(msg.distance) && msg.distance > 0.0f &&
    std::isfinite(msg.pixel_angle) &&
    std::abs(msg.distance - kNoTargetDistance) > 1e-3f &&
    std::abs(msg.pixel_angle - kNoTargetAngle) > 1e-3f;
}

bool isNoTargetPacket(const auto_aim_interfaces::msg::Send &msg)
{
  return !hasValidTarget(msg);
}

void fillNoTargetPacket(auto_aim_interfaces::msg::Send &msg)
{
  msg.distance = kNoTargetDistance;
  msg.angle = kNoTargetAngle;
  msg.pixel_angle = kNoTargetAngle;
  msg.longitudinal_distance = kNoTargetDistance;
  msg.lateral_distance = kNoTargetDistance;
  msg.u = 0.0f;
  msg.v = 0.0f;
  msg.roi_radius = 0.0f;
  msg.door_nearest_distance = kNoTargetDistance;
  msg.stability = 0;
  msg.light_detected = kLightNotDetected;
}
}  // namespace

RangeFusionNode::RangeFusionNode()
: Node("range_fusion_node")
{
  camera_optical_frame_ =
    this->declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
  camera_info_topic_ =
    this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
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
  range_filter_alpha_ =
    this->declare_parameter<double>("range_filter_alpha", 1.0);
  range_filter_jump_threshold_ =
    this->declare_parameter<double>("range_filter_jump_threshold", 1.0);
  range_filter_deadband_ =
    this->declare_parameter<double>("range_filter_deadband", 0.0);
  if (range_filter_alpha_ <= 0.0 || range_filter_alpha_ > 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "range_filter_alpha must be in (0, 1], fallback to 1.0");
    range_filter_alpha_ = 1.0;
  }
  if (range_filter_jump_threshold_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "range_filter_jump_threshold < 0, fallback to 0.0");
    range_filter_jump_threshold_ = 0.0;
  }
  if (range_filter_deadband_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "range_filter_deadband < 0, fallback to 0.0");
    range_filter_deadband_ = 0.0;
  }
  fallback_to_pnp_ = this->declare_parameter<bool>("fallback_to_pnp", true);
  door_state_enable_ = this->declare_parameter<bool>("door_state_enable", true);
  door_cloud_topic_ =
    this->declare_parameter<std::string>("door_cloud_topic", "/livox/lidar");
  door_open_distance_threshold_ =
    this->declare_parameter<double>("door_open_distance_threshold", 1.0);
  door_opening_width_ = this->declare_parameter<double>("door_opening_width", 0.45);
  door_opening_height_ = this->declare_parameter<double>("door_opening_height", 0.70);
  door_roi_margin_ = this->declare_parameter<double>("door_roi_margin", 0.0);
  door_roi_center_lateral_ =
    this->declare_parameter<double>("door_roi_center_lateral", 0.0);
  door_roi_center_vertical_ =
    this->declare_parameter<double>("door_roi_center_vertical", 0.0);
  door_front_min_ = this->declare_parameter<double>("door_front_min", 0.15);
  door_front_max_ = this->declare_parameter<double>("door_front_max", 1.3);
  door_open_evidence_max_ =
    this->declare_parameter<double>("door_open_evidence_max", 30.0);
  door_min_points_ =
    static_cast<size_t>(this->declare_parameter<int64_t>("door_min_points", 5));
  door_open_min_points_ =
    static_cast<size_t>(this->declare_parameter<int64_t>("door_open_min_points", 3));
  door_confirm_frames_ =
    static_cast<size_t>(this->declare_parameter<int64_t>("door_confirm_frames", 3));
  door_cloud_timeout_sec_ =
    this->declare_parameter<double>("door_cloud_timeout_sec", 0.2);
  const std::string door_forward_axis =
    this->declare_parameter<std::string>("door_forward_axis", "x");
  const std::string door_lateral_axis =
    this->declare_parameter<std::string>("door_lateral_axis", "y");
  const std::string door_vertical_axis =
    this->declare_parameter<std::string>("door_vertical_axis", "z");
  door_forward_axis_ = parseDoorAxis(door_forward_axis, 0);
  door_lateral_axis_ = parseDoorAxis(door_lateral_axis, 1);
  door_vertical_axis_ = parseDoorAxis(door_vertical_axis, 2);
  door_forward_sign_ = parseDoorAxisSign(door_forward_axis);
  door_lateral_sign_ = parseDoorAxisSign(door_lateral_axis);
  door_vertical_sign_ = parseDoorAxisSign(door_vertical_axis);
  if (door_open_distance_threshold_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "door_open_distance_threshold must be positive, fallback to 1.0");
    door_open_distance_threshold_ = 1.0;
  }
  if (door_opening_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "door_opening_width must be positive, fallback to 0.45");
    door_opening_width_ = 0.45;
  }
  if (door_opening_height_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "door_opening_height must be positive, fallback to 0.70");
    door_opening_height_ = 0.70;
  }
  if (door_roi_margin_ < 0.0) {
    door_roi_margin_ = 0.0;
  }
  if (door_front_min_ < 0.0) {
    door_front_min_ = 0.0;
  }
  if (door_front_max_ <= door_front_min_) {
    RCLCPP_WARN(get_logger(), "door_front_max <= door_front_min, fallback to 1.3");
    door_front_max_ = 1.3;
  }
  if (door_open_evidence_max_ <= door_open_distance_threshold_) {
    RCLCPP_WARN(
      get_logger(), "door_open_evidence_max <= door_open_distance_threshold, fallback to 30.0");
    door_open_evidence_max_ = 30.0;
  }
  if (door_min_points_ < 1) {
    door_min_points_ = 1;
  }
  if (door_open_min_points_ < 1) {
    door_open_min_points_ = 1;
  }
  if (door_confirm_frames_ < 1) {
    door_confirm_frames_ = 1;
  }
  if (door_cloud_timeout_sec_ <= 0.0) {
    door_cloud_timeout_sec_ = 0.2;
  }
  output_stability_logic_ =
    this->declare_parameter<std::string>("output_stability_logic", "and");
  executor_threads_ =
    static_cast<size_t>(std::max<int64_t>(
      1, this->declare_parameter<int64_t>("executor_threads", 3)));

  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  door_cloud_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
  rclcpp::SubscriptionOptions door_cloud_sub_options;
  door_cloud_sub_options.callback_group = door_cloud_callback_group_;
  door_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    door_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RangeFusionNode::doorCloudCallback, this, std::placeholders::_1),
    door_cloud_sub_options);
  rclcpp::SubscriptionOptions camera_info_sub_options;
  camera_info_sub_options.callback_group = camera_info_callback_group_;
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
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

void RangeFusionNode::doorCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(door_cloud_mutex_);
  last_door_cloud_ = msg;
  last_door_cloud_received_time_ = now();
  has_door_cloud_ = true;
}

void RangeFusionNode::sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  if (isNoTargetPacket(*msg)) {
    resetRangeFilter();
    auto out_msg = *msg;
    const uint8_t door_state = evaluateDoorState();
    fillDoorStatePacket(out_msg, door_state);
    send_pub_->publish(out_msg);
    return;
  }

  auto out_msg = auto_aim_interfaces::msg::Send();
  out_msg.header = msg->header;
  out_msg.angle = kNoTargetAngle;
  out_msg.pixel_angle = msg->pixel_angle;
  out_msg.longitudinal_distance = 0.0f;
  out_msg.lateral_distance = 0.0f;
  out_msg.u = msg->u;
  out_msg.v = msg->v;
  out_msg.roi_radius = msg->roi_radius;
  out_msg.door_nearest_distance = kNoTargetDistance;
  out_msg.light_detected = kLightNotDetected;

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
    resetRangeFilter();
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
    updateRangeFilter(
      lidar_distance, lateral_distance, longitudinal_distance,
      lidar_distance, lateral_distance, longitudinal_distance);
    actual_angle = std::atan2(lateral_distance, longitudinal_distance);
    out_msg.distance = static_cast<float>(lidar_distance);
    out_msg.lateral_distance = static_cast<float>(lateral_distance);
    out_msg.longitudinal_distance = static_cast<float>(longitudinal_distance);
    if (angle_unit_ == "rad") {
      out_msg.angle = static_cast<float>(actual_angle);
    } else {
      out_msg.angle = static_cast<float>(actual_angle * 180.0 / kPi);
    }
    out_msg.light_detected = kLightVisible;
  } else if (fallback_to_pnp_) {
    resetRangeFilter();
    out_msg.distance = msg->distance;
    out_msg.angle = msg->angle;
    out_msg.pixel_angle = msg->pixel_angle;
    out_msg.longitudinal_distance = msg->longitudinal_distance;
    out_msg.lateral_distance = msg->lateral_distance;
    out_msg.light_detected = hasValidTarget(*msg) ? kLightVisible : kLightNotDetected;
  } else {
    resetRangeFilter();
    fillNoTargetPacket(out_msg);
  }

  out_msg.stability =
    out_msg.light_detected == kLightVisible ? computeStability(msg->stability, roi_ok) : 0;
  send_pub_->publish(out_msg);
}

void RangeFusionNode::handleNoCloud(
  const auto_aim_interfaces::msg::Send &in,
  auto_aim_interfaces::msg::Send &out)
{
  out.angle = kNoTargetAngle;
  out.pixel_angle = in.pixel_angle;
  out.longitudinal_distance = 0.0f;
  out.lateral_distance = 0.0f;
  if (fallback_to_pnp_ && hasValidTarget(in)) {
    resetRangeFilter();
    out.distance = in.distance;
    out.angle = in.angle;
    out.longitudinal_distance = in.longitudinal_distance;
    out.lateral_distance = in.lateral_distance;
    out.stability = in.stability;
    out.light_detected = kLightVisible;
    return;
  }
  fillNoTargetPacket(out);
  resetRangeFilter();
}

uint8_t RangeFusionNode::evaluateDoorState()
{
  last_door_nearest_distance_ = kNoTargetDistance;
  if (!door_state_enable_) {
    return kLightNotDetected;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  rclcpp::Time received_time;
  bool has_cloud = false;
  {
    std::lock_guard<std::mutex> lock(door_cloud_mutex_);
    cloud = last_door_cloud_;
    received_time = last_door_cloud_received_time_;
    has_cloud = has_door_cloud_;
  }

  if (!has_cloud || !cloud) {
    return confirmDoorState(kLightNotDetected);
  }
  if ((now() - received_time).seconds() > door_cloud_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Door cloud timeout: %.3fs", (now() - received_time).seconds());
    return confirmDoorState(kLightNotDetected);
  }

  const double half_width = door_opening_width_ * 0.5 + door_roi_margin_;
  const double half_height = door_opening_height_ * 0.5 + door_roi_margin_;
  size_t valid_points = 0;
  size_t roi_points = 0;
  size_t blocked_points = 0;
  size_t open_evidence_points = 0;
  double nearest_range = std::numeric_limits<double>::infinity();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double source_x = static_cast<double>(*iter_x);
    const double source_y = static_cast<double>(*iter_y);
    const double source_z = static_cast<double>(*iter_z);
    if (!std::isfinite(source_x) || !std::isfinite(source_y) || !std::isfinite(source_z)) {
      continue;
    }
    ++valid_points;

    const double forward =
      selectDoorAxis(source_x, source_y, source_z, door_forward_axis_, door_forward_sign_);
    if (forward < door_front_min_ || forward > door_open_evidence_max_) {
      continue;
    }

    const double lateral =
      selectDoorAxis(source_x, source_y, source_z, door_lateral_axis_, door_lateral_sign_);
    if (std::abs(lateral - door_roi_center_lateral_) > half_width) {
      continue;
    }

    const double vertical =
      selectDoorAxis(source_x, source_y, source_z, door_vertical_axis_, door_vertical_sign_);
    if (std::abs(vertical - door_roi_center_vertical_) > half_height) {
      continue;
    }

    ++roi_points;
    nearest_range = std::min(nearest_range, forward);
    if (forward <= door_open_distance_threshold_ && forward <= door_front_max_) {
      ++blocked_points;
    } else if (forward > door_open_distance_threshold_) {
      ++open_evidence_points;
    }
  }

  uint8_t candidate_state = kLightNotDetected;
  if (valid_points == 0) {
    candidate_state = kLightNotDetected;
  } else if (blocked_points >= door_min_points_) {
    candidate_state = kDoorBlocked;
  } else if (open_evidence_points >= door_open_min_points_) {
    candidate_state = kDoorOpenLightOccluded;
  }
  if (std::isfinite(nearest_range)) {
    last_door_nearest_distance_ = nearest_range;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Door ROI state=%u valid=%zu points=%zu blocked=%zu open=%zu nearest=%.3f",
    candidate_state, valid_points, roi_points, blocked_points, open_evidence_points,
    std::isfinite(nearest_range) ? nearest_range : -1.0);

  return confirmDoorState(candidate_state);
}

uint8_t RangeFusionNode::confirmDoorState(uint8_t candidate_state)
{
  if (candidate_state != pending_door_state_) {
    pending_door_state_ = candidate_state;
    pending_door_state_count_ = 1;
  } else {
    ++pending_door_state_count_;
  }

  if (candidate_state == kLightNotDetected) {
    confirmed_door_state_ = kLightNotDetected;
    return confirmed_door_state_;
  }

  if (pending_door_state_count_ >= door_confirm_frames_) {
    confirmed_door_state_ = candidate_state;
  }
  return confirmed_door_state_;
}

void RangeFusionNode::fillDoorStatePacket(
  auto_aim_interfaces::msg::Send &msg, uint8_t door_state) const
{
  fillNoTargetPacket(msg);
  msg.light_detected = door_state;
  msg.door_nearest_distance = static_cast<float>(last_door_nearest_distance_);
}

double RangeFusionNode::selectDoorAxis(
  double x, double y, double z, int axis, double sign) const
{
  double value = x;
  if (axis == 1) {
    value = y;
  } else if (axis == 2) {
    value = z;
  }
  return sign * value;
}

int RangeFusionNode::parseDoorAxis(const std::string &axis_name, int fallback_axis) const
{
  std::string normalized = axis_name;
  if (!normalized.empty() && (normalized.front() == '-' || normalized.front() == '+')) {
    normalized.erase(normalized.begin());
  }
  if (normalized == "x") {
    return 0;
  }
  if (normalized == "y") {
    return 1;
  }
  if (normalized == "z") {
    return 2;
  }
  RCLCPP_WARN(
    get_logger(), "Invalid door axis '%s', using fallback axis index %d",
    axis_name.c_str(), fallback_axis);
  return fallback_axis;
}

double RangeFusionNode::parseDoorAxisSign(const std::string &axis_name) const
{
  return (!axis_name.empty() && axis_name.front() == '-') ? -1.0 : 1.0;
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

void RangeFusionNode::resetRangeFilter()
{
  has_filtered_range_ = false;
  filtered_range_ = 0.0;
  filtered_lateral_ = 0.0;
  filtered_longitudinal_ = 0.0;
}

void RangeFusionNode::updateRangeFilter(
  double raw_range,
  double raw_lateral,
  double raw_longitudinal,
  double &filtered_range,
  double &filtered_lateral,
  double &filtered_longitudinal)
{
  const bool jump_reset =
    has_filtered_range_ && range_filter_jump_threshold_ > 0.0 &&
    std::abs(raw_range - filtered_range_) > range_filter_jump_threshold_;
  if (!has_filtered_range_ || range_filter_alpha_ >= 1.0 || jump_reset) {
    filtered_range_ = raw_range;
    filtered_lateral_ = raw_lateral;
    filtered_longitudinal_ = raw_longitudinal;
    has_filtered_range_ = true;
  } else if (
    range_filter_deadband_ > 0.0 &&
    std::abs(raw_range - filtered_range_) <= range_filter_deadband_) {
    // Keep the previous value when the lidar range only jitters within the noise band.
  } else {
    const double keep = 1.0 - range_filter_alpha_;
    filtered_range_ = keep * filtered_range_ + range_filter_alpha_ * raw_range;
    filtered_lateral_ =
      keep * filtered_lateral_ + range_filter_alpha_ * raw_lateral;
    filtered_longitudinal_ =
      keep * filtered_longitudinal_ + range_filter_alpha_ * raw_longitudinal;
  }
  filtered_range = filtered_range_;
  filtered_lateral = filtered_lateral_;
  filtered_longitudinal = filtered_longitudinal_;
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
