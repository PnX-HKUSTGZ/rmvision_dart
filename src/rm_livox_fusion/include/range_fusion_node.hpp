#pragma once

#include <auto_aim_interfaces/msg/send.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <string>
#include <vector>

namespace rm_livox_fusion
{
class RangeFusionNode : public rclcpp::Node
{
public:
  RangeFusionNode();
  size_t executorThreads() const;

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void doorCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);

  void handleNoCloud(
    const auto_aim_interfaces::msg::Send &in,
    auto_aim_interfaces::msg::Send &out);

  rclcpp::Time getLookupTime(
    const auto_aim_interfaces::msg::Send &in,
    const sensor_msgs::msg::PointCloud2 &cloud) const;

  double toRadians(double angle) const;
  double computeMedian(std::vector<double> values) const;
  uint8_t computeStability(uint8_t input, bool roi_ok) const;
  uint8_t evaluateDoorState();
  uint8_t confirmDoorState(uint8_t candidate_state);
  void fillDoorStatePacket(auto_aim_interfaces::msg::Send &msg, uint8_t door_state) const;
  double selectDoorAxis(double x, double y, double z, int axis, double sign) const;
  int parseDoorAxis(const std::string &axis_name, int fallback_axis) const;
  double parseDoorAxisSign(const std::string &axis_name) const;
  void resetRangeFilter();
  void updateRangeFilter(
    double raw_range,
    double raw_lateral,
    double raw_longitudinal,
    double &filtered_range,
    double &filtered_lateral,
    double &filtered_longitudinal);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr door_cloud_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr door_cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr send_callback_group_;
  rclcpp::CallbackGroup::SharedPtr camera_info_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  std::mutex door_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_door_cloud_;
  rclcpp::Time last_door_cloud_received_time_;
  bool has_door_cloud_{false};

  std::string camera_optical_frame_;
  std::string camera_info_topic_;
  std::string accum_cloud_frame_;
  std::string angle_unit_;
  std::string output_stability_logic_;
  double gate_yaw_;
  double roi_scale_;
  bool use_z_as_range_;
  double valid_range_min_;
  double valid_range_max_;
  double pnp_range_gate_;
  double pnp_fallback_lidar_min_{24.0};
  double pnp_fallback_lidar_max_{26.0};
  size_t min_points_;
  double mad_thresh_;
  double range_filter_alpha_;
  double range_filter_jump_threshold_;
  double range_filter_deadband_;
  bool fallback_to_pnp_;
  bool door_state_enable_{true};
  std::string door_cloud_topic_;
  double door_open_distance_threshold_{1.0};
  double door_opening_width_{0.45};
  double door_opening_height_{0.70};
  double door_roi_margin_{0.0};
  double door_roi_center_lateral_{0.0};
  double door_roi_center_vertical_{0.0};
  double door_front_min_{0.15};
  double door_front_max_{1.3};
  double door_open_evidence_max_{30.0};
  size_t door_min_points_{5};
  size_t door_open_min_points_{3};
  size_t door_confirm_frames_{3};
  double door_cloud_timeout_sec_{0.2};
  int door_forward_axis_{0};
  int door_lateral_axis_{1};
  int door_vertical_axis_{2};
  double door_forward_sign_{1.0};
  double door_lateral_sign_{1.0};
  double door_vertical_sign_{1.0};
  double last_door_nearest_distance_{-1.0};
  uint8_t pending_door_state_{0};
  size_t pending_door_state_count_{0};
  uint8_t confirmed_door_state_{0};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool has_camera_info_{false};
  size_t executor_threads_{3};
  bool has_filtered_range_{false};
  double filtered_range_{0.0};
  double filtered_lateral_{0.0};
  double filtered_longitudinal_{0.0};

  static constexpr double kPi = 3.14159265358979323846;
};
}  // namespace rm_livox_fusion
