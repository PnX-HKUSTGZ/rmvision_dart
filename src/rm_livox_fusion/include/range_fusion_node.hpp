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

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr send_callback_group_;
  rclcpp::CallbackGroup::SharedPtr camera_info_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;

  std::string camera_optical_frame_;
  std::string accum_cloud_frame_;
  std::string angle_unit_;
  std::string output_stability_logic_;
  double gate_yaw_;
  double roi_scale_;
  bool use_z_as_range_;
  size_t min_points_;
  double mad_thresh_;
  bool fallback_to_pnp_;
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool has_camera_info_{false};
  size_t executor_threads_{3};

  static constexpr double kPi = 3.14159265358979323846;
};
}  // namespace rm_livox_fusion
