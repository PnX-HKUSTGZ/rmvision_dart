#pragma once

#include <auto_aim_interfaces/msg/send.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rm_livox_fusion
{
class RangeFusionNode : public rclcpp::Node
{
public:
  RangeFusionNode();

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg);

  void handleNoCloud(
    const auto_aim_interfaces::msg::Send &in,
    auto_aim_interfaces::msg::Send &out);

  void pruneCloudBufferLocked(const rclcpp::Time &newest_stamp);
  bool isZeroStamp(const builtin_interfaces::msg::Time &stamp) const;
  double computeWeightedMedian(
    const std::vector<double> &values,
    const std::vector<double> &weights) const;
  double toRadians(double angle) const;
  double computeMedian(std::vector<double> values) const;
  bool estimateDistanceRobust(
    const std::vector<double> &ranges,
    const std::vector<double> &ages_sec,
    double &distance,
    double &mad_value,
    size_t &inlier_count) const;
  uint8_t computeStability(uint8_t input, bool roi_ok) const;

  using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
  struct CloudFrame
  {
    rclcpp::Time stamp;
    CloudPtr cloud;
  };

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex cloud_mutex_;
  std::deque<CloudFrame> cloud_buffer_;

  std::string camera_optical_frame_;
  std::string accum_cloud_frame_;
  std::string angle_unit_;
  std::string output_stability_logic_;
  double gate_yaw_;
  double roi_scale_;
  bool use_z_as_range_;
  size_t min_points_;
  double mad_thresh_;
  double window_min_sec_;
  double window_max_sec_;
  double window_step_sec_;
  size_t target_points_;
  double age_tau_sec_;
  double mad_k_;
  double cloud_sync_tolerance_sec_;
  double range_min_;
  double range_max_;
  bool use_pnp_prior_gate_;
  double pnp_prior_rel_tol_;
  double pnp_prior_abs_tol_;
  bool hold_last_lidar_on_failure_;
  bool fallback_to_pnp_;
  float last_lidar_distance_{0.0f};
  bool has_last_lidar_distance_{false};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  bool has_camera_info_{false};

  static constexpr double kPi = 3.14159265358979323846;
};
}  // namespace rm_livox_fusion
