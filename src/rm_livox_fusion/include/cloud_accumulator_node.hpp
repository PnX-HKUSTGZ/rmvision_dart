#pragma once

#include <rclcpp/rclcpp.hpp>
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
class CloudAccumulatorNode : public rclcpp::Node
{
public:
  CloudAccumulatorNode();

private:
  using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  CloudPtr filterCloud(const CloudPtr &input) const;
  void pruneByTimeLocked();
  void pruneBySizeLocked();
  void publishTimer();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<std::pair<rclcpp::Time, CloudPtr>> cloud_queue_;
  std::mutex queue_mutex_;
  size_t total_points_;
  rclcpp::Time last_stamp_;
  bool has_stamp_;

  std::string target_frame_;
  double window_sec_;
  double publish_hz_;
  double voxel_leaf_;
  double range_min_;
  double range_max_;
  size_t max_points_;
  bool use_tf_at_cloud_stamp_;
};
}  // namespace rm_livox_fusion
