#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/callback_group.hpp>
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
  void markPublishStateLocked();

public:
  size_t executorThreads() const;

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr publish_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<std::pair<rclcpp::Time, CloudPtr>> cloud_queue_;
  std::mutex queue_mutex_;
  size_t total_points_;
  rclcpp::Time last_stamp_;
  bool has_stamp_;
  size_t queue_version_{0};
  size_t published_version_{0};
  bool pending_publish_{false};

  std::string target_frame_;
  double window_sec_;
  double max_publish_hz_;
  double voxel_leaf_;
  double range_min_;
  double range_max_;
  size_t max_points_;
  bool use_tf_at_cloud_stamp_;
  bool publish_only_on_new_cloud_;
  size_t executor_threads_;
};
}  // namespace rm_livox_fusion
