#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rm_livox_fusion
{
class CloudAccumulatorNode : public rclcpp::Node
{
public:
  CloudAccumulatorNode()
  : Node("cloud_accumulator_node"),
    total_points_(0),
    has_stamp_(false)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");
    window_sec_ = this->declare_parameter<double>("window_sec", 3.0);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 5.0);
    voxel_leaf_ = this->declare_parameter<double>("voxel_leaf", 0.03);
    range_min_ = this->declare_parameter<double>("range_min", 2.0);
    range_max_ = this->declare_parameter<double>("range_max", 40.0);
    max_points_ =
      static_cast<size_t>(this->declare_parameter<int64_t>("max_points", 2000000));
    use_tf_at_cloud_stamp_ =
      this->declare_parameter<bool>("use_tf_at_cloud_stamp", true);

    if (publish_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_hz <= 0, fallback to 1.0 Hz");
      publish_hz_ = 1.0;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output_cloud", rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", rclcpp::SensorDataQoS(),
      std::bind(&CloudAccumulatorNode::cloudCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CloudAccumulatorNode::publishTimer, this));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Input cloud frame_id is empty");
      return;
    }

    rclcpp::Time lookup_time = use_tf_at_cloud_stamp_
      ? rclcpp::Time(msg->header.stamp)
      : rclcpp::Time(0, 0, get_clock()->get_clock_type());

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, lookup_time,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
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
    auto filtered = filterCloud(pcl_cloud);
    if (!filtered || filtered->empty()) {
      return;
    }

    rclcpp::Time stamp(cloud_tf.header.stamp);
    std::lock_guard<std::mutex> lock(queue_mutex_);
    cloud_queue_.emplace_back(stamp, filtered);
    total_points_ += filtered->size();
    last_stamp_ = stamp;
    has_stamp_ = true;

    pruneByTimeLocked();
    pruneBySizeLocked();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) const
  {
    auto range_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    range_filtered->points.reserve(input->points.size());

    double min_range = std::max(0.0, range_min_);
    double max_range = range_max_ > 0.0 ? range_max_ : std::numeric_limits<double>::infinity();
    double min2 = min_range * min_range;
    double max2 = max_range * max_range;

    for (const auto &pt : input->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      double d2 = static_cast<double>(pt.x) * pt.x +
        static_cast<double>(pt.y) * pt.y +
        static_cast<double>(pt.z) * pt.z;
      if (d2 < min2 || d2 > max2) {
        continue;
      }
      range_filtered->points.push_back(pt);
    }
    range_filtered->width = static_cast<uint32_t>(range_filtered->points.size());
    range_filtered->height = 1;
    range_filtered->is_dense = true;

    if (voxel_leaf_ <= 0.0) {
      return range_filtered;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_),
      static_cast<float>(voxel_leaf_),
      static_cast<float>(voxel_leaf_));
    voxel.setInputCloud(range_filtered);
    auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel.filter(*downsampled);
    return downsampled;
  }

  void pruneByTimeLocked()
  {
    if (!has_stamp_) {
      return;
    }
    while (!cloud_queue_.empty()) {
      double age_sec = (last_stamp_ - cloud_queue_.front().first).seconds();
      if (age_sec <= window_sec_) {
        break;
      }
      total_points_ -= cloud_queue_.front().second->size();
      cloud_queue_.pop_front();
    }
  }

  void pruneBySizeLocked()
  {
    if (max_points_ == 0) {
      cloud_queue_.clear();
      total_points_ = 0;
      return;
    }
    while (total_points_ > max_points_ && !cloud_queue_.empty()) {
      total_points_ -= cloud_queue_.front().second->size();
      cloud_queue_.pop_front();
    }
  }

  void publishTimer()
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    rclcpp::Time stamp;
    size_t total_points = 0;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (cloud_queue_.empty()) {
        return;
      }
      clouds.reserve(cloud_queue_.size());
      for (const auto &entry : cloud_queue_) {
        clouds.push_back(entry.second);
      }
      stamp = has_stamp_ ? last_stamp_ : now();
      total_points = total_points_;
    }

    auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    merged->points.reserve(total_points);
    for (const auto &cloud : clouds) {
      *merged += *cloud;
    }
    merged->width = static_cast<uint32_t>(merged->points.size());
    merged->height = 1;
    merged->is_dense = true;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*merged, out_msg);
    out_msg.header.frame_id = target_frame_;
    out_msg.header.stamp = stamp;
    cloud_pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<std::pair<rclcpp::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_queue_;
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_livox_fusion::CloudAccumulatorNode>());
  rclcpp::shutdown();
  return 0;
}
