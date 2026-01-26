#include <auto_aim_interfaces/msg/send.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rm_livox_fusion
{
class RangeFusionNode : public rclcpp::Node
{
public:
  RangeFusionNode()
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

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
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

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_cloud_ = msg;
  }

  void sendCallback(const auto_aim_interfaces::msg::Send::SharedPtr msg)
  {
    auto out_msg = auto_aim_interfaces::msg::Send();
    out_msg.header = msg->header;
    out_msg.angle = msg->angle;
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

    sensor_msgs::msg::PointCloud2 cloud_tf;
    try {
      tf2::doTransform(*cloud, cloud_tf, transform);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cloud transform failed: %s", ex.what());
      handleNoCloud(*msg, out_msg);
      send_pub_->publish(out_msg);
      return;
    }

    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(cloud_tf, *pcl_cloud);

    double theta = toRadians(msg->angle);
    double gate = toRadians(gate_yaw_);
    bool use_roi = has_camera_info_ && (msg->roi_radius > 0.0f);
    double roi_u = static_cast<double>(msg->u);
    double roi_v = static_cast<double>(msg->v);
    double roi_r = static_cast<double>(msg->roi_radius);
    double roi_scale = roi_scale_ > 0.0 ? roi_scale_ : 1.0;
    roi_r *= roi_scale;
    double roi_r2 = roi_r * roi_r;

    std::vector<double> ranges;
    ranges.reserve(pcl_cloud->points.size());
    for (const auto &pt : pcl_cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      if (pt.z <= 0.0f) {
        continue;
      }
      if (use_roi) {
        double u = fx_ * static_cast<double>(pt.x) / static_cast<double>(pt.z) + cx_;
        double v = fy_ * static_cast<double>(pt.y) / static_cast<double>(pt.z) + cy_;
        double du = u - roi_u;
        double dv = v - roi_v;
        if (du * du + dv * dv > roi_r2) {
          continue;
        }
      } else {
        double bearing = std::atan2(static_cast<double>(pt.x), static_cast<double>(pt.z));
        if (std::abs(bearing - theta) > gate) {
          continue;
        }
      }
      double range = 0.0;
      if (use_z_as_range_) {
        range = static_cast<double>(pt.z);
      } else {
        range = std::sqrt(static_cast<double>(pt.x) * pt.x +
                          static_cast<double>(pt.z) * pt.z);
      }
      ranges.push_back(range);
    }

    bool roi_ok = false;
    double lidar_distance = 0.0;
    if (ranges.size() >= min_points_) {
      double median = computeMedian(ranges);
      std::vector<double> deviations;
      deviations.reserve(ranges.size());
      for (double value : ranges) {
        deviations.push_back(std::abs(value - median));
      }
      double mad = computeMedian(deviations);
      if (mad <= mad_thresh_) {
        roi_ok = true;
        lidar_distance = median;
      }
    }

    if (roi_ok) {
      out_msg.distance = static_cast<float>(lidar_distance);
    } else if (fallback_to_pnp_) {
      out_msg.distance = msg->distance;
    } else {
      out_msg.distance = 0.0f;
    }

    out_msg.stability = computeStability(msg->stability, roi_ok);
    send_pub_->publish(out_msg);
  }

  void handleNoCloud(
    const auto_aim_interfaces::msg::Send &in,
    auto_aim_interfaces::msg::Send &out)
  {
    if (fallback_to_pnp_) {
      out.distance = in.distance;
      out.stability = in.stability;
      return;
    }
    out.distance = 0.0f;
    out.stability = 0;
  }

  rclcpp::Time getLookupTime(
    const auto_aim_interfaces::msg::Send &in,
    const sensor_msgs::msg::PointCloud2 &cloud) const
  {
    bool has_send_stamp = (in.header.stamp.sec != 0 || in.header.stamp.nanosec != 0);
    if (has_send_stamp) {
      return rclcpp::Time(in.header.stamp);
    }
    return rclcpp::Time(cloud.header.stamp);
  }

  double toRadians(double angle) const
  {
    if (angle_unit_ == "rad") {
      return angle;
    }
    return angle * kPi / 180.0;
  }

  double computeMedian(std::vector<double> values) const
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

  uint8_t computeStability(uint8_t input, bool roi_ok) const
  {
    if (output_stability_logic_ == "or") {
      return static_cast<uint8_t>((input != 0) || roi_ok);
    }
    return static_cast<uint8_t>((input != 0) && roi_ok);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

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

  static constexpr double kPi = 3.14159265358979323846;
};
}  // namespace rm_livox_fusion

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_livox_fusion::RangeFusionNode>());
  rclcpp::shutdown();
  return 0;
}
