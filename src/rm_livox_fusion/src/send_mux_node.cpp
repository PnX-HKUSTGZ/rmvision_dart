#include <auto_aim_interfaces/msg/send.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cmath>
#include <string>

namespace rm_livox_fusion
{
namespace
{
constexpr float kNoTargetDistance = 666.0f;
constexpr float kNoTargetAngle = 1234.0f;

bool isNoTargetPacket(const auto_aim_interfaces::msg::Send & msg)
{
  return (std::abs(msg.distance - kNoTargetDistance) < 1e-3f &&
    (std::abs(msg.angle - kNoTargetAngle) < 1e-3f ||
     std::abs(msg.pixel_angle - kNoTargetAngle) < 1e-3f)) ||
    (msg.distance <= 0.0f && std::abs(msg.angle - kNoTargetAngle) < 1e-3f);
}
}  // namespace

class SendMuxNode : public rclcpp::Node
{
public:
  SendMuxNode()
  : Node("send_mux")
  {
    base_topic_ =
      declare_parameter<std::string>("base_topic", "/base/Send_fused");
    outpost_topic_ =
      declare_parameter<std::string>("outpost_topic", "/outpost/Send_fused");
    target_id_topic_ =
      declare_parameter<std::string>("target_id_topic", "/target_id");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/Send");
    timeout_sec_ = declare_parameter<double>("timeout_sec", 0.2);
    if (timeout_sec_ <= 0.0) {
      timeout_sec_ = 0.2;
    }

    send_pub_ = create_publisher<auto_aim_interfaces::msg::Send>(
      output_topic_, rclcpp::SensorDataQoS());
    base_sub_ = create_subscription<auto_aim_interfaces::msg::Send>(
      base_topic_, rclcpp::SensorDataQoS(),
      [this](auto_aim_interfaces::msg::Send::SharedPtr msg) {
        base_msg_ = *msg;
        base_stamp_ = now();
        has_base_ = true;
        publishIfActive(1);
      });
    outpost_sub_ = create_subscription<auto_aim_interfaces::msg::Send>(
      outpost_topic_, rclcpp::SensorDataQoS(),
      [this](auto_aim_interfaces::msg::Send::SharedPtr msg) {
        outpost_msg_ = *msg;
        outpost_stamp_ = now();
        has_outpost_ = true;
        publishIfActive(0);
      });
    target_id_sub_ = create_subscription<std_msgs::msg::UInt8>(
      target_id_topic_, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        if (has_target_id_ && target_id_ == msg->data) {
          return;
        }
        target_id_ = msg->data;
        has_target_id_ = true;
        RCLCPP_INFO(get_logger(), "send_mux target_id=%u", target_id_);
        publishSelectedOrNoTarget();
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SendMuxNode::checkSelectedTimeout, this));
  }

private:
  void publishIfActive(uint8_t source_target_id)
  {
    if (target_id_ == source_target_id) {
      publishSelectedOrNoTarget();
    }
  }

  void publishSelectedOrNoTarget()
  {
    if (target_id_ == 1) {
      publishCandidate(base_msg_, base_stamp_, has_base_);
      return;
    }
    if (target_id_ == 0) {
      publishCandidate(outpost_msg_, outpost_stamp_, has_outpost_);
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Unknown target_id=%u, publishing no target", target_id_);
    publishNoTarget();
  }

  void publishCandidate(
    const auto_aim_interfaces::msg::Send & msg,
    const rclcpp::Time & stamp,
    bool has_msg)
  {
    if (!has_msg || (now() - stamp).seconds() > timeout_sec_ || isNoTargetPacket(msg)) {
      publishNoTarget();
      return;
    }
    send_pub_->publish(msg);
  }

  void publishNoTarget()
  {
    auto msg = auto_aim_interfaces::msg::Send();
    msg.header.stamp = now();
    msg.distance = kNoTargetDistance;
    msg.angle = kNoTargetAngle;
    msg.pixel_angle = kNoTargetAngle;
    msg.longitudinal_distance = 1111.0f;
    msg.lateral_distance = 2222.0f;
    msg.u = 0.0f;
    msg.v = 0.0f;
    msg.roi_radius = 0.0f;
    msg.stability = 0;
    send_pub_->publish(msg);
  }

  void checkSelectedTimeout()
  {
    if (target_id_ == 1) {
      publishNoTargetIfStale(base_stamp_, has_base_);
      return;
    }
    if (target_id_ == 0) {
      publishNoTargetIfStale(outpost_stamp_, has_outpost_);
      return;
    }
    publishNoTarget();
  }

  void publishNoTargetIfStale(const rclcpp::Time & stamp, bool has_msg)
  {
    if (!has_msg || (now() - stamp).seconds() > timeout_sec_) {
      publishNoTarget();
    }
  }

  std::string base_topic_;
  std::string outpost_topic_;
  std::string target_id_topic_;
  std::string output_topic_;
  double timeout_sec_{0.2};
  uint8_t target_id_{1};
  bool has_target_id_{false};

  auto_aim_interfaces::msg::Send base_msg_;
  auto_aim_interfaces::msg::Send outpost_msg_;
  rclcpp::Time base_stamp_;
  rclcpp::Time outpost_stamp_;
  bool has_base_{false};
  bool has_outpost_{false};

  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr base_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr outpost_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_id_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rm_livox_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_livox_fusion::SendMuxNode>());
  rclcpp::shutdown();
  return 0;
}
