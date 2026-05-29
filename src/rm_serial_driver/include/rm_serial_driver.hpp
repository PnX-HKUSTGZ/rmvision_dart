// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/send.hpp"

namespace rm_serial_driver
{
  class RMSerialDriver : public rclcpp::Node
  {
  public:
    explicit RMSerialDriver(const rclcpp::NodeOptions &options);

    ~RMSerialDriver() override;

  private:
    void getParams();

    void receiveData();

    void sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg);

    void reopenPort();

    void setParam(const rclcpp::Parameter &param);

    void resetTracker();

    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    // Param client to set detect_colr
    using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 0;
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    ResultFuturePtr set_param_future_;

    // Service client to reset tracker
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

    // Aimimg point receiving from serial port for visualization
    visualization_msgs::msg::Marker aiming_point_;

    // Broadcast tf from odom to gimbal_link
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr target_sub_;

    // For debug usage
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // dart_id
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dart_pub_; // 新增

    // --- 新增：比赛模式发布 ---
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr competition_mode_pub_;
    // --- 新增：目标ID发布 ---
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr target_id_pub_;
    // <<< NEW: publisher for incoming offset >>>
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr offset_pub_;

    std::atomic<bool> has_received_dart_id_{false};
    std::atomic<uint8_t> last_received_dart_id_{0};
    std::atomic<bool> dart_id_changed_pending_{false};
    bool initial_activation_sent_{false};
    bool initial_activation_timing_{false};
    rclcpp::Time initial_activation_start_time_;

    std::thread receive_thread_;

    double light_radius_ = 130; // 用于Rivz，需要直接给出数值可视化
    bool force_stability_ = false;
  };
} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
