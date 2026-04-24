// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstring>

#include "../include/crc.hpp"
#include "../include/packet.hpp"
#include "../include/rm_serial_driver.hpp"
#include "auto_aim_interfaces/msg/send.hpp"

namespace rm_serial_driver
{
  RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
      : Node("rm_serial_driver", options),
        owned_ctx_{new IoContext(2)},
        serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
  {
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

    getParams();

    // TF broadcaster
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create Publisher
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    dart_pub_ = this->create_publisher<std_msgs::msg::UInt8>("current_dart_id", 10);

    // --- 新增：初始化比赛模式 publisher ---
    competition_mode_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("competition_mode", 10);
    // --- 新增：初始化目标ID publisher ---
    target_id_pub_ =
        this->create_publisher<std_msgs::msg::UInt8>("target_id", 10);

    // <<< NEW: initialize offset publisher >>>
    offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("offset", 10);

    // Detect parameter client
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "light_detector");

    // Tracker reset service client
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

    try
    {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open())
      {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
      throw ex;
    }

    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::CYLINDER;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = 2 * light_radius_;
    aiming_point_.scale.y = 2 * light_radius_;
    aiming_point_.scale.z = 0.01;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Create Subscription
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
        "/Send", rclcpp::SensorDataQoS(),
        std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  }

  RMSerialDriver::~RMSerialDriver()
  {
    if (receive_thread_.joinable())
    {
      receive_thread_.join();
    }

    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
      owned_ctx_->waitForExit();
    }
  }

  void RMSerialDriver::receiveData()
  {
    constexpr size_t PACKET_SIZE = sizeof(ReceivePacket);
    std::vector<uint8_t> header_buf(1);
    std::vector<uint8_t> buf(PACKET_SIZE - 1); // exclude header

    while (rclcpp::ok())
    {
      try
      {
        serial_driver_->port()->receive(header_buf);
        uint8_t header = header_buf[0];

        if (header == 0x5A)
        {
          serial_driver_->port()->receive(buf);

          std::vector<uint8_t> raw;
          raw.reserve(PACKET_SIZE);
          raw.push_back(header);
          raw.insert(raw.end(), buf.begin(), buf.end());

          if (!crc16::Verify_CRC16_Check_Sum(raw.data(), raw.size()))
          {
            RCLCPP_WARN(get_logger(), "CRC16 verification failed.");
            continue;
          }

          // 手动解析字段
          ReceivePacket packet;
          packet.header = raw[0];
          packet.competition_mode_ = raw[1];
          packet.target_id_ = raw[2];
          packet.dart_id = raw[3];
          // 解析 float offset（使用 memcpy 以避免别名和对齐问题）
          std::memcpy(&packet.offset, &raw[4], sizeof(float));

          // 解析 checksum（little-endian）
          packet.checksum = static_cast<uint16_t>(raw[8]) |
                            (static_cast<uint16_t>(raw[9]) << 8);

          // 发布字段
          std_msgs::msg::UInt8 comp_msg;
          comp_msg.data = packet.competition_mode_;
          competition_mode_pub_->publish(comp_msg);

          std_msgs::msg::UInt8 dart_msg;
          dart_msg.data = packet.dart_id;
          dart_pub_->publish(dart_msg);
          if (has_received_dart_id_.load(std::memory_order_relaxed) &&
              packet.dart_id != last_received_dart_id_.load(std::memory_order_relaxed))
          {
            dart_id_changed_pending_.store(true, std::memory_order_relaxed);
          }
          last_received_dart_id_.store(packet.dart_id, std::memory_order_relaxed);
          has_received_dart_id_.store(true, std::memory_order_relaxed);

          std_msgs::msg::UInt8 target_msg;
          target_msg.data = packet.target_id_;
          target_id_pub_->publish(target_msg);

          std_msgs::msg::Float32 offset_msg;
          offset_msg.data = packet.offset;
          offset_pub_->publish(offset_msg);

          RCLCPP_DEBUG(get_logger(),
                       "Parsed packet: target_id=%u, dart_id=%u, offset=%.3f",
                       packet.target_id_, packet.dart_id, packet.offset);
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Invalid header: 0x%02X", header);
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Serial read error: %s", e.what());
        reopenPort();
      }
    }
  }

  void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(),
                "[SerialDriver] 收到 Send 消息:distance=%.2f, pixel_angle=%.2f, real_angle=%.2f, long=%.2f, lat=%.2f",
                msg->distance, msg->pixel_angle, msg->angle,
                msg->longitudinal_distance, msg->lateral_distance);

    const static std::map<std::string, uint8_t> id_unit8_map{
        {"", 0}, {"outpost", 0}, {"1", 1}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}};

    try
    {
      SendPacket packet;
      packet.distance = msg->distance;
      // 电控当前需要视觉像素角；雷达融合角仅用于日志调试。
      packet.angle = msg->pixel_angle;
      packet.longitudinal_distance = msg->longitudinal_distance;
      packet.lateral_distance = msg->lateral_distance;
      packet.dart_id_change_flag = 1;
      // 将收到的 stability 放入包中
      packet.stability = msg->stability;

      // 先计算 CRC
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

      // 转成字节向量
      std::vector<uint8_t> data = toVector(packet);

      // —— 在这里插入日志输出 —— //

      // 1) 打印逻辑字段
      RCLCPP_INFO(get_logger(),
                  ">> Sending packet: distance=%.2f, sent_angle=%.2f, pixel_angle=%.2f, real_angle=%.2f, long=%.2f, lat=%.2f, dart_flag=%u, stability=%u",
                  packet.distance,
                  packet.angle,
                  msg->pixel_angle,
                  msg->angle,
                  packet.longitudinal_distance,
                  packet.lateral_distance,
                  packet.dart_id_change_flag,
                  packet.stability);

      // 2) 打印原始字节（十六进制）
      {
        std::ostringstream ss;
        ss << std::hex << std::setfill('0');
        for (auto byte : data)
        {
          ss << std::setw(2) << static_cast<int>(byte) << " ";
        }
        RCLCPP_DEBUG(get_logger(), ">> Raw bytes: %s", ss.str().c_str());
      }

      // 真正发送
      serial_driver_->port()->send(data);

      // 计算并发布延迟
      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
      RCLCPP_INFO_STREAM(get_logger(),
                         "Total latency: " << latency.data << " ms");
      latency_pub_->publish(latency);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(),
                   "Error while sending data: %s", ex.what());
      reopenPort();
    }
  }

  void RMSerialDriver::getParams()
  {
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
      device_name_ = declare_parameter<std::string>("device_name", "");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      const auto fc_string = declare_parameter<std::string>("flow_control", "");

      if (fc_string == "none")
      {
        fc = FlowControl::NONE;
      }
      else if (fc_string == "hardware")
      {
        fc = FlowControl::HARDWARE;
      }
      else if (fc_string == "software")
      {
        fc = FlowControl::SOFTWARE;
      }
      else
      {
        throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBits::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBits::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBits::TWO;
      }
      else
      {
        throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  }

  void RMSerialDriver::reopenPort()
  {
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
      if (serial_driver_->port()->is_open())
      {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok())
      {
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
      }
    }
  }

  void RMSerialDriver::setParam(const rclcpp::Parameter &param)
  {
    if (!detector_param_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
      return;
    }

    if (
        !set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr &results)
          {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true; });
    }
  }

  void RMSerialDriver::resetTracker()
  {
    if (!reset_tracker_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Reset tracker!");
  }

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
