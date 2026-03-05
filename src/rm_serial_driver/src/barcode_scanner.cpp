// Copyright (c) 2026
// Licensed under the Apache-2.0 License.

#include <rclcpp/qos.hpp>
#include <serial_driver/serial_driver.hpp>

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "../include/barcode_scanner.hpp"

namespace rm_serial_driver
{
BarcodeScannerNode::BarcodeScannerNode(const rclcpp::NodeOptions & options)
: Node("barcode_scanner", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start BarcodeScannerNode!");

  getParams();
  profile_pub_ = this->create_publisher<auto_aim_interfaces::msg::DartProfile>(
    profile_topic_, rclcpp::SensorDataQoS());

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&BarcodeScannerNode::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating barcode scanner port: %s - %s", device_name_.c_str(),
      ex.what());
    throw;
  }
}

BarcodeScannerNode::~BarcodeScannerNode()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void BarcodeScannerNode::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  baud_rate = declare_parameter<int>("baud_rate", 115200);
  profile_topic_ = declare_parameter<std::string>("profile_topic", "barcode/scan_profile");
  dart_id_min_ = declare_parameter<int>("dart_id_min", 1);
  dart_id_max_ = declare_parameter<int>("dart_id_max", 16);
  offset_min_deg_ = declare_parameter<double>("offset_min_deg", -10.0);
  offset_max_deg_ = declare_parameter<double>("offset_max_deg", 10.0);
  slot_count_ = declare_parameter<int>("slot_count", 4);
  const std::string regex_string = declare_parameter<std::string>(
    "barcode_regex", "^D([0-9]+),O([+-]?[0-9]*\\.?[0-9]+)$");
  barcode_regex_ = std::regex(regex_string);

  const auto fc_string = declare_parameter<std::string>("flow_control", "none");
  if (fc_string == "none") {
    fc = FlowControl::NONE;
  } else if (fc_string == "hardware") {
    fc = FlowControl::HARDWARE;
  } else if (fc_string == "software") {
    fc = FlowControl::SOFTWARE;
  } else {
    throw std::invalid_argument{
      "The flow_control parameter must be one of: none, software, or hardware."};
  }

  const auto pt_string = declare_parameter<std::string>("parity", "none");
  if (pt_string == "none") {
    pt = Parity::NONE;
  } else if (pt_string == "odd") {
    pt = Parity::ODD;
  } else if (pt_string == "even") {
    pt = Parity::EVEN;
  } else {
    throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
  }

  const auto sb_string = declare_parameter<std::string>("stop_bits", "1");
  if (sb_string == "1" || sb_string == "1.0") {
    sb = StopBits::ONE;
  } else if (sb_string == "1.5") {
    sb = StopBits::ONE_POINT_FIVE;
  } else if (sb_string == "2" || sb_string == "2.0") {
    sb = StopBits::TWO;
  } else {
    throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
  }

  if (dart_id_min_ > dart_id_max_) {
    throw std::invalid_argument{"dart_id_min must be <= dart_id_max"};
  }
  if (offset_min_deg_ > offset_max_deg_) {
    throw std::invalid_argument{"offset_min_deg must be <= offset_max_deg"};
  }
  if (slot_count_ < 1) {
    throw std::invalid_argument{"slot_count must be >= 1"};
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

bool BarcodeScannerNode::parseBarcodeLine(
  const std::string & line, uint8_t & dart_id, float & offset_deg)
{
  std::smatch matches;
  if (!std::regex_match(line, matches, barcode_regex_) || matches.size() != 3) {
    return false;
  }

  int parsed_dart_id = 0;
  double parsed_offset = 0.0;
  try {
    parsed_dart_id = std::stoi(matches[1].str());
    parsed_offset = std::stod(matches[2].str());
  } catch (const std::exception &) {
    return false;
  }

  if (parsed_dart_id < dart_id_min_ || parsed_dart_id > dart_id_max_) {
    return false;
  }
  if (parsed_offset < offset_min_deg_ || parsed_offset > offset_max_deg_) {
    return false;
  }

  dart_id = static_cast<uint8_t>(parsed_dart_id);
  offset_deg = static_cast<float>(parsed_offset);
  return true;
}

void BarcodeScannerNode::receiveData()
{
  std::vector<uint8_t> byte_buf(1);
  std::string line_buf;
  line_buf.reserve(64);

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(byte_buf);
      const char ch = static_cast<char>(byte_buf[0]);

      if (ch == '\r' || ch == '\n') {
        if (!line_buf.empty()) {
          uint8_t dart_id = 0;
          float offset_deg = 0.0F;
          if (parseBarcodeLine(line_buf, dart_id, offset_deg)) {
            auto msg = auto_aim_interfaces::msg::DartProfile();
            msg.header.stamp = this->now();
            msg.header.frame_id = "barcode_scanner";
            msg.dart_id = dart_id;
            msg.offset_deg = offset_deg;
            msg.scan_slot = static_cast<uint8_t>(next_scan_slot_);
            profile_pub_->publish(msg);

            RCLCPP_INFO(
              get_logger(), "Accepted scan line '%s' -> dart_id=%u offset=%.3f slot=%d",
              line_buf.c_str(), dart_id, offset_deg, next_scan_slot_);
            next_scan_slot_++;
            if (next_scan_slot_ > slot_count_) {
              next_scan_slot_ = 1;
            }
          } else {
            RCLCPP_WARN(
              get_logger(), "Rejected scan line '%s' (format/range invalid)", line_buf.c_str());
          }
          line_buf.clear();
        }
        continue;
      }

      if (std::isprint(static_cast<unsigned char>(ch)) != 0) {
        line_buf.push_back(ch);
      }

      if (line_buf.size() > 256U) {
        RCLCPP_WARN(get_logger(), "Scan line too long, dropping current buffer");
        line_buf.clear();
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Barcode scanner serial read error: %s", ex.what());
      reopenPort();
    }
  }
}

void BarcodeScannerNode::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen barcode scanner port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened barcode scanner port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening barcode scanner port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}
}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::BarcodeScannerNode)
