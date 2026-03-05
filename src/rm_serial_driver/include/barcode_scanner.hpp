// Copyright (c) 2026
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__BARCODE_SCANNER_HPP_
#define RM_SERIAL_DRIVER__BARCODE_SCANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <serial_driver/serial_driver.hpp>

#include <cstdint>
#include <memory>
#include <regex>
#include <string>
#include <thread>

#include "auto_aim_interfaces/msg/dart_profile.hpp"

namespace rm_serial_driver
{
class BarcodeScannerNode : public rclcpp::Node
{
public:
  explicit BarcodeScannerNode(const rclcpp::NodeOptions & options);
  ~BarcodeScannerNode() override;

private:
  void getParams();
  void receiveData();
  void reopenPort();
  bool parseBarcodeLine(const std::string & line, uint8_t & dart_id, float & offset_deg);

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::thread receive_thread_;

  rclcpp::Publisher<auto_aim_interfaces::msg::DartProfile>::SharedPtr profile_pub_;

  std::regex barcode_regex_;
  std::string profile_topic_;
  int dart_id_min_{1};
  int dart_id_max_{16};
  double offset_min_deg_{-10.0};
  double offset_max_deg_{10.0};
  int slot_count_{4};
  int next_scan_slot_{1};
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__BARCODE_SCANNER_HPP_
