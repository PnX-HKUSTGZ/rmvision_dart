// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

// 飞镖对应进行修改
namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t competition_mode_;
  uint8_t target_id_; // 0-outpost 1-base
  uint8_t dart_id;
  float offset;
  uint16_t checksum = 0;
} __attribute__((packed));

struct LoggerPacket
{
  uint8_t header = 0xD5;

  uint8_t state;
  uint8_t prepare_state;
  uint8_t launch_station_status;
  uint8_t is_fire_finished;  // 0-false, 1-true

  uint8_t fired_count_this_open;
  uint8_t current_shot_number;
  uint8_t current_dart_id;

  float string_L_force;
  float string_R_force;

  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
//   uint8_t tracking : 1;
//   uint8_t iffire : 1;
//   uint8_t id : 3;    0-outpost 6-guard 7-base
//   uint8_t reserved : 3;
  
  float distance;
  float angle;
  float longitudinal_distance;
  float lateral_distance;
  uint8_t dart_id_change_flag;
  uint8_t stability;
  // 0: unknown/no target, 1: green light visible and aim data valid,
  // 2: door open but green light occluded, 3: door not fully open/blocked.
  uint8_t light_detected;
  
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
