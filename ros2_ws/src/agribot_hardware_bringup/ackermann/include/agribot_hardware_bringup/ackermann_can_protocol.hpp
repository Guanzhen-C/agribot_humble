#ifndef AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace agribot_hardware_bringup::ackermann_can
{

constexpr uint32_t kCommandId = 0x181;
constexpr uint32_t kFeedbackPart1Id = 0x101;
constexpr uint32_t kFeedbackPart2Id = 0x102;
constexpr uint32_t kFeedbackPart3Id = 0x103;
constexpr std::size_t kTelemetrySize = 24;

using TelemetryPayload = std::array<uint8_t, kTelemetrySize>;

struct Command
{
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
};

struct Kinematics
{
  double wheelbase_m{0.65};
  double max_steering_angle_rad{0.30};
  double max_linear_velocity{0.80};
  double max_angular_velocity{0.65};
  double minimum_motion_speed{0.02};
};

struct Telemetry
{
  uint8_t stop_flag{0};
  double linear_velocity_x{0.0};
  double linear_velocity_y{0.0};
  double angular_velocity_z{0.0};
  double linear_acceleration_x{0.0};
  double linear_acceleration_y{0.0};
  double linear_acceleration_z{0.0};
  double angular_velocity_x{0.0};
  double angular_velocity_y{0.0};
  double imu_angular_velocity_z{0.0};
  double battery_voltage{0.0};
};

chassis_can::Frame encodeCommand(
  const Command & command,
  uint32_t command_id = kCommandId);

std::optional<Telemetry> decodeTelemetry(const TelemetryPayload & payload);

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake = false);

}  // namespace agribot_hardware_bringup::ackermann_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_
