#ifndef AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace agribot_hardware_bringup::ackermann_can
{

// Software-side reference IDs until the controller supplier confirms them.
constexpr uint32_t kCommandId = 0x515;
constexpr uint32_t kChassisStateId = 0x535;
constexpr uint32_t kDriveStateId = 0x536;
constexpr uint32_t kSteeringStateId = 0x537;

struct Command
{
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
  bool enabled{false};
  bool brake{true};
  bool headlight{false};
};

struct Kinematics
{
  double wheelbase_m{0.65};
  double max_steering_angle_rad{0.60};
  double max_linear_velocity{0.80};
  double max_angular_velocity{0.65};
  double minimum_motion_speed{0.02};
};

struct ChassisState
{
  bool enabled{false};
  bool emergency_stop{false};
  bool running{false};
  bool fault{false};
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
  double battery_voltage{0.0};
  uint8_t rolling_counter{0};
};

chassis_can::Frame encodeCommand(
  const Command & command,
  uint8_t rolling_counter,
  uint32_t command_id = kCommandId);

std::optional<ChassisState> decodeChassisState(
  const chassis_can::Frame & frame,
  uint32_t state_id = kChassisStateId);

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake = false,
  bool headlight = false);

}  // namespace agribot_hardware_bringup::ackermann_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_CAN_PROTOCOL_HPP_
