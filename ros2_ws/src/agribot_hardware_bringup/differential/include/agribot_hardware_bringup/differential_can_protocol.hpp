#ifndef AGRIBOT_HARDWARE_BRINGUP__DIFFERENTIAL_CAN_PROTOCOL_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__DIFFERENTIAL_CAN_PROTOCOL_HPP_

#include <cstdint>
#include <optional>

#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace agribot_hardware_bringup::differential_can
{

constexpr uint32_t kCommandId = 0x514;
constexpr uint32_t kChassisStateId = 0x532;
constexpr uint32_t kLeftMotorStateId = 0x533;
constexpr uint32_t kRightMotorStateId = 0x534;

struct Command
{
  double left_percent{0.0};
  double right_percent{0.0};
  bool brake{true};
  bool headlight{false};
};

struct Kinematics
{
  double track_width_m{0.94};
  double wheel_diameter_m{0.20};
  double reduction_ratio{30.0};
  double max_motor_rpm{3000.0};
  double max_linear_velocity{1.0};
  double max_angular_velocity{1.4};
};

struct ChassisState
{
  uint8_t work_mode{0};
  bool emergency_stop{false};
  bool running{false};
  bool headlight{false};
  double battery_voltage{0.0};
  bool remote_comm_fault{false};
  bool autonomy_comm_fault{false};
  bool motor_comm_fault{false};
  bool bms_comm_fault{false};
  uint8_t rolling_counter{0};
};

chassis_can::Frame encodeCommand(
  const Command & command,
  uint8_t rolling_counter,
  bool legacy_brake_byte = true);

std::optional<ChassisState> decodeChassisState(const chassis_can::Frame & frame);

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake = false,
  bool headlight = false);

void motorRpmToTwist(
  int16_t left_rpm,
  int16_t right_rpm,
  const Kinematics & config,
  double & linear_velocity,
  double & angular_velocity);

}  // namespace agribot_hardware_bringup::differential_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__DIFFERENTIAL_CAN_PROTOCOL_HPP_
