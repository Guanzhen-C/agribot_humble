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
  bool headlight{false};
};

struct Kinematics
{
  double track_width_m{0.590224};
  double command_full_scale_wheel_speed_mps{0.80};
  double feedback_wheel_speed_mps_per_speed_unit{0.000436332313};
  double max_linear_velocity{1.0};
  double max_angular_velocity{1.4};
};

struct ChassisState
{
  uint8_t work_mode{0};
  bool emergency_stop{false};
  bool running{false};
  uint8_t remote_connection_status{0};
  bool headlight{false};
  double battery_voltage{0.0};
  uint8_t rolling_counter{0};
};

struct MotorState
{
  uint32_t frame_id{0};
  bool over_current_protection{false};
  bool load_fault{false};
  bool over_temperature_protection{false};
  bool over_voltage_protection{false};
  bool under_voltage_protection{false};
  bool locked_rotor_protection{false};
  bool hall_fault{false};
  bool shake_fault{false};
  int16_t speed{0};
  uint8_t motor_voltage{0};
  int8_t running_current{0};
  int16_t temperature{0};
  uint8_t rolling_counter{0};

  bool hasFault() const;
};

chassis_can::Frame encodeCommand(const Command & command, uint8_t rolling_counter);

std::optional<ChassisState> decodeChassisState(const chassis_can::Frame & frame);
std::optional<MotorState> decodeMotorState(const chassis_can::Frame & frame);

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake = false,
  bool headlight = false);

void motorSpeedToTwist(
  int32_t left_speed_units,
  int32_t right_speed_units,
  const Kinematics & config,
  double & linear_velocity,
  double & angular_velocity);

}  // namespace agribot_hardware_bringup::differential_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__DIFFERENTIAL_CAN_PROTOCOL_HPP_
