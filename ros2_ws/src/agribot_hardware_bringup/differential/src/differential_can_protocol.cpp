#include "agribot_hardware_bringup/differential_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace agribot_hardware_bringup::differential_can
{
namespace
{

void requirePositive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

void validateConfig(const Kinematics & config)
{
  requirePositive(config.track_width_m, "track_width_m");
  requirePositive(
    config.command_full_scale_wheel_speed_mps,
    "command_full_scale_wheel_speed_mps");
  requirePositive(
    config.feedback_wheel_speed_mps_per_speed_unit,
    "feedback_wheel_speed_mps_per_speed_unit");
  requirePositive(config.max_linear_velocity, "max_linear_velocity");
  requirePositive(config.max_angular_velocity, "max_angular_velocity");
}

int8_t percentToByte(double percent)
{
  if (!std::isfinite(percent)) {
    throw std::invalid_argument("motor percentage must be finite");
  }
  return static_cast<int8_t>(std::lround(std::clamp(percent, -100.0, 100.0)));
}

}  // namespace

bool MotorState::hasFault() const
{
  return hall_fault || controller_fault || phase_loss || under_voltage_protection ||
         over_current_protection || locked_rotor_protection || runaway_protection ||
         other_controller_protection;
}

int32_t MotorState::signedSpeed() const
{
  const int32_t magnitude = static_cast<int32_t>(speed);
  return reverse ? -magnitude : magnitude;
}

chassis_can::Frame encodeCommand(const Command & command, uint8_t rolling_counter)
{
  chassis_can::Frame frame;
  frame.id = kCommandId;
  frame.data[0] = static_cast<uint8_t>(
    (command.left_brake ? 0x01U : 0x00U) |
    (command.right_brake ? 0x02U : 0x00U));
  frame.data[1] = static_cast<uint8_t>(
    percentToByte(command.left_brake ? 0.0 : command.left_percent));
  frame.data[2] = static_cast<uint8_t>(
    percentToByte(command.right_brake ? 0.0 : command.right_percent));
  frame.data[3] = static_cast<uint8_t>(
    (command.headlight ? 0x01U : 0x00U) |
    ((static_cast<uint8_t>(command.turn_light) & 0x03U) << 1U));
  frame.data[6] = rolling_counter & 0x0fU;
  frame.data[7] = chassis_can::xorChecksum(frame.data);
  return frame;
}

std::optional<ChassisState> decodeChassisState(const chassis_can::Frame & frame)
{
  if (frame.id != kChassisStateId || !chassis_can::hasValidChecksum(frame.data)) {
    return std::nullopt;
  }

  ChassisState state;
  state.work_mode = frame.data[0] & 0x03U;
  state.emergency_stop = ((frame.data[0] >> 2U) & 0x01U) != 0U;
  state.running = ((frame.data[0] >> 3U) & 0x01U) != 0U;
  state.remote_connection_status = (frame.data[0] >> 4U) & 0x03U;
  state.turn_light = static_cast<TurnLight>(frame.data[1] & 0x03U);
  state.headlight = ((frame.data[1] >> 2U) & 0x01U) != 0U;
  state.battery_voltage = static_cast<double>(frame.data[2]);
  state.rolling_counter = chassis_can::rollingCounter(frame.data);
  return state;
}

std::optional<MotorState> decodeMotorState(const chassis_can::Frame & frame)
{
  if ((frame.id != kLeftMotorStateId && frame.id != kRightMotorStateId) ||
    !chassis_can::hasValidChecksum(frame.data))
  {
    return std::nullopt;
  }

  MotorState state;
  state.frame_id = frame.id;
  state.hall_fault = (frame.data[0] & 0x01U) != 0U;
  state.controller_fault = (frame.data[0] & 0x02U) != 0U;
  state.phase_loss = (frame.data[0] & 0x04U) != 0U;
  state.under_voltage_protection = (frame.data[0] & 0x08U) != 0U;
  state.over_current_protection = (frame.data[0] & 0x10U) != 0U;
  state.locked_rotor_protection = (frame.data[0] & 0x20U) != 0U;
  state.runaway_protection = (frame.data[0] & 0x40U) != 0U;
  state.other_controller_protection = (frame.data[0] & 0x80U) != 0U;
  state.pwm_output = (frame.data[1] & 0x10U) != 0U;
  state.reverse = (frame.data[1] & 0x20U) != 0U;
  state.brake = (frame.data[1] & 0x40U) != 0U;
  state.electronic_brake = (frame.data[1] & 0x80U) != 0U;
  state.speed = chassis_can::getUint16Le(frame.data, 2);
  state.running_current = chassis_can::getInt16Le(frame.data, 4);
  state.rolling_counter = chassis_can::rollingCounter(frame.data);
  return state;
}

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake,
  bool headlight)
{
  validateConfig(config);
  if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
    throw std::invalid_argument("velocity command must be finite");
  }

  const double linear = std::clamp(
    linear_velocity, -config.max_linear_velocity, config.max_linear_velocity);
  const double angular = std::clamp(
    angular_velocity, -config.max_angular_velocity, config.max_angular_velocity);
  double left_speed = linear - angular * config.track_width_m * 0.5;
  double right_speed = linear + angular * config.track_width_m * 0.5;

  const double maximum_wheel_speed = config.command_full_scale_wheel_speed_mps;
  const double requested_max = std::max(std::abs(left_speed), std::abs(right_speed));
  if (requested_max > maximum_wheel_speed) {
    const double scale = maximum_wheel_speed / requested_max;
    left_speed *= scale;
    right_speed *= scale;
  }

  const auto speedToPercent = [&](double speed) {
    return std::clamp(speed / maximum_wheel_speed * 100.0, -100.0, 100.0);
  };

  Command command;
  command.left_percent = speedToPercent(left_speed);
  command.right_percent = speedToPercent(right_speed);
  command.left_brake = brake;
  command.right_brake = brake;
  command.headlight = headlight;
  return command;
}

void motorSpeedToTwist(
  int32_t left_speed_units,
  int32_t right_speed_units,
  const Kinematics & config,
  double & linear_velocity,
  double & angular_velocity)
{
  validateConfig(config);
  const double left_speed =
    static_cast<double>(left_speed_units) * config.feedback_wheel_speed_mps_per_speed_unit;
  const double right_speed =
    static_cast<double>(right_speed_units) * config.feedback_wheel_speed_mps_per_speed_unit;
  linear_velocity = (left_speed + right_speed) * 0.5;
  angular_velocity = (right_speed - left_speed) / config.track_width_m;
}

}  // namespace agribot_hardware_bringup::differential_can
