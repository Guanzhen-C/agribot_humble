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

void validateCalibration(const Kinematics & config)
{
  const auto & levels = config.command_calibration_levels;
  const auto & speeds = config.command_calibration_wheel_speeds_mps;
  if (levels.size() < 2U || levels.size() != speeds.size()) {
    throw std::invalid_argument(
            "command calibration arrays must have the same size >= 2");
  }
  if (levels.front() != 0.0 || speeds.front() != 0.0) {
    throw std::invalid_argument("command calibration must start at (0, 0)");
  }
  for (std::size_t index = 0; index < levels.size(); ++index) {
    if (!std::isfinite(levels[index]) || !std::isfinite(speeds[index]) ||
      levels[index] < 0.0 || speeds[index] < 0.0)
    {
      throw std::invalid_argument(
              "command calibration values must be finite and nonnegative");
    }
    if (index > 0U &&
      (levels[index] <= levels[index - 1U] || speeds[index] <= speeds[index - 1U]))
    {
      throw std::invalid_argument(
              "command calibration levels and speeds must increase strictly");
    }
  }
  if (levels.back() > config.command_full_scale_level) {
    throw std::invalid_argument(
            "last command calibration level exceeds full-scale level");
  }
}

void validateConfig(const Kinematics & config)
{
  requirePositive(config.track_width_m, "track_width_m");
  requirePositive(config.command_full_scale_level, "command_full_scale_level");
  validateCalibration(config);
  requirePositive(
    config.feedback_wheel_speed_mps_per_speed_unit,
    "feedback_wheel_speed_mps_per_speed_unit");
  requirePositive(config.max_linear_velocity, "max_linear_velocity");
  requirePositive(config.max_angular_velocity, "max_angular_velocity");
}

double interpolateOrExtrapolate(
  double value,
  const std::vector<double> & input,
  const std::vector<double> & output)
{
  std::size_t upper = 1U;
  while (upper + 1U < input.size() && value > input[upper]) {
    ++upper;
  }
  const std::size_t lower = upper - 1U;
  const double ratio =
    (value - input[lower]) / (input[upper] - input[lower]);
  return output[lower] + ratio * (output[upper] - output[lower]);
}

double maximumCommandWheelSpeed(const Kinematics & config)
{
  return interpolateOrExtrapolate(
    config.command_full_scale_level,
    config.command_calibration_levels,
    config.command_calibration_wheel_speeds_mps);
}

double wheelSpeedToPercent(double speed, const Kinematics & config)
{
  const double level = interpolateOrExtrapolate(
    std::abs(speed),
    config.command_calibration_wheel_speeds_mps,
    config.command_calibration_levels);
  const double percent = std::clamp(
    level / config.command_full_scale_level * 100.0, 0.0, 100.0);
  return std::copysign(percent, speed);
}

int8_t percentToByte(double percent)
{
  if (!std::isfinite(percent)) {
    throw std::invalid_argument("motor percentage must be finite");
  }
  return static_cast<int8_t>(std::lround(std::clamp(percent, -100.0, 100.0)));
}

int8_t decodeInt8(uint8_t value)
{
  const int16_t signed_value = value < 0x80U ?
    static_cast<int16_t>(value) : static_cast<int16_t>(value) - 0x100;
  return static_cast<int8_t>(signed_value);
}

}  // namespace

bool ChassisState::hasFault() const
{
  return vrc_communication_fault || autonomous_communication_fault ||
         motor_driver_communication_fault || bms_communication_fault;
}

bool MotorState::hasFault() const
{
  return over_voltage_protection || under_voltage_protection || temperature_fault ||
         over_current_protection || overload_protection || hall_fault ||
         locked_rotor_protection || other_fault;
}

chassis_can::Frame encodeCommand(const Command & command, uint8_t rolling_counter)
{
  chassis_can::Frame frame;
  frame.id = kCommandId;
  frame.data[1] = static_cast<uint8_t>(percentToByte(command.left_percent));
  frame.data[2] = static_cast<uint8_t>(percentToByte(command.right_percent));
  frame.data[3] = command.headlight ? 0x01U : 0x00U;
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
  state.headlight = ((frame.data[1] >> 2U) & 0x01U) != 0U;
  state.battery_voltage =
    static_cast<double>(chassis_can::getUint16Le(frame.data, 2)) * 0.1;
  state.vrc_communication_fault = (frame.data[4] & 0x01U) != 0U;
  state.autonomous_communication_fault = (frame.data[4] & 0x02U) != 0U;
  state.motor_driver_communication_fault = (frame.data[4] & 0x04U) != 0U;
  state.bms_communication_fault = (frame.data[4] & 0x08U) != 0U;
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
  state.over_voltage_protection = (frame.data[0] & 0x01U) != 0U;
  state.under_voltage_protection = (frame.data[0] & 0x02U) != 0U;
  state.temperature_fault = (frame.data[0] & 0x04U) != 0U;
  state.over_current_protection = (frame.data[0] & 0x08U) != 0U;
  state.overload_protection = (frame.data[0] & 0x10U) != 0U;
  state.hall_fault = (frame.data[0] & 0x20U) != 0U;
  state.locked_rotor_protection = (frame.data[0] & 0x40U) != 0U;
  state.other_fault = (frame.data[0] & 0x80U) != 0U;
  state.speed = chassis_can::getInt16Le(frame.data, 1);
  state.motor_voltage = frame.data[3];
  state.running_current = decodeInt8(frame.data[4]);
  state.temperature = static_cast<int16_t>(frame.data[5]) - 40;
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

  const double maximum_wheel_speed = maximumCommandWheelSpeed(config);
  const double requested_max = std::max(std::abs(left_speed), std::abs(right_speed));
  if (requested_max > maximum_wheel_speed) {
    const double scale = maximum_wheel_speed / requested_max;
    left_speed *= scale;
    right_speed *= scale;
  }

  Command command;
  command.left_percent = brake ? 0.0 : wheelSpeedToPercent(left_speed, config);
  command.right_percent = brake ? 0.0 : wheelSpeedToPercent(right_speed, config);
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
