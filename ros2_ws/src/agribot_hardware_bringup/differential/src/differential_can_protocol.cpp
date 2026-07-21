#include "agribot_hardware_bringup/differential_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace agribot_hardware_bringup::differential_can
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void requirePositive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

void validateConfig(const Kinematics & config)
{
  requirePositive(config.track_width_m, "track_width_m");
  requirePositive(config.wheel_diameter_m, "wheel_diameter_m");
  requirePositive(config.reduction_ratio, "reduction_ratio");
  requirePositive(config.max_motor_rpm, "max_motor_rpm");
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

chassis_can::Frame encodeCommand(
  const Command & command,
  uint8_t rolling_counter,
  bool legacy_brake_byte)
{
  chassis_can::Frame frame;
  frame.id = kCommandId;
  frame.data[0] = command.brake && legacy_brake_byte ? 0x03U : 0x00U;
  frame.data[1] = static_cast<uint8_t>(percentToByte(command.brake ? 0.0 : command.left_percent));
  frame.data[2] = static_cast<uint8_t>(percentToByte(command.brake ? 0.0 : command.right_percent));
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
  state.battery_voltage = static_cast<double>(chassis_can::getUint16Le(frame.data, 2)) * 0.1;
  state.remote_comm_fault = (frame.data[4] & 0x01U) != 0U;
  state.autonomy_comm_fault = (frame.data[4] & 0x02U) != 0U;
  state.motor_comm_fault = (frame.data[4] & 0x04U) != 0U;
  state.bms_comm_fault = (frame.data[4] & 0x08U) != 0U;
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

  const double wheel_circumference = kPi * config.wheel_diameter_m;
  const double maximum_wheel_speed =
    config.max_motor_rpm / config.reduction_ratio * wheel_circumference / 60.0;
  const double requested_max = std::max(std::abs(left_speed), std::abs(right_speed));
  if (requested_max > maximum_wheel_speed) {
    const double scale = maximum_wheel_speed / requested_max;
    left_speed *= scale;
    right_speed *= scale;
  }

  const auto speedToPercent = [&](double speed) {
      const double motor_rpm =
        speed * 60.0 / wheel_circumference * config.reduction_ratio;
      return std::clamp(motor_rpm / config.max_motor_rpm * 100.0, -100.0, 100.0);
    };

  Command command;
  command.left_percent = speedToPercent(left_speed);
  command.right_percent = speedToPercent(right_speed);
  command.brake = brake;
  command.headlight = headlight;
  return command;
}

void motorRpmToTwist(
  int16_t left_rpm,
  int16_t right_rpm,
  const Kinematics & config,
  double & linear_velocity,
  double & angular_velocity)
{
  validateConfig(config);
  const double circumference = kPi * config.wheel_diameter_m;
  const double left_speed =
    static_cast<double>(left_rpm) / config.reduction_ratio * circumference / 60.0;
  const double right_speed =
    static_cast<double>(right_rpm) / config.reduction_ratio * circumference / 60.0;
  linear_velocity = (left_speed + right_speed) * 0.5;
  angular_velocity = (right_speed - left_speed) / config.track_width_m;
}

}  // namespace agribot_hardware_bringup::differential_can
