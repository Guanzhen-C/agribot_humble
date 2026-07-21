#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace agribot_hardware_bringup::ackermann_can
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
  requirePositive(config.wheelbase_m, "wheelbase_m");
  requirePositive(config.max_steering_angle_rad, "max_steering_angle_rad");
  requirePositive(config.max_linear_velocity, "max_linear_velocity");
  requirePositive(config.max_angular_velocity, "max_angular_velocity");
  requirePositive(config.minimum_motion_speed, "minimum_motion_speed");
}

}  // namespace

chassis_can::Frame encodeCommand(
  const Command & command,
  uint8_t rolling_counter,
  uint32_t command_id)
{
  chassis_can::Frame frame;
  frame.id = command_id;
  frame.data[0] = (command.enabled ? 0x01U : 0x00U) |
    (command.brake ? 0x02U : 0x00U);
  chassis_can::putInt16Le(
    frame.data, 1,
    chassis_can::scaledInt16(command.brake ? 0.0 : command.speed_mps, 0.001));
  chassis_can::putInt16Le(
    frame.data, 3,
    chassis_can::scaledInt16(command.brake ? 0.0 : command.steering_angle_rad, 0.001));
  frame.data[5] = command.headlight ? 0x01U : 0x00U;
  frame.data[6] = rolling_counter & 0x0fU;
  frame.data[7] = chassis_can::xorChecksum(frame.data);
  return frame;
}

std::optional<ChassisState> decodeChassisState(
  const chassis_can::Frame & frame,
  uint32_t state_id)
{
  if (frame.id != state_id || !chassis_can::hasValidChecksum(frame.data)) {
    return std::nullopt;
  }

  ChassisState state;
  state.enabled = (frame.data[0] & 0x01U) != 0U;
  state.emergency_stop = (frame.data[0] & 0x02U) != 0U;
  state.running = (frame.data[0] & 0x04U) != 0U;
  state.fault = (frame.data[0] & 0x08U) != 0U;
  state.speed_mps = static_cast<double>(chassis_can::getInt16Le(frame.data, 1)) * 0.001;
  state.steering_angle_rad =
    static_cast<double>(chassis_can::getInt16Le(frame.data, 3)) * 0.001;
  state.battery_voltage = static_cast<double>(frame.data[5]);
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

  Command command;
  command.speed_mps = std::clamp(
    linear_velocity, -config.max_linear_velocity, config.max_linear_velocity);
  const double angular = std::clamp(
    angular_velocity, -config.max_angular_velocity, config.max_angular_velocity);

  if (std::abs(command.speed_mps) < config.minimum_motion_speed) {
    command.speed_mps = 0.0;
    command.steering_angle_rad = 0.0;
  } else {
    command.steering_angle_rad = std::clamp(
      std::atan(config.wheelbase_m * angular / command.speed_mps),
      -config.max_steering_angle_rad,
      config.max_steering_angle_rad);
  }

  command.enabled = !brake;
  command.brake = brake;
  command.headlight = headlight;
  return command;
}

}  // namespace agribot_hardware_bringup::ackermann_can
