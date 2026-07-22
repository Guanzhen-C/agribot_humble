#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace agribot_hardware_bringup::ackermann_can
{
namespace
{

constexpr double kVelocityResolution = 0.001;
constexpr double kAccelerationRawPerMeterPerSecondSquared = 1671.84;
constexpr double kGyroscopeRadiansPerSecondPerRaw = 0.00026644;

void putInt16Be(chassis_can::Payload & payload, std::size_t offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  payload[offset] = static_cast<uint8_t>((raw >> 8U) & 0xffU);
  payload[offset + 1] = static_cast<uint8_t>(raw & 0xffU);
}

int16_t getInt16Be(const TelemetryPayload & payload, std::size_t offset)
{
  const auto raw = (static_cast<uint16_t>(payload[offset]) << 8U) |
    static_cast<uint16_t>(payload[offset + 1]);
  return static_cast<int16_t>(raw);
}

uint16_t getUint16Be(const TelemetryPayload & payload, std::size_t offset)
{
  return (static_cast<uint16_t>(payload[offset]) << 8U) |
         static_cast<uint16_t>(payload[offset + 1]);
}

uint8_t telemetryChecksum(const TelemetryPayload & payload)
{
  uint8_t checksum = 0;
  for (std::size_t index = 0; index < kTelemetrySize - 2; ++index) {
    checksum ^= payload[index];
  }
  return checksum;
}

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
  uint32_t command_id)
{
  if (!std::isfinite(command.speed_mps) || !std::isfinite(command.steering_angle_rad)) {
    throw std::invalid_argument("command values must be finite");
  }

  chassis_can::Frame frame;
  frame.id = command_id;
  putInt16Be(
    frame.data, 0, chassis_can::scaledInt16(command.speed_mps, kVelocityResolution));
  putInt16Be(
    frame.data, 4,
    chassis_can::scaledInt16(command.steering_angle_rad, kVelocityResolution));
  return frame;
}

std::optional<Telemetry> decodeTelemetry(const TelemetryPayload & payload)
{
  if (payload.front() != 0x7bU || payload.back() != 0x7dU ||
    payload[kTelemetrySize - 2] != telemetryChecksum(payload))
  {
    return std::nullopt;
  }

  Telemetry telemetry;
  telemetry.stop_flag = payload[1];
  telemetry.linear_velocity_x =
    static_cast<double>(getInt16Be(payload, 2)) * kVelocityResolution;
  telemetry.linear_velocity_y =
    static_cast<double>(getInt16Be(payload, 4)) * kVelocityResolution;
  telemetry.angular_velocity_z =
    static_cast<double>(getInt16Be(payload, 6)) * kVelocityResolution;
  telemetry.linear_acceleration_x =
    static_cast<double>(getInt16Be(payload, 8)) /
    kAccelerationRawPerMeterPerSecondSquared;
  telemetry.linear_acceleration_y =
    static_cast<double>(getInt16Be(payload, 10)) /
    kAccelerationRawPerMeterPerSecondSquared;
  telemetry.linear_acceleration_z =
    static_cast<double>(getInt16Be(payload, 12)) /
    kAccelerationRawPerMeterPerSecondSquared;
  telemetry.angular_velocity_x =
    static_cast<double>(getInt16Be(payload, 14)) * kGyroscopeRadiansPerSecondPerRaw;
  telemetry.angular_velocity_y =
    static_cast<double>(getInt16Be(payload, 16)) * kGyroscopeRadiansPerSecondPerRaw;
  telemetry.imu_angular_velocity_z =
    static_cast<double>(getInt16Be(payload, 18)) * kGyroscopeRadiansPerSecondPerRaw;
  telemetry.battery_voltage =
    static_cast<double>(getUint16Be(payload, 20)) * kVelocityResolution;
  return telemetry;
}

Command fromTwist(
  double linear_velocity,
  double angular_velocity,
  const Kinematics & config,
  bool brake)
{
  validateConfig(config);
  if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
    throw std::invalid_argument("velocity command must be finite");
  }

  Command command;
  command.speed_mps = brake ? 0.0 : std::clamp(
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
  return command;
}

}  // namespace agribot_hardware_bringup::ackermann_can
