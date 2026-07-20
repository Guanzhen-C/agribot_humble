#include "agribot_hardware_bringup/chassis_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace agribot_hardware_bringup::chassis_can
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void requireFinite(double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void requirePositive(double value, const char * name)
{
  requireFinite(value, name);
  if (value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

int8_t percentToByte(double percent)
{
  requireFinite(percent, "motor percentage");
  const auto bounded = std::clamp(percent, -100.0, 100.0);
  return static_cast<int8_t>(std::lround(bounded));
}

void putInt16Le(Payload & payload, std::size_t offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  payload[offset] = static_cast<uint8_t>(raw & 0xffU);
  payload[offset + 1] = static_cast<uint8_t>((raw >> 8U) & 0xffU);
}

int16_t getInt16Le(const Payload & payload, std::size_t offset)
{
  const auto raw = static_cast<uint16_t>(payload[offset]) |
    (static_cast<uint16_t>(payload[offset + 1]) << 8U);
  return static_cast<int16_t>(raw);
}

uint16_t getUint16Le(const Payload & payload, std::size_t offset)
{
  return static_cast<uint16_t>(payload[offset]) |
         (static_cast<uint16_t>(payload[offset + 1]) << 8U);
}

int16_t scaledInt16(double value, double units_per_raw)
{
  requireFinite(value, "encoded value");
  requirePositive(units_per_raw, "encoding resolution");
  const double raw = std::round(value / units_per_raw);
  return static_cast<int16_t>(std::clamp(
      raw,
      static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max())));
}

void validateDifferentialConfig(const DifferentialKinematics & config)
{
  requirePositive(config.track_width_m, "track_width_m");
  requirePositive(config.wheel_diameter_m, "wheel_diameter_m");
  requirePositive(config.reduction_ratio, "reduction_ratio");
  requirePositive(config.max_motor_rpm, "max_motor_rpm");
  requirePositive(config.max_linear_velocity, "max_linear_velocity");
  requirePositive(config.max_angular_velocity, "max_angular_velocity");
}

void validateAckermannConfig(const AckermannKinematics & config)
{
  requirePositive(config.wheelbase_m, "wheelbase_m");
  requirePositive(config.max_steering_angle_rad, "max_steering_angle_rad");
  requirePositive(config.max_linear_velocity, "max_linear_velocity");
  requirePositive(config.max_angular_velocity, "max_angular_velocity");
  requirePositive(config.minimum_motion_speed, "minimum_motion_speed");
}

}  // namespace

bool MotorState::hasFault() const
{
  return over_voltage || under_voltage || temperature_fault || over_current ||
         overload || hall_fault || locked_rotor || other_fault;
}

uint8_t xorChecksum(const Payload & payload)
{
  uint8_t checksum = 0;
  for (std::size_t index = 0; index < kPayloadSize - 1; ++index) {
    checksum ^= payload[index];
  }
  return checksum;
}

bool hasValidChecksum(const Payload & payload)
{
  return payload[7] == xorChecksum(payload);
}

uint8_t rollingCounter(const Payload & payload)
{
  return payload[6] & 0x0fU;
}

Frame encodeDifferentialCommand(
  const DifferentialCommand & command,
  uint8_t rolling_counter,
  bool legacy_brake_byte)
{
  Frame frame;
  frame.id = kDifferentialCommandId;
  frame.data[0] = command.brake && legacy_brake_byte ? 0x03U : 0x00U;
  frame.data[1] = static_cast<uint8_t>(percentToByte(command.brake ? 0.0 : command.left_percent));
  frame.data[2] = static_cast<uint8_t>(percentToByte(command.brake ? 0.0 : command.right_percent));
  frame.data[3] = command.headlight ? 0x01U : 0x00U;
  frame.data[6] = rolling_counter & 0x0fU;
  frame.data[7] = xorChecksum(frame.data);
  return frame;
}

std::optional<ChassisState> decodeChassisState(const Frame & frame)
{
  if (frame.id != kDifferentialStateId || !hasValidChecksum(frame.data)) {
    return std::nullopt;
  }

  ChassisState state;
  state.work_mode = frame.data[0] & 0x03U;
  state.emergency_stop = ((frame.data[0] >> 2U) & 0x01U) != 0U;
  state.running = ((frame.data[0] >> 3U) & 0x01U) != 0U;
  state.headlight = ((frame.data[1] >> 2U) & 0x01U) != 0U;
  state.battery_voltage = static_cast<double>(getUint16Le(frame.data, 2)) * 0.1;
  state.remote_comm_fault = (frame.data[4] & 0x01U) != 0U;
  state.autonomy_comm_fault = (frame.data[4] & 0x02U) != 0U;
  state.motor_comm_fault = (frame.data[4] & 0x04U) != 0U;
  state.bms_comm_fault = (frame.data[4] & 0x08U) != 0U;
  state.rolling_counter = rollingCounter(frame.data);
  return state;
}

std::optional<MotorState> decodeMotorState(const Frame & frame)
{
  if ((frame.id != kLeftMotorStateId && frame.id != kRightMotorStateId) ||
    !hasValidChecksum(frame.data))
  {
    return std::nullopt;
  }

  MotorState state;
  state.frame_id = frame.id;
  state.over_voltage = (frame.data[0] & 0x01U) != 0U;
  state.under_voltage = (frame.data[0] & 0x02U) != 0U;
  state.temperature_fault = (frame.data[0] & 0x04U) != 0U;
  state.over_current = (frame.data[0] & 0x08U) != 0U;
  state.overload = (frame.data[0] & 0x10U) != 0U;
  state.hall_fault = (frame.data[0] & 0x20U) != 0U;
  state.locked_rotor = (frame.data[0] & 0x40U) != 0U;
  state.other_fault = (frame.data[0] & 0x80U) != 0U;
  state.rpm = getInt16Le(frame.data, 1);
  state.voltage = static_cast<double>(frame.data[3]);
  state.current = static_cast<double>(static_cast<int8_t>(frame.data[4]));
  state.temperature_c = static_cast<double>(frame.data[5]) - 40.0;
  state.rolling_counter = rollingCounter(frame.data);
  return state;
}

DifferentialCommand differentialFromTwist(
  double linear_velocity,
  double angular_velocity,
  const DifferentialKinematics & config,
  bool brake,
  bool headlight)
{
  validateDifferentialConfig(config);
  requireFinite(linear_velocity, "linear_velocity");
  requireFinite(angular_velocity, "angular_velocity");

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

  DifferentialCommand command;
  command.left_percent = speedToPercent(left_speed);
  command.right_percent = speedToPercent(right_speed);
  command.brake = brake;
  command.headlight = headlight;
  return command;
}

void motorRpmToTwist(
  int16_t left_rpm,
  int16_t right_rpm,
  const DifferentialKinematics & config,
  double & linear_velocity,
  double & angular_velocity)
{
  validateDifferentialConfig(config);
  const double circumference = kPi * config.wheel_diameter_m;
  const double left_speed =
    static_cast<double>(left_rpm) / config.reduction_ratio * circumference / 60.0;
  const double right_speed =
    static_cast<double>(right_rpm) / config.reduction_ratio * circumference / 60.0;
  linear_velocity = (left_speed + right_speed) * 0.5;
  angular_velocity = (right_speed - left_speed) / config.track_width_m;
}

Frame encodeReferenceAckermannCommand(
  const AckermannCommand & command,
  uint8_t rolling_counter,
  uint32_t command_id)
{
  Frame frame;
  frame.id = command_id;
  frame.data[0] = (command.enabled ? 0x01U : 0x00U) |
    (command.brake ? 0x02U : 0x00U);
  putInt16Le(frame.data, 1, scaledInt16(command.brake ? 0.0 : command.speed_mps, 0.001));
  putInt16Le(
    frame.data, 3,
    scaledInt16(command.brake ? 0.0 : command.steering_angle_rad, 0.001));
  frame.data[5] = command.headlight ? 0x01U : 0x00U;
  frame.data[6] = rolling_counter & 0x0fU;
  frame.data[7] = xorChecksum(frame.data);
  return frame;
}

std::optional<AckermannState> decodeReferenceAckermannState(
  const Frame & frame,
  uint32_t state_id)
{
  if (frame.id != state_id || !hasValidChecksum(frame.data)) {
    return std::nullopt;
  }

  AckermannState state;
  state.enabled = (frame.data[0] & 0x01U) != 0U;
  state.emergency_stop = (frame.data[0] & 0x02U) != 0U;
  state.running = (frame.data[0] & 0x04U) != 0U;
  state.fault = (frame.data[0] & 0x08U) != 0U;
  state.speed_mps = static_cast<double>(getInt16Le(frame.data, 1)) * 0.001;
  state.steering_angle_rad = static_cast<double>(getInt16Le(frame.data, 3)) * 0.001;
  state.battery_voltage = static_cast<double>(frame.data[5]);
  state.rolling_counter = rollingCounter(frame.data);
  return state;
}

AckermannCommand ackermannFromTwist(
  double linear_velocity,
  double angular_velocity,
  const AckermannKinematics & config,
  bool brake,
  bool headlight)
{
  validateAckermannConfig(config);
  requireFinite(linear_velocity, "linear_velocity");
  requireFinite(angular_velocity, "angular_velocity");

  AckermannCommand command;
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

}  // namespace agribot_hardware_bringup::chassis_can
