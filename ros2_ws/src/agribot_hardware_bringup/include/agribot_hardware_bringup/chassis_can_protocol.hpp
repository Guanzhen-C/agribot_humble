#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_PROTOCOL_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace agribot_hardware_bringup::chassis_can
{

constexpr std::size_t kPayloadSize = 8;

constexpr uint32_t kDifferentialCommandId = 0x514;
constexpr uint32_t kDifferentialStateId = 0x532;
constexpr uint32_t kLeftMotorStateId = 0x533;
constexpr uint32_t kRightMotorStateId = 0x534;

// This layout is a software-side reference until the Ackermann controller
// supplier confirms its CAN IDs and signal definitions.
constexpr uint32_t kReferenceAckermannCommandId = 0x515;
constexpr uint32_t kReferenceAckermannStateId = 0x535;

using Payload = std::array<uint8_t, kPayloadSize>;

struct Frame
{
  uint32_t id{0};
  Payload data{};
};

struct DifferentialCommand
{
  double left_percent{0.0};
  double right_percent{0.0};
  bool brake{true};
  bool headlight{false};
};

struct DifferentialKinematics
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

struct MotorState
{
  uint32_t frame_id{0};
  bool over_voltage{false};
  bool under_voltage{false};
  bool temperature_fault{false};
  bool over_current{false};
  bool overload{false};
  bool hall_fault{false};
  bool locked_rotor{false};
  bool other_fault{false};
  int16_t rpm{0};
  double voltage{0.0};
  double current{0.0};
  double temperature_c{0.0};
  uint8_t rolling_counter{0};

  bool hasFault() const;
};

struct AckermannCommand
{
  double speed_mps{0.0};
  double steering_angle_rad{0.0};
  bool enabled{false};
  bool brake{true};
  bool headlight{false};
};

struct AckermannKinematics
{
  double wheelbase_m{0.65};
  double max_steering_angle_rad{0.60};
  double max_linear_velocity{0.80};
  double max_angular_velocity{0.65};
  double minimum_motion_speed{0.02};
};

struct AckermannState
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

uint8_t xorChecksum(const Payload & payload);
bool hasValidChecksum(const Payload & payload);
uint8_t rollingCounter(const Payload & payload);

Frame encodeDifferentialCommand(
  const DifferentialCommand & command,
  uint8_t rolling_counter,
  bool legacy_brake_byte = true);

std::optional<ChassisState> decodeChassisState(const Frame & frame);
std::optional<MotorState> decodeMotorState(const Frame & frame);

DifferentialCommand differentialFromTwist(
  double linear_velocity,
  double angular_velocity,
  const DifferentialKinematics & config,
  bool brake = false,
  bool headlight = false);

void motorRpmToTwist(
  int16_t left_rpm,
  int16_t right_rpm,
  const DifferentialKinematics & config,
  double & linear_velocity,
  double & angular_velocity);

// Reference Ackermann layout:
// byte0 bit0 enable, bit1 brake; byte1..2 speed in 0.001 m/s;
// byte3..4 steering in 0.001 rad; byte5 bit0 headlight; byte6 counter;
// byte7 XOR checksum. It must not be enabled on real hardware until confirmed.
Frame encodeReferenceAckermannCommand(
  const AckermannCommand & command,
  uint8_t rolling_counter,
  uint32_t command_id = kReferenceAckermannCommandId);

std::optional<AckermannState> decodeReferenceAckermannState(
  const Frame & frame,
  uint32_t state_id = kReferenceAckermannStateId);

AckermannCommand ackermannFromTwist(
  double linear_velocity,
  double angular_velocity,
  const AckermannKinematics & config,
  bool brake = false,
  bool headlight = false);

}  // namespace agribot_hardware_bringup::chassis_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_PROTOCOL_HPP_
