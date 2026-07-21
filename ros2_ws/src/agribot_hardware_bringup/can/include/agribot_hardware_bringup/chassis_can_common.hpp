#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace agribot_hardware_bringup::chassis_can
{

constexpr std::size_t kPayloadSize = 8;

using Payload = std::array<uint8_t, kPayloadSize>;

struct Frame
{
  uint32_t id{0};
  Payload data{};
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

uint8_t xorChecksum(const Payload & payload);
bool hasValidChecksum(const Payload & payload);
uint8_t rollingCounter(const Payload & payload);

void putInt16Le(Payload & payload, std::size_t offset, int16_t value);
int16_t getInt16Le(const Payload & payload, std::size_t offset);
uint16_t getUint16Le(const Payload & payload, std::size_t offset);
int16_t scaledInt16(double value, double units_per_raw);

std::optional<MotorState> decodeMotorState(
  const Frame & frame,
  uint32_t first_state_id,
  uint32_t second_state_id);

}  // namespace agribot_hardware_bringup::chassis_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_
