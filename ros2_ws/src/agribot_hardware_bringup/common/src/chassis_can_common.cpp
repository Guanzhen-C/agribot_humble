#include "agribot_hardware_bringup/chassis_can_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace agribot_hardware_bringup::chassis_can
{

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
  if (!std::isfinite(value)) {
    throw std::invalid_argument("encoded value must be finite");
  }
  if (!std::isfinite(units_per_raw) || units_per_raw <= 0.0) {
    throw std::invalid_argument("encoding resolution must be positive");
  }
  const double raw = std::round(value / units_per_raw);
  return static_cast<int16_t>(std::clamp(
           raw,
           static_cast<double>(std::numeric_limits<int16_t>::min()),
           static_cast<double>(std::numeric_limits<int16_t>::max())));
}

std::optional<MotorState> decodeMotorState(
  const Frame & frame,
  uint32_t first_state_id,
  uint32_t second_state_id)
{
  if ((frame.id != first_state_id && frame.id != second_state_id) ||
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

}  // namespace agribot_hardware_bringup::chassis_can
