#include "agribot_hardware_bringup/chassis_can_common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace agribot_hardware_bringup::chassis_can
{

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

}  // namespace agribot_hardware_bringup::chassis_can
