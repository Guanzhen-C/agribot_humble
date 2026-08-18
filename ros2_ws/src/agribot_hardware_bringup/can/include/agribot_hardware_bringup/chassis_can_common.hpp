#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace agribot_hardware_bringup::chassis_can
{

constexpr std::size_t kPayloadSize = 8;

using Payload = std::array<uint8_t, kPayloadSize>;

struct Frame
{
  uint32_t id{0};
  Payload data{};
};

uint8_t xorChecksum(const Payload & payload);
bool hasValidChecksum(const Payload & payload);
uint8_t rollingCounter(const Payload & payload);

void putInt16Le(Payload & payload, std::size_t offset, int16_t value);
int16_t getInt16Le(const Payload & payload, std::size_t offset);
uint16_t getUint16Le(const Payload & payload, std::size_t offset);
int16_t scaledInt16(double value, double units_per_raw);

}  // namespace agribot_hardware_bringup::chassis_can

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_COMMON_HPP_
