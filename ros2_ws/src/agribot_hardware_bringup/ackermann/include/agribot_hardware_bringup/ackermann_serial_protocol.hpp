#ifndef AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_SERIAL_PROTOCOL_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_SERIAL_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"

namespace agribot_hardware_bringup::ackermann_serial
{

constexpr uint8_t kFrameHeader = 0x7bU;
constexpr uint8_t kFrameTail = 0x7dU;
constexpr std::size_t kCommandSize = 11U;

using CommandFrame = std::array<uint8_t, kCommandSize>;

uint8_t xorChecksum(const uint8_t * data, std::size_t size);

CommandFrame encodeCommand(const ackermann_can::Command & command);

class TelemetryParser
{
public:
  std::vector<ackermann_can::Telemetry> feed(
    const uint8_t * data,
    std::size_t size);

  void reset();

  std::size_t discardedBytes() const;

  std::size_t invalidFrames() const;

private:
  std::vector<uint8_t> buffer_;
  std::size_t discarded_bytes_{0U};
  std::size_t invalid_frames_{0U};
};

}  // namespace agribot_hardware_bringup::ackermann_serial

#endif  // AGRIBOT_HARDWARE_BRINGUP__ACKERMANN_SERIAL_PROTOCOL_HPP_
