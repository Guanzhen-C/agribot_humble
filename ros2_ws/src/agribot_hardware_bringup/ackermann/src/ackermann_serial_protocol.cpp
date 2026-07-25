#include "agribot_hardware_bringup/ackermann_serial_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace agribot_hardware_bringup::ackermann_serial
{

uint8_t xorChecksum(const uint8_t * data, std::size_t size)
{
  uint8_t checksum = 0U;
  for (std::size_t index = 0; index < size; ++index) {
    checksum ^= data[index];
  }
  return checksum;
}

CommandFrame encodeCommand(const ackermann_can::Command & command)
{
  const auto can_frame = ackermann_can::encodeCommand(command);
  CommandFrame frame{};
  frame[0] = kFrameHeader;
  std::copy_n(can_frame.data.begin(), 6U, frame.begin() + 3);
  frame[kCommandSize - 2U] = xorChecksum(frame.data(), kCommandSize - 2U);
  frame[kCommandSize - 1U] = kFrameTail;
  return frame;
}

std::vector<ackermann_can::Telemetry> TelemetryParser::feed(
  const uint8_t * data,
  std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<ackermann_can::Telemetry> telemetry;
  while (true) {
    const auto header = std::find(buffer_.begin(), buffer_.end(), kFrameHeader);
    if (header == buffer_.end()) {
      discarded_bytes_ += buffer_.size();
      buffer_.clear();
      break;
    }
    if (header != buffer_.begin()) {
      const auto skipped = static_cast<std::size_t>(header - buffer_.begin());
      discarded_bytes_ += skipped;
      buffer_.erase(buffer_.begin(), header);
    }
    if (buffer_.size() < ackermann_can::kTelemetrySize) {
      break;
    }

    ackermann_can::TelemetryPayload candidate{};
    std::copy_n(buffer_.begin(), candidate.size(), candidate.begin());
    const auto decoded = ackermann_can::decodeTelemetry(candidate);
    if (decoded.has_value()) {
      telemetry.push_back(*decoded);
      buffer_.erase(
        buffer_.begin(),
        buffer_.begin() + static_cast<std::ptrdiff_t>(candidate.size()));
    } else {
      ++invalid_frames_;
      ++discarded_bytes_;
      buffer_.erase(buffer_.begin());
    }
  }
  return telemetry;
}

void TelemetryParser::reset()
{
  buffer_.clear();
}

std::size_t TelemetryParser::discardedBytes() const
{
  return discarded_bytes_;
}

std::size_t TelemetryParser::invalidFrames() const
{
  return invalid_frames_;
}

}  // namespace agribot_hardware_bringup::ackermann_serial
