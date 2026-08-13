#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_TRANSPORT_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_TRANSPORT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace agribot_hardware_bringup
{

struct ChassisCanReadResult
{
  std::vector<chassis_can::Frame> frames;
  uint64_t invalid_frames{0};
};

struct ChassisCanTransportConfig
{
  std::string backend{"socketcan"};
  std::string socketcan_interface{"can0"};
  std::string zqwl_port;
  int zqwl_channel{0};
  int zqwl_bitrate{1000000};
};

class ChassisCanTransport
{
public:
  virtual ~ChassisCanTransport() = default;

  virtual std::string type() const = 0;
  virtual std::string hardwareId() const = 0;
  virtual void restart() = 0;
  virtual void writeFrame(const chassis_can::Frame & frame) = 0;
  virtual ChassisCanReadResult readFrames(std::size_t max_frames) = 0;
};

std::unique_ptr<ChassisCanTransport> makeChassisCanTransport(
  const ChassisCanTransportConfig & config,
  const std::vector<uint32_t> & feedback_ids);

namespace zqwl_cdc
{

constexpr std::size_t kParameterPacketSize = 22;
constexpr std::size_t kClassicCanPacketSize = 16;

using ParameterPacket = std::array<uint8_t, kParameterPacketSize>;
using ClassicCanPacket = std::array<uint8_t, kClassicCanPacketSize>;

ParameterPacket makeStartPacket(int channel, int bitrate);
ParameterPacket makeStopPacket();
ClassicCanPacket encodeFrame(const chassis_can::Frame & frame, int channel);

class FrameDecoder
{
public:
  ChassisCanReadResult append(
    const uint8_t * data,
    std::size_t size,
    std::size_t max_frames);

private:
  std::vector<uint8_t> buffer_;
};

}  // namespace zqwl_cdc
}  // namespace agribot_hardware_bringup

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_TRANSPORT_HPP_
