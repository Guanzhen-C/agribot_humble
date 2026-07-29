#include "agribot_hardware_bringup/chassis_can_transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <unordered_set>
#include <utility>

namespace agribot_hardware_bringup
{
namespace
{

using namespace std::chrono_literals;

constexpr uint8_t kZqwlPacketStart = 0x5a;
constexpr uint8_t kZqwlPacketEnd = 0xa5;
constexpr uint8_t kZqwlStatusPacketType = 0xfe;
constexpr std::size_t kZqwlHeaderSize = 7;
constexpr std::size_t kZqwlStatusPacketSize = 32;
constexpr std::size_t kMaxZqwlBufferSize = 128 * 1024;

void validateStandardCanId(uint32_t id)
{
  if (id > CAN_SFF_MASK) {
    throw std::invalid_argument("only standard 11-bit CAN IDs are supported");
  }
}

class SocketCanTransport final : public ChassisCanTransport
{
public:
  SocketCanTransport(
    std::string interface_name,
    const std::vector<uint32_t> & feedback_ids)
  : interface_name_(std::move(interface_name))
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "socket(PF_CAN)");
    }

    try {
      struct ifreq request {};
      if (interface_name_.size() >= IFNAMSIZ) {
        throw std::invalid_argument("CAN interface name is too long");
      }
      std::strncpy(request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
      if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
        throw std::system_error(
                errno, std::generic_category(), "CAN interface lookup");
      }

      std::vector<struct can_filter> filters;
      filters.reserve(feedback_ids.size());
      for (const auto id : feedback_ids) {
        validateStandardCanId(id);
        filters.push_back({id, CAN_SFF_MASK});
      }
      if (setsockopt(
          socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
          static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0)
      {
        throw std::system_error(errno, std::generic_category(), "CAN_RAW_FILTER");
      }

      struct sockaddr_can address {};
      address.can_family = AF_CAN;
      address.can_ifindex = request.ifr_ifindex;
      if (bind(
          socket_fd_, reinterpret_cast<struct sockaddr *>(&address),
          sizeof(address)) < 0)
      {
        throw std::system_error(errno, std::generic_category(), "bind(SocketCAN)");
      }
    } catch (...) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw;
    }
  }

  ~SocketCanTransport() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  std::string type() const override
  {
    return "socketcan";
  }

  std::string hardwareId() const override
  {
    return interface_name_;
  }

  void writeFrame(const chassis_can::Frame & frame) override
  {
    validateStandardCanId(frame.id);
    struct can_frame raw_frame {};
    raw_frame.can_id = frame.id;
    raw_frame.can_dlc = chassis_can::kPayloadSize;
    std::copy(frame.data.begin(), frame.data.end(), raw_frame.data);
    const auto bytes = write(socket_fd_, &raw_frame, sizeof(raw_frame));
    if (bytes != static_cast<ssize_t>(sizeof(raw_frame))) {
      throw std::system_error(errno, std::generic_category(), "write(SocketCAN)");
    }
  }

  ChassisCanReadResult readFrames(std::size_t max_frames) override
  {
    ChassisCanReadResult result;
    result.frames.reserve(max_frames);
    while (result.frames.size() < max_frames) {
      struct can_frame raw_frame {};
      const auto bytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));
      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (bytes < 0) {
        throw std::system_error(errno, std::generic_category(), "read(SocketCAN)");
      }
      if (bytes != static_cast<ssize_t>(sizeof(raw_frame)) ||
        raw_frame.can_dlc != chassis_can::kPayloadSize ||
        (raw_frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U)
      {
        ++result.invalid_frames;
        continue;
      }

      chassis_can::Frame frame;
      frame.id = raw_frame.can_id & CAN_SFF_MASK;
      std::copy(std::begin(raw_frame.data), std::end(raw_frame.data), frame.data.begin());
      result.frames.push_back(frame);
    }
    return result;
  }

private:
  std::string interface_name_;
  int socket_fd_{-1};
};

class ZqwlCdcTransport final : public ChassisCanTransport
{
public:
  ZqwlCdcTransport(
    std::string port,
    int channel,
    int bitrate,
    const std::vector<uint32_t> & feedback_ids)
  : port_(std::move(port)),
    channel_(channel),
    feedback_ids_(feedback_ids.begin(), feedback_ids.end())
  {
    for (const auto id : feedback_ids) {
      validateStandardCanId(id);
    }

    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "open(ZQWL CDC)");
    }

    try {
      if (ioctl(serial_fd_, TIOCEXCL) < 0) {
        throw std::system_error(errno, std::generic_category(), "TIOCEXCL(ZQWL CDC)");
      }
      configureSerialPort();
      if (tcflush(serial_fd_, TCIOFLUSH) < 0) {
        throw std::system_error(errno, std::generic_category(), "tcflush(ZQWL CDC)");
      }
      const auto packet = zqwl_cdc::makeStartPacket(channel_, bitrate);
      writePacket(packet.data(), packet.size());
    } catch (...) {
      close(serial_fd_);
      serial_fd_ = -1;
      throw;
    }
  }

  ~ZqwlCdcTransport() override
  {
    if (serial_fd_ < 0) {
      return;
    }
    try {
      const auto packet = zqwl_cdc::makeStopPacket();
      writePacket(packet.data(), packet.size());
    } catch (...) {
    }
    close(serial_fd_);
  }

  std::string type() const override
  {
    return "zqwl_cdc";
  }

  std::string hardwareId() const override
  {
    return port_;
  }

  void writeFrame(const chassis_can::Frame & frame) override
  {
    const auto packet = zqwl_cdc::encodeFrame(frame, channel_);
    writePacket(packet.data(), packet.size());
  }

  ChassisCanReadResult readFrames(std::size_t max_frames) override
  {
    ChassisCanReadResult result = decoder_.append(nullptr, 0, max_frames);
    filterFrames(result);

    while (result.frames.size() < max_frames) {
      std::array<uint8_t, 4096> bytes {};
      const auto count = read(serial_fd_, bytes.data(), bytes.size());
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (count < 0) {
        throw std::system_error(errno, std::generic_category(), "read(ZQWL CDC)");
      }
      if (count == 0) {
        break;
      }

      auto decoded = decoder_.append(
        bytes.data(), static_cast<std::size_t>(count),
        max_frames - result.frames.size());
      result.invalid_frames += decoded.invalid_frames;
      filterFrames(decoded);
      result.frames.insert(
        result.frames.end(), decoded.frames.begin(), decoded.frames.end());
    }
    return result;
  }

private:
  void configureSerialPort()
  {
    struct termios attributes {};
    if (tcgetattr(serial_fd_, &attributes) < 0) {
      throw std::system_error(errno, std::generic_category(), "tcgetattr(ZQWL CDC)");
    }
    cfmakeraw(&attributes);
    cfsetispeed(&attributes, B115200);
    cfsetospeed(&attributes, B115200);
    attributes.c_cflag |= CLOCAL | CREAD;
    attributes.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    attributes.c_cflag |= CS8;
    if (tcsetattr(serial_fd_, TCSANOW, &attributes) < 0) {
      throw std::system_error(errno, std::generic_category(), "tcsetattr(ZQWL CDC)");
    }
  }

  void writePacket(const uint8_t * data, std::size_t size)
  {
    std::size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (offset < size) {
      const auto written = write(serial_fd_, data + offset, size - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        throw std::system_error(errno, std::generic_category(), "write(ZQWL CDC)");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("write(ZQWL CDC) timed out");
      }
      struct pollfd descriptor {serial_fd_, POLLOUT, 0};
      (void)poll(&descriptor, 1, 20);
    }
  }

  void filterFrames(ChassisCanReadResult & result) const
  {
    result.frames.erase(
      std::remove_if(
        result.frames.begin(), result.frames.end(),
        [this](const chassis_can::Frame & frame) {
          return feedback_ids_.find(frame.id) == feedback_ids_.end();
        }),
      result.frames.end());
  }

  std::string port_;
  int channel_{0};
  int serial_fd_{-1};
  std::unordered_set<uint32_t> feedback_ids_;
  zqwl_cdc::FrameDecoder decoder_;
};

}  // namespace

namespace zqwl_cdc
{

ParameterPacket makeStartPacket(int channel, int bitrate)
{
  if (channel != 0) {
    throw std::invalid_argument("ZQWL CDC currently supports channel 0 only");
  }
  if (bitrate != 1000000) {
    throw std::invalid_argument("ZQWL CDC currently supports 1000000 bit/s only");
  }

  ParameterPacket packet {
    0x49, 0x3b, 0x42, 0x57, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x2e};
  packet[4] = static_cast<uint8_t>(channel);
  return packet;
}

ParameterPacket makeStopPacket()
{
  return {
    0x49, 0x3b, 0x44, 0x57, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x2e};
}

ClassicCanPacket encodeFrame(const chassis_can::Frame & frame, int channel)
{
  if (channel != 0) {
    throw std::invalid_argument("ZQWL CDC currently supports channel 0 only");
  }
  validateStandardCanId(frame.id);

  ClassicCanPacket packet {};
  packet[0] = kZqwlPacketStart;
  packet[1] = static_cast<uint8_t>(chassis_can::kPayloadSize);
  packet[2] = static_cast<uint8_t>(channel);
  packet[3] = static_cast<uint8_t>((frame.id >> 24U) & 0xffU);
  packet[4] = static_cast<uint8_t>((frame.id >> 16U) & 0xffU);
  packet[5] = static_cast<uint8_t>((frame.id >> 8U) & 0xffU);
  packet[6] = static_cast<uint8_t>(frame.id & 0xffU);
  std::copy(frame.data.begin(), frame.data.end(), packet.begin() + kZqwlHeaderSize);
  packet.back() = kZqwlPacketEnd;
  return packet;
}

ChassisCanReadResult FrameDecoder::append(
  const uint8_t * data,
  std::size_t size,
  std::size_t max_frames)
{
  if (data != nullptr && size > 0) {
    buffer_.insert(buffer_.end(), data, data + size);
  }
  if (buffer_.size() > kMaxZqwlBufferSize) {
    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + static_cast<std::ptrdiff_t>(buffer_.size() - kMaxZqwlBufferSize));
  }

  ChassisCanReadResult result;
  result.frames.reserve(max_frames);
  while (result.frames.size() < max_frames) {
    const auto start = std::find(buffer_.begin(), buffer_.end(), kZqwlPacketStart);
    if (start == buffer_.end()) {
      buffer_.clear();
      break;
    }
    if (start != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), start);
    }
    if (buffer_.size() < 2) {
      break;
    }

    const std::size_t payload_size = buffer_[1];
    if (payload_size == kZqwlStatusPacketType) {
      if (buffer_.size() < kZqwlStatusPacketSize) {
        break;
      }
      if (buffer_[kZqwlStatusPacketSize - 1] != kZqwlPacketEnd) {
        buffer_.erase(buffer_.begin());
        ++result.invalid_frames;
        continue;
      }
      buffer_.erase(
        buffer_.begin(),
        buffer_.begin() + static_cast<std::ptrdiff_t>(kZqwlStatusPacketSize));
      continue;
    }
    if (payload_size > CAN_MAX_DLEN) {
      buffer_.erase(buffer_.begin());
      ++result.invalid_frames;
      continue;
    }
    const std::size_t packet_size = payload_size + 8;
    if (buffer_.size() < packet_size) {
      break;
    }
    if (buffer_[packet_size - 1] != kZqwlPacketEnd) {
      buffer_.erase(buffer_.begin());
      ++result.invalid_frames;
      continue;
    }
    if (buffer_[2] != 0U) {
      buffer_.erase(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
      ++result.invalid_frames;
      continue;
    }
    if (payload_size != chassis_can::kPayloadSize) {
      buffer_.erase(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
      ++result.invalid_frames;
      continue;
    }

    const uint32_t can_id =
      (static_cast<uint32_t>(buffer_[3]) << 24U) |
      (static_cast<uint32_t>(buffer_[4]) << 16U) |
      (static_cast<uint32_t>(buffer_[5]) << 8U) |
      static_cast<uint32_t>(buffer_[6]);
    if (can_id > CAN_SFF_MASK) {
      buffer_.erase(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
      ++result.invalid_frames;
      continue;
    }

    chassis_can::Frame frame;
    frame.id = can_id;
    std::copy_n(buffer_.begin() + kZqwlHeaderSize, chassis_can::kPayloadSize, frame.data.begin());
    result.frames.push_back(frame);
    buffer_.erase(
      buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
  }
  return result;
}

}  // namespace zqwl_cdc

std::unique_ptr<ChassisCanTransport> makeChassisCanTransport(
  const ChassisCanTransportConfig & config,
  const std::vector<uint32_t> & feedback_ids)
{
  if (config.backend == "socketcan") {
    return std::make_unique<SocketCanTransport>(
      config.socketcan_interface, feedback_ids);
  }
  if (config.backend == "zqwl_cdc") {
    if (config.zqwl_port.empty()) {
      throw std::invalid_argument("zqwl_port must not be empty");
    }
    return std::make_unique<ZqwlCdcTransport>(
      config.zqwl_port, config.zqwl_channel, config.zqwl_bitrate, feedback_ids);
  }
  throw std::invalid_argument(
          "can_transport must be 'socketcan' or 'zqwl_cdc'");
}

}  // namespace agribot_hardware_bringup
