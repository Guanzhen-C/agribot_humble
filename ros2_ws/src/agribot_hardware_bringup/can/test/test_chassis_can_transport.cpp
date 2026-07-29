#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "agribot_hardware_bringup/chassis_can_transport.hpp"

namespace agribot_hardware_bringup
{
namespace
{

TEST(ZqwlCdcProtocol, BuildsVerifiedOneMegabitStartAndStopPackets)
{
  EXPECT_EQ(
    zqwl_cdc::makeStartPacket(0, 1000000),
    (zqwl_cdc::ParameterPacket {
        0x49, 0x3b, 0x42, 0x57, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x2e}));
  EXPECT_EQ(
    zqwl_cdc::makeStopPacket(),
    (zqwl_cdc::ParameterPacket {
        0x49, 0x3b, 0x44, 0x57, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x2e}));
  EXPECT_THROW(zqwl_cdc::makeStartPacket(1, 1000000), std::invalid_argument);
  EXPECT_THROW(zqwl_cdc::makeStartPacket(0, 500000), std::invalid_argument);
}

TEST(ZqwlCdcProtocol, EncodesClassicCanFrame)
{
  chassis_can::Frame frame;
  frame.id = 0x181;
  frame.data = {0x00, 0x64, 0x00, 0x00, 0xff, 0x9c, 0x00, 0x00};

  EXPECT_EQ(
    zqwl_cdc::encodeFrame(frame, 0),
    (zqwl_cdc::ClassicCanPacket {
        0x5a, 0x08, 0x00, 0x00, 0x00, 0x01, 0x81, 0x00,
        0x64, 0x00, 0x00, 0xff, 0x9c, 0x00, 0x00, 0xa5}));
}

TEST(ZqwlCdcProtocol, DecodesFragmentedAndCoalescedFeedback)
{
  const std::array<uint8_t, 4> noise {0x49, 0x3b, 0x42, 0x57};
  const std::array<chassis_can::Frame, 3> source {{
    {0x101, {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x102, {0xff, 0xf5, 0xff, 0xef, 0x40, 0x16, 0xff, 0xfc}},
    {0x103, {0x00, 0x04, 0xff, 0xfc, 0x57, 0x47, 0x23, 0x7d}},
  }};

  std::vector<uint8_t> stream(noise.begin(), noise.end());
  for (const auto & frame : source) {
    const auto packet = zqwl_cdc::encodeFrame(frame, 0);
    stream.insert(stream.end(), packet.begin(), packet.end());
  }

  zqwl_cdc::FrameDecoder decoder;
  auto decoded = decoder.append(stream.data(), 11, 64);
  EXPECT_TRUE(decoded.frames.empty());
  decoded = decoder.append(stream.data() + 11, stream.size() - 11, 2);
  ASSERT_EQ(decoded.frames.size(), 2U);
  EXPECT_EQ(decoded.frames[0].id, 0x101U);
  EXPECT_EQ(decoded.frames[0].data, source[0].data);
  EXPECT_EQ(decoded.frames[1].id, 0x102U);
  EXPECT_EQ(decoded.frames[1].data, source[1].data);

  decoded = decoder.append(nullptr, 0, 64);
  ASSERT_EQ(decoded.frames.size(), 1U);
  EXPECT_EQ(decoded.frames[0].id, 0x103U);
  EXPECT_EQ(decoded.frames[0].data, source[2].data);
}

TEST(ZqwlCdcProtocol, ResynchronizesAfterMalformedPacket)
{
  const std::array<uint8_t, 5> malformed {0x5a, 0xfe, 0x00, 0x00, 0x00};
  chassis_can::Frame source;
  source.id = 0x101;
  source.data = {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const auto valid = zqwl_cdc::encodeFrame(source, 0);

  std::vector<uint8_t> stream(malformed.begin(), malformed.end());
  stream.insert(stream.end(), valid.begin(), valid.end());

  zqwl_cdc::FrameDecoder decoder;
  const auto decoded = decoder.append(stream.data(), stream.size(), 64);
  ASSERT_EQ(decoded.frames.size(), 1U);
  EXPECT_EQ(decoded.frames.front().id, 0x101U);
  EXPECT_GT(decoded.invalid_frames, 0U);
}

TEST(ZqwlCdcProtocol, RejectsFramesFromAnInactiveChannel)
{
  chassis_can::Frame source;
  source.id = 0x101;
  source.data = {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  auto inactive_channel = zqwl_cdc::encodeFrame(source, 0);
  inactive_channel[2] = 1;
  const auto valid = zqwl_cdc::encodeFrame(source, 0);

  std::vector<uint8_t> stream(inactive_channel.begin(), inactive_channel.end());
  stream.insert(stream.end(), valid.begin(), valid.end());

  zqwl_cdc::FrameDecoder decoder;
  const auto decoded = decoder.append(stream.data(), stream.size(), 64);
  ASSERT_EQ(decoded.frames.size(), 1U);
  EXPECT_EQ(decoded.frames.front().id, 0x101U);
  EXPECT_EQ(decoded.invalid_frames, 1U);
}

}  // namespace
}  // namespace agribot_hardware_bringup
