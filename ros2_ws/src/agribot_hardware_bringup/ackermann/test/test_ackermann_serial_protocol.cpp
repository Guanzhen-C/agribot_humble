#include <array>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/ackermann_serial_protocol.hpp"

namespace ackermann = agribot_hardware_bringup::ackermann_can;
namespace serial = agribot_hardware_bringup::ackermann_serial;

TEST(AckermannSerialProtocol, EncodesCapturedCommandFrames)
{
  struct Case
  {
    ackermann::Command command;
    serial::CommandFrame expected;
  };

  const std::array<Case, 5> cases{{
    {{0.0, 0.0}, {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x7d}},
    {{0.1, 0.0}, {0x7b, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x7d}},
    {{-0.1, 0.0}, {0x7b, 0x00, 0x00, 0xff, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7d}},
    {{0.0, 0.12}, {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x03, 0x7d}},
    {{0.0, -0.12}, {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x88, 0x0c, 0x7d}},
  }};

  for (const auto & test_case : cases) {
    EXPECT_EQ(serial::encodeCommand(test_case.command), test_case.expected);
  }
}

TEST(AckermannSerialProtocol, RecoversFragmentedTelemetryAfterNoise)
{
  const ackermann::TelemetryPayload payload{
    0x7b, 0x00, 0x00, 0x64, 0x00, 0x00, 0xff, 0xce,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x5b, 0x7d};

  serial::TelemetryParser parser;
  const std::array<uint8_t, 4> noise{0x01, 0x02, 0x03, 0x04};
  EXPECT_TRUE(parser.feed(noise.data(), noise.size()).empty());
  EXPECT_TRUE(parser.feed(payload.data(), 7U).empty());
  const auto decoded = parser.feed(payload.data() + 7U, payload.size() - 7U);

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.front().linear_velocity_x, 0.1);
  EXPECT_DOUBLE_EQ(decoded.front().angular_velocity_z, -0.05);
  EXPECT_DOUBLE_EQ(decoded.front().battery_voltage, 25.442);
  EXPECT_EQ(parser.discardedBytes(), noise.size());
  EXPECT_EQ(parser.invalidFrames(), 0U);
}

TEST(AckermannSerialProtocol, RejectsCorruptFrameAndResynchronizes)
{
  ackermann::TelemetryPayload corrupt{
    0x7b, 0x00, 0x00, 0x64, 0x00, 0x00, 0xff, 0xce,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x5b, 0x7d};
  corrupt[22] ^= 0x01U;
  const ackermann::TelemetryPayload valid{
    0x7b, 0x00, 0x00, 0x64, 0x00, 0x00, 0xff, 0xce,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x5b, 0x7d};

  std::vector<uint8_t> stream(corrupt.begin(), corrupt.end());
  stream.insert(stream.end(), valid.begin(), valid.end());
  serial::TelemetryParser parser;
  const auto decoded = parser.feed(stream.data(), stream.size());

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(parser.invalidFrames(), 1U);
  EXPECT_EQ(parser.discardedBytes(), corrupt.size());
}
