#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"
#include "agribot_hardware_bringup/chassis_adapter.hpp"

namespace ackermann = agribot_hardware_bringup::ackermann_can;

TEST(AckermannCanProtocol, EncodesVerifiedC50cCommandLayout)
{
  struct Case
  {
    ackermann::Command command;
    std::array<uint8_t, 8> expected;
  };

  const std::array<Case, 5> cases{{
    {{0.0, 0.0}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0.1, 0.0}, {0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{-0.1, 0.0}, {0xff, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {{0.0, 0.12}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00}},
    {{0.1, -0.12}, {0x00, 0x64, 0x00, 0x00, 0xff, 0x88, 0x00, 0x00}},
  }};

  for (const auto & test_case : cases) {
    const auto frame = ackermann::encodeCommand(test_case.command);
    EXPECT_EQ(frame.id, ackermann::kCommandId);
    EXPECT_EQ(frame.data, test_case.expected);
  }
}

TEST(AckermannCanProtocol, RejectsNonFiniteCommands)
{
  ackermann::Command command;
  command.speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ackermann::encodeCommand(command), std::invalid_argument);
}

TEST(AckermannCanProtocol, DecodesCapturedC50cTelemetry)
{
  const ackermann::TelemetryPayload payload{
    0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x0e, 0x7d};

  const auto telemetry = ackermann::decodeTelemetry(payload);
  ASSERT_TRUE(telemetry.has_value());
  EXPECT_EQ(telemetry->stop_flag, 0U);
  EXPECT_DOUBLE_EQ(telemetry->linear_velocity_x, 0.0);
  EXPECT_DOUBLE_EQ(telemetry->linear_velocity_y, 0.0);
  EXPECT_DOUBLE_EQ(telemetry->angular_velocity_z, 0.0);
  EXPECT_NEAR(telemetry->linear_acceleration_x, 3416.0 / 1671.84, 1e-9);
  EXPECT_NEAR(telemetry->linear_acceleration_y, 1038.0 / 1671.84, 1e-9);
  EXPECT_NEAR(telemetry->linear_acceleration_z, 17002.0 / 1671.84, 1e-9);
  EXPECT_NEAR(telemetry->angular_velocity_x, 2.0 * 0.00026644, 1e-12);
  EXPECT_NEAR(telemetry->angular_velocity_y, 0.00026644, 1e-12);
  EXPECT_DOUBLE_EQ(telemetry->imu_angular_velocity_z, 0.0);
  EXPECT_DOUBLE_EQ(telemetry->battery_voltage, 25.442);
}

TEST(AckermannCanProtocol, RejectsCorruptTelemetry)
{
  ackermann::TelemetryPayload payload{
    0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x0e, 0x7d};

  payload[2] ^= 0x01U;
  EXPECT_FALSE(ackermann::decodeTelemetry(payload).has_value());
  payload[2] ^= 0x01U;
  payload[23] = 0x00U;
  EXPECT_FALSE(ackermann::decodeTelemetry(payload).has_value());
}

TEST(AckermannCanKinematics, ConvertsYawRateAndDoesNotRequestInPlaceRotation)
{
  ackermann::Kinematics config;
  const auto stopped = ackermann::fromTwist(0.0, 0.5, config);
  EXPECT_DOUBLE_EQ(stopped.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(stopped.steering_angle_rad, 0.0);

  const auto moving = ackermann::fromTwist(0.5, 0.1, config);
  EXPECT_NEAR(
    moving.steering_angle_rad,
    std::atan(config.wheelbase_m * 0.1 / 0.5),
    1e-9);

  const auto limited = ackermann::fromTwist(0.1, 0.65, config);
  EXPECT_DOUBLE_EQ(limited.steering_angle_rad, config.max_steering_angle_rad);
}

TEST(AckermannCanKinematics, StopRequestProducesAllZeroCommand)
{
  ackermann::Kinematics config;
  const auto stopped = ackermann::fromTwist(0.5, 0.2, config, true);
  const auto frame = ackermann::encodeCommand(stopped);
  EXPECT_EQ(frame.data, (std::array<uint8_t, 8>{}));
}

TEST(AckermannCanKinematics, RejectsInvalidConfiguration)
{
  ackermann::Kinematics config;
  config.wheelbase_m = 0.0;
  EXPECT_THROW(ackermann::fromTwist(0.1, 0.0, config), std::invalid_argument);
}

TEST(AckermannCanAdapter, ReassemblesTelemetryAndRejectsBadBcc)
{
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>("ackermann_can_adapter_test");
  auto adapter = agribot_hardware_bringup::makeAckermannChassisAdapter(*node);
  EXPECT_FALSE(adapter->usesPerFrameIntegrity());
  EXPECT_EQ(
    adapter->feedbackIds(),
    (std::vector<uint32_t>{
      ackermann::kFeedbackPart1Id,
      ackermann::kFeedbackPart2Id,
      ackermann::kFeedbackPart3Id}));

  ackermann::TelemetryPayload payload{
    0x7b, 0x00, 0x00, 0x64, 0x00, 0x00, 0xff, 0xce,
    0x0d, 0x58, 0x04, 0x0e, 0x42, 0x6a, 0x00, 0x02,
    0x00, 0x01, 0x00, 0x00, 0x63, 0x62, 0x5b, 0x7d};
  const std::array<uint32_t, 3> ids{
    ackermann::kFeedbackPart1Id,
    ackermann::kFeedbackPart2Id,
    ackermann::kFeedbackPart3Id};
  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);

  agribot_hardware_bringup::FrameUpdate update;
  for (std::size_t part = 0; part < ids.size(); ++part) {
    agribot_hardware_bringup::chassis_can::Frame frame;
    frame.id = ids[part];
    std::copy_n(
      payload.begin() + static_cast<std::ptrdiff_t>(part * 8), 8, frame.data.begin());
    update = adapter->processFrame(frame, stamp);
    EXPECT_TRUE(update.valid);
  }
  ASSERT_TRUE(update.motion.has_value());
  EXPECT_DOUBLE_EQ(update.motion->linear_velocity, 0.1);
  EXPECT_DOUBLE_EQ(update.motion->angular_velocity, -0.05);
  EXPECT_TRUE(adapter->feedbackFresh(rclcpp::Time(10, 500000000, RCL_ROS_TIME), 0.6));
  EXPECT_FALSE(adapter->feedbackFresh(rclcpp::Time(10, 700000000, RCL_ROS_TIME), 0.6));
  EXPECT_TRUE(adapter->feedbackAllowsMotion(false));
  EXPECT_FALSE(adapter->feedbackAllowsMotion(true));

  payload[22] ^= 0x01U;
  for (std::size_t part = 0; part < ids.size(); ++part) {
    agribot_hardware_bringup::chassis_can::Frame frame;
    frame.id = ids[part];
    std::copy_n(
      payload.begin() + static_cast<std::ptrdiff_t>(part * 8), 8, frame.data.begin());
    update = adapter->processFrame(frame, stamp);
  }
  EXPECT_FALSE(update.valid);
  EXPECT_TRUE(update.checksum_error);
  rclcpp::shutdown();
}
