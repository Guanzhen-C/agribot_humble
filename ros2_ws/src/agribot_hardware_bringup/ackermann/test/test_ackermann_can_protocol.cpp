#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"
#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace ackermann = agribot_hardware_bringup::ackermann_can;
namespace common = agribot_hardware_bringup::chassis_can;

TEST(AckermannCanProtocol, EncodesCommandLayout)
{
  ackermann::Command command;
  command.speed_mps = 0.8;
  command.steering_angle_rad = -0.2;
  command.enabled = true;
  command.brake = false;
  command.headlight = true;

  const auto frame = ackermann::encodeCommand(command, 4);
  EXPECT_EQ(frame.id, ackermann::kCommandId);
  EXPECT_EQ(frame.data[0], 0x01);
  EXPECT_EQ(frame.data[1], 0x20);
  EXPECT_EQ(frame.data[2], 0x03);
  EXPECT_EQ(frame.data[3], 0x38);
  EXPECT_EQ(frame.data[4], 0xff);
  EXPECT_EQ(frame.data[5], 0x01);
  EXPECT_EQ(frame.data[6], 0x04);
  EXPECT_TRUE(common::hasValidChecksum(frame.data));
}

TEST(AckermannCanKinematics, DoesNotRequestInPlaceRotation)
{
  ackermann::Kinematics config;
  const auto stopped = ackermann::fromTwist(0.0, 0.5, config);
  EXPECT_DOUBLE_EQ(stopped.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(stopped.steering_angle_rad, 0.0);

  const auto moving = ackermann::fromTwist(0.5, 0.4, config);
  EXPECT_NEAR(
    moving.steering_angle_rad,
    std::atan(config.wheelbase_m * 0.4 / 0.5),
    1e-9);
}

TEST(AckermannCanProtocol, DecodesChassisFeedback)
{
  common::Frame frame;
  frame.id = ackermann::kChassisStateId;
  frame.data[0] = 0x05;
  frame.data[1] = 0x0c;
  frame.data[2] = 0xfe;
  frame.data[3] = 0xfa;
  frame.data[4] = 0x00;
  frame.data[5] = 48;
  frame.data[6] = 9;
  frame.data[7] = common::xorChecksum(frame.data);

  const auto state = ackermann::decodeChassisState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->enabled);
  EXPECT_TRUE(state->running);
  EXPECT_FALSE(state->emergency_stop);
  EXPECT_FALSE(state->fault);
  EXPECT_DOUBLE_EQ(state->speed_mps, -0.5);
  EXPECT_DOUBLE_EQ(state->steering_angle_rad, 0.25);
  EXPECT_DOUBLE_EQ(state->battery_voltage, 48.0);
  EXPECT_EQ(state->rolling_counter, 9);
}

TEST(AckermannCanProtocol, DecodesMotorFeedback)
{
  common::Frame drive;
  drive.id = ackermann::kDriveStateId;
  drive.data[0] = 0x08;
  drive.data[1] = 0x2e;
  drive.data[2] = 0xfb;
  drive.data[3] = 48;
  drive.data[4] = static_cast<uint8_t>(static_cast<int8_t>(-7));
  drive.data[5] = 70;
  drive.data[6] = 0x0b;
  drive.data[7] = common::xorChecksum(drive.data);

  const auto drive_state = common::decodeMotorState(
    drive, ackermann::kDriveStateId, ackermann::kSteeringStateId);
  ASSERT_TRUE(drive_state.has_value());
  EXPECT_TRUE(drive_state->over_current);
  EXPECT_TRUE(drive_state->hasFault());
  EXPECT_EQ(drive_state->rpm, -1234);
  EXPECT_DOUBLE_EQ(drive_state->voltage, 48.0);
  EXPECT_DOUBLE_EQ(drive_state->current, -7.0);
  EXPECT_DOUBLE_EQ(drive_state->temperature_c, 30.0);
  EXPECT_EQ(drive_state->rolling_counter, 11);

  drive.id = 0x538;
  drive.data[7] = common::xorChecksum(drive.data);
  EXPECT_FALSE(
    common::decodeMotorState(
      drive, ackermann::kDriveStateId,
      ackermann::kSteeringStateId).has_value());
}

TEST(AckermannCanKinematics, RejectsInvalidConfiguration)
{
  ackermann::Kinematics config;
  config.wheelbase_m = 0.0;
  EXPECT_THROW(ackermann::fromTwist(0.1, 0.0, config), std::invalid_argument);
}
