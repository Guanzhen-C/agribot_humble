#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/chassis_can_common.hpp"
#include "agribot_hardware_bringup/differential_can_protocol.hpp"

namespace common = agribot_hardware_bringup::chassis_can;
namespace differential = agribot_hardware_bringup::differential_can;

TEST(DifferentialCanProtocol, EncodesCommandFromExcelLayout)
{
  differential::Command command;
  command.left_percent = 50.0;
  command.right_percent = -25.0;
  command.brake = false;
  command.headlight = true;

  const auto frame = differential::encodeCommand(command, 0x12);
  EXPECT_EQ(frame.id, differential::kCommandId);
  EXPECT_EQ(frame.data[0], 0x00);
  EXPECT_EQ(frame.data[1], 0x32);
  EXPECT_EQ(frame.data[2], 0xe7);
  EXPECT_EQ(frame.data[3], 0x01);
  EXPECT_EQ(frame.data[6], 0x02);
  EXPECT_EQ(frame.data[7], common::xorChecksum(frame.data));
}

TEST(DifferentialCanProtocol, BrakeZerosMotorsAndUsesLegacyByte)
{
  differential::Command command;
  command.left_percent = 80.0;
  command.right_percent = 80.0;
  command.brake = true;

  const auto legacy = differential::encodeCommand(command, 3, true);
  EXPECT_EQ(legacy.data[0], 0x03);
  EXPECT_EQ(legacy.data[1], 0x00);
  EXPECT_EQ(legacy.data[2], 0x00);
  EXPECT_TRUE(common::hasValidChecksum(legacy.data));

  const auto excel_only = differential::encodeCommand(command, 3, false);
  EXPECT_EQ(excel_only.data[0], 0x00);
  EXPECT_TRUE(common::hasValidChecksum(excel_only.data));
}

TEST(DifferentialCanProtocol, DecodesLittleEndianChassisState)
{
  common::Frame frame;
  frame.id = differential::kChassisStateId;
  frame.data[0] = 0x0d;
  frame.data[1] = 0x04;
  frame.data[2] = 0xf4;
  frame.data[3] = 0x01;
  frame.data[4] = 0x0a;
  frame.data[6] = 0x0f;
  frame.data[7] = common::xorChecksum(frame.data);

  const auto state = differential::decodeChassisState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->work_mode, 1);
  EXPECT_TRUE(state->emergency_stop);
  EXPECT_TRUE(state->running);
  EXPECT_TRUE(state->headlight);
  EXPECT_DOUBLE_EQ(state->battery_voltage, 50.0);
  EXPECT_FALSE(state->remote_comm_fault);
  EXPECT_TRUE(state->autonomy_comm_fault);
  EXPECT_FALSE(state->motor_comm_fault);
  EXPECT_TRUE(state->bms_comm_fault);
  EXPECT_EQ(state->rolling_counter, 15);
}

TEST(DifferentialCanProtocol, RejectsBadChecksum)
{
  common::Frame frame;
  frame.id = differential::kChassisStateId;
  frame.data[7] = 0x55;
  EXPECT_FALSE(differential::decodeChassisState(frame).has_value());
}

TEST(DifferentialCanProtocol, DecodesSignedMotorFeedback)
{
  common::Frame frame;
  frame.id = differential::kLeftMotorStateId;
  frame.data[0] = 0x44;
  frame.data[1] = 0xd4;
  frame.data[2] = 0xfe;
  frame.data[3] = 48;
  frame.data[4] = static_cast<uint8_t>(static_cast<int8_t>(-12));
  frame.data[5] = 75;
  frame.data[6] = 7;
  frame.data[7] = common::xorChecksum(frame.data);

  const auto state = common::decodeMotorState(
    frame, differential::kLeftMotorStateId, differential::kRightMotorStateId);
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->temperature_fault);
  EXPECT_TRUE(state->locked_rotor);
  EXPECT_TRUE(state->hasFault());
  EXPECT_EQ(state->rpm, -300);
  EXPECT_DOUBLE_EQ(state->voltage, 48.0);
  EXPECT_DOUBLE_EQ(state->current, -12.0);
  EXPECT_DOUBLE_EQ(state->temperature_c, 35.0);
}

TEST(DifferentialCanKinematics, RoundTrip)
{
  differential::Kinematics config;
  const auto command = differential::fromTwist(0.5, 0.4, config);
  EXPECT_LT(command.left_percent, command.right_percent);
  EXPECT_LE(std::abs(command.left_percent), 100.0);
  EXPECT_LE(std::abs(command.right_percent), 100.0);

  const double circumference = M_PI * config.wheel_diameter_m;
  const auto left_rpm = static_cast<int16_t>(std::lround(
      (0.5 - 0.4 * config.track_width_m * 0.5) * 60.0 /
      circumference * config.reduction_ratio));
  const auto right_rpm = static_cast<int16_t>(std::lround(
      (0.5 + 0.4 * config.track_width_m * 0.5) * 60.0 /
      circumference * config.reduction_ratio));
  double linear = 0.0;
  double angular = 0.0;
  differential::motorRpmToTwist(left_rpm, right_rpm, config, linear, angular);
  EXPECT_NEAR(linear, 0.5, 0.001);
  EXPECT_NEAR(angular, 0.4, 0.001);
}

TEST(DifferentialCanKinematics, RejectsInvalidConfiguration)
{
  differential::Kinematics config;
  config.track_width_m = 0.0;
  EXPECT_THROW(differential::fromTwist(0.1, 0.0, config), std::invalid_argument);
}
