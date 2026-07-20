#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/chassis_can_protocol.hpp"

namespace can = agribot_hardware_bringup::chassis_can;

TEST(ChassisCanProtocol, EncodesDifferentialCommandFromExcelLayout)
{
  can::DifferentialCommand command;
  command.left_percent = 50.0;
  command.right_percent = -25.0;
  command.brake = false;
  command.headlight = true;

  const auto frame = can::encodeDifferentialCommand(command, 0x12);
  EXPECT_EQ(frame.id, can::kDifferentialCommandId);
  EXPECT_EQ(frame.data[0], 0x00);
  EXPECT_EQ(frame.data[1], 0x32);
  EXPECT_EQ(frame.data[2], 0xe7);
  EXPECT_EQ(frame.data[3], 0x01);
  EXPECT_EQ(frame.data[6], 0x02);
  EXPECT_EQ(frame.data[7], can::xorChecksum(frame.data));
}

TEST(ChassisCanProtocol, BrakeZerosMotorsAndUsesLegacyBrakeByte)
{
  can::DifferentialCommand command;
  command.left_percent = 80.0;
  command.right_percent = 80.0;
  command.brake = true;

  const auto legacy = can::encodeDifferentialCommand(command, 3, true);
  EXPECT_EQ(legacy.data[0], 0x03);
  EXPECT_EQ(legacy.data[1], 0x00);
  EXPECT_EQ(legacy.data[2], 0x00);
  EXPECT_TRUE(can::hasValidChecksum(legacy.data));

  const auto excel_only = can::encodeDifferentialCommand(command, 3, false);
  EXPECT_EQ(excel_only.data[0], 0x00);
  EXPECT_TRUE(can::hasValidChecksum(excel_only.data));
}

TEST(ChassisCanProtocol, DecodesLittleEndianChassisState)
{
  can::Frame frame;
  frame.id = can::kDifferentialStateId;
  frame.data[0] = 0x0d;  // autonomous, emergency stop and running
  frame.data[1] = 0x04;
  frame.data[2] = 0xf4;
  frame.data[3] = 0x01;  // 500 * 0.1 = 50 V, Intel byte order
  frame.data[4] = 0x0a;
  frame.data[6] = 0x0f;
  frame.data[7] = can::xorChecksum(frame.data);

  const auto state = can::decodeChassisState(frame);
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

TEST(ChassisCanProtocol, RejectsBadChecksum)
{
  can::Frame frame;
  frame.id = can::kDifferentialStateId;
  frame.data[7] = 0x55;
  EXPECT_FALSE(can::decodeChassisState(frame).has_value());
}

TEST(ChassisCanProtocol, DecodesSignedMotorFeedback)
{
  can::Frame frame;
  frame.id = can::kLeftMotorStateId;
  frame.data[0] = 0x44;
  frame.data[1] = 0xd4;
  frame.data[2] = 0xfe;  // -300 rpm, Intel byte order
  frame.data[3] = 48;
  frame.data[4] = static_cast<uint8_t>(static_cast<int8_t>(-12));
  frame.data[5] = 75;  // 35 C after -40 offset
  frame.data[6] = 7;
  frame.data[7] = can::xorChecksum(frame.data);

  const auto state = can::decodeMotorState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->temperature_fault);
  EXPECT_TRUE(state->locked_rotor);
  EXPECT_TRUE(state->hasFault());
  EXPECT_EQ(state->rpm, -300);
  EXPECT_DOUBLE_EQ(state->voltage, 48.0);
  EXPECT_DOUBLE_EQ(state->current, -12.0);
  EXPECT_DOUBLE_EQ(state->temperature_c, 35.0);
}

TEST(ChassisCanKinematics, DifferentialRoundTrip)
{
  can::DifferentialKinematics config;
  const auto command = can::differentialFromTwist(0.5, 0.4, config);
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
  can::motorRpmToTwist(left_rpm, right_rpm, config, linear, angular);
  EXPECT_NEAR(linear, 0.5, 0.001);
  EXPECT_NEAR(angular, 0.4, 0.001);
}

TEST(ChassisCanProtocol, EncodesReferenceAckermannLayout)
{
  can::AckermannCommand command;
  command.speed_mps = 0.8;
  command.steering_angle_rad = -0.2;
  command.enabled = true;
  command.brake = false;
  command.headlight = true;

  const auto frame = can::encodeReferenceAckermannCommand(command, 4);
  EXPECT_EQ(frame.id, can::kReferenceAckermannCommandId);
  EXPECT_EQ(frame.data[0], 0x01);
  EXPECT_EQ(frame.data[1], 0x20);
  EXPECT_EQ(frame.data[2], 0x03);
  EXPECT_EQ(frame.data[3], 0x38);
  EXPECT_EQ(frame.data[4], 0xff);
  EXPECT_EQ(frame.data[5], 0x01);
  EXPECT_EQ(frame.data[6], 0x04);
  EXPECT_TRUE(can::hasValidChecksum(frame.data));
}

TEST(ChassisCanKinematics, AckermannDoesNotRequestInPlaceRotation)
{
  can::AckermannKinematics config;
  const auto stopped = can::ackermannFromTwist(0.0, 0.5, config);
  EXPECT_DOUBLE_EQ(stopped.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(stopped.steering_angle_rad, 0.0);

  const auto moving = can::ackermannFromTwist(0.5, 0.4, config);
  EXPECT_NEAR(
    moving.steering_angle_rad,
    std::atan(config.wheelbase_m * 0.4 / 0.5),
    1e-9);
}

TEST(ChassisCanProtocol, DecodesReferenceAckermannFeedback)
{
  can::Frame frame;
  frame.id = can::kReferenceAckermannStateId;
  frame.data[0] = 0x05;  // enabled and running
  frame.data[1] = 0x0c;
  frame.data[2] = 0xfe;  // -0.500 m/s, Intel byte order
  frame.data[3] = 0xfa;
  frame.data[4] = 0x00;  // 0.250 rad, Intel byte order
  frame.data[5] = 48;
  frame.data[6] = 9;
  frame.data[7] = can::xorChecksum(frame.data);

  const auto state = can::decodeReferenceAckermannState(frame);
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

TEST(ChassisCanKinematics, RejectsInvalidConfiguration)
{
  can::DifferentialKinematics differential;
  differential.track_width_m = 0.0;
  EXPECT_THROW(
    can::differentialFromTwist(0.1, 0.0, differential),
    std::invalid_argument);

  can::AckermannKinematics ackermann;
  ackermann.wheelbase_m = 0.0;
  EXPECT_THROW(
    can::ackermannFromTwist(0.1, 0.0, ackermann),
    std::invalid_argument);
}
