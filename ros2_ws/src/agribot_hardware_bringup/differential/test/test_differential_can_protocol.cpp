#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "agribot_hardware_bringup/chassis_adapter.hpp"
#include "agribot_hardware_bringup/chassis_can_common.hpp"
#include "agribot_hardware_bringup/differential_can_protocol.hpp"

namespace common = agribot_hardware_bringup::chassis_can;
namespace differential = agribot_hardware_bringup::differential_can;

namespace
{

common::Frame finalizeFrame(uint32_t id, const common::Payload & data)
{
  common::Frame frame{id, data};
  frame.data[7] = common::xorChecksum(frame.data);
  return frame;
}

}  // namespace

TEST(DifferentialCanProtocol, EncodesThreeInOneCommandLayout)
{
  differential::Command command;
  command.left_percent = 50.0;
  command.right_percent = -25.0;
  command.headlight = true;

  const auto frame = differential::encodeCommand(command, 0x12);
  EXPECT_EQ(frame.id, differential::kCommandId);
  EXPECT_EQ(
    frame.data,
    (common::Payload {0x00, 0x32, 0xe7, 0x01, 0x00, 0x00, 0x02, 0xd6}));
  EXPECT_TRUE(common::hasValidChecksum(frame.data));
}

TEST(DifferentialCanProtocol, UsesZeroPwmForStopAndKeepsReservedBytesZero)
{
  differential::Command command;
  command.left_percent = 80.0;
  command.right_percent = -40.0;

  const auto frame = differential::encodeCommand(command, 3);
  EXPECT_EQ(frame.data[0], 0x00);
  EXPECT_EQ(frame.data[1], 0x50);
  EXPECT_EQ(frame.data[2], 0xd8);
  EXPECT_EQ(frame.data[3], 0x00);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_TRUE(common::hasValidChecksum(frame.data));

  const auto stopped = differential::fromTwist(0.5, 0.2, {}, true);
  const auto stopped_frame = differential::encodeCommand(stopped, 4);
  EXPECT_EQ(stopped_frame.data[0], 0x00);
  EXPECT_EQ(stopped_frame.data[1], 0x00);
  EXPECT_EQ(stopped_frame.data[2], 0x00);
}

TEST(DifferentialCanProtocol, DecodesThreeInOneChassisState)
{
  const auto frame = finalizeFrame(
    differential::kChassisStateId,
    {0x2d, 0x04, 0xe7, 0x01, 0x00, 0x00, 0x0f, 0x00});

  const auto state = differential::decodeChassisState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->work_mode, 1);
  EXPECT_TRUE(state->emergency_stop);
  EXPECT_TRUE(state->running);
  EXPECT_EQ(state->remote_connection_status, 2);
  EXPECT_TRUE(state->headlight);
  EXPECT_DOUBLE_EQ(state->battery_voltage, 48.7);
  EXPECT_EQ(state->rolling_counter, 15);
}

TEST(DifferentialCanProtocol, RejectsInvalidChassisFrames)
{
  common::Frame frame;
  frame.id = differential::kChassisStateId;
  frame.data[7] = 0x55;
  EXPECT_FALSE(differential::decodeChassisState(frame).has_value());

  frame = finalizeFrame(0x531, {});
  EXPECT_FALSE(differential::decodeChassisState(frame).has_value());
}

TEST(DifferentialCanProtocol, DecodesThreeInOneMotorFeedback)
{
  const auto frame = finalizeFrame(
    differential::kLeftMotorStateId,
    {0xad, 0xf4, 0xd4, 0xfe, 0x7b, 0x00, 0x07, 0x00});

  const auto state = differential::decodeMotorState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_TRUE(state->over_current_protection);
  EXPECT_FALSE(state->load_fault);
  EXPECT_TRUE(state->over_temperature_protection);
  EXPECT_TRUE(state->over_voltage_protection);
  EXPECT_FALSE(state->under_voltage_protection);
  EXPECT_TRUE(state->locked_rotor_protection);
  EXPECT_FALSE(state->hall_fault);
  EXPECT_TRUE(state->shake_fault);
  EXPECT_EQ(state->temperature, -12);
  EXPECT_EQ(state->speed, -300);
  EXPECT_EQ(state->running_current, 123);
  EXPECT_EQ(state->rolling_counter, 7);
  EXPECT_TRUE(state->hasFault());
}

TEST(DifferentialCanProtocol, DecodesSignedSpeedTemperatureAndCurrent)
{
  const auto frame = finalizeFrame(
    differential::kRightMotorStateId,
    {0x00, 0x50, 0xff, 0x7f, 0x00, 0x80, 0x0a, 0x00});

  const auto state = differential::decodeMotorState(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->temperature, 80);
  EXPECT_EQ(state->speed, 32767);
  EXPECT_EQ(state->running_current, -32768);
  EXPECT_FALSE(state->hasFault());
}

TEST(DifferentialCanProtocol, RejectsInvalidMotorFrames)
{
  auto frame = finalizeFrame(differential::kLeftMotorStateId, {});
  frame.data[7] ^= 0x01U;
  EXPECT_FALSE(differential::decodeMotorState(frame).has_value());

  frame = finalizeFrame(0x535, {});
  EXPECT_FALSE(differential::decodeMotorState(frame).has_value());
}

TEST(DifferentialCanKinematics, RoundTrip)
{
  differential::Kinematics config;
  const auto command = differential::fromTwist(0.5, 0.4, config);
  EXPECT_LT(command.left_percent, command.right_percent);
  EXPECT_LE(std::abs(command.left_percent), 100.0);
  EXPECT_LE(std::abs(command.right_percent), 100.0);

  const auto left_speed_units = static_cast<int32_t>(std::lround(
      (0.5 - 0.4 * config.track_width_m * 0.5) /
      config.feedback_wheel_speed_mps_per_speed_unit));
  const auto right_speed_units = static_cast<int32_t>(std::lround(
      (0.5 + 0.4 * config.track_width_m * 0.5) /
      config.feedback_wheel_speed_mps_per_speed_unit));
  double linear = 0.0;
  double angular = 0.0;
  differential::motorSpeedToTwist(
    left_speed_units, right_speed_units, config, linear, angular);
  EXPECT_NEAR(linear, 0.5, config.feedback_wheel_speed_mps_per_speed_unit);
  EXPECT_NEAR(
    angular, 0.4,
    2.0 * config.feedback_wheel_speed_mps_per_speed_unit / config.track_width_m);
}

TEST(DifferentialCanKinematics, MapsWheelSpeedDirectlyToCommandPercent)
{
  differential::Kinematics config;
  config.max_linear_velocity = config.command_full_scale_wheel_speed_mps;
  const auto command = differential::fromTwist(
    config.command_full_scale_wheel_speed_mps * 0.25, 0.0, config);
  EXPECT_DOUBLE_EQ(command.left_percent, 25.0);
  EXPECT_DOUBLE_EQ(command.right_percent, 25.0);
}

TEST(DifferentialCanKinematics, RejectsInvalidConfiguration)
{
  differential::Kinematics config;
  config.track_width_m = 0.0;
  EXPECT_THROW(differential::fromTwist(0.1, 0.0, config), std::invalid_argument);
}

TEST(DifferentialCanAdapter, UsesThreeInOneFeedbackForMotionAndSafety)
{
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>("differential_can_adapter_test");
  auto adapter = agribot_hardware_bringup::makeDifferentialChassisAdapter(*node);
  EXPECT_TRUE(adapter->usesPerFrameIntegrity());
  EXPECT_EQ(
    adapter->feedbackIds(),
    (std::vector<uint32_t>{
      differential::kChassisStateId,
      differential::kLeftMotorStateId,
      differential::kRightMotorStateId}));

  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  auto chassis = finalizeFrame(
    differential::kChassisStateId,
    {0x21, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x01, 0x00});
  auto left = finalizeFrame(
    differential::kLeftMotorStateId,
    {0x00, 0x14, 0xe8, 0x03, 0x04, 0x00, 0x01, 0x00});
  auto right = finalizeFrame(
    differential::kRightMotorStateId,
    {0x00, 0x15, 0xb0, 0x04, 0x05, 0x00, 0x01, 0x00});

  EXPECT_TRUE(adapter->processFrame(chassis, stamp).valid);
  EXPECT_TRUE(adapter->processFrame(left, stamp).valid);
  const auto update = adapter->processFrame(right, stamp);
  ASSERT_TRUE(update.motion.has_value());
  EXPECT_TRUE(adapter->feedbackFresh(rclcpp::Time(10, 500000000, RCL_ROS_TIME), 0.6));
  EXPECT_TRUE(adapter->feedbackAllowsMotion(true));

  scout_msgs::msg::ScoutStatus status;
  adapter->populateStatus(status);
  EXPECT_DOUBLE_EQ(status.battery_voltage, 48.0);
  EXPECT_DOUBLE_EQ(
    status.motor_states[scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_LEFT].rpm,
    1000.0);
  EXPECT_DOUBLE_EQ(
    status.motor_states[scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_RIGHT].rpm,
    1200.0);
  EXPECT_DOUBLE_EQ(
    status.motor_states[scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_LEFT].temperature,
    20.0);
  EXPECT_DOUBLE_EQ(
    status.motor_states[scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_RIGHT].temperature,
    21.0);

  left.data[0] = 0x01;
  left.data[6] = 0x02;
  left.data[7] = common::xorChecksum(left.data);
  EXPECT_TRUE(adapter->processFrame(left, stamp).valid);
  EXPECT_FALSE(adapter->feedbackAllowsMotion(true));
  rclcpp::shutdown();
}
