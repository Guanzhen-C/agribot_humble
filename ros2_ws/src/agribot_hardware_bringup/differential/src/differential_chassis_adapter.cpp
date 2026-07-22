#include "agribot_hardware_bringup/chassis_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "agribot_hardware_bringup/differential_can_protocol.hpp"

namespace agribot_hardware_bringup
{
namespace
{

class DifferentialChassisAdapter final : public ChassisAdapter
{
public:
  explicit DifferentialChassisAdapter(rclcpp::Node & node)
  {
    legacy_brake_byte_ = node.declare_parameter<bool>("legacy_brake_byte", true);
    invert_left_motor_ = node.declare_parameter<bool>("invert_left_motor", false);
    invert_right_motor_ = node.declare_parameter<bool>("invert_right_motor", false);
    config_.track_width_m = node.declare_parameter<double>("track_width_m", 0.94);
    config_.wheel_diameter_m = node.declare_parameter<double>("wheel_diameter_m", 0.20);
    config_.reduction_ratio = node.declare_parameter<double>("reduction_ratio", 30.0);
    config_.max_motor_rpm = node.declare_parameter<double>("max_motor_rpm", 3000.0);
    config_.max_linear_velocity =
      node.declare_parameter<double>("max_linear_velocity", 0.80);
    config_.max_angular_velocity =
      node.declare_parameter<double>("max_angular_velocity", 1.4);
    (void)differential_can::fromTwist(0.0, 0.0, config_, true);
  }

  std::string type() const override
  {
    return "differential";
  }

  uint32_t commandId() const override
  {
    return differential_can::kCommandId;
  }

  std::vector<uint32_t> feedbackIds() const override
  {
    return {
      differential_can::kChassisStateId,
      differential_can::kLeftMotorStateId,
      differential_can::kRightMotorStateId};
  }

  bool usesPerFrameIntegrity() const override
  {
    return true;
  }

  chassis_can::Frame commandFromTwist(
    const geometry_msgs::msg::Twist & message,
    bool brake,
    bool headlight,
    uint8_t rolling_counter) const override
  {
    auto command = differential_can::fromTwist(
      message.linear.x, message.angular.z, config_, brake, headlight);
    if (invert_left_motor_) {
      command.left_percent *= -1.0;
    }
    if (invert_right_motor_) {
      command.right_percent *= -1.0;
    }
    return differential_can::encodeCommand(command, rolling_counter, legacy_brake_byte_);
  }

  FrameUpdate processFrame(
    const chassis_can::Frame & frame,
    const rclcpp::Time & stamp) override
  {
    FrameUpdate update;
    if (frame.id == differential_can::kChassisStateId) {
      chassis_state_ = differential_can::decodeChassisState(frame);
      if (!chassis_state_.has_value()) {
        return update;
      }
      chassis_state_time_ = stamp;
      chassis_state_received_ = true;
      update.valid = true;
      update.emergency_stop = chassis_state_->emergency_stop;
      return update;
    }

    const auto motor = chassis_can::decodeMotorState(
      frame, differential_can::kLeftMotorStateId,
      differential_can::kRightMotorStateId);
    if (!motor.has_value()) {
      return update;
    }

    if (frame.id == differential_can::kLeftMotorStateId) {
      left_motor_state_ = motor;
      left_motor_time_ = stamp;
      left_motor_received_ = true;
      left_motor_updated_ = true;
    } else {
      right_motor_state_ = motor;
      right_motor_time_ = stamp;
      right_motor_received_ = true;
      right_motor_updated_ = true;
    }
    update.valid = true;

    if (left_motor_updated_ && right_motor_updated_) {
      int16_t left_rpm = left_motor_state_->rpm;
      int16_t right_rpm = right_motor_state_->rpm;
      if (invert_left_motor_) {
        left_rpm = static_cast<int16_t>(-left_rpm);
      }
      if (invert_right_motor_) {
        right_rpm = static_cast<int16_t>(-right_rpm);
      }
      MeasuredMotion motion;
      differential_can::motorRpmToTwist(
        left_rpm, right_rpm, config_,
        motion.linear_velocity, motion.angular_velocity);
      update.motion = motion;
      left_motor_updated_ = false;
      right_motor_updated_ = false;
    }
    return update;
  }

  bool feedbackFresh(
    const rclcpp::Time & current_time,
    double timeout_sec) const override
  {
    return isFresh(chassis_state_received_, chassis_state_time_, current_time, timeout_sec) &&
           isFresh(left_motor_received_, left_motor_time_, current_time, timeout_sec) &&
           isFresh(right_motor_received_, right_motor_time_, current_time, timeout_sec);
  }

  bool feedbackAllowsMotion(bool require_autonomous_mode) const override
  {
    if (!chassis_state_.has_value() || !left_motor_state_.has_value() ||
      !right_motor_state_.has_value() || chassis_state_->emergency_stop)
    {
      return false;
    }
    if (require_autonomous_mode && chassis_state_->work_mode != 1U) {
      return false;
    }
    if (chassis_state_->remote_comm_fault || chassis_state_->autonomy_comm_fault ||
      chassis_state_->motor_comm_fault || chassis_state_->bms_comm_fault)
    {
      return false;
    }
    return !left_motor_state_->hasFault() && !right_motor_state_->hasFault();
  }

  void populateStatus(scout_msgs::msg::ScoutStatus & status) const override
  {
    if (!chassis_state_.has_value()) {
      return;
    }
    status.control_mode = chassis_state_->work_mode;
    status.battery_voltage = chassis_state_->battery_voltage;
    const bool chassis_fault = chassis_state_->emergency_stop ||
      chassis_state_->remote_comm_fault || chassis_state_->autonomy_comm_fault ||
      chassis_state_->motor_comm_fault || chassis_state_->bms_comm_fault;
    status.base_state = chassis_fault ? 1U : 0U;
    status.fault_code = chassis_fault ? 1U : 0U;

    const auto fillMotor = [&](std::size_t index, const chassis_can::MotorState & motor) {
        status.motor_states[index].current = motor.current;
        status.motor_states[index].rpm = motor.rpm;
        status.motor_states[index].temperature = motor.temperature_c;
        status.motor_states[index].motor_pose = 0.0;
      };
    if (left_motor_state_.has_value()) {
      fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_LEFT, *left_motor_state_);
      fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_REAR_LEFT, *left_motor_state_);
      if (left_motor_state_->hasFault()) {
        status.fault_code |= 0x02U;
      }
    }
    if (right_motor_state_.has_value()) {
      fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_FRONT_RIGHT, *right_motor_state_);
      fillMotor(scout_msgs::msg::ScoutStatus::MOTOR_ID_REAR_RIGHT, *right_motor_state_);
      if (right_motor_state_->hasFault()) {
        status.fault_code |= 0x04U;
      }
    }
  }

private:
  static bool isFresh(
    bool received,
    const rclcpp::Time & stamp,
    const rclcpp::Time & current_time,
    double timeout_sec)
  {
    if (!received) {
      return false;
    }
    const double age = (current_time - stamp).seconds();
    return age >= 0.0 && age <= timeout_sec;
  }

  differential_can::Kinematics config_;
  bool legacy_brake_byte_{true};
  bool invert_left_motor_{false};
  bool invert_right_motor_{false};
  rclcpp::Time chassis_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time left_motor_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_motor_time_{0, 0, RCL_ROS_TIME};
  bool chassis_state_received_{false};
  bool left_motor_received_{false};
  bool right_motor_received_{false};
  bool left_motor_updated_{false};
  bool right_motor_updated_{false};
  std::optional<differential_can::ChassisState> chassis_state_;
  std::optional<chassis_can::MotorState> left_motor_state_;
  std::optional<chassis_can::MotorState> right_motor_state_;
};

}  // namespace

std::unique_ptr<ChassisAdapter> makeDifferentialChassisAdapter(rclcpp::Node & node)
{
  return std::make_unique<DifferentialChassisAdapter>(node);
}

}  // namespace agribot_hardware_bringup
