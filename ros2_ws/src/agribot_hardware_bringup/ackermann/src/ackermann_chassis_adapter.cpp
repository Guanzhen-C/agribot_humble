#include "agribot_hardware_bringup/chassis_adapter.hpp"

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "agribot_hardware_bringup/ackermann_can_protocol.hpp"

namespace agribot_hardware_bringup
{
namespace
{

class AckermannChassisAdapter final : public ChassisAdapter
{
public:
  explicit AckermannChassisAdapter(rclcpp::Node & node)
  {
    config_.wheelbase_m = node.declare_parameter<double>("wheelbase_m", 0.65);
    config_.max_steering_angle_rad =
      node.declare_parameter<double>("max_steering_angle_rad", 0.60);
    config_.max_linear_velocity =
      node.declare_parameter<double>("max_linear_velocity", 0.80);
    config_.max_angular_velocity =
      node.declare_parameter<double>("max_angular_velocity", 0.65);
    config_.minimum_motion_speed =
      node.declare_parameter<double>("minimum_motion_speed", 0.02);
    command_id_ = declareId(node, "command_id", ackermann_can::kCommandId);
    chassis_state_id_ = declareId(node, "chassis_state_id", ackermann_can::kChassisStateId);
    drive_state_id_ = declareId(node, "drive_state_id", ackermann_can::kDriveStateId);
    steering_state_id_ = declareId(node, "steering_state_id", ackermann_can::kSteeringStateId);
    const bool allow_unverified =
      node.declare_parameter<bool>("allow_unverified_protocol", false);
    if (!allow_unverified) {
      throw std::invalid_argument(
              "Ackermann CAN layout is a reference only; set "
              "allow_unverified_protocol:=true only after controller confirmation");
    }
    (void)ackermann_can::fromTwist(0.0, 0.0, config_, true);
  }

  std::string type() const override
  {
    return "ackermann";
  }

  uint32_t commandId() const override
  {
    return command_id_;
  }

  std::vector<uint32_t> feedbackIds() const override
  {
    return {chassis_state_id_, drive_state_id_, steering_state_id_};
  }

  chassis_can::Frame commandFromTwist(
    const geometry_msgs::msg::Twist & message,
    bool brake,
    bool headlight,
    uint8_t rolling_counter) const override
  {
    const auto command = ackermann_can::fromTwist(
      message.linear.x, message.angular.z, config_, brake, headlight);
    return ackermann_can::encodeCommand(command, rolling_counter, command_id_);
  }

  FrameUpdate processFrame(
    const chassis_can::Frame & frame,
    const rclcpp::Time & stamp) override
  {
    FrameUpdate update;
    if (frame.id == chassis_state_id_) {
      chassis_state_ = ackermann_can::decodeChassisState(frame, chassis_state_id_);
      if (!chassis_state_.has_value()) {
        return update;
      }
      chassis_state_time_ = stamp;
      chassis_state_received_ = true;
      update.valid = true;
      update.emergency_stop = chassis_state_->emergency_stop;
      MeasuredMotion motion;
      motion.linear_velocity = chassis_state_->speed_mps;
      motion.angular_velocity = std::abs(chassis_state_->speed_mps) < 1e-6 ? 0.0 :
        chassis_state_->speed_mps * std::tan(chassis_state_->steering_angle_rad) /
        config_.wheelbase_m;
      update.motion = motion;
      return update;
    }

    const auto motor = chassis_can::decodeMotorState(
      frame, drive_state_id_, steering_state_id_);
    if (!motor.has_value()) {
      return update;
    }
    update.valid = true;
    if (frame.id == drive_state_id_) {
      drive_state_ = motor;
      drive_state_time_ = stamp;
      drive_state_received_ = true;
    } else {
      steering_state_ = motor;
      steering_state_time_ = stamp;
      steering_state_received_ = true;
    }
    return update;
  }

  bool feedbackFresh(
    const rclcpp::Time & current_time,
    double timeout_sec) const override
  {
    return isFresh(chassis_state_received_, chassis_state_time_, current_time, timeout_sec) &&
           isFresh(drive_state_received_, drive_state_time_, current_time, timeout_sec) &&
           isFresh(steering_state_received_, steering_state_time_, current_time, timeout_sec);
  }

  bool feedbackAllowsMotion(bool) const override
  {
    return chassis_state_.has_value() && drive_state_.has_value() &&
           steering_state_.has_value() && chassis_state_->enabled &&
           !chassis_state_->emergency_stop && !chassis_state_->fault &&
           !drive_state_->hasFault() && !steering_state_->hasFault();
  }

  void populateStatus(scout_msgs::msg::ScoutStatus & status) const override
  {
    if (!chassis_state_.has_value()) {
      return;
    }
    status.control_mode = chassis_state_->enabled ? 1U : 0U;
    status.battery_voltage = chassis_state_->battery_voltage;
    const bool chassis_fault = chassis_state_->emergency_stop || chassis_state_->fault;
    const bool drive_fault = drive_state_.has_value() && drive_state_->hasFault();
    const bool steering_fault = steering_state_.has_value() && steering_state_->hasFault();
    status.base_state = (chassis_fault || drive_fault || steering_fault) ? 1U : 0U;
    status.fault_code = (chassis_fault ? 0x01U : 0x00U) |
      (drive_fault ? 0x02U : 0x00U) | (steering_fault ? 0x04U : 0x00U);
  }

private:
  static uint32_t declareId(rclcpp::Node & node, const char * name, uint32_t value)
  {
    return static_cast<uint32_t>(
      node.declare_parameter<int64_t>(name, static_cast<int64_t>(value)));
  }

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

  ackermann_can::Kinematics config_;
  uint32_t command_id_{ackermann_can::kCommandId};
  uint32_t chassis_state_id_{ackermann_can::kChassisStateId};
  uint32_t drive_state_id_{ackermann_can::kDriveStateId};
  uint32_t steering_state_id_{ackermann_can::kSteeringStateId};
  rclcpp::Time chassis_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time drive_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time steering_state_time_{0, 0, RCL_ROS_TIME};
  bool chassis_state_received_{false};
  bool drive_state_received_{false};
  bool steering_state_received_{false};
  std::optional<ackermann_can::ChassisState> chassis_state_;
  std::optional<chassis_can::MotorState> drive_state_;
  std::optional<chassis_can::MotorState> steering_state_;
};

}  // namespace

std::unique_ptr<ChassisAdapter> makeAckermannChassisAdapter(rclcpp::Node & node)
{
  return std::make_unique<AckermannChassisAdapter>(node);
}

}  // namespace agribot_hardware_bringup
