#include "agribot_hardware_bringup/chassis_adapter.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
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
      node.declare_parameter<double>("max_steering_angle_rad", 0.30);
    config_.max_linear_velocity =
      node.declare_parameter<double>("max_linear_velocity", 0.80);
    config_.max_angular_velocity =
      node.declare_parameter<double>("max_angular_velocity", 0.65);
    config_.minimum_motion_speed =
      node.declare_parameter<double>("minimum_motion_speed", 0.02);
    command_id_ = declareId(node, "command_id", ackermann_can::kCommandId);
    feedback_ids_[0] = declareId(
      node, "feedback_part1_id", ackermann_can::kFeedbackPart1Id);
    feedback_ids_[1] = declareId(
      node, "feedback_part2_id", ackermann_can::kFeedbackPart2Id);
    feedback_ids_[2] = declareId(
      node, "feedback_part3_id", ackermann_can::kFeedbackPart3Id);
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
    return {feedback_ids_.begin(), feedback_ids_.end()};
  }

  bool usesPerFrameIntegrity() const override
  {
    return false;
  }

  chassis_can::Frame commandFromTwist(
    const geometry_msgs::msg::Twist & message,
    bool brake,
    bool headlight,
    uint8_t rolling_counter) const override
  {
    (void)headlight;
    (void)rolling_counter;
    const auto command = ackermann_can::fromTwist(
      message.linear.x, message.angular.z, config_, brake);
    return ackermann_can::encodeCommand(command, command_id_);
  }

  FrameUpdate processFrame(
    const chassis_can::Frame & frame,
    const rclcpp::Time & stamp) override
  {
    FrameUpdate update;
    const auto found = std::find(feedback_ids_.begin(), feedback_ids_.end(), frame.id);
    if (found == feedback_ids_.end()) {
      return update;
    }

    const auto index = static_cast<std::size_t>(found - feedback_ids_.begin());
    if (index == 0U) {
      cycle_started_ = true;
      second_part_received_ = false;
    } else if (index == 1U) {
      if (!cycle_started_) {
        return update;
      }
      second_part_received_ = true;
    } else if (!cycle_started_ || !second_part_received_) {
      return update;
    }

    feedback_parts_[index] = frame.data;
    feedback_times_[index] = stamp;
    feedback_received_[index] = true;
    update.valid = true;

    if (index != 2U) {
      return update;
    }

    cycle_started_ = false;
    second_part_received_ = false;
    ackermann_can::TelemetryPayload payload{};
    for (std::size_t part = 0; part < feedback_parts_.size(); ++part) {
      std::copy(
        feedback_parts_[part].begin(), feedback_parts_[part].end(),
        payload.begin() + static_cast<std::ptrdiff_t>(part * chassis_can::kPayloadSize));
    }

    const auto telemetry = ackermann_can::decodeTelemetry(payload);
    if (!telemetry.has_value()) {
      update.valid = false;
      update.checksum_error = true;
      return update;
    }

    telemetry_ = telemetry;
    telemetry_time_ = stamp;
    telemetry_received_ = true;
    MeasuredMotion motion;
    motion.linear_velocity = telemetry_->linear_velocity_x;
    motion.angular_velocity = telemetry_->angular_velocity_z;
    update.motion = motion;
    return update;
  }

  bool feedbackFresh(
    const rclcpp::Time & current_time,
    double timeout_sec) const override
  {
    if (!isFresh(telemetry_received_, telemetry_time_, current_time, timeout_sec)) {
      return false;
    }
    for (std::size_t index = 0; index < feedback_received_.size(); ++index) {
      if (!isFresh(
          feedback_received_[index], feedback_times_[index], current_time, timeout_sec))
      {
        return false;
      }
    }
    return true;
  }

  bool feedbackAllowsMotion(bool require_autonomous_mode) const override
  {
    return telemetry_.has_value() && !require_autonomous_mode;
  }

  void populateStatus(scout_msgs::msg::ScoutStatus & status) const override
  {
    if (!telemetry_.has_value()) {
      return;
    }
    status.battery_voltage = telemetry_->battery_voltage;
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
  std::array<uint32_t, 3> feedback_ids_{
    ackermann_can::kFeedbackPart1Id,
    ackermann_can::kFeedbackPart2Id,
    ackermann_can::kFeedbackPart3Id};
  std::array<chassis_can::Payload, 3> feedback_parts_{};
  std::array<rclcpp::Time, 3> feedback_times_{
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME)};
  std::array<bool, 3> feedback_received_{false, false, false};
  rclcpp::Time telemetry_time_{0, 0, RCL_ROS_TIME};
  bool cycle_started_{false};
  bool second_part_received_{false};
  bool telemetry_received_{false};
  std::optional<ackermann_can::Telemetry> telemetry_;
};

}  // namespace

std::unique_ptr<ChassisAdapter> makeAckermannChassisAdapter(rclcpp::Node & node)
{
  return std::make_unique<AckermannChassisAdapter>(node);
}

}  // namespace agribot_hardware_bringup
