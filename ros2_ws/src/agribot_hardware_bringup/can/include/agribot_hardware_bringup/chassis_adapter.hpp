#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_ADAPTER_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_ADAPTER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scout_msgs/msg/scout_status.hpp"

#include "agribot_hardware_bringup/chassis_can_common.hpp"

namespace agribot_hardware_bringup
{

struct MeasuredMotion
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

struct FrameUpdate
{
  bool valid{false};
  std::optional<bool> emergency_stop;
  std::optional<MeasuredMotion> motion;
};

class ChassisAdapter
{
public:
  virtual ~ChassisAdapter() = default;

  virtual std::string type() const = 0;
  virtual uint32_t commandId() const = 0;
  virtual std::vector<uint32_t> feedbackIds() const = 0;
  virtual chassis_can::Frame commandFromTwist(
    const geometry_msgs::msg::Twist & command,
    bool brake,
    bool headlight,
    uint8_t rolling_counter) const = 0;
  virtual FrameUpdate processFrame(
    const chassis_can::Frame & frame,
    const rclcpp::Time & stamp) = 0;
  virtual bool feedbackFresh(
    const rclcpp::Time & current_time,
    double timeout_sec) const = 0;
  virtual bool feedbackAllowsMotion(bool require_autonomous_mode) const = 0;
  virtual void populateStatus(scout_msgs::msg::ScoutStatus & status) const = 0;
};

using ChassisAdapterFactory = std::unique_ptr<ChassisAdapter>(*)(rclcpp::Node & node);

std::unique_ptr<ChassisAdapter> makeDifferentialChassisAdapter(rclcpp::Node & node);
std::unique_ptr<ChassisAdapter> makeAckermannChassisAdapter(rclcpp::Node & node);

}  // namespace agribot_hardware_bringup

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_ADAPTER_HPP_
