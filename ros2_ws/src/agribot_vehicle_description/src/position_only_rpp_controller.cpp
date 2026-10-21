#include "position_only_rpp_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agribot_vehicle_description
{
geometry_msgs::msg::TwistStamped
PositionOnlyRegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  if (goal_checker == nullptr) {
    throw nav2_core::PlannerException("Position-only RPP requires the original goal checker");
  }
  detail::PositionOnlyGoalCheckerView view(*goal_checker);
  return RegulatedPurePursuitController::computeVelocityCommands(pose, speed, &view);
}
}  // namespace agribot_vehicle_description

PLUGINLIB_EXPORT_CLASS(
  agribot_vehicle_description::PositionOnlyRegulatedPurePursuitController,
  nav2_core::Controller)
