#ifndef AGRIBOT_VEHICLE_DESCRIPTION__POSITION_ONLY_RPP_CONTROLLER_HPP_
#define AGRIBOT_VEHICLE_DESCRIPTION__POSITION_ONLY_RPP_CONTROLLER_HPP_

#include <stdexcept>
#include "nav2_core/goal_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

namespace agribot_vehicle_description
{
namespace detail
{
class PositionOnlyGoalCheckerView : public nav2_core::GoalChecker
{
public:
  explicit PositionOnlyGoalCheckerView(nav2_core::GoalChecker & checker) : checker_(checker) {}

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    throw std::logic_error("An internal RPP view cannot initialize the owned goal checker");
  }

  void reset() override {checker_.reset();}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & current, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    return checker_.isGoalReached(current, goal, velocity);
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    if (!checker_.getTolerances(pose, velocity)) {return false;}
    // Humble RPP uses this radius only to enter final-yaw rotation. Its caller
    // still checks the real half-wheelbase tolerance on the original checker.
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    return true;
  }

private:
  nav2_core::GoalChecker & checker_;
};
}  // namespace detail

class PositionOnlyRegulatedPurePursuitController
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;
};
}  // namespace agribot_vehicle_description

#endif
