#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agribot_vehicle_description
{
// Reuse DWB's velocity limits, integration and critics with bicycle-feasible commands.
class AckermannTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr & node,
    const std::string & name) override
  {
    StandardTrajectoryGenerator::initialize(node, name);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".min_turning_radius", rclcpp::ParameterValue(1.3));
    node->get_parameter(name + ".min_turning_radius", radius_);
    if (!std::isfinite(radius_) || radius_ <= 0.0) {
      throw std::invalid_argument("DWB min_turning_radius must be positive");
    }
  }

  void startNewIteration(const nav_2d_msgs::msg::Twist2D & velocity) override
  {
    StandardTrajectoryGenerator::startNewIteration(velocity);
    samples_.clear();
    index_ = 0;
    auto limits = kinematics_handler_->getKinematics();
    const double max_yaw_rate = limits.getMaxTheta();
    while (StandardTrajectoryGenerator::hasMoreTwists()) {
      auto sample = StandardTrajectoryGenerator::nextTwist();
      // The drive cannot rotate when stopped. Sample curvature at every speed,
      // rather than retaining high yaw-rate samples that the chassis will clip.
      if (std::abs(sample.x) < 0.02) {continue;}
      sample.y = 0.0;
      sample.theta = max_yaw_rate > 0.0 ?
        sample.theta / max_yaw_rate * std::abs(sample.x) / radius_ : 0.0;
      sample.theta = std::clamp(sample.theta, -max_yaw_rate, max_yaw_rate);
      samples_.push_back(sample);
    }
  }

  bool hasMoreTwists() override {return index_ < samples_.size();}

  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    return samples_.at(index_++);
  }

protected:
  nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & command,
    const nav_2d_msgs::msg::Twist2D & velocity, const double dt) override
  {
    auto result = StandardTrajectoryGenerator::computeNewVelocity(command, velocity, dt);
    result.y = 0.0;
    const double bound = std::abs(result.x) / radius_;
    result.theta = std::clamp(result.theta, -bound, bound);
    if (std::abs(result.x) < 0.02) {result.theta = 0.0;}
    return result;
  }

private:
  double radius_{1.3};
  std::vector<nav_2d_msgs::msg::Twist2D> samples_;
  std::size_t index_{0};
};
}  // namespace agribot_vehicle_description

PLUGINLIB_EXPORT_CLASS(
  agribot_vehicle_description::AckermannTrajectoryGenerator, dwb_core::TrajectoryGenerator)
