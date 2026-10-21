#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/bool.hpp"

namespace agribot_vehicle_description
{

// Retain the selected Nav2 goal checker, but do not finish a looping route
// on its first passage through the final pose.
class RouteProgressGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Route goal checker requires a live node");
    }
    const auto parameter = name + ".wrapped_plugin";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, "nav2_controller::SimpleGoalChecker");
    }
    const auto type = node->get_parameter(parameter).as_string();
    if (type == "agribot_vehicle_description::RouteProgressGoalChecker") {
      throw std::runtime_error("Route goal checker cannot wrap itself");
    }
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
    checker_ = loader_->createSharedInstance(type);
    checker_->initialize(parent, name, costmap);
    const auto topic_parameter = name + ".requested_goal_topic";
    const auto frame_parameter = name + ".requested_goal_frame";
    if (!node->has_parameter(topic_parameter)) {
      node->declare_parameter(topic_parameter, "");
    }
    if (!node->has_parameter(frame_parameter)) {
      node->declare_parameter(frame_parameter, costmap ? costmap->getGlobalFrameID() : "");
    }
    const auto topic = node->get_parameter(topic_parameter).as_string();
    requested_goal_frame_ = node->get_parameter(frame_parameter).as_string();
    use_requested_goal_ = !topic.empty();
    if (use_requested_goal_) {
      if (requested_goal_frame_.empty() ||
        (costmap && requested_goal_frame_ != costmap->getGlobalFrameID()))
      {
        throw std::runtime_error("Requested route goal must use the controller costmap frame");
      }
      requested_goal_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
          const auto & p = message->pose.position;
          const auto & q = message->pose.orientation;
          std::lock_guard<std::mutex> lock(checker_mutex_);
          have_requested_goal_ = message->header.frame_id == requested_goal_frame_ &&
            std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
            std::isfinite(q.w) &&
            std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) < 1e-3;
          if (have_requested_goal_) {
            requested_goal_ = message->pose;
          }
          checker_->reset();
        });
    }
    ready_.store(false);
    subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      "/waypoint_goal_ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {ready_.store(message->data);});
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(checker_mutex_);
    checker_->reset();
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & current,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    // Do not let a stateful inner checker latch XY success on an earlier lap.
    if (!ready_.load()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(checker_mutex_);
    if (use_requested_goal_ && !have_requested_goal_) {
      return false;
    }
    // Smac's last grid pose may differ from the original requested position.
    // Keep the original tolerance, but center it on that requested position.
    return checker_->isGoalReached(
      current, use_requested_goal_ ? requested_goal_ : goal, velocity);
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override
  {
    std::lock_guard<std::mutex> lock(checker_mutex_);
    return checker_->getTolerances(pose, velocity);
  }

private:
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> loader_;
  nav2_core::GoalChecker::Ptr checker_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr requested_goal_subscription_;
  std::mutex checker_mutex_;
  geometry_msgs::msg::Pose requested_goal_;
  std::string requested_goal_frame_;
  bool use_requested_goal_{false};
  bool have_requested_goal_{false};
  std::atomic<bool> ready_{false};
};

}  // namespace agribot_vehicle_description

PLUGINLIB_EXPORT_CLASS(
  agribot_vehicle_description::RouteProgressGoalChecker, nav2_core::GoalChecker)
