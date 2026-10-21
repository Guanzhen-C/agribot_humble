#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_default_plugins/displays/path/path_display.hpp>
#include "navigation_task_state.hpp"

namespace scout_viz
{
class NavigationPathDisplay : public rviz_default_plugins::displays::PathDisplay
{
public:
  void processMessage(nav_msgs::msg::Path::ConstSharedPtr message) override
  {
    consumeStatus();
    if (!task_.hidden()) {PathDisplay::processMessage(message);}
  }

  void update(float wall_dt, float ros_dt) override
  {
    PathDisplay::update(wall_dt, ros_dt);
    consumeStatus();
  }

  void reset() override
  {
    PathDisplay::reset();
    task_ = NavigationTaskState();
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    inbox_->pending.clear();
  }

protected:
  void onInitialize() override
  {
    PathDisplay::onInitialize();
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    for (const std::string source :
      {"navigate_to_pose", "navigate_through_poses", "follow_waypoints", "follow_path"})
    {
      // Callbacks own only the inbox, not the display or its rendering objects.
      subscriptions_.push_back(node->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/" + source + "/_action/status", qos,
        [inbox = inbox_, source](action_msgs::msg::GoalStatusArray::ConstSharedPtr status) {
          std::lock_guard<std::mutex> lock(inbox->mutex);
          inbox->pending[source] = status;
        }));
    }
  }

private:
  struct Inbox
  {
    std::mutex mutex;
    std::map<std::string, action_msgs::msg::GoalStatusArray::ConstSharedPtr> pending;
  };
  void consumeStatus()
  {
    std::map<std::string, action_msgs::msg::GoalStatusArray::ConstSharedPtr> pending;
    {
      std::lock_guard<std::mutex> lock(inbox_->mutex);
      pending.swap(inbox_->pending);
    }
    bool clear = false;
    for (const std::string source :
      {"follow_waypoints", "navigate_to_pose", "navigate_through_poses", "follow_path"})
    {
      const auto entry = pending.find(source);
      if (entry != pending.end()) {clear = task_.update(source, *entry->second) || clear;}
    }
    if (clear && task_.hidden()) {
      PathDisplay::reset();
      context_->queueRender();
    }
  }
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> subscriptions_;
  NavigationTaskState task_;
};
}  // namespace scout_viz

PLUGINLIB_EXPORT_CLASS(scout_viz::NavigationPathDisplay, rviz_common::Display)
