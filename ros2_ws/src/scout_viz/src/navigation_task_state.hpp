#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include "action_msgs/msg/goal_status_array.hpp"

namespace scout_viz
{
class NavigationTaskState
{
public:
  bool hidden() const {return hidden_;}

  bool update(const std::string & source, const action_msgs::msg::GoalStatusArray & message)
  {
    if (message.status_list.empty()) {return false;}
    const auto newest = std::max_element(message.status_list.begin(), message.status_list.end(),
      [](const auto & a, const auto & b) {return stamp(a) < stamp(b);});
    Record next{stamp(*newest), newest->goal_info.goal_id.uuid, newest->status};
    const auto previous = records_.find(source);
    if (previous != records_.end() && next.time < previous->second.time) {return false;}
    const bool changed = previous == records_.end() || !sameGoal(previous->second, next);
    const bool changed_status = changed || previous->second.status != next.status;
    if (!changed && terminal(previous->second.status) && active(next.status)) {return false;}
    records_[source] = next;

    // A controller child cannot start a new navigator mission. FollowPath-only
    // runs use their own status until a higher-level navigator is observed.
    if (source == "follow_path" && !root_source_.empty() && root_source_ != source) {
      return false;
    }

    // Child actions finishing an intermediate waypoint do not finish its parent task.
    if ((source != "follow_waypoints" && activeSource("follow_waypoints")) ||
      (source == "follow_path" &&
      (activeSource("navigate_to_pose") || activeSource("navigate_through_poses"))))
    {
      return false;
    }
    if (changed && active(next.status)) {
      const bool parent = !root_source_.empty() && rank(source) < rank(root_source_) && !hidden_;
      if ((next.time >= root_.time || parent) && next.time > closed_at_) {
        root_ = next;
        root_source_ = source;
        hidden_ = false;
      }
    } else if (terminal(next.status) && changed_status &&
      ((source == root_source_ && sameGoal(root_, next)) || next.time >= root_.time))
    {
      root_ = next;
      root_source_ = source;
      closed_at_ = std::max(closed_at_, next.time);
      hidden_ = true;
      return true;
    }
    return false;
  }

private:
  struct Record
  {
    int64_t time = -1;
    std::array<uint8_t, 16> id{};
    int8_t status = 0;
  };
  static int64_t stamp(const action_msgs::msg::GoalStatus & value)
  {
    return static_cast<int64_t>(value.goal_info.stamp.sec) * 1000000000LL +
           value.goal_info.stamp.nanosec;
  }
  static bool sameGoal(const Record & a, const Record & b) {return a.time == b.time && a.id == b.id;}
  static bool active(int8_t status) {return status >= 1 && status <= 3;}
  static bool terminal(int8_t status) {return status >= 4 && status <= 6;}
  static int rank(const std::string & source)
  {
    return source == "follow_waypoints" ? 0 : source == "follow_path" ? 2 : 1;
  }
  bool activeSource(const std::string & source) const
  {
    const auto it = records_.find(source);
    return it != records_.end() && active(it->second.status);
  }
  std::map<std::string, Record> records_;
  Record root_;
  std::string root_source_;
  int64_t closed_at_ = -1;
  bool hidden_ = false;
};
}  // namespace scout_viz
