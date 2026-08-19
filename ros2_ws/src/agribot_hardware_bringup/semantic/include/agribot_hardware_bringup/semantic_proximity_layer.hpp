#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agribot_hardware_bringup
{

class SemanticProximityLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
    int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  struct Bounds
  {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
  };

  void sourceCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  static std::optional<Bounds> occupiedBounds(
    const nav_msgs::msg::OccupancyGrid & grid);
  static void expandBounds(const Bounds & bounds, double * min_x, double * min_y,
    double * max_x, double * max_y);
  static void mergeBounds(
    std::optional<Bounds> * target, const Bounds & value);
  static bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y,
    unsigned int * column, unsigned int * row);

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr source_grid_;
  std::optional<Bounds> current_bounds_;
  std::optional<Bounds> stale_bounds_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr source_subscription_;
  std::string source_topic_;
  unsigned char maximum_cost_{200};
};

}  // namespace agribot_hardware_bringup
