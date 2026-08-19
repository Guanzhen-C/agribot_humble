#include "agribot_hardware_bringup/semantic_proximity_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agribot_hardware_bringup
{
namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

}  // namespace

void SemanticProximityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("semantic proximity layer has no lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "source_topic",
    rclcpp::ParameterValue("/semantic_navigation/proximity_costmap"));
  declareParameter("maximum_cost", rclcpp::ParameterValue(200));
  declareParameter("obstacle_costmap_topic", rclcpp::ParameterValue(""));
  declareParameter("obstacle_costmap_publish_frequency", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".source_topic", source_topic_);
  int configured_maximum_cost = 200;
  node->get_parameter(name_ + ".maximum_cost", configured_maximum_cost);
  node->get_parameter(name_ + ".obstacle_costmap_topic", obstacle_costmap_topic_);
  node->get_parameter(
    name_ + ".obstacle_costmap_publish_frequency", obstacle_costmap_publish_frequency_);
  configured_maximum_cost = std::clamp(configured_maximum_cost, 1, 252);
  maximum_cost_ = static_cast<unsigned char>(configured_maximum_cost);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  source_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic_, qos,
    std::bind(&SemanticProximityLayer::sourceCallback, this, std::placeholders::_1));
  if (!obstacle_costmap_topic_.empty()) {
    obstacle_costmap_publish_frequency_ = std::max(0.01, obstacle_costmap_publish_frequency_);
    obstacle_costmap_publisher_ = rclcpp::create_publisher<nav_msgs::msg::OccupancyGrid>(
      *node, obstacle_costmap_topic_, qos);
  }
  current_ = true;
  RCLCPP_INFO(
    logger_, "Semantic route layer subscribed to '%s' with additive maximum cost %u",
    source_topic_.c_str(), static_cast<unsigned int>(maximum_cost_));
}

void SemanticProximityLayer::activate()
{
  if (obstacle_costmap_publisher_) {
    const auto * costmap = layered_costmap_->getCostmap();
    const Bounds full_bounds{
      costmap->getOriginX(), costmap->getOriginY(),
      costmap->getOriginX() + costmap->getSizeInMetersX(),
      costmap->getOriginY() + costmap->getSizeInMetersY()};
    std::lock_guard<std::mutex> lock(mutex_);
    mergeBounds(&stale_bounds_, full_bounds);
  }
}

void SemanticProximityLayer::sourceCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  const auto bounds = occupiedBounds(*message);
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_bounds_.has_value()) {
    mergeBounds(&stale_bounds_, *current_bounds_);
  }
  source_grid_ = message;
  current_bounds_ = bounds;
  if (current_bounds_.has_value()) {
    mergeBounds(&stale_bounds_, *current_bounds_);
  }
  current_ = true;
}

void SemanticProximityLayer::mergeBounds(
  std::optional<Bounds> * target, const Bounds & value)
{
  if (!target->has_value()) {
    *target = value;
    return;
  }
  target->value().min_x = std::min(target->value().min_x, value.min_x);
  target->value().min_y = std::min(target->value().min_y, value.min_y);
  target->value().max_x = std::max(target->value().max_x, value.max_x);
  target->value().max_y = std::max(target->value().max_y, value.max_y);
}

std::optional<SemanticProximityLayer::Bounds> SemanticProximityLayer::occupiedBounds(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  const auto width = grid.info.width;
  const auto height = grid.info.height;
  if (width == 0U || height == 0U || grid.info.resolution <= 0.0F ||
    grid.data.size() != static_cast<std::size_t>(width) * height)
  {
    return std::nullopt;
  }

  unsigned int min_column = width;
  unsigned int min_row = height;
  unsigned int max_column = 0U;
  unsigned int max_row = 0U;
  bool found = false;
  for (unsigned int row = 0U; row < height; ++row) {
    for (unsigned int column = 0U; column < width; ++column) {
      if (grid.data[static_cast<std::size_t>(row) * width + column] <= 0) {
        continue;
      }
      found = true;
      min_column = std::min(min_column, column);
      min_row = std::min(min_row, row);
      max_column = std::max(max_column, column);
      max_row = std::max(max_row, row);
    }
  }
  if (!found) {
    return std::nullopt;
  }

  const double yaw = yawFromQuaternion(grid.info.origin.orientation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Bounds result{
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  const double resolution = grid.info.resolution;
  for (const auto column : {min_column, max_column + 1U}) {
    for (const auto row : {min_row, max_row + 1U}) {
      const double local_x = static_cast<double>(column) * resolution;
      const double local_y = static_cast<double>(row) * resolution;
      const double world_x = grid.info.origin.position.x + cosine * local_x - sine * local_y;
      const double world_y = grid.info.origin.position.y + sine * local_x + cosine * local_y;
      result.min_x = std::min(result.min_x, world_x);
      result.min_y = std::min(result.min_y, world_y);
      result.max_x = std::max(result.max_x, world_x);
      result.max_y = std::max(result.max_y, world_y);
    }
  }
  return result;
}

void SemanticProximityLayer::expandBounds(
  const Bounds & bounds, double * min_x, double * min_y, double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, bounds.min_x);
  *min_y = std::min(*min_y, bounds.min_y);
  *max_x = std::max(*max_x, bounds.max_x);
  *max_y = std::max(*max_y, bounds.max_y);
}

void SemanticProximityLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stale_bounds_.has_value()) {
    expandBounds(*stale_bounds_, min_x, min_y, max_x, max_y);
    stale_bounds_.reset();
  }
}

bool SemanticProximityLayer::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y,
  unsigned int * column, unsigned int * row)
{
  const double yaw = yawFromQuaternion(grid.info.origin.orientation);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double dx = world_x - grid.info.origin.position.x;
  const double dy = world_y - grid.info.origin.position.y;
  const auto local_column = static_cast<long>(std::floor(
      (cosine * dx + sine * dy) / grid.info.resolution));
  const auto local_row = static_cast<long>(std::floor(
      (-sine * dx + cosine * dy) / grid.info.resolution));
  if (local_column < 0L || local_row < 0L ||
    local_column >= static_cast<long>(grid.info.width) ||
    local_row >= static_cast<long>(grid.info.height))
  {
    return false;
  }
  *column = static_cast<unsigned int>(local_column);
  *row = static_cast<unsigned int>(local_row);
  return true;
}

void SemanticProximityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // Capture the static/dynamic obstacle and inflation layers before semantic
  // route preference is added. RViz can display this without exposing the
  // internal A* corridor cost used by Smac.
  publishObstacleCostmap(master_grid, min_i, min_j, max_i, max_j);

  nav_msgs::msg::OccupancyGrid::SharedPtr source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    source = source_grid_;
  }
  if (!source || source->data.empty()) {
    return;
  }

  const auto width = master_grid.getSizeInCellsX();
  const auto height = master_grid.getSizeInCellsY();
  const int first_column = std::max(0, min_i);
  const int first_row = std::max(0, min_j);
  const int last_column = std::min(static_cast<int>(width), max_i);
  const int last_row = std::min(static_cast<int>(height), max_j);
  for (int row = first_row; row < last_row; ++row) {
    for (int column = first_column; column < last_column; ++column) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(column), static_cast<unsigned int>(row), world_x, world_y);
      unsigned int source_column = 0U;
      unsigned int source_row = 0U;
      if (!worldToGrid(*source, world_x, world_y, &source_column, &source_row)) {
        continue;
      }
      const int normalized = source->data[
        static_cast<std::size_t>(source_row) * source->info.width + source_column];
      if (normalized <= 0) {
        continue;
      }
      const auto semantic_cost = static_cast<unsigned char>(std::clamp(
        static_cast<int>(std::lround(
          static_cast<double>(maximum_cost_) * std::min(normalized, 100) / 100.0)),
        1, static_cast<int>(maximum_cost_)));
      const auto existing = master_grid.getCost(
        static_cast<unsigned int>(column), static_cast<unsigned int>(row));
      if (existing == nav2_costmap_2d::NO_INFORMATION ||
        existing >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }
      const auto combined = static_cast<unsigned char>(std::min(
        static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1,
        static_cast<int>(existing) + static_cast<int>(semantic_cost)));
      master_grid.setCost(
        static_cast<unsigned int>(column), static_cast<unsigned int>(row), combined);
    }
  }
}

void SemanticProximityLayer::publishObstacleCostmap(
  const nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!obstacle_costmap_publisher_) {
    return;
  }

  const auto width = master_grid.getSizeInCellsX();
  const auto height = master_grid.getSizeInCellsY();
  const auto cell_count = static_cast<std::size_t>(width) * height;
  const bool geometry_changed =
    obstacle_costmap_.info.width != width ||
    obstacle_costmap_.info.height != height ||
    obstacle_costmap_.info.resolution != master_grid.getResolution() ||
    obstacle_costmap_.info.origin.position.x != master_grid.getOriginX() ||
    obstacle_costmap_.info.origin.position.y != master_grid.getOriginY();
  if (geometry_changed) {
    obstacle_costmap_.info.resolution = master_grid.getResolution();
    obstacle_costmap_.info.width = width;
    obstacle_costmap_.info.height = height;
    obstacle_costmap_.info.origin.position.x = master_grid.getOriginX();
    obstacle_costmap_.info.origin.position.y = master_grid.getOriginY();
    obstacle_costmap_.info.origin.orientation.w = 1.0;
    obstacle_costmap_.data.assign(cell_count, -1);
    min_i = 0;
    min_j = 0;
    max_i = static_cast<int>(width);
    max_j = static_cast<int>(height);
  }

  const int first_column = std::max(0, min_i);
  const int first_row = std::max(0, min_j);
  const int last_column = std::min(static_cast<int>(width), max_i);
  const int last_row = std::min(static_cast<int>(height), max_j);
  const auto * costs = master_grid.getCharMap();
  for (int row = first_row; row < last_row; ++row) {
    for (int column = first_column; column < last_column; ++column) {
      const auto index = static_cast<std::size_t>(row) * width + column;
      const auto cost = costs[index];
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        obstacle_costmap_.data[index] = -1;
      } else {
        obstacle_costmap_.data[index] = static_cast<int8_t>(std::lround(
          100.0 * static_cast<double>(cost) /
          static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)));
      }
    }
  }

  const auto now = clock_->now();
  const auto period = rclcpp::Duration::from_seconds(
    1.0 / obstacle_costmap_publish_frequency_);
  if (last_obstacle_costmap_publish_time_.nanoseconds() != 0 &&
    now - last_obstacle_costmap_publish_time_ < period)
  {
    return;
  }

  obstacle_costmap_.header.stamp = now;
  obstacle_costmap_.header.frame_id = layered_costmap_->getGlobalFrameID();
  obstacle_costmap_.info.map_load_time = now;
  obstacle_costmap_publisher_->publish(obstacle_costmap_);
  last_obstacle_costmap_publish_time_ = now;
}

void SemanticProximityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = true;
  if (current_bounds_.has_value()) {
    mergeBounds(&stale_bounds_, *current_bounds_);
  }
}

}  // namespace agribot_hardware_bringup

PLUGINLIB_EXPORT_CLASS(
  agribot_hardware_bringup::SemanticProximityLayer, nav2_costmap_2d::Layer)
