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
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".source_topic", source_topic_);
  int configured_maximum_cost = 200;
  node->get_parameter(name_ + ".maximum_cost", configured_maximum_cost);
  configured_maximum_cost = std::clamp(configured_maximum_cost, 1, 252);
  maximum_cost_ = static_cast<unsigned char>(configured_maximum_cost);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  source_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    source_topic_, qos,
    std::bind(&SemanticProximityLayer::sourceCallback, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    logger_, "Semantic proximity layer subscribed to '%s' with maximum cost %u",
    source_topic_.c_str(), static_cast<unsigned int>(maximum_cost_));
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
  if (current_bounds_.has_value()) {
    expandBounds(*current_bounds_, min_x, min_y, max_x, max_y);
  }
  if (stale_bounds_.has_value()) {
    expandBounds(*stale_bounds_, min_x, min_y, max_x, max_y);
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

  nav_msgs::msg::OccupancyGrid::SharedPtr source;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    source = source_grid_;
    stale_bounds_.reset();
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
      if (semantic_cost > existing) {
        master_grid.setCost(
          static_cast<unsigned int>(column), static_cast<unsigned int>(row), semantic_cost);
      }
    }
  }
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
