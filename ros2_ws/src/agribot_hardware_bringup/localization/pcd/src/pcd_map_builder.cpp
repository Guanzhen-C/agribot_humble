#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace agribot_hardware_bringup
{
namespace
{

Eigen::Isometry3d pose_to_isometry(
  const geometry_msgs::msg::Point & position,
  const geometry_msgs::msg::Quaternion & orientation)
{
  Eigen::Quaterniond quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("received a zero-length pose quaternion");
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.normalized().toRotationMatrix();
  transform.translation() =
    Eigen::Vector3d(position.x, position.y, position.z);
  return transform;
}

geometry_msgs::msg::Transform isometry_to_transform(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform output;
  const Eigen::Quaterniond quaternion(transform.rotation());
  output.translation.x = transform.translation().x();
  output.translation.y = transform.translation().y();
  output.translation.z = transform.translation().z();
  output.rotation.x = quaternion.x();
  output.rotation.y = quaternion.y();
  output.rotation.z = quaternion.z();
  output.rotation.w = quaternion.w();
  return output;
}

struct VoxelKey
{
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<int64_t>{}(key.x);
    seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
      (seed >> 2U);
    seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
      (seed >> 2U);
    return seed;
  }
};

bool key_less(const VoxelKey & left, const VoxelKey & right)
{
  if (left.x != right.x) {
    return left.x < right.x;
  }
  if (left.y != right.y) {
    return left.y < right.y;
  }
  return left.z < right.z;
}

struct VoxelAccumulator
{
  Eigen::Vector3d position_sum{Eigen::Vector3d::Zero()};
  double intensity_sum{0.0};
  std::size_t samples{0U};
  std::size_t observations{0U};
};

struct FrameAccumulator
{
  Eigen::Vector3d position_sum{Eigen::Vector3d::Zero()};
  double intensity_sum{0.0};
  std::size_t samples{0U};
};

struct OccupancyMap
{
  uint32_t width{0U};
  uint32_t height{0U};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> cells;
  std::vector<uint8_t> pixels;
};

std::filesystem::path normalized_base_path(const std::string & value)
{
  if (value.empty()) {
    throw std::runtime_error("map_base_path must not be empty");
  }

  std::filesystem::path path(value);
  const std::string extension = path.extension().string();
  if (extension == ".pcd" || extension == ".pgm" || extension == ".yaml") {
    path.replace_extension();
  }
  return path;
}

}  // namespace

class PcdMapBuilder : public rclcpp::Node
{
public:
  PcdMapBuilder()
  : Node("pcd_map_builder"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered");
    cloud_frame_ = declare_parameter<std::string>(
      "cloud_frame", "camera_init");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/fastlio/odometry");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    map_base_path_ = declare_parameter<std::string>(
      "map_base_path", "/tmp/agribot_map");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    min_observations_ = declare_parameter<int>("min_observations", 2);
    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    publish_period_ = declare_parameter<double>("publish_period", 2.0);
    occupancy_resolution_ = declare_parameter<double>(
      "occupancy_resolution", 0.05);
    occupancy_min_z_ = declare_parameter<double>("occupancy_min_z", 0.05);
    occupancy_max_z_ = declare_parameter<double>("occupancy_max_z", 1.80);
    occupancy_padding_ = declare_parameter<double>("occupancy_padding", 1.0);
    occupancy_dilation_radius_ = declare_parameter<double>(
      "occupancy_dilation_radius", 0.05);
    save_on_shutdown_ = declare_parameter<bool>("save_on_shutdown", false);

    if (voxel_size_ <= 0.0 || occupancy_resolution_ <= 0.0) {
      throw std::runtime_error("voxel and occupancy resolutions must be positive");
    }
    if (min_observations_ < 1) {
      throw std::runtime_error("min_observations must be at least one");
    }
    if (min_range_ < 0.0 || max_range_ <= min_range_) {
      throw std::runtime_error("invalid mapping range limits");
    }
    if (occupancy_max_z_ <= occupancy_min_z_) {
      throw std::runtime_error("occupancy_max_z must exceed occupancy_min_z");
    }

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_map", map_qos);
    occupancy_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&PcdMapBuilder::handle_odometry, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdMapBuilder::handle_cloud, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      std::bind(
        &PcdMapBuilder::handle_save, this, std::placeholders::_1,
        std::placeholders::_2));

    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&PcdMapBuilder::publish_transform, this));
    const auto publish_duration = std::chrono::duration<double>(
      std::max(0.2, publish_period_));
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_duration),
      std::bind(&PcdMapBuilder::publish_map_cloud, this));

    RCLCPP_INFO(
      get_logger(),
      "Building a voxelized 3D map from %s; save with /pcd_map_builder/save_map",
      cloud_topic_.c_str());
  }

  ~PcdMapBuilder() override
  {
    if (!save_on_shutdown_) {
      return;
    }
    try {
      save_map();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed saving map during shutdown: %s", error.what());
    }
  }

private:
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

  VoxelKey voxel_key(const Eigen::Vector3d & point) const
  {
    return VoxelKey{
      static_cast<int64_t>(std::floor(point.x() / voxel_size_)),
      static_cast<int64_t>(std::floor(point.y() / voxel_size_)),
      static_cast<int64_t>(std::floor(point.z() / voxel_size_))};
  }

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    Eigen::Isometry3d odom_to_base;
    try {
      odom_to_base = pose_to_isometry(
        message->pose.pose.position, message->pose.pose.orientation);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid odometry: %s", error.what());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_to_base_ = odom_to_base;
    have_odometry_ = true;
    if (!origin_initialized_) {
      // Make the first rear-axle pose the map origin.
      map_to_odom_ = odom_to_base.inverse();
      origin_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initialized map origin from the first %s -> base_link pose",
        odom_frame_.c_str());
    }
  }

  void handle_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!cloud_frame_.empty() && message->header.frame_id != cloud_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring %s cloud in frame '%s'; expected '%s'",
        cloud_topic_.c_str(), message->header.frame_id.c_str(),
        cloud_frame_.c_str());
      return;
    }

    Eigen::Isometry3d map_to_odom;
    Eigen::Vector3d sensor_origin;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!origin_initialized_ || !have_odometry_) {
        return;
      }
      map_to_odom = map_to_odom_;
      sensor_origin = (map_to_odom_ * latest_odom_to_base_).translation();
    }

    PointCloud input;
    pcl::fromROSMsg(*message, input);
    std::unordered_map<VoxelKey, FrameAccumulator, VoxelKeyHash> frame_voxels;
    frame_voxels.reserve(input.size());
    const double min_range_squared = min_range_ * min_range_;
    const double max_range_squared = max_range_ * max_range_;

    for (const pcl::PointXYZI & point : input.points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector3d odom_point(point.x, point.y, point.z);
      const Eigen::Vector3d map_point = map_to_odom * odom_point;
      const double range_squared = (map_point - sensor_origin).squaredNorm();
      if (range_squared < min_range_squared || range_squared > max_range_squared) {
        continue;
      }

      FrameAccumulator & accumulator = frame_voxels[voxel_key(map_point)];
      accumulator.position_sum += map_point;
      accumulator.intensity_sum += point.intensity;
      ++accumulator.samples;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [key, frame] : frame_voxels) {
      if (frame.samples == 0U) {
        continue;
      }
      VoxelAccumulator & global = voxels_[key];
      global.position_sum += frame.position_sum /
        static_cast<double>(frame.samples);
      global.intensity_sum += frame.intensity_sum /
        static_cast<double>(frame.samples);
      ++global.samples;
      ++global.observations;
    }
    ++cloud_count_;
  }

  PointCloud::Ptr snapshot_map() const
  {
    std::vector<std::pair<VoxelKey, VoxelAccumulator>> selected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      selected.reserve(voxels_.size());
      for (const auto & item : voxels_) {
        if (
          item.second.observations >=
          static_cast<std::size_t>(min_observations_))
        {
          selected.push_back(item);
        }
      }
    }

    std::sort(
      selected.begin(), selected.end(),
      [](const auto & left, const auto & right) {
        return key_less(left.first, right.first);
      });

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(selected.size());
    for (const auto & [key, accumulator] : selected) {
      (void)key;
      const double divisor = static_cast<double>(accumulator.samples);
      const Eigen::Vector3d position = accumulator.position_sum / divisor;
      pcl::PointXYZI point;
      point.x = static_cast<float>(position.x());
      point.y = static_cast<float>(position.y());
      point.z = static_cast<float>(position.z());
      point.intensity = static_cast<float>(accumulator.intensity_sum / divisor);
      cloud->push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1U;
    cloud->is_dense = true;
    return cloud;
  }

  OccupancyMap build_occupancy_map(const PointCloud & cloud) const
  {
    double minimum_x = std::numeric_limits<double>::infinity();
    double minimum_y = std::numeric_limits<double>::infinity();
    double maximum_x = -std::numeric_limits<double>::infinity();
    double maximum_y = -std::numeric_limits<double>::infinity();
    std::vector<const pcl::PointXYZI *> obstacles;
    obstacles.reserve(cloud.size());

    for (const pcl::PointXYZI & point : cloud.points) {
      if (point.z < occupancy_min_z_ || point.z > occupancy_max_z_) {
        continue;
      }
      obstacles.push_back(&point);
      minimum_x = std::min(minimum_x, static_cast<double>(point.x));
      minimum_y = std::min(minimum_y, static_cast<double>(point.y));
      maximum_x = std::max(maximum_x, static_cast<double>(point.x));
      maximum_y = std::max(maximum_y, static_cast<double>(point.y));
    }
    if (obstacles.empty()) {
      throw std::runtime_error(
              "no map points remain inside the configured occupancy height band");
    }

    OccupancyMap map;
    map.origin_x = std::floor(
      (minimum_x - occupancy_padding_) / occupancy_resolution_) *
      occupancy_resolution_;
    map.origin_y = std::floor(
      (minimum_y - occupancy_padding_) / occupancy_resolution_) *
      occupancy_resolution_;
    const double extent_x = maximum_x + occupancy_padding_ - map.origin_x;
    const double extent_y = maximum_y + occupancy_padding_ - map.origin_y;
    map.width = static_cast<uint32_t>(
      std::ceil(extent_x / occupancy_resolution_) + 1.0);
    map.height = static_cast<uint32_t>(
      std::ceil(extent_y / occupancy_resolution_) + 1.0);
    const uint64_t cell_count =
      static_cast<uint64_t>(map.width) * static_cast<uint64_t>(map.height);
    if (cell_count == 0U || cell_count > 100000000U) {
      throw std::runtime_error("generated occupancy map has an invalid size");
    }

    map.cells.assign(static_cast<std::size_t>(cell_count), 0);
    map.pixels.assign(static_cast<std::size_t>(cell_count), 254U);
    const int dilation_cells = static_cast<int>(
      std::ceil(occupancy_dilation_radius_ / occupancy_resolution_));

    for (const pcl::PointXYZI * point : obstacles) {
      const int center_x = static_cast<int>(
        std::floor((point->x - map.origin_x) / occupancy_resolution_));
      const int center_y = static_cast<int>(
        std::floor((point->y - map.origin_y) / occupancy_resolution_));
      for (int offset_y = -dilation_cells; offset_y <= dilation_cells; ++offset_y) {
        for (int offset_x = -dilation_cells; offset_x <= dilation_cells; ++offset_x) {
          if (
            offset_x * offset_x + offset_y * offset_y >
            dilation_cells * dilation_cells)
          {
            continue;
          }
          const int x = center_x + offset_x;
          const int y = center_y + offset_y;
          if (
            x < 0 || y < 0 || x >= static_cast<int>(map.width) ||
            y >= static_cast<int>(map.height))
          {
            continue;
          }
          const std::size_t index =
            static_cast<std::size_t>(y) * map.width +
            static_cast<std::size_t>(x);
          map.cells[index] = 100;
          map.pixels[index] = 0U;
        }
      }
    }
    return map;
  }

  void write_occupancy_files(
    const OccupancyMap & map, const std::filesystem::path & base) const
  {
    const std::filesystem::path pgm_path = base.string() + ".pgm";
    const std::filesystem::path yaml_path = base.string() + ".yaml";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      throw std::runtime_error("failed opening " + pgm_path.string());
    }
    pgm << "P5\n" << map.width << " " << map.height << "\n255\n";
    for (int row = static_cast<int>(map.height) - 1; row >= 0; --row) {
      const std::size_t offset = static_cast<std::size_t>(row) * map.width;
      pgm.write(
        reinterpret_cast<const char *>(map.pixels.data() + offset),
        static_cast<std::streamsize>(map.width));
    }
    if (!pgm) {
      throw std::runtime_error("failed writing " + pgm_path.string());
    }

    std::ofstream yaml(yaml_path);
    if (!yaml) {
      throw std::runtime_error("failed opening " + yaml_path.string());
    }
    yaml << "image: " << pgm_path.filename().string() << "\n"
         << "mode: trinary\n"
         << "resolution: " << occupancy_resolution_ << "\n"
         << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
    if (!yaml) {
      throw std::runtime_error("failed writing " + yaml_path.string());
    }
  }

  void publish_occupancy_map(const OccupancyMap & map)
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.info.resolution = static_cast<float>(occupancy_resolution_);
    message.info.width = map.width;
    message.info.height = map.height;
    message.info.origin.position.x = map.origin_x;
    message.info.origin.position.y = map.origin_y;
    message.info.origin.orientation.w = 1.0;
    message.data = map.cells;
    occupancy_publisher_->publish(message);
  }

  std::string save_map()
  {
    const PointCloud::Ptr cloud = snapshot_map();
    if (cloud->empty()) {
      throw std::runtime_error(
              "the map is empty; move the vehicle before saving");
    }

    const std::filesystem::path base = normalized_base_path(map_base_path_);
    if (base.has_parent_path()) {
      std::filesystem::create_directories(base.parent_path());
    }
    const std::filesystem::path pcd_path = base.string() + ".pcd";
    if (pcl::io::savePCDFileBinary(pcd_path.string(), *cloud) != 0) {
      throw std::runtime_error("failed writing " + pcd_path.string());
    }

    const OccupancyMap occupancy = build_occupancy_map(*cloud);
    write_occupancy_files(occupancy, base);
    publish_occupancy_map(occupancy);
    return "Saved " + std::to_string(cloud->size()) + " voxels to " +
           pcd_path.string() + " and aligned Nav2 map files";
  }

  void handle_save(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    (void)request;
    try {
      response->message = save_map();
      response->success = true;
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
    }
  }

  void publish_map_cloud()
  {
    const PointCloud::Ptr cloud = snapshot_map();
    if (cloud->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    map_publisher_->publish(message);
  }

  void publish_transform()
  {
    Eigen::Isometry3d map_to_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!origin_initialized_) {
        return;
      }
      map_to_odom = map_to_odom_;
    }

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = isometry_to_transform(map_to_odom);
    tf_broadcaster_->sendTransform(message);
  }

  std::string cloud_topic_;
  std::string cloud_frame_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string map_base_path_;
  double voxel_size_{0.10};
  int min_observations_{2};
  double min_range_{0.5};
  double max_range_{30.0};
  double publish_period_{2.0};
  double occupancy_resolution_{0.05};
  double occupancy_min_z_{0.05};
  double occupancy_max_z_{1.80};
  double occupancy_padding_{1.0};
  double occupancy_dilation_radius_{0.05};
  bool save_on_shutdown_{false};

  mutable std::mutex mutex_;
  bool have_odometry_{false};
  bool origin_initialized_{false};
  Eigen::Isometry3d latest_odom_to_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
  std::size_t cloud_count_{0U};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<agribot_hardware_bringup::PcdMapBuilder>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pcd_map_builder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
