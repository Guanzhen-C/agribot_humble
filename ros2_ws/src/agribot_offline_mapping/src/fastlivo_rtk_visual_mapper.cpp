#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

struct TimedPose
{
  rclcpp::Time stamp;
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
};

struct VoxelKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) +
      (seed >> 2U);
    seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) +
      (seed >> 2U);
    return seed;
  }
};

struct VoxelAccumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double r{0.0};
  double g{0.0};
  double b{0.0};
  std::uint64_t count{0};
};

Eigen::Isometry3d odometryPose(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & orientation = message.pose.pose.orientation;
  Eigen::Quaterniond quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return transform;
}

bool hasColorField(const sensor_msgs::msg::PointCloud2 & message)
{
  return std::any_of(
    message.fields.begin(), message.fields.end(),
    [](const sensor_msgs::msg::PointField & field) {
      return field.name == "rgb" || field.name == "rgba";
    });
}

}  // namespace

class FastlivoRtkVisualMapper : public rclcpp::Node
{
public:
  FastlivoRtkVisualMapper()
  : Node("fastlivo_rtk_visual_mapper")
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered");
    local_odom_topic_ = declare_parameter<std::string>(
      "local_odom_topic", "/fastlivo/odometry");
    fused_odom_topic_ = declare_parameter<std::string>(
      "fused_odom_topic", "/fastlivo_rtk/odometry");
    output_file_ = declare_parameter<std::string>("output_file", "");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    sync_tolerance_sec_ = declare_parameter<double>(
      "sync_tolerance_sec", 0.12);
    minimum_observations_ = declare_parameter<int>("minimum_observations", 1);
    max_pose_buffer_ = declare_parameter<int>("max_pose_buffer", 2000);
    max_pending_clouds_ = declare_parameter<int>("max_pending_clouds", 8);
    max_voxels_ = declare_parameter<int>("max_voxels", 8000000);
    allow_overwrite_ = declare_parameter<bool>("allow_overwrite", false);

    if (output_file_.empty()) {
      throw std::runtime_error("output_file must not be empty");
    }
    if (!(voxel_size_ > 0.0) || !(sync_tolerance_sec_ > 0.0)) {
      throw std::runtime_error("voxel_size and sync_tolerance_sec must be positive");
    }
    if (minimum_observations_ < 1 || max_pose_buffer_ < 10 ||
      max_pending_clouds_ < 1 || max_voxels_ < 1000)
    {
      throw std::runtime_error("visual map buffer limits are invalid");
    }

    local_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, rclcpp::QoS(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        appendPose(*message, local_poses_);
      });
    fused_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      fused_odom_topic_, rclcpp::QoS(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        appendPose(*message, fused_poses_);
      });
    auto cloud_qos = rclcpp::SensorDataQoS().keep_last(2);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        if (pending_clouds_.size() >= static_cast<std::size_t>(max_pending_clouds_)) {
          pending_clouds_.pop_front();
          ++dropped_clouds_;
        }
        pending_clouds_.push_back(std::move(message));
        processPendingClouds();
      });
    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {processPendingClouds();});
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        processPendingClouds();
        try {
          const std::size_t points = saveMap();
          response->success = true;
          response->message = "saved " + std::to_string(points) +
            " colored voxels to " + output_file_;
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Accumulating RTK-corrected FAST-LIVO2 color clouds at %.3f m resolution",
      voxel_size_);
  }

private:
  void appendPose(
    const nav_msgs::msg::Odometry & message,
    std::deque<TimedPose> & buffer)
  {
    const rclcpp::Time stamp(message.header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() <= 0) {
      return;
    }
    if (!buffer.empty() && stamp < buffer.back().stamp) {
      buffer.clear();
    }
    buffer.push_back(TimedPose{stamp, odometryPose(message)});
    while (buffer.size() > static_cast<std::size_t>(max_pose_buffer_)) {
      buffer.pop_front();
    }
    processPendingClouds();
  }

  const TimedPose * nearestPose(
    const std::deque<TimedPose> & buffer,
    const rclcpp::Time & stamp) const
  {
    const TimedPose * nearest = nullptr;
    double nearest_difference = std::numeric_limits<double>::infinity();
    for (auto iterator = buffer.rbegin(); iterator != buffer.rend(); ++iterator) {
      const double difference = std::abs((iterator->stamp - stamp).seconds());
      if (difference < nearest_difference) {
        nearest = &*iterator;
        nearest_difference = difference;
      }
      if (iterator->stamp < stamp && difference > nearest_difference) {
        break;
      }
    }
    if (nearest_difference > sync_tolerance_sec_) {
      return nullptr;
    }
    return nearest;
  }

  bool poseWindowPassed(
    const std::deque<TimedPose> & buffer,
    const rclcpp::Time & stamp) const
  {
    return !buffer.empty() &&
           (buffer.back().stamp - stamp).seconds() > sync_tolerance_sec_;
  }

  void processPendingClouds()
  {
    while (!pending_clouds_.empty()) {
      const auto & cloud = pending_clouds_.front();
      const rclcpp::Time stamp(cloud->header.stamp, RCL_ROS_TIME);
      const TimedPose * local_pose = nearestPose(local_poses_, stamp);
      const TimedPose * fused_pose = nearestPose(fused_poses_, stamp);
      if (local_pose != nullptr && fused_pose != nullptr) {
        accumulateCloud(*cloud, local_pose->transform, fused_pose->transform);
        pending_clouds_.pop_front();
        continue;
      }
      if (poseWindowPassed(local_poses_, stamp) &&
        poseWindowPassed(fused_poses_, stamp))
      {
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }
      break;
    }
  }

  void accumulateCloud(
    const sensor_msgs::msg::PointCloud2 & message,
    const Eigen::Isometry3d & local_base,
    const Eigen::Isometry3d & fused_base)
  {
    if (!hasColorField(message)) {
      ++uncolored_clouds_;
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(message, cloud);
    const Eigen::Isometry3d map_from_local = fused_base * local_base.inverse();
    for (const auto & point : cloud.points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector3d mapped = map_from_local *
        Eigen::Vector3d(point.x, point.y, point.z);
      const VoxelKey key{
        static_cast<std::int32_t>(std::floor(mapped.x() / voxel_size_)),
        static_cast<std::int32_t>(std::floor(mapped.y() / voxel_size_)),
        static_cast<std::int32_t>(std::floor(mapped.z() / voxel_size_))};
      auto iterator = voxels_.find(key);
      if (iterator == voxels_.end()) {
        if (voxels_.size() >= static_cast<std::size_t>(max_voxels_)) {
          ++dropped_points_;
          continue;
        }
        iterator = voxels_.emplace(key, VoxelAccumulator{}).first;
      }
      auto & accumulator = iterator->second;
      accumulator.x += mapped.x();
      accumulator.y += mapped.y();
      accumulator.z += mapped.z();
      accumulator.r += point.r;
      accumulator.g += point.g;
      accumulator.b += point.b;
      ++accumulator.count;
    }
    ++processed_clouds_;
    if (processed_clouds_ % 100 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Visual map: clouds=%zu voxels=%zu dropped_clouds=%zu",
        processed_clouds_, voxels_.size(), dropped_clouds_);
    }
  }

  std::size_t saveMap()
  {
    if (processed_clouds_ == 0 || voxels_.empty()) {
      throw std::runtime_error("no synchronized colored clouds were accumulated");
    }
    const std::filesystem::path output(output_file_);
    if (std::filesystem::exists(output) && !allow_overwrite_) {
      throw std::runtime_error("visual map already exists: " + output.string());
    }
    if (output.has_parent_path()) {
      std::filesystem::create_directories(output.parent_path());
    }

    pcl::PointCloud<pcl::PointXYZRGB> result;
    result.header.frame_id = "map";
    result.reserve(voxels_.size());
    for (const auto & item : voxels_) {
      const auto & accumulator = item.second;
      if (accumulator.count < static_cast<std::uint64_t>(minimum_observations_)) {
        continue;
      }
      const double inverse_count = 1.0 / static_cast<double>(accumulator.count);
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(accumulator.x * inverse_count);
      point.y = static_cast<float>(accumulator.y * inverse_count);
      point.z = static_cast<float>(accumulator.z * inverse_count);
      point.r = static_cast<std::uint8_t>(std::clamp(
        accumulator.r * inverse_count, 0.0, 255.0));
      point.g = static_cast<std::uint8_t>(std::clamp(
        accumulator.g * inverse_count, 0.0, 255.0));
      point.b = static_cast<std::uint8_t>(std::clamp(
        accumulator.b * inverse_count, 0.0, 255.0));
      result.push_back(point);
    }
    if (result.empty()) {
      throw std::runtime_error("all visual voxels were rejected before saving");
    }
    result.width = static_cast<std::uint32_t>(result.size());
    result.height = 1;
    result.is_dense = false;

    std::filesystem::path temporary = output;
    temporary += ".tmp";
    if (pcl::io::savePCDFileBinary(temporary.string(), result) != 0) {
      throw std::runtime_error("failed to write visual map: " + temporary.string());
    }
    if (std::filesystem::exists(output)) {
      std::filesystem::remove(output);
    }
    std::filesystem::rename(temporary, output);
    RCLCPP_INFO(
      get_logger(),
      "Saved %zu colored voxels from %zu clouds to %s",
      result.size(), processed_clouds_, output.c_str());
    return result.size();
  }

  std::string cloud_topic_;
  std::string local_odom_topic_;
  std::string fused_odom_topic_;
  std::string output_file_;
  double voxel_size_{0.10};
  double sync_tolerance_sec_{0.12};
  int minimum_observations_{1};
  int max_pose_buffer_{2000};
  int max_pending_clouds_{8};
  int max_voxels_{8000000};
  bool allow_overwrite_{false};

  std::deque<TimedPose> local_poses_;
  std::deque<TimedPose> fused_poses_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
  std::size_t processed_clouds_{0};
  std::size_t dropped_clouds_{0};
  std::size_t uncolored_clouds_{0};
  std::size_t dropped_points_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_odom_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr process_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FastlivoRtkVisualMapper>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("fastlivo_rtk_visual_mapper"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
