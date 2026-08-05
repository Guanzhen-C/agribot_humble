#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace agribot_hardware_bringup
{
namespace
{

using namespace std::chrono_literals;
using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("pose quaternion is invalid");
  }
  quaternion.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("pose contains a non-finite value");
  }
  return result;
}

Eigen::Isometry3d xyzRpyToIsometry(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  if (xyz.size() != 3U || rpy.size() != 3U) {
    throw std::runtime_error("XYZ and RPY parameters must contain three values");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  result.linear() =
    (
    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

Eigen::Vector3d rotationRpy(const Eigen::Matrix3d & rotation)
{
  return Eigen::Vector3d(
    std::atan2(rotation(2, 1), rotation(2, 2)),
    std::atan2(-rotation(2, 0), std::hypot(rotation(2, 1), rotation(2, 2))),
    std::atan2(rotation(1, 0), rotation(0, 0)));
}

geometry_msgs::msg::Quaternion quaternionMessage(const Eigen::Matrix3d & rotation)
{
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

class PcdInitialLocalizer final : public rclcpp::Node
{
public:
  PcdInitialLocalizer()
  : Node("pcd_initial_localizer")
  {
    declareParameters();
    validateParameters();
    loadMap();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, latched_qos);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
    aligned_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/aligned_cloud", rclcpp::QoS(1).reliable().transient_local());
    map_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/pcd_map", latched_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100),
      std::bind(&PcdInitialLocalizer::handleOdometry, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdInitialLocalizer::handleCloud, this, std::placeholders::_1));
    initial_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10),
      std::bind(
        &PcdInitialLocalizer::handleInitialPose, this,
        std::placeholders::_1));
    if (!external_ready_topic_.empty()) {
      external_ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
        external_ready_topic_, latched_qos,
        std::bind(
          &PcdInitialLocalizer::handleExternalReady, this,
          std::placeholders::_1));
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    matching_timer_ =
      create_wall_timer(100ms, std::bind(&PcdInitialLocalizer::tryInitialMatch, this));
    heartbeat_timer_ =
      create_wall_timer(1s, std::bind(&PcdInitialLocalizer::publishHeartbeat, this));

    if (automatic_global_localization_) {
      initial_pose_prior_ = Eigen::Isometry3d::Identity();
      pending_attempt_ = true;
      pending_global_search_ = true;
      setStatus("collecting scans for one-shot global FPFH initialization");
    } else {
      setStatus("waiting for RViz 2D Pose Estimate on " + initial_pose_topic_);
    }
    publishHeartbeat();
    publishMap();
    RCLCPP_INFO(
      get_logger(),
      "3D map localizer ready: map=%s points=%zu; initial FPFH=%s; "
      "one-shot initial localization enabled; automatic global search=%s",
      map_file_path_.c_str(), registration_map_->size(),
      enable_fpfh_ ? "enabled" : "disabled",
      automatic_global_localization_ ? "enabled" : "disabled");
  }

private:
  struct TimedPose
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d odom_to_base{Eigen::Isometry3d::Identity()};
  };

  struct ScanSample
  {
    rclcpp::Time stamp;
    PointCloud::Ptr cloud_base;
    Eigen::Isometry3d odom_to_base{Eigen::Isometry3d::Identity()};
  };

  struct MatchResult
  {
    bool accepted{false};
    Eigen::Isometry3d map_to_base{Eigen::Isometry3d::Identity()};
    double overlap{0.0};
    double inlier_rmse{std::numeric_limits<double>::infinity()};
    double elapsed_seconds{0.0};
    std::string reason;
  };

  struct RegistrationQuality
  {
    double overlap{0.0};
    double inlier_rmse{std::numeric_limits<double>::infinity()};
  };

  void declareParameters()
  {
    map_file_path_ = declare_parameter<std::string>("map_file_path", "");
    cloud_topic_ =
      declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "body");
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/fastlio/odometry");
    initial_pose_topic_ =
      declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    pose_topic_ =
      declare_parameter<std::string>("pose_topic", "/localization_pose");
    ready_topic_ =
      declare_parameter<std::string>("ready_topic", "/localization/ready");
    status_topic_ =
      declare_parameter<std::string>("status_topic", "/localization/status");
    external_ready_topic_ =
      declare_parameter<std::string>("external_ready_topic", "");
    external_ready_timeout_sec_ =
      declare_parameter<double>("external_ready_timeout_sec", 0.5);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    enable_fpfh_ = declare_parameter<bool>("enable_fpfh", false);
    automatic_global_localization_ =
      declare_parameter<bool>("automatic_global_localization", false);

    base_to_body_ = xyzRpyToIsometry(
      declare_parameter<std::vector<double>>(
        "base_to_body_xyz", {0.1425, 0.0, 0.143}),
      declare_parameter<std::vector<double>>(
        "base_to_body_rpy", {0.0, 0.0, 0.0}));

    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.15);
    coarse_voxel_size_ = declare_parameter<double>("coarse_voxel_size", 0.30);
    scan_voxel_size_ = declare_parameter<double>("scan_voxel_size", 0.10);
    feature_voxel_size_ = declare_parameter<double>("feature_voxel_size", 0.35);
    normal_radius_ = declare_parameter<double>("normal_radius", 0.70);
    feature_radius_ = declare_parameter<double>("feature_radius", 1.10);
    min_range_ = declare_parameter<double>("min_range", 0.50);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    min_z_ = declare_parameter<double>("min_z", -0.50);
    max_z_ = declare_parameter<double>("max_z", 2.50);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 250);
    initial_scan_count_ = declare_parameter<int>("initial_scan_count", 5);
    max_odom_age_ = declare_parameter<double>("max_odom_age", 0.15);
    initial_search_radius_ = declare_parameter<double>("initial_search_radius", 8.0);
    local_submap_radius_ = declare_parameter<double>("local_submap_radius", 8.0);
    local_submap_min_points_ =
      declare_parameter<int>("local_submap_min_points", 300);

    fpfh_max_iterations_ =
      declare_parameter<int>("fpfh_max_iterations", 12000);
    fpfh_correspondence_randomness_ =
      declare_parameter<int>("fpfh_correspondence_randomness", 5);
    fpfh_similarity_threshold_ =
      declare_parameter<double>("fpfh_similarity_threshold", 0.85);
    fpfh_max_correspondence_distance_ =
      declare_parameter<double>("fpfh_max_correspondence_distance", 0.90);
    fpfh_inlier_fraction_ =
      declare_parameter<double>("fpfh_inlier_fraction", 0.12);

    coarse_ndt_resolution_ =
      declare_parameter<double>("coarse_ndt_resolution", 0.75);
    coarse_ndt_step_size_ =
      declare_parameter<double>("coarse_ndt_step_size", 0.25);
    coarse_ndt_max_iterations_ =
      declare_parameter<int>("coarse_ndt_max_iterations", 40);
    fine_ndt_resolution_ =
      declare_parameter<double>("fine_ndt_resolution", 0.35);
    fine_ndt_step_size_ =
      declare_parameter<double>("fine_ndt_step_size", 0.10);
    fine_ndt_max_iterations_ =
      declare_parameter<int>("fine_ndt_max_iterations", 40);
    ndt_transformation_epsilon_ =
      declare_parameter<double>("ndt_transformation_epsilon", 0.005);
    gicp_max_correspondence_distance_ =
      declare_parameter<double>("gicp_max_correspondence_distance", 0.50);
    gicp_max_iterations_ =
      declare_parameter<int>("gicp_max_iterations", 40);
    gicp_correspondence_randomness_ =
      declare_parameter<int>("gicp_correspondence_randomness", 20);

    overlap_distance_ = declare_parameter<double>("overlap_distance", 0.50);
    maximum_inlier_rmse_ =
      declare_parameter<double>("maximum_inlier_rmse", 0.20);
  }

  void validateParameters() const
  {
    if (map_file_path_.size() < 4U ||
      map_file_path_.substr(map_file_path_.size() - 4U) != ".pcd")
    {
      throw std::runtime_error("map_file_path must point to a PCD map");
    }
    if (map_voxel_size_ <= 0.0 || coarse_voxel_size_ < map_voxel_size_ ||
      scan_voxel_size_ <= 0.0 || feature_voxel_size_ <= 0.0 ||
      normal_radius_ <= feature_voxel_size_ || feature_radius_ <= normal_radius_ ||
      min_range_ < 0.0 || max_range_ <= min_range_ ||
      max_z_ <= min_z_ || min_scan_points_ < 20 || initial_scan_count_ < 1 ||
      max_odom_age_ <= 0.0 || initial_search_radius_ <= 0.0 ||
      initial_search_radius_ > local_submap_radius_ || local_submap_radius_ <= 0.0 ||
      local_submap_min_points_ < 100)
    {
      throw std::runtime_error("invalid map, scan, or local-submap parameters");
    }
    if (fpfh_max_iterations_ < 1 || fpfh_correspondence_randomness_ < 1 ||
      fpfh_similarity_threshold_ <= 0.0 || fpfh_similarity_threshold_ > 1.0 ||
      fpfh_max_correspondence_distance_ <= 0.0 ||
      fpfh_inlier_fraction_ <= 0.0 || fpfh_inlier_fraction_ > 1.0)
    {
      throw std::runtime_error("invalid FPFH prerejective registration parameters");
    }
    if (coarse_ndt_resolution_ <= fine_ndt_resolution_ ||
      fine_ndt_resolution_ <= 0.0 || coarse_ndt_step_size_ <= 0.0 ||
      fine_ndt_step_size_ <= 0.0 || coarse_ndt_max_iterations_ < 1 ||
      fine_ndt_max_iterations_ < 1 || ndt_transformation_epsilon_ <= 0.0 ||
      gicp_max_correspondence_distance_ <= 0.0 || gicp_max_iterations_ < 1 ||
      gicp_correspondence_randomness_ < 5)
    {
      throw std::runtime_error("invalid NDT or GICP parameters");
    }
    if (overlap_distance_ <= 0.0 || maximum_inlier_rmse_ <= 0.0 ||
      external_ready_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("invalid registration validation parameters");
    }
    if (automatic_global_localization_ && !enable_fpfh_) {
      throw std::runtime_error(
              "automatic_global_localization requires enable_fpfh");
    }
  }

  static PointCloud::Ptr voxelize(
    const PointCloud::ConstPtr & input,
    double leaf_size)
  {
    auto output = std::make_shared<PointCloud>();
    pcl::VoxelGrid<Point> filter;
    const float leaf = static_cast<float>(leaf_size);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(input);
    filter.filter(*output);
    return output;
  }

  PointCloud::Ptr filterScan(const PointCloud::ConstPtr & input) const
  {
    auto output = std::make_shared<PointCloud>();
    output->reserve(input->size());
    const double minimum_squared = min_range_ * min_range_;
    const double maximum_squared = max_range_ * max_range_;
    for (const auto & point : input->points) {
      if (!pcl::isFinite(point) || point.z < min_z_ || point.z > max_z_) {
        continue;
      }
      const double squared_range =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z;
      if (squared_range >= minimum_squared && squared_range <= maximum_squared) {
        output->push_back(point);
      }
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  static PointCloud::Ptr cropMap(
    const PointCloud::ConstPtr & input,
    const Eigen::Vector3d & center,
    double radius)
  {
    auto output = std::make_shared<PointCloud>();
    output->reserve(input->size());
    const double radius_squared = radius * radius;
    for (const auto & point : input->points) {
      const double delta_x = static_cast<double>(point.x) - center.x();
      const double delta_y = static_cast<double>(point.y) - center.y();
      if (delta_x * delta_x + delta_y * delta_y <= radius_squared) {
        output->push_back(point);
      }
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  std::pair<PointCloud::Ptr, FeatureCloud::Ptr> computeFeatures(
    const PointCloud::ConstPtr & input) const
  {
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(std::make_shared<pcl::search::KdTree<Point>>());
    normal_estimation.setRadiusSearch(normal_radius_);
    normal_estimation.compute(*normals);

    auto raw_features = std::make_shared<FeatureCloud>();
    pcl::FPFHEstimation<Point, pcl::Normal, Feature> feature_estimation;
    feature_estimation.setInputCloud(input);
    feature_estimation.setInputNormals(normals);
    feature_estimation.setSearchMethod(std::make_shared<pcl::search::KdTree<Point>>());
    feature_estimation.setRadiusSearch(feature_radius_);
    feature_estimation.compute(*raw_features);

    auto points = std::make_shared<PointCloud>();
    auto features = std::make_shared<FeatureCloud>();
    points->reserve(input->size());
    features->reserve(input->size());
    for (std::size_t index = 0; index < input->size(); ++index) {
      const auto & normal = normals->points[index];
      const auto & feature = raw_features->points[index];
      bool finite = std::isfinite(normal.normal_x) &&
        std::isfinite(normal.normal_y) && std::isfinite(normal.normal_z);
      for (float value : feature.histogram) {
        finite = finite && std::isfinite(value);
      }
      if (finite) {
        points->push_back(input->points[index]);
        features->push_back(feature);
      }
    }
    points->width = static_cast<std::uint32_t>(points->size());
    points->height = 1;
    points->is_dense = true;
    features->width = static_cast<std::uint32_t>(features->size());
    features->height = 1;
    features->is_dense = true;
    return {points, features};
  }

  std::pair<PointCloud::Ptr, FeatureCloud::Ptr> cropFeatureMap(
    const Eigen::Vector3d & center) const
  {
    auto points = std::make_shared<PointCloud>();
    auto features = std::make_shared<FeatureCloud>();
    const double radius_squared = initial_search_radius_ * initial_search_radius_;
    for (std::size_t index = 0; index < feature_map_->size(); ++index) {
      const auto & point = feature_map_->points[index];
      const double delta_x = static_cast<double>(point.x) - center.x();
      const double delta_y = static_cast<double>(point.y) - center.y();
      if (delta_x * delta_x + delta_y * delta_y <= radius_squared) {
        points->push_back(point);
        features->push_back(map_features_->points[index]);
      }
    }
    points->width = static_cast<std::uint32_t>(points->size());
    points->height = 1;
    points->is_dense = true;
    features->width = static_cast<std::uint32_t>(features->size());
    features->height = 1;
    features->is_dense = true;
    return {points, features};
  }

  void loadMap()
  {
    auto raw_map = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile<Point>(map_file_path_, *raw_map) != 0) {
      throw std::runtime_error("could not load PCD map: " + map_file_path_);
    }
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw_map, *raw_map, valid_indices);
    registration_map_ = voxelize(raw_map, map_voxel_size_);
    coarse_registration_map_ = voxelize(raw_map, coarse_voxel_size_);
    if (registration_map_->size() < 100U) {
      throw std::runtime_error("PCD map has too few usable points");
    }
    if (enable_fpfh_) {
      const auto feature_input = voxelize(registration_map_, feature_voxel_size_);
      const auto feature_pair = computeFeatures(feature_input);
      feature_map_ = feature_pair.first;
      map_features_ = feature_pair.second;
      if (feature_map_->size() < 100U) {
        throw std::runtime_error("PCD map has too few valid FPFH features");
      }
    }
    map_tree_.setInputCloud(registration_map_);
  }

  std::optional<TimedPose> closestOdometry(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    if (odometry_buffer_.empty()) {
      return std::nullopt;
    }
    const auto closest = std::min_element(
      odometry_buffer_.begin(), odometry_buffer_.end(),
      [&stamp](const TimedPose & left, const TimedPose & right) {
        return std::abs((left.stamp - stamp).seconds()) <
        std::abs((right.stamp - stamp).seconds());
      });
    if (std::abs((closest->stamp - stamp).seconds()) > max_odom_age_) {
      return std::nullopt;
    }
    return *closest;
  }

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    TimedPose sample{
      rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()),
      Eigen::Isometry3d::Identity()};
    try {
      sample.odom_to_base = poseToIsometry(message->pose.pose);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid localization odometry: %s", exception.what());
      return;
    }

    std::optional<Eigen::Isometry3d> map_to_odom;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      odometry_buffer_.push_back(sample);
      while (odometry_buffer_.size() > 2U &&
        (sample.stamp - odometry_buffer_.front().stamp).seconds() > 2.0)
      {
        odometry_buffer_.pop_front();
      }
      if (localized_) {
        map_to_odom = map_to_odom_;
      }
    }
    if (map_to_odom.has_value()) {
      publishTransform(message->header.stamp, *map_to_odom);
      publishPose(message->header.stamp, *map_to_odom * sample.odom_to_base, *message);
    }
  }

  void handleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      if (localized_) {
        return;
      }
    }
    if (!cloud_frame_.empty() && message->header.frame_id != cloud_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Expected %s point cloud frame but received %s",
        cloud_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    const auto odometry = closestOdometry(stamp);
    if (!odometry.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No localization odometry within %.3f s of the registration scan",
        max_odom_age_);
      return;
    }

    auto body_cloud = std::make_shared<PointCloud>();
    pcl::fromROSMsg(*message, *body_cloud);
    auto base_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
      *body_cloud, *base_cloud, base_to_body_.matrix().cast<float>());
    base_cloud = voxelize(filterScan(base_cloud), scan_voxel_size_);
    if (base_cloud->size() < static_cast<std::size_t>(min_scan_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Registration scan has only %zu usable points; need at least %d",
        base_cloud->size(), min_scan_points_);
      return;
    }

    std::lock_guard<std::mutex> guard(state_mutex_);
    scan_buffer_.push_back(ScanSample{stamp, base_cloud, odometry->odom_to_base});
    while (scan_buffer_.size() > static_cast<std::size_t>(initial_scan_count_)) {
      scan_buffer_.pop_front();
    }
  }

  void handleInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring initial pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    Eigen::Isometry3d planar_pose;
    try {
      const auto input = poseToIsometry(message->pose.pose);
      const double yaw = rotationRpy(input.linear()).z();
      planar_pose = Eigen::Isometry3d::Identity();
      planar_pose.translation() = Eigen::Vector3d(
        input.translation().x(), input.translation().y(), 0.0);
      planar_pose.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid initial pose: %s", exception.what());
      return;
    }

    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      localized_ = false;
      scan_buffer_.clear();
      initial_pose_prior_ = planar_pose;
      pending_attempt_ = true;
      pending_global_search_ = false;
      setStatusLocked(
        enable_fpfh_ ?
        "initial pose received; collecting scans for local FPFH initialization" :
        "initial pose received; collecting scans for local NDT and GICP initialization");
    }
    const auto rpy = rotationRpy(planar_pose.linear());
    RCLCPP_INFO(
      get_logger(), "Initial pose prior: x=%.2f y=%.2f yaw=%.1f deg",
      planar_pose.translation().x(), planar_pose.translation().y(),
      rpy.z() * 180.0 / M_PI);
    publishHeartbeat();
  }

  void handleExternalReady(const std_msgs::msg::Bool::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      external_ready_ = message->data;
      external_ready_stamp_ = now();
    }
    publishHeartbeat();
  }

  ScanSample aggregateScans(const std::deque<ScanSample> & scans) const
  {
    const ScanSample & reference = scans.back();
    auto aggregate = std::make_shared<PointCloud>();
    for (const auto & scan : scans) {
      PointCloud transformed;
      const Eigen::Isometry3d reference_to_scan =
        reference.odom_to_base.inverse() * scan.odom_to_base;
      pcl::transformPointCloud(
        *scan.cloud_base, transformed, reference_to_scan.matrix().cast<float>());
      *aggregate += transformed;
    }
    ScanSample result = reference;
    result.cloud_base = voxelize(aggregate, scan_voxel_size_);
    return result;
  }

  std::optional<Eigen::Isometry3d> alignNdt(
    const PointCloud::ConstPtr & source,
    const PointCloud::ConstPtr & target,
    const Eigen::Isometry3d & initial_guess,
    double resolution,
    double step_size,
    int maximum_iterations,
    std::string & failure_reason) const
  {
    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    ndt.setResolution(static_cast<float>(resolution));
    ndt.setStepSize(step_size);
    ndt.setMaximumIterations(maximum_iterations);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    PointCloud aligned;
    ndt.align(aligned, initial_guess.matrix().cast<float>());
    if (!ndt.hasConverged()) {
      failure_reason = "NDT did not converge";
      return std::nullopt;
    }
    return Eigen::Isometry3d(ndt.getFinalTransformation().cast<double>());
  }

  RegistrationQuality registrationQuality(
    const PointCloud::ConstPtr & source,
    const Eigen::Isometry3d & map_to_base)
  {
    PointCloud aligned;
    pcl::transformPointCloud(
      *source, aligned, map_to_base.matrix().cast<float>());
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t inliers = 0U;
    double squared_error = 0.0;
    const float maximum_squared =
      static_cast<float>(overlap_distance_ * overlap_distance_);
    for (const auto & point : aligned.points) {
      if (map_tree_.nearestKSearch(point, 1, indices, distances) == 1 &&
        distances[0] <= maximum_squared)
      {
        ++inliers;
        squared_error += distances[0];
      }
    }
    if (inliers == 0U || source->empty()) {
      return {};
    }
    return {
      static_cast<double>(inliers) / static_cast<double>(source->size()),
      std::sqrt(squared_error / static_cast<double>(inliers))};
  }

  std::optional<Eigen::Isometry3d> initialFeatureRegistration(
    const ScanSample & sample,
    const Eigen::Isometry3d & initial_pose,
    bool global_search,
    std::string & failure_reason) const
  {
    const auto feature_input = voxelize(sample.cloud_base, feature_voxel_size_);
    const auto source_pair = computeFeatures(feature_input);
    const auto target_pair = global_search ?
      std::make_pair(feature_map_, map_features_) :
      cropFeatureMap(initial_pose.translation());
    if (source_pair.first->size() < 50U) {
      failure_reason = "too few source FPFH features";
      return std::nullopt;
    }
    if (target_pair.first->size() < 100U) {
      failure_reason = "local PCD submap has too few FPFH features";
      return std::nullopt;
    }

    pcl::SampleConsensusPrerejective<Point, Point, Feature> registration;
    registration.setInputSource(source_pair.first);
    registration.setSourceFeatures(source_pair.second);
    registration.setInputTarget(target_pair.first);
    registration.setTargetFeatures(target_pair.second);
    registration.setMaximumIterations(fpfh_max_iterations_);
    registration.setNumberOfSamples(3);
    registration.setCorrespondenceRandomness(fpfh_correspondence_randomness_);
    registration.setSimilarityThreshold(
      static_cast<float>(fpfh_similarity_threshold_));
    registration.setMaxCorrespondenceDistance(
      static_cast<float>(fpfh_max_correspondence_distance_));
    registration.setInlierFraction(static_cast<float>(fpfh_inlier_fraction_));

    PointCloud aligned;
    registration.align(aligned);
    if (!registration.hasConverged()) {
      failure_reason = "local FPFH prerejective registration did not converge";
      return std::nullopt;
    }
    return Eigen::Isometry3d(registration.getFinalTransformation().cast<double>());
  }

  MatchResult runRegistration(
    const ScanSample & sample,
    const Eigen::Isometry3d & initial_pose)
  {
    MatchResult result;
    const auto started = std::chrono::steady_clock::now();
    const auto coarse_source = voxelize(sample.cloud_base, coarse_voxel_size_);
    const auto coarse_target = cropMap(
      coarse_registration_map_, initial_pose.translation(), local_submap_radius_);
    if (coarse_target->size() < static_cast<std::size_t>(local_submap_min_points_)) {
      result.reason = "local PCD submap has too few points";
      return result;
    }

    std::string failure_reason;
    const auto coarse = alignNdt(
      coarse_source, coarse_target, initial_pose,
      coarse_ndt_resolution_, coarse_ndt_step_size_,
      coarse_ndt_max_iterations_, failure_reason);
    if (!coarse.has_value()) {
      result.reason = "coarse " + failure_reason;
      return result;
    }

    const auto fine_target = cropMap(
      registration_map_, coarse->translation(), local_submap_radius_);
    const auto fine = alignNdt(
      sample.cloud_base, fine_target, *coarse,
      fine_ndt_resolution_, fine_ndt_step_size_,
      fine_ndt_max_iterations_, failure_reason);
    if (!fine.has_value()) {
      result.reason = "fine " + failure_reason;
      return result;
    }

    pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
    gicp.setInputSource(sample.cloud_base);
    gicp.setInputTarget(fine_target);
    gicp.setMaxCorrespondenceDistance(gicp_max_correspondence_distance_);
    gicp.setMaximumIterations(gicp_max_iterations_);
    gicp.setCorrespondenceRandomness(gicp_correspondence_randomness_);
    gicp.setTransformationEpsilon(1.0e-5);
    gicp.setRotationEpsilon(1.0e-5);
    gicp.setEuclideanFitnessEpsilon(1.0e-4);
    PointCloud aligned;
    gicp.align(aligned, fine->matrix().cast<float>());
    result.elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    if (!gicp.hasConverged()) {
      result.reason = "GICP did not converge";
      return result;
    }

    result.map_to_base = Eigen::Isometry3d(
      gicp.getFinalTransformation().cast<double>());
    const RegistrationQuality quality =
      registrationQuality(sample.cloud_base, result.map_to_base);
    result.overlap = quality.overlap;
    result.inlier_rmse = quality.inlier_rmse;

    if (result.inlier_rmse > maximum_inlier_rmse_) {
      result.reason = "scan-to-map inlier RMSE is above the acceptance limit";
    } else {
      result.accepted = true;
      result.reason = "one-shot NDT and GICP initial localization accepted";
    }
    return result;
  }

  void tryInitialMatch()
  {
    std::deque<ScanSample> scans;
    Eigen::Isometry3d pose_seed = Eigen::Isometry3d::Identity();
    bool global_feature_search = false;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      if (matching_ || localized_) {
        return;
      }
      if (!pending_attempt_ || !initial_pose_prior_.has_value() ||
        scan_buffer_.size() < static_cast<std::size_t>(initial_scan_count_))
      {
        return;
      }
      scans = scan_buffer_;
      pose_seed = *initial_pose_prior_;
      global_feature_search = pending_global_search_;
      pending_attempt_ = false;
      setStatusLocked(
        enable_fpfh_ ?
        "running RViz-guided local FPFH coarse registration" :
        "running RViz-guided local NDT and GICP registration");
      matching_ = true;
    }

    const ScanSample sample = aggregateScans(scans);
    MatchResult result;
    const auto started = std::chrono::steady_clock::now();
    if (enable_fpfh_) {
      std::string failure_reason;
      const auto feature_pose =
        initialFeatureRegistration(
        sample, pose_seed, global_feature_search, failure_reason);
      if (feature_pose.has_value()) {
        setStatus("refining local FPFH result with NDT and GICP");
        result = runRegistration(sample, *feature_pose);
      } else {
        result.reason = failure_reason;
      }
    } else {
      result = runRegistration(sample, pose_seed);
    }
    result.elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();

    Eigen::Isometry3d published_map_to_odom = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      matching_ = false;
      if (result.accepted) {
        map_to_odom_ =
          result.map_to_base * sample.odom_to_base.inverse();
        initial_pose_prior_.reset();
        scan_buffer_.clear();
        setStatusLocked("initial localization accepted; map-to-odom correction fixed");
        localized_ = true;
        published_map_to_odom = map_to_odom_;
      } else {
        setStatusLocked(result.reason + "; set a new RViz initial pose to retry");
      }
    }

    if (!result.accepted) {
      RCLCPP_WARN(
        get_logger(),
        "Initial registration rejected in %.3f s: overlap=%.3f "
        "inlier_rmse=%.3f m (%s)", result.elapsed_seconds,
        result.overlap, result.inlier_rmse,
        result.reason.c_str());
      publishHeartbeat();
      return;
    }

    const auto localized_rpy = rotationRpy(result.map_to_base.linear());
    RCLCPP_INFO(
      get_logger(),
      "Initial registration accepted in %.3f s: pose=(%.3f, %.3f, %.3f, "
      "yaw %.2f deg) overlap=%.3f inlier_rmse=%.3f m",
      result.elapsed_seconds, result.map_to_base.translation().x(),
      result.map_to_base.translation().y(), result.map_to_base.translation().z(),
      localized_rpy.z() * 180.0 / M_PI, result.overlap, result.inlier_rmse);

    publishAlignedCloud(sample, result.map_to_base);
    publishTransform(sample.stamp, published_map_to_odom);
    publishHeartbeat();
  }

  void setStatusLocked(const std::string & status)
  {
    status_ = status;
    std_msgs::msg::String message;
    message.data = status_;
    status_publisher_->publish(message);
  }

  void setStatus(const std::string & status)
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    setStatusLocked(status);
  }

  void publishHeartbeat()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    std_msgs::msg::Bool ready;
    bool external_ready = true;
    if (!external_ready_topic_.empty()) {
      external_ready = external_ready_ && external_ready_stamp_.has_value();
      if (external_ready) {
        const double age = (now() - *external_ready_stamp_).seconds();
        external_ready = age >= 0.0 && age <= external_ready_timeout_sec_;
      }
    }
    ready.data = localized_ && external_ready;
    ready_publisher_->publish(ready);
    std_msgs::msg::String status;
    status.data = status_;
    status_publisher_->publish(status);
  }

  void publishTransform(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Isometry3d & map_to_odom)
  {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform.translation.x = map_to_odom.translation().x();
    message.transform.translation.y = map_to_odom.translation().y();
    message.transform.translation.z = map_to_odom.translation().z();
    message.transform.rotation = quaternionMessage(map_to_odom.linear());
    tf_broadcaster_->sendTransform(message);
  }

  void publishPose(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Isometry3d & map_to_base,
    const nav_msgs::msg::Odometry & odometry)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = map_to_base.translation().x();
    message.pose.pose.position.y = map_to_base.translation().y();
    message.pose.pose.position.z = map_to_base.translation().z();
    message.pose.pose.orientation = quaternionMessage(map_to_base.linear());
    message.pose.covariance = odometry.pose.covariance;
    pose_publisher_->publish(message);
  }

  void publishAlignedCloud(
    const ScanSample & sample,
    const Eigen::Isometry3d & map_to_base)
  {
    PointCloud aligned;
    pcl::transformPointCloud(
      *sample.cloud_base, aligned, map_to_base.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = sample.stamp;
    message.header.frame_id = map_frame_;
    aligned_cloud_publisher_->publish(message);
  }

  void publishMap()
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*registration_map_, message);
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    map_cloud_publisher_->publish(message);
  }

  std::string map_file_path_;
  std::string cloud_topic_;
  std::string cloud_frame_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string pose_topic_;
  std::string ready_topic_;
  std::string status_topic_;
  std::string external_ready_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  bool enable_fpfh_{false};
  bool automatic_global_localization_{false};
  bool pending_global_search_{false};
  Eigen::Isometry3d base_to_body_{Eigen::Isometry3d::Identity()};

  double map_voxel_size_{0.15};
  double coarse_voxel_size_{0.30};
  double scan_voxel_size_{0.10};
  double feature_voxel_size_{0.35};
  double normal_radius_{0.70};
  double feature_radius_{1.10};
  double min_range_{0.50};
  double max_range_{30.0};
  double min_z_{-0.50};
  double max_z_{2.50};
  int min_scan_points_{250};
  int initial_scan_count_{5};
  double max_odom_age_{0.15};
  double initial_search_radius_{8.0};
  double local_submap_radius_{8.0};
  int local_submap_min_points_{300};

  int fpfh_max_iterations_{12000};
  int fpfh_correspondence_randomness_{5};
  double fpfh_similarity_threshold_{0.85};
  double fpfh_max_correspondence_distance_{0.90};
  double fpfh_inlier_fraction_{0.12};

  double coarse_ndt_resolution_{0.75};
  double coarse_ndt_step_size_{0.25};
  int coarse_ndt_max_iterations_{40};
  double fine_ndt_resolution_{0.35};
  double fine_ndt_step_size_{0.10};
  int fine_ndt_max_iterations_{40};
  double ndt_transformation_epsilon_{0.005};
  double gicp_max_correspondence_distance_{0.50};
  int gicp_max_iterations_{40};
  int gicp_correspondence_randomness_{20};

  double overlap_distance_{0.50};
  double maximum_inlier_rmse_{0.20};
  double external_ready_timeout_sec_{0.5};

  PointCloud::Ptr registration_map_;
  PointCloud::Ptr coarse_registration_map_;
  PointCloud::Ptr feature_map_;
  FeatureCloud::Ptr map_features_;
  pcl::KdTreeFLANN<Point> map_tree_;

  std::mutex state_mutex_;
  std::deque<TimedPose> odometry_buffer_;
  std::deque<ScanSample> scan_buffer_;
  std::optional<Eigen::Isometry3d> initial_pose_prior_;
  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  bool pending_attempt_{false};
  bool matching_{false};
  bool localized_{false};
  bool external_ready_{false};
  std::optional<rclcpp::Time> external_ready_stamp_;
  std::string status_{"starting"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_ready_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    aligned_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr matching_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace
}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<agribot_hardware_bringup::PcdInitialLocalizer>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pcd_initial_localizer"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
