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
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
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
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
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

diagnostic_msgs::msg::KeyValue keyValue(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

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
  result.translation() =
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
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

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);
  return std::abs(angle_axis.angle());
}

Eigen::Vector3d rotationRpy(const Eigen::Matrix3d & rotation)
{
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::atan2(
    -rotation(2, 0),
    std::hypot(rotation(2, 1), rotation(2, 2)));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Isometry3d interpolateTransform(
  const Eigen::Isometry3d & from,
  const Eigen::Isometry3d & to,
  double alpha)
{
  const double bounded_alpha = std::clamp(alpha, 0.0, 1.0);
  Eigen::Quaterniond from_quaternion(from.linear());
  Eigen::Quaterniond to_quaternion(to.linear());
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() =
    from_quaternion.slerp(bounded_alpha, to_quaternion).normalized().toRotationMatrix();
  result.translation() =
    (1.0 - bounded_alpha) * from.translation() +
    bounded_alpha * to.translation();
  return result;
}

class PcdGlobalLocalizer final : public rclcpp::Node
{
public:
  PcdGlobalLocalizer()
  : Node("pcd_global_localizer")
  {
    declareParameters();
    validateParameters();
    loadMap();

    const auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
    ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(ready_topic_, ready_qos);
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 20);
    fitness_publisher_ =
      create_publisher<std_msgs::msg::Float32>(
      "/localization/fitness_score", 10);
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      "/localization/status", ready_qos);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());
    aligned_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/aligned_cloud", rclcpp::SensorDataQoS());
    map_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/pcd_map", rclcpp::QoS(1).reliable().transient_local());

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100),
      std::bind(&PcdGlobalLocalizer::handleOdometry, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdGlobalLocalizer::handleCloud, this, std::placeholders::_1));

    relocalize_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/relocalize",
      std::bind(
        &PcdGlobalLocalizer::handleRelocalize, this,
        std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    work_timer_ = create_wall_timer(500ms, std::bind(&PcdGlobalLocalizer::runMatching, this));
    heartbeat_timer_ =
      create_wall_timer(1s, std::bind(&PcdGlobalLocalizer::publishHeartbeat, this));
    diagnostics_timer_ =
      create_wall_timer(1s, std::bind(&PcdGlobalLocalizer::publishDiagnostics, this));

    publishMap();
    setStatus("waiting for synchronized FAST-LIO odometry and body-frame scans");
    publishReady();
    RCLCPP_INFO(
      get_logger(),
      "Automatic 3D relocalization ready: map=%s points=%zu update=%.2f Hz",
      map_file_path_.c_str(), registration_map_->size(), matching_rate_hz_);
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
    double fitness{std::numeric_limits<double>::infinity()};
    double overlap{0.0};
    std::string reason;
  };

  void declareParameters()
  {
    map_file_path_ = declare_parameter<std::string>("map_file_path", "");
    cloud_topic_ =
      declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "body");
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/fastlio/odometry");
    pose_topic_ =
      declare_parameter<std::string>("pose_topic", "/localization_pose");
    ready_topic_ =
      declare_parameter<std::string>("ready_topic", "/localization/ready");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const auto base_to_body_xyz =
      declare_parameter<std::vector<double>>(
      "base_to_body_xyz", {0.1425, 0.0, 0.143});
    const auto base_to_body_rpy =
      declare_parameter<std::vector<double>>(
      "base_to_body_rpy", {0.0, 0.0, 0.0});
    base_to_body_ = xyzRpyToIsometry(base_to_body_xyz, base_to_body_rpy);

    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.25);
    scan_voxel_size_ = declare_parameter<double>("scan_voxel_size", 0.20);
    feature_voxel_size_ =
      declare_parameter<double>("feature_voxel_size", 0.35);
    normal_radius_ = declare_parameter<double>("normal_radius", 0.70);
    feature_radius_ = declare_parameter<double>("feature_radius", 1.10);
    min_range_ = declare_parameter<double>("min_range", 0.50);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    min_z_ = declare_parameter<double>("min_z", -0.50);
    max_z_ = declare_parameter<double>("max_z", 2.50);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 250);
    initial_scan_count_ = declare_parameter<int>("initial_scan_count", 3);
    max_odom_age_ = declare_parameter<double>("max_odom_age", 0.10);

    global_max_iterations_ =
      declare_parameter<int>("global_max_iterations", 40000);
    global_correspondence_randomness_ =
      declare_parameter<int>("global_correspondence_randomness", 5);
    global_similarity_threshold_ =
      declare_parameter<double>("global_similarity_threshold", 0.85);
    global_max_correspondence_distance_ =
      declare_parameter<double>("global_max_correspondence_distance", 0.90);
    global_inlier_fraction_ =
      declare_parameter<double>("global_inlier_fraction", 0.12);
    global_retry_period_ =
      declare_parameter<double>("global_retry_period", 5.0);

    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 0.80);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.10);
    ndt_transformation_epsilon_ =
      declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 35);
    fitness_max_range_ =
      declare_parameter<double>("fitness_max_range", 1.50);
    max_fitness_score_ =
      declare_parameter<double>("max_fitness_score", 0.45);
    overlap_distance_ =
      declare_parameter<double>("overlap_distance", 0.50);
    minimum_overlap_ = declare_parameter<double>("minimum_overlap", 0.20);
    maximum_tilt_ = declare_parameter<double>("maximum_tilt", 0.35);
    maximum_base_height_ =
      declare_parameter<double>("maximum_base_height", 0.50);

    matching_rate_hz_ = declare_parameter<double>("matching_rate_hz", 0.25);
    required_consecutive_matches_ =
      declare_parameter<int>("required_consecutive_matches", 2);
    runtime_failure_limit_ =
      declare_parameter<int>("runtime_failure_limit", 3);
    relocalize_failure_limit_ =
      declare_parameter<int>("relocalize_failure_limit", 5);
    maximum_translation_correction_ =
      declare_parameter<double>("maximum_translation_correction", 0.75);
    maximum_rotation_correction_ =
      declare_parameter<double>("maximum_rotation_correction", 0.35);
    correction_alpha_ = declare_parameter<double>("correction_alpha", 0.25);
  }

  void validateParameters() const
  {
    if (map_file_path_.size() < 4U ||
      map_file_path_.substr(map_file_path_.size() - 4U) != ".pcd")
    {
      throw std::runtime_error("map_file_path must point to a PCD map");
    }
    if (
      map_voxel_size_ <= 0.0 || scan_voxel_size_ <= 0.0 ||
      feature_voxel_size_ <= 0.0 || normal_radius_ <= feature_voxel_size_ ||
      feature_radius_ <= normal_radius_)
    {
      throw std::runtime_error(
              "voxel sizes must be positive and feature_radius > normal_radius > feature voxel");
    }
    if (
      min_range_ < 0.0 || max_range_ <= min_range_ ||
      max_z_ <= min_z_ || min_scan_points_ < 20 ||
      initial_scan_count_ < 1 || max_odom_age_ <= 0.0)
    {
      throw std::runtime_error("invalid scan filtering or synchronization parameters");
    }
    if (
      global_max_iterations_ < 1 || global_correspondence_randomness_ < 1 ||
      global_similarity_threshold_ <= 0.0 || global_similarity_threshold_ > 1.0 ||
      global_max_correspondence_distance_ <= 0.0 ||
      global_inlier_fraction_ <= 0.0 || global_inlier_fraction_ > 1.0 ||
      global_retry_period_ <= 0.0)
    {
      throw std::runtime_error("invalid global registration parameters");
    }
    if (
      ndt_resolution_ <= 0.0 || ndt_step_size_ <= 0.0 ||
      ndt_transformation_epsilon_ <= 0.0 || ndt_max_iterations_ < 1 ||
      fitness_max_range_ <= 0.0 || max_fitness_score_ <= 0.0 ||
      overlap_distance_ <= 0.0 || minimum_overlap_ <= 0.0 ||
      minimum_overlap_ > 1.0)
    {
      throw std::runtime_error("invalid NDT or match validation parameters");
    }
    if (
      matching_rate_hz_ < 0.2 || matching_rate_hz_ > 0.5 ||
      required_consecutive_matches_ < 1 || runtime_failure_limit_ < 1 ||
      relocalize_failure_limit_ < runtime_failure_limit_ ||
      maximum_translation_correction_ <= 0.0 ||
      maximum_rotation_correction_ <= 0.0 ||
      correction_alpha_ <= 0.0 || correction_alpha_ > 1.0)
    {
      throw std::runtime_error("invalid runtime correction parameters");
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

  PointCloud::Ptr filterByHeightAndRange(
    const PointCloud::ConstPtr & input,
    bool apply_range) const
  {
    auto filtered = std::make_shared<PointCloud>();
    filtered->reserve(input->size());
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
      if (apply_range &&
        (squared_range < minimum_squared || squared_range > maximum_squared))
      {
        continue;
      }
      filtered->push_back(point);
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
  }

  std::pair<PointCloud::Ptr, FeatureCloud::Ptr> computeFeatures(
    const PointCloud::ConstPtr & input) const
  {
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(input);
    normal_estimation.setSearchMethod(
      std::make_shared<pcl::search::KdTree<Point>>());
    normal_estimation.setRadiusSearch(normal_radius_);
    normal_estimation.compute(*normals);

    auto raw_features = std::make_shared<FeatureCloud>();
    pcl::FPFHEstimation<Point, pcl::Normal, Feature> feature_estimation;
    feature_estimation.setInputCloud(input);
    feature_estimation.setInputNormals(normals);
    feature_estimation.setSearchMethod(
      std::make_shared<pcl::search::KdTree<Point>>());
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
    features->width = static_cast<std::uint32_t>(features->size());
    features->height = 1;
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
    registration_map_ =
      voxelize(filterByHeightAndRange(raw_map, false), map_voxel_size_);
    if (registration_map_->size() < 100U) {
      throw std::runtime_error("PCD map has too few usable points");
    }

    auto feature_input = voxelize(registration_map_, feature_voxel_size_);
    const auto feature_pair = computeFeatures(feature_input);
    feature_map_ = feature_pair.first;
    map_features_ = feature_pair.second;
    if (feature_map_->size() < 100U) {
      throw std::runtime_error("PCD map has too few valid FPFH features");
    }

    map_tree_.setInputCloud(registration_map_);
    map_minimum_ = Eigen::Vector3d(
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
    map_maximum_ = -map_minimum_;
    for (const auto & point : registration_map_->points) {
      map_minimum_ = map_minimum_.cwiseMin(
        Eigen::Vector3d(point.x, point.y, point.z));
      map_maximum_ = map_maximum_.cwiseMax(
        Eigen::Vector3d(point.x, point.y, point.z));
    }
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
        "Ignoring invalid FAST-LIO odometry: %s", exception.what());
      return;
    }

    std::optional<Eigen::Isometry3d> map_to_odom;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      odometry_buffer_.push_back(sample);
      while (
        odometry_buffer_.size() > 2U &&
        (sample.stamp - odometry_buffer_.front().stamp).seconds() > 2.0)
      {
        odometry_buffer_.pop_front();
      }
      if (initialized_) {
        map_to_odom = map_to_odom_;
      }
    }
    if (!map_to_odom.has_value()) {
      return;
    }
    publishTransform(message->header.stamp, *map_to_odom);
    publishPose(message->header.stamp, *map_to_odom * sample.odom_to_base, *message);
  }

  void handleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
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
        "No FAST-LIO odometry within %.3f s of the body-frame scan",
        max_odom_age_);
      return;
    }

    auto body_cloud = std::make_shared<PointCloud>();
    pcl::fromROSMsg(*message, *body_cloud);
    auto base_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(
      *body_cloud, *base_cloud, base_to_body_.matrix().cast<float>());
    base_cloud = filterByHeightAndRange(base_cloud, true);
    base_cloud = voxelize(base_cloud, scan_voxel_size_);
    if (base_cloud->size() < static_cast<std::size_t>(min_scan_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Body-frame scan has only %zu usable points; need at least %d",
        base_cloud->size(), min_scan_points_);
      return;
    }

    ScanSample sample{stamp, base_cloud, odometry->odom_to_base};
    std::lock_guard<std::mutex> guard(state_mutex_);
    latest_scan_ = sample;
    if (!initialized_) {
      initialization_scans_.push_back(sample);
      while (
        initialization_scans_.size() >
        static_cast<std::size_t>(initial_scan_count_))
      {
        initialization_scans_.pop_front();
      }
    }
  }

  ScanSample aggregateInitialScans(
    const std::deque<ScanSample> & scans) const
  {
    const ScanSample & reference = scans.back();
    auto aggregate = std::make_shared<PointCloud>();
    for (const auto & scan : scans) {
      PointCloud transformed;
      const Eigen::Isometry3d reference_to_scan =
        reference.odom_to_base.inverse() * scan.odom_to_base;
      pcl::transformPointCloud(
        *scan.cloud_base, transformed,
        reference_to_scan.matrix().cast<float>());
      *aggregate += transformed;
    }
    ScanSample result = reference;
    result.cloud_base = voxelize(aggregate, scan_voxel_size_);
    return result;
  }

  MatchResult globalRegistration(const ScanSample & sample)
  {
    setStatus("running full-map FPFH global search");
    auto feature_input = voxelize(sample.cloud_base, feature_voxel_size_);
    const auto feature_pair = computeFeatures(feature_input);
    const auto & source_points = feature_pair.first;
    const auto & source_features = feature_pair.second;
    if (source_points->size() < 50U) {
      return MatchResult{
        false, Eigen::Isometry3d::Identity(),
        std::numeric_limits<double>::infinity(), 0.0,
        "too few source FPFH features"};
    }

    pcl::SampleConsensusPrerejective<Point, Point, Feature> registration;
    registration.setInputSource(source_points);
    registration.setSourceFeatures(source_features);
    registration.setInputTarget(feature_map_);
    registration.setTargetFeatures(map_features_);
    registration.setMaximumIterations(global_max_iterations_);
    registration.setNumberOfSamples(3);
    registration.setCorrespondenceRandomness(global_correspondence_randomness_);
    registration.setSimilarityThreshold(
      static_cast<float>(global_similarity_threshold_));
    registration.setMaxCorrespondenceDistance(
      static_cast<float>(global_max_correspondence_distance_));
    registration.setInlierFraction(static_cast<float>(global_inlier_fraction_));

    PointCloud aligned;
    registration.align(aligned);
    if (!registration.hasConverged()) {
      return MatchResult{
        false, Eigen::Isometry3d::Identity(),
        std::numeric_limits<double>::infinity(), 0.0,
        "FPFH global search did not converge"};
    }
    const Eigen::Isometry3d global_guess(
      registration.getFinalTransformation().cast<double>());
    return refineNdt(sample.cloud_base, global_guess);
  }

  MatchResult refineNdt(
    const PointCloud::ConstPtr & source,
    const Eigen::Isometry3d & initial_guess)
  {
    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setInputSource(source);
    ndt.setInputTarget(registration_map_);
    ndt.setResolution(static_cast<float>(ndt_resolution_));
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);

    PointCloud aligned;
    ndt.align(aligned, initial_guess.matrix().cast<float>());
    if (!ndt.hasConverged()) {
      return MatchResult{
        false, Eigen::Isometry3d::Identity(),
        std::numeric_limits<double>::infinity(), 0.0,
        "NDT refinement did not converge"};
    }
    const Eigen::Isometry3d transform(
      ndt.getFinalTransformation().cast<double>());
    const double fitness = ndt.getFitnessScore(fitness_max_range_);
    const double overlap = overlapRatio(source, transform);
    const auto validation_error = validatePose(transform, fitness, overlap);
    if (validation_error.has_value()) {
      return MatchResult{false, transform, fitness, overlap, *validation_error};
    }
    return MatchResult{true, transform, fitness, overlap, "match accepted"};
  }

  double overlapRatio(
    const PointCloud::ConstPtr & source,
    const Eigen::Isometry3d & map_to_base)
  {
    if (source->empty()) {
      return 0.0;
    }
    PointCloud aligned;
    pcl::transformPointCloud(
      *source, aligned, map_to_base.matrix().cast<float>());
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    std::size_t inliers = 0U;
    const float maximum_squared =
      static_cast<float>(overlap_distance_ * overlap_distance_);
    for (const auto & point : aligned.points) {
      if (
        map_tree_.nearestKSearch(point, 1, indices, distances) == 1 &&
        distances[0] <= maximum_squared)
      {
        ++inliers;
      }
    }
    return static_cast<double>(inliers) / static_cast<double>(source->size());
  }

  std::optional<std::string> validatePose(
    const Eigen::Isometry3d & map_to_base,
    double fitness,
    double overlap) const
  {
    if (!map_to_base.matrix().allFinite() || !std::isfinite(fitness)) {
      return "registration returned a non-finite result";
    }
    if (fitness > max_fitness_score_) {
      return "NDT fitness score exceeds the configured limit";
    }
    if (overlap < minimum_overlap_) {
      return "3D scan-to-map overlap is below the configured limit";
    }
    const Eigen::Vector3d rpy = rotationRpy(map_to_base.linear());
    if (std::abs(rpy.x()) > maximum_tilt_ || std::abs(rpy.y()) > maximum_tilt_) {
      return "matched vehicle pose has implausible roll or pitch";
    }
    if (std::abs(map_to_base.translation().z()) > maximum_base_height_) {
      return "matched rear-axle height is outside the mapped floor";
    }
    if (
      map_to_base.translation().x() < map_minimum_.x() - 1.0 ||
      map_to_base.translation().x() > map_maximum_.x() + 1.0 ||
      map_to_base.translation().y() < map_minimum_.y() - 1.0 ||
      map_to_base.translation().y() > map_maximum_.y() + 1.0)
    {
      return "matched vehicle pose lies outside the PCD map";
    }
    return std::nullopt;
  }

  void runMatching()
  {
    ScanSample sample;
    bool initialized = false;
    Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      if (!latest_scan_.has_value()) {
        return;
      }
      initialized = initialized_;
      if (initialized) {
        const double period = 1.0 / matching_rate_hz_;
        if (
          last_match_time_.nanoseconds() > 0 &&
          (now() - last_match_time_).seconds() < period)
        {
          return;
        }
        sample = *latest_scan_;
        map_to_odom = map_to_odom_;
      } else {
        if (
          initialization_scans_.size() <
          static_cast<std::size_t>(initial_scan_count_))
        {
          return;
        }
        if (
          last_global_attempt_time_.nanoseconds() > 0 &&
          (now() - last_global_attempt_time_).seconds() < global_retry_period_)
        {
          return;
        }
        sample = aggregateInitialScans(initialization_scans_);
        last_global_attempt_time_ = now();
        ++global_attempts_;
      }
      last_match_time_ = now();
    }

    MatchResult result;
    if (!initialized) {
      result = globalRegistration(sample);
    } else {
      setStatus("running low-rate NDT map correction");
      const Eigen::Isometry3d predicted_map_to_base =
        map_to_odom * sample.odom_to_base;
      result = refineNdt(sample.cloud_base, predicted_map_to_base);
      if (result.accepted) {
        const Eigen::Isometry3d correction =
          predicted_map_to_base.inverse() * result.map_to_base;
        if (
          correction.translation().norm() > maximum_translation_correction_ ||
          rotationAngle(correction.linear()) > maximum_rotation_correction_)
        {
          result.accepted = false;
          result.reason = "NDT correction jump exceeds the configured limit";
        }
      }
    }
    processMatch(sample, result, initialized);
  }

  void processMatch(
    const ScanSample & sample,
    const MatchResult & result,
    bool was_initialized)
  {
    bool publish_aligned = false;
    Eigen::Isometry3d aligned_pose = result.map_to_base;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      last_fitness_ = result.fitness;
      last_overlap_ = result.overlap;
      last_match_reason_ = result.reason;
      if (result.accepted) {
        const Eigen::Isometry3d candidate =
          result.map_to_base * sample.odom_to_base.inverse();
        if (!was_initialized) {
          map_to_odom_ = candidate;
          initialized_ = true;
          initialization_scans_.clear();
          consecutive_matches_ = 1;
          consecutive_failures_ = 0;
          setStatusLocked("global pose candidate found; validating with another scan");
        } else {
          map_to_odom_ =
            interpolateTransform(map_to_odom_, candidate, correction_alpha_);
          ++consecutive_matches_;
          consecutive_failures_ = 0;
          if (
            ready_ ||
            consecutive_matches_ >= required_consecutive_matches_)
          {
            setReadyLocked(true);
            setStatusLocked("automatic 3D localization healthy");
          } else {
            setStatusLocked("3D pose candidate accepted; validation pending");
          }
        }
        publish_aligned = true;
      } else {
        consecutive_matches_ = 0;
        ++consecutive_failures_;
        setStatusLocked(result.reason);
        if (!ready_) {
          initialized_ = false;
          initialization_scans_.clear();
        } else if (consecutive_failures_ >= runtime_failure_limit_) {
          setReadyLocked(false);
        }
        if (consecutive_failures_ >= relocalize_failure_limit_) {
          initialized_ = false;
          initialization_scans_.clear();
          setStatusLocked("map correction repeatedly failed; restarting global search");
        }
      }
    }

    std_msgs::msg::Float32 fitness;
    fitness.data = static_cast<float>(result.fitness);
    fitness_publisher_->publish(fitness);
    if (publish_aligned) {
      publishAlignedCloud(sample, aligned_pose);
    }
    if (!result.accepted) {
      RCLCPP_WARN(
        get_logger(), "3D localization rejected: %s (fitness=%.3f overlap=%.2f)",
        result.reason.c_str(), result.fitness, result.overlap);
    } else {
      RCLCPP_INFO(
        get_logger(), "3D localization accepted: fitness=%.3f overlap=%.2f",
        result.fitness, result.overlap);
    }
  }

  void handleRelocalize(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      initialized_ = false;
      setReadyLocked(false);
      consecutive_matches_ = 0;
      consecutive_failures_ = 0;
      initialization_scans_.clear();
      last_global_attempt_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      setStatusLocked("global relocalization requested");
    }
    response->success = true;
    response->message = "Automatic full-map relocalization restarted";
  }

  void setReadyLocked(bool ready)
  {
    if (ready_ == ready) {
      return;
    }
    ready_ = ready;
    publishReadyLocked();
    RCLCPP_WARN(
      get_logger(), "Localization readiness changed to %s",
      ready ? "READY" : "NOT READY");
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

  void publishReadyLocked()
  {
    std_msgs::msg::Bool message;
    message.data = ready_;
    ready_publisher_->publish(message);
  }

  void publishReady()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    publishReadyLocked();
  }

  void publishHeartbeat()
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    publishReadyLocked();
    std_msgs::msg::String message;
    message.data = status_;
    status_publisher_->publish(message);
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

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "agribot/pcd_global_localizer";
    diagnostic.hardware_id = map_file_path_;
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      if (ready_) {
        diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      } else if (initialized_) {
        diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      } else {
        diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      diagnostic.message = status_;
      diagnostic.values.push_back(
        keyValue("initialized", initialized_ ? "true" : "false"));
      diagnostic.values.push_back(keyValue("ready", ready_ ? "true" : "false"));
      diagnostic.values.push_back(
        keyValue("matching_rate_hz", std::to_string(matching_rate_hz_)));
      diagnostic.values.push_back(
        keyValue("fitness_score", std::to_string(last_fitness_)));
      diagnostic.values.push_back(
        keyValue("overlap_ratio", std::to_string(last_overlap_)));
      diagnostic.values.push_back(
        keyValue("global_attempts", std::to_string(global_attempts_)));
      diagnostic.values.push_back(
        keyValue("consecutive_matches", std::to_string(consecutive_matches_)));
      diagnostic.values.push_back(
        keyValue("consecutive_failures", std::to_string(consecutive_failures_)));
      diagnostic.values.push_back(keyValue("last_result", last_match_reason_));
    }
    array.status.push_back(std::move(diagnostic));
    diagnostics_publisher_->publish(array);
  }

  std::string map_file_path_;
  std::string cloud_topic_;
  std::string cloud_frame_;
  std::string odom_topic_;
  std::string pose_topic_;
  std::string ready_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  Eigen::Isometry3d base_to_body_{Eigen::Isometry3d::Identity()};

  double map_voxel_size_{0.25};
  double scan_voxel_size_{0.20};
  double feature_voxel_size_{0.35};
  double normal_radius_{0.70};
  double feature_radius_{1.10};
  double min_range_{0.50};
  double max_range_{30.0};
  double min_z_{-0.50};
  double max_z_{2.50};
  int min_scan_points_{250};
  int initial_scan_count_{3};
  double max_odom_age_{0.10};

  int global_max_iterations_{40000};
  int global_correspondence_randomness_{5};
  double global_similarity_threshold_{0.85};
  double global_max_correspondence_distance_{0.90};
  double global_inlier_fraction_{0.12};
  double global_retry_period_{5.0};

  double ndt_resolution_{0.80};
  double ndt_step_size_{0.10};
  double ndt_transformation_epsilon_{0.01};
  int ndt_max_iterations_{35};
  double fitness_max_range_{1.50};
  double max_fitness_score_{0.45};
  double overlap_distance_{0.50};
  double minimum_overlap_{0.20};
  double maximum_tilt_{0.35};
  double maximum_base_height_{0.50};

  double matching_rate_hz_{0.25};
  int required_consecutive_matches_{2};
  int runtime_failure_limit_{3};
  int relocalize_failure_limit_{5};
  double maximum_translation_correction_{0.75};
  double maximum_rotation_correction_{0.35};
  double correction_alpha_{0.25};

  PointCloud::Ptr registration_map_;
  PointCloud::Ptr feature_map_;
  FeatureCloud::Ptr map_features_;
  pcl::KdTreeFLANN<Point> map_tree_;
  Eigen::Vector3d map_minimum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d map_maximum_{Eigen::Vector3d::Zero()};

  std::mutex state_mutex_;
  std::deque<TimedPose> odometry_buffer_;
  std::deque<ScanSample> initialization_scans_;
  std::optional<ScanSample> latest_scan_;
  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  bool initialized_{false};
  bool ready_{false};
  int consecutive_matches_{0};
  int consecutive_failures_{0};
  int global_attempts_{0};
  double last_fitness_{std::numeric_limits<double>::infinity()};
  double last_overlap_{0.0};
  std::string status_{"starting"};
  std::string last_match_reason_{"no match attempted"};
  rclcpp::Time last_match_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_attempt_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    aligned_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr work_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace
}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node =
      std::make_shared<agribot_hardware_bringup::PcdGlobalLocalizer>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pcd_global_localizer"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
