#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "agribot_hardware_bringup/horizontal_antenna_factor.hpp"
#include "agribot_hardware_bringup/map_georeference.hpp"

namespace agribot_hardware_bringup::fusion
{

namespace
{

constexpr double kMinimumQuaternionNorm = 1.0e-9;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

gtsam::Rot3 rotationFromMessage(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm) {
    throw std::runtime_error("invalid zero-norm quaternion");
  }
  return gtsam::Rot3::Quaternion(
    quaternion.w / norm, quaternion.x / norm, quaternion.y / norm,
    quaternion.z / norm);
}

gtsam::Pose3 poseFromMessage(const geometry_msgs::msg::Pose & pose)
{
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z))
  {
    throw std::runtime_error("pose contains non-finite position");
  }
  return gtsam::Pose3(
    rotationFromMessage(pose.orientation),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose poseMessage(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose message;
  const gtsam::Point3 translation = pose.translation();
  const Eigen::Quaterniond quaternion(pose.rotation().matrix());
  message.position.x = translation.x();
  message.position.y = translation.y();
  message.position.z = translation.z();
  message.orientation.w = quaternion.w();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  return message;
}

gtsam::Rot3 rotationFromRpy(const std::vector<double> & rpy)
{
  if (rpy.size() != 3U) {
    throw std::runtime_error("RPY parameter must contain three values");
  }
  return gtsam::Rot3::RzRyRx(rpy[0], rpy[1], rpy[2]);
}

gtsam::Pose3 poseFromEigen(const Eigen::Isometry3d & transform)
{
  return gtsam::Pose3(
    gtsam::Rot3(transform.linear()),
    gtsam::Point3(transform.translation()));
}

diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class FastLivoRtkFusionNode final : public rclcpp::Node
{
public:
  FastLivoRtkFusionNode()
  : Node("fastlivo_rtk_fusion")
  {
    declareAndLoadParameters();
    loadGeoreference();
    configureNoiseModels();
    createRosInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "FAST-LIVO2/RTK fixed-lag fusion ready: odom=%s fix=%s quality=%s lag=%.1fs",
      odom_topic_.c_str(), fix_topic_.c_str(), quality_topic_.c_str(), fixed_lag_sec_);
    RCLCPP_INFO(
      get_logger(),
      "RTK policy: quality=%d only, XY only, base-to-master-antenna=[%.4f %.4f %.4f] m",
      required_fix_quality_, base_to_antenna_.x(), base_to_antenna_.y(),
      base_to_antenna_.z());
  }

private:
  struct OdomSample
  {
    double stamp_sec{0.0};
    builtin_interfaces::msg::Time stamp;
    gtsam::Pose3 odom_from_base;
    geometry_msgs::msg::TwistWithCovariance twist;
  };

  struct GravitySample
  {
    double stamp_sec{0.0};
    gtsam::Vector3 up_in_base{0.0, 0.0, 1.0};
  };

  struct FixedRtkSample
  {
    std::uint64_t sequence{0U};
    double stamp_sec{0.0};
    gtsam::Point3 antenna_in_map{0.0, 0.0, 0.0};
    double sigma_x_m{0.1};
    double sigma_y_m{0.1};
  };

  struct Keyframe
  {
    gtsam::Key key{0U};
    double stamp_sec{0.0};
    gtsam::Pose3 odom_from_base;
    bool has_rtk_factor{false};
  };

  void declareAndLoadParameters()
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/fastlivo/odometry");
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/rtk/fix");
    quality_topic_ = declare_parameter<std::string>(
      "quality_topic", "/rtk/fix_quality");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    seed_pose_topic_ = declare_parameter<std::string>(
      "seed_pose_topic", "/localization_pose");
    fused_odom_topic_ = declare_parameter<std::string>(
      "fused_odom_topic", "/fastlivo_rtk/odometry");
    fused_pose_topic_ = declare_parameter<std::string>(
      "fused_pose_topic", "/fastlivo_rtk/pose");
    fused_path_topic_ = declare_parameter<std::string>(
      "fused_path_topic", "/fastlivo_rtk/path");
    local_path_topic_ = declare_parameter<std::string>(
      "local_path_topic", "/fastlivo_rtk/fastlivo_path");
    rtk_path_topic_ = declare_parameter<std::string>(
      "rtk_path_topic", "/fastlivo_rtk/fixed_rtk_path");
    ready_topic_ = declare_parameter<std::string>(
      "ready_topic", "/fastlivo_rtk/ready");
    fixed_active_topic_ = declare_parameter<std::string>(
      "fixed_active_topic", "/fastlivo_rtk/fixed_active");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    georeference_file_ = declare_parameter<std::string>("georeference_file", "");
    map_file_ = declare_parameter<std::string>("map_file", "");

    publish_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", true);
    auto_initialize_from_fixed_rtk_ = declare_parameter<bool>(
      "auto_initialize_from_fixed_rtk", true);
    initial_map_from_odom_yaw_rad_ = declare_parameter<double>(
      "initial_map_from_odom_yaw_rad", 0.0);
    initial_base_z_m_ = declare_parameter<double>("initial_base_z_m", 0.0);
    required_fix_quality_ = declare_parameter<int>("required_fix_quality", 4);
    required_consecutive_fixed_ = declare_parameter<int>(
      "required_consecutive_fixed", 3);
    quality_timeout_sec_ = declare_parameter<double>("quality_timeout_sec", 0.30);
    rtk_sync_tolerance_sec_ = declare_parameter<double>(
      "rtk_sync_tolerance_sec", 0.15);
    imu_sync_tolerance_sec_ = declare_parameter<double>(
      "imu_sync_tolerance_sec", 0.08);
    seed_sync_tolerance_sec_ = declare_parameter<double>(
      "seed_sync_tolerance_sec", 0.30);
    fixed_lag_sec_ = declare_parameter<double>("fixed_lag_sec", 60.0);
    keyframe_period_sec_ = declare_parameter<double>("keyframe_period_sec", 0.50);
    keyframe_distance_m_ = declare_parameter<double>("keyframe_distance_m", 0.30);
    keyframe_angle_rad_ = declare_parameter<double>(
      "keyframe_angle_rad", 0.08726646259971647);
    rtk_factor_min_distance_m_ = declare_parameter<double>(
      "rtk_factor_min_distance_m", 0.30);
    rtk_max_innovation_m_ = declare_parameter<double>(
      "rtk_max_innovation_m", 1.50);
    rtk_horizontal_sigma_floor_m_ = declare_parameter<double>(
      "rtk_horizontal_sigma_floor_m", 0.10);
    rtk_huber_delta_ = declare_parameter<double>("rtk_huber_delta", 1.345);
    gravity_sigma_rad_ = declare_parameter<double>(
      "gravity_sigma_rad", 0.017453292519943295);
    gravity_huber_delta_ = declare_parameter<double>(
      "gravity_huber_delta", 1.345);

    odom_rotation_sigma_rad_ = declare_parameter<double>(
      "odom_rotation_sigma_rad", 0.008726646259971648);
    odom_yaw_sigma_rad_ = declare_parameter<double>(
      "odom_yaw_sigma_rad", 0.017453292519943295);
    odom_horizontal_sigma_m_ = declare_parameter<double>(
      "odom_horizontal_sigma_m", 0.03);
    odom_vertical_sigma_m_ = declare_parameter<double>(
      "odom_vertical_sigma_m", 0.05);
    seed_rotation_sigma_rad_ = declare_parameter<double>(
      "seed_rotation_sigma_rad", 0.08726646259971647);
    seed_position_sigma_m_ = declare_parameter<double>("seed_position_sigma_m", 0.20);
    automatic_yaw_sigma_rad_ = declare_parameter<double>(
      "automatic_yaw_sigma_rad", 0.7853981633974483);
    automatic_position_sigma_m_ = declare_parameter<double>(
      "automatic_position_sigma_m", 1.0);
    initial_roll_pitch_sigma_rad_ = declare_parameter<double>(
      "initial_roll_pitch_sigma_rad", 0.008726646259971648);
    initial_z_sigma_m_ = declare_parameter<double>("initial_z_sigma_m", 0.10);
    correction_translation_rate_mps_ = declare_parameter<double>(
      "correction_translation_rate_mps", 0.50);
    correction_rotation_rate_radps_ = declare_parameter<double>(
      "correction_rotation_rate_radps", 0.20);
    fixed_recent_timeout_sec_ = declare_parameter<double>(
      "fixed_recent_timeout_sec", 0.50);
    path_max_poses_ = declare_parameter<int>("path_max_poses", 6000);
    path_sample_period_sec_ = declare_parameter<double>(
      "path_sample_period_sec", 0.10);

    const std::vector<double> antenna = declare_parameter<std::vector<double>>(
      "base_to_antenna_xyz", {0.1425, 0.2952585, 0.78476});
    if (antenna.size() != 3U) {
      throw std::runtime_error("base_to_antenna_xyz must contain three values");
    }
    base_to_antenna_ = gtsam::Point3(antenna[0], antenna[1], antenna[2]);
    base_from_imu_ = rotationFromRpy(declare_parameter<std::vector<double>>(
      "base_from_imu_rpy", {0.000572424, -0.009139547, -0.000002616}));

    const std::array<double, 24> positive_parameters{
      quality_timeout_sec_, rtk_sync_tolerance_sec_, imu_sync_tolerance_sec_,
      seed_sync_tolerance_sec_, fixed_lag_sec_, keyframe_period_sec_,
      keyframe_distance_m_, keyframe_angle_rad_, rtk_factor_min_distance_m_,
      rtk_max_innovation_m_, rtk_horizontal_sigma_floor_m_, rtk_huber_delta_,
      gravity_sigma_rad_, gravity_huber_delta_, odom_rotation_sigma_rad_,
      odom_yaw_sigma_rad_, odom_horizontal_sigma_m_, odom_vertical_sigma_m_,
      seed_rotation_sigma_rad_, seed_position_sigma_m_, automatic_yaw_sigma_rad_,
      automatic_position_sigma_m_, initial_roll_pitch_sigma_rad_, initial_z_sigma_m_};
    if (required_fix_quality_ < 1 || required_consecutive_fixed_ < 1 ||
      path_max_poses_ < 2 || path_sample_period_sec_ <= 0.0 ||
      correction_translation_rate_mps_ <= 0.0 || correction_rotation_rate_radps_ <= 0.0 ||
      fixed_recent_timeout_sec_ <= 0.0 ||
      !std::all_of(
        positive_parameters.begin(), positive_parameters.end(),
        [](const double value) {return std::isfinite(value) && value > 0.0;}))
    {
      throw std::runtime_error("FAST-LIVO2/RTK fusion parameters are invalid");
    }
    if (georeference_file_.empty()) {
      throw std::runtime_error("georeference_file is required");
    }
  }

  void loadGeoreference()
  {
    const auto georeference = navsat::loadMapGeoreference(
      std::filesystem::path(georeference_file_));
    if (!map_file_.empty()) {
      const std::string fingerprint = navsat::fingerprintFile(
        std::filesystem::path(map_file_));
      if (fingerprint != georeference.map_fingerprint) {
        throw std::runtime_error(
                "map PCD fingerprint does not match the georeference");
      }
    }
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
      georeference.reference_latitude_deg,
      georeference.reference_longitude_deg,
      georeference.reference_altitude_m);
    map_from_enu_ = poseFromEigen(navsat::mapFromEnuTransform(georeference));
    georeference_horizontal_rmse_m_ = georeference.horizontal_rmse_m;
    RCLCPP_INFO(
      get_logger(),
      "Loaded map georeference '%s': horizontal RMSE %.3f m, %zu samples",
      georeference.map_id.c_str(), georeference.horizontal_rmse_m,
      georeference.sample_count);
  }

  void configureNoiseModels()
  {
    gtsam::Vector6 odom_sigmas;
    odom_sigmas << odom_rotation_sigma_rad_, odom_rotation_sigma_rad_,
      odom_yaw_sigma_rad_, odom_horizontal_sigma_m_, odom_horizontal_sigma_m_,
      odom_vertical_sigma_m_;
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

    const auto gravity_gaussian = gtsam::noiseModel::Isotropic::Sigma(
      2, gravity_sigma_rad_);
    gravity_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(gravity_huber_delta_),
      gravity_gaussian);
  }

  void createRosInterfaces()
  {
    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(100);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&FastLivoRtkFusionNode::handleOdom, this, std::placeholders::_1));
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, sensor_qos,
      std::bind(&FastLivoRtkFusionNode::handleFix, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&FastLivoRtkFusionNode::handleImu, this, std::placeholders::_1));
    quality_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      quality_topic_, rclcpp::QoS(20),
      std::bind(&FastLivoRtkFusionNode::handleQuality, this, std::placeholders::_1));
    seed_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      seed_pose_topic_, rclcpp::QoS(10),
      std::bind(&FastLivoRtkFusionNode::handleSeedPose, this, std::placeholders::_1));

    fused_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      fused_odom_topic_, rclcpp::QoS(20));
    fused_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      fused_pose_topic_, rclcpp::QoS(20));
    auto path_qos = rclcpp::QoS(1).transient_local().reliable();
    fused_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      fused_path_topic_, path_qos);
    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic_, path_qos);
    rtk_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      rtk_path_topic_, path_qos);
    auto state_qos = rclcpp::QoS(1).transient_local().reliable();
    ready_publisher_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, state_qos);
    fixed_active_publisher_ = create_publisher<std_msgs::msg::Bool>(
      fixed_active_topic_, state_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&FastLivoRtkFusionNode::publishDiagnostics, this));

    fused_path_.header.frame_id = map_frame_;
    local_path_.header.frame_id = map_frame_;
    rtk_path_.header.frame_id = map_frame_;
    publishStateFlags();
  }

  void handleQuality(const std_msgs::msg::UInt8::SharedPtr message)
  {
    latest_quality_ = static_cast<int>(message->data);
    latest_quality_receipt_sec_ = now().seconds();
    if (latest_quality_ == required_fix_quality_) {
      ++consecutive_fixed_count_;
    } else {
      consecutive_fixed_count_ = 0;
    }
    publishStateFlags();
  }

  void handleImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (message->orientation_covariance[0] < 0.0) {
      ++rejected_gravity_count_;
      return;
    }
    try {
      const gtsam::Rot3 world_from_imu = rotationFromMessage(message->orientation);
      const gtsam::Rot3 world_from_base = world_from_imu.compose(base_from_imu_.inverse());
      const gtsam::Vector3 up_in_base =
        world_from_base.unrotate(gtsam::Vector3(0.0, 0.0, 1.0)).normalized();
      const double stamp_sec = messageStampOrNow(message->header.stamp);
      gravity_buffer_.push_back({stamp_sec, up_in_base});
      while (!gravity_buffer_.empty() &&
        gravity_buffer_.front().stamp_sec < stamp_sec - 2.0)
      {
        gravity_buffer_.pop_front();
      }
    } catch (const std::exception & error) {
      ++rejected_gravity_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Ignoring IMU attitude: %s", error.what());
    }
  }

  void handleSeedPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring seed pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    if (initialized_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Fusion is already initialized; additional seed poses are ignored");
      return;
    }
    try {
      pending_seed_map_from_base_ = poseFromMessage(message->pose.pose);
      pending_seed_stamp_sec_ = messageStampOrNow(message->header.stamp);
      initialization_status_ = "map pose seed received; waiting for synchronized FAST-LIVO2 odometry";
      tryInitializeFromSeed();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid seed pose: %s", error.what());
    }
  }

  void handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (!fixedQualityAvailable() ||
      message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude))
    {
      ++rejected_nonfixed_count_;
      publishStateFlags();
      return;
    }

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_->Forward(
      message->latitude, message->longitude, message->altitude,
      east, north, up);
    const gtsam::Point3 antenna_in_map = map_from_enu_.transformFrom(
      gtsam::Point3(east, north, up));

    const auto covarianceSigma = [this](const double variance) {
        if (!std::isfinite(variance) || variance <= 0.0) {
          return rtk_horizontal_sigma_floor_m_;
        }
        return std::max(std::sqrt(variance), rtk_horizontal_sigma_floor_m_);
      };
    FixedRtkSample sample;
    sample.sequence = ++rtk_sequence_;
    sample.stamp_sec = messageStampOrNow(message->header.stamp);
    sample.antenna_in_map = antenna_in_map;
    sample.sigma_x_m = covarianceSigma(message->position_covariance[0]);
    sample.sigma_y_m = covarianceSigma(message->position_covariance[4]);
    latest_fixed_rtk_ = sample;
    last_fixed_fix_receipt_sec_ = now().seconds();

    if (!initialized_) {
      tryInitializeAutomatically();
    }
    if (initialized_) {
      tryAddRtkFactor(sample);
    }
    publishStateFlags();
  }

  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    try {
      OdomSample sample;
      sample.stamp_sec = messageStampOrNow(message->header.stamp);
      sample.stamp = message->header.stamp;
      if (stampSeconds(sample.stamp) <= 0.0) {
        sample.stamp = now();
      }
      sample.odom_from_base = poseFromMessage(message->pose.pose);
      sample.twist = message->twist;
      latest_odom_ = sample;

      if (!initialized_) {
        tryInitializeFromSeed();
        tryInitializeAutomatically();
      } else if (shouldCreateKeyframe(sample)) {
        addKeyframe(sample);
      }

      if (initialized_) {
        if (latest_fixed_rtk_.has_value() &&
          latest_fixed_rtk_->sequence != last_consumed_rtk_sequence_)
        {
          tryAddRtkFactor(*latest_fixed_rtk_);
        }
        publishCurrentEstimate(sample);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FAST-LIVO2/RTK fusion rejected odometry: %s", error.what());
    }
  }

  bool fixedQualityAvailable() const
  {
    if (latest_quality_ != required_fix_quality_ ||
      consecutive_fixed_count_ < required_consecutive_fixed_ ||
      !latest_quality_receipt_sec_.has_value())
    {
      return false;
    }
    const double age = now().seconds() - *latest_quality_receipt_sec_;
    return age >= 0.0 && age <= quality_timeout_sec_;
  }

  bool fixedRecentlyActive() const
  {
    if (!fixedQualityAvailable() || !last_fixed_fix_receipt_sec_.has_value()) {
      return false;
    }
    const double age = now().seconds() - *last_fixed_fix_receipt_sec_;
    return age >= 0.0 && age <= fixed_recent_timeout_sec_;
  }

  double messageStampOrNow(const builtin_interfaces::msg::Time & stamp) const
  {
    const double value = stampSeconds(stamp);
    return value > 0.0 ? value : now().seconds();
  }

  void tryInitializeFromSeed()
  {
    if (initialized_ || !pending_seed_map_from_base_.has_value() ||
      !pending_seed_stamp_sec_.has_value() || !latest_odom_.has_value())
    {
      return;
    }
    const double offset = std::abs(
      latest_odom_->stamp_sec - *pending_seed_stamp_sec_);
    if (offset > seed_sync_tolerance_sec_) {
      initialization_status_ = "map pose seed is not synchronized with FAST-LIVO2 odometry";
      return;
    }
    initializeGraph(*pending_seed_map_from_base_, *latest_odom_, false);
    pending_seed_map_from_base_.reset();
    pending_seed_stamp_sec_.reset();
  }

  void tryInitializeAutomatically()
  {
    if (initialized_ || !auto_initialize_from_fixed_rtk_ ||
      !latest_odom_.has_value() || !latest_fixed_rtk_.has_value())
    {
      return;
    }
    if (std::abs(
        latest_odom_->stamp_sec - latest_fixed_rtk_->stamp_sec) >
      rtk_sync_tolerance_sec_)
    {
      initialization_status_ = "waiting for synchronized fixed RTK and FAST-LIVO2 odometry";
      return;
    }

    const gtsam::Rot3 map_from_odom_rotation = gtsam::Rot3::Rz(
      initial_map_from_odom_yaw_rad_);
    const gtsam::Rot3 map_from_base_rotation = map_from_odom_rotation.compose(
      latest_odom_->odom_from_base.rotation());
    const gtsam::Point3 rotated_lever = map_from_base_rotation.rotate(base_to_antenna_);
    const gtsam::Point3 base_translation(
      latest_fixed_rtk_->antenna_in_map.x() - rotated_lever.x(),
      latest_fixed_rtk_->antenna_in_map.y() - rotated_lever.y(),
      initial_base_z_m_);
    initializeGraph(
      gtsam::Pose3(map_from_base_rotation, base_translation), *latest_odom_, true);
  }

  void initializeGraph(
    const gtsam::Pose3 & map_from_base,
    const OdomSample & odom_sample,
    const bool automatic_seed)
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = 1;
    parameters.findUnusedFactorSlots = true;
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
      fixed_lag_sec_, parameters);

    const gtsam::Key key = gtsam::Symbol('x', 0U);
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    gtsam::Vector6 prior_sigmas;
    const double yaw_sigma = automatic_seed ?
      automatic_yaw_sigma_rad_ : seed_rotation_sigma_rad_;
    const double horizontal_sigma = automatic_seed ?
      automatic_position_sigma_m_ : seed_position_sigma_m_;
    prior_sigmas << initial_roll_pitch_sigma_rad_, initial_roll_pitch_sigma_rad_,
      yaw_sigma, horizontal_sigma, horizontal_sigma, initial_z_sigma_m_;
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        key, map_from_base, gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
    addGravityFactor(factors, key, odom_sample.stamp_sec);
    values.insert(key, map_from_base);
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
    timestamps[key] = odom_sample.stamp_sec;

    const auto started = std::chrono::steady_clock::now();
    smoother_->update(factors, values, timestamps);
    last_optimization_ms_ = millisecondsSince(started);
    latest_key_ = key;
    next_key_index_ = 1U;
    keyframes_.clear();
    keyframes_.push_back({key, odom_sample.stamp_sec, odom_sample.odom_from_base, false});
    latest_optimized_map_from_base_ = smoother_->calculateEstimate<gtsam::Pose3>(key);
    target_map_from_odom_ = latest_optimized_map_from_base_.compose(
      odom_sample.odom_from_base.inverse());
    applied_map_from_odom_ = target_map_from_odom_;
    initial_map_from_odom_ = target_map_from_odom_;
    applied_correction_stamp_sec_ = odom_sample.stamp_sec;
    initialized_ = true;
    initialization_status_ = automatic_seed ?
      "initialized from synchronized fixed RTK with weak yaw prior" :
      "initialized from 3D map pose seed";
    updateMarginalCovariance();
    publishStateFlags();
    RCLCPP_INFO(
      get_logger(), "%s at map position [%.3f %.3f %.3f]",
      initialization_status_.c_str(), map_from_base.x(), map_from_base.y(), map_from_base.z());
  }

  bool shouldCreateKeyframe(const OdomSample & sample) const
  {
    if (keyframes_.empty()) {
      return true;
    }
    const Keyframe & previous = keyframes_.back();
    const gtsam::Pose3 increment = previous.odom_from_base.between(sample.odom_from_base);
    return sample.stamp_sec - previous.stamp_sec >= keyframe_period_sec_ ||
           increment.translation().norm() >= keyframe_distance_m_ ||
           gtsam::Rot3::Logmap(increment.rotation()).norm() >= keyframe_angle_rad_;
  }

  void addKeyframe(const OdomSample & sample)
  {
    if (!smoother_ || keyframes_.empty()) {
      throw std::runtime_error("fixed-lag smoother is not initialized");
    }
    const Keyframe & previous = keyframes_.back();
    const gtsam::Pose3 odom_increment = previous.odom_from_base.between(
      sample.odom_from_base);
    const gtsam::Pose3 initial_pose = latest_optimized_map_from_base_.compose(
      odom_increment);
    const gtsam::Key key = gtsam::Symbol('x', next_key_index_++);

    gtsam::NonlinearFactorGraph factors;
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        previous.key, key, odom_increment, odom_noise_));
    addGravityFactor(factors, key, sample.stamp_sec);
    gtsam::Values values;
    values.insert(key, initial_pose);
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
    timestamps[key] = sample.stamp_sec;

    const auto started = std::chrono::steady_clock::now();
    smoother_->update(factors, values, timestamps);
    last_optimization_ms_ = millisecondsSince(started);
    latest_key_ = key;
    keyframes_.push_back({key, sample.stamp_sec, sample.odom_from_base, false});
    pruneKeyframeIndex(sample.stamp_sec);
    refreshOptimizedCorrection();
  }

  void addGravityFactor(
    gtsam::NonlinearFactorGraph & factors,
    const gtsam::Key key,
    const double stamp_sec)
  {
    const auto gravity = nearestGravity(stamp_sec);
    if (!gravity.has_value()) {
      ++rejected_gravity_count_;
      return;
    }
    factors.add(gtsam::Pose3AttitudeFactor(
        key, gtsam::Unit3(0.0, 0.0, 1.0), gravity_noise_,
        gtsam::Unit3(*gravity)));
    ++accepted_gravity_count_;
  }

  std::optional<gtsam::Vector3> nearestGravity(const double stamp_sec) const
  {
    if (gravity_buffer_.empty()) {
      return std::nullopt;
    }
    const GravitySample * nearest = nullptr;
    double nearest_offset = std::numeric_limits<double>::infinity();
    for (auto iterator = gravity_buffer_.rbegin(); iterator != gravity_buffer_.rend(); ++iterator) {
      const double offset = std::abs(iterator->stamp_sec - stamp_sec);
      if (offset < nearest_offset) {
        nearest = &*iterator;
        nearest_offset = offset;
      }
      if (iterator->stamp_sec < stamp_sec - imu_sync_tolerance_sec_) {
        break;
      }
    }
    if (nearest == nullptr || nearest_offset > imu_sync_tolerance_sec_) {
      return std::nullopt;
    }
    return nearest->up_in_base;
  }

  void tryAddRtkFactor(const FixedRtkSample & sample)
  {
    if (!smoother_ || keyframes_.empty() || sample.sequence == last_consumed_rtk_sequence_) {
      return;
    }
    auto nearest = keyframes_.end();
    double nearest_offset = std::numeric_limits<double>::infinity();
    for (auto iterator = keyframes_.begin(); iterator != keyframes_.end(); ++iterator) {
      const double offset = std::abs(iterator->stamp_sec - sample.stamp_sec);
      if (offset < nearest_offset) {
        nearest = iterator;
        nearest_offset = offset;
      }
    }
    if (nearest == keyframes_.end() || nearest_offset > rtk_sync_tolerance_sec_) {
      return;
    }
    if (nearest->has_rtk_factor) {
      last_consumed_rtk_sequence_ = sample.sequence;
      return;
    }
    if (last_accepted_rtk_antenna_.has_value()) {
      const gtsam::Vector2 displacement(
        sample.antenna_in_map.x() - last_accepted_rtk_antenna_->x(),
        sample.antenna_in_map.y() - last_accepted_rtk_antenna_->y());
      if (displacement.norm() < rtk_factor_min_distance_m_) {
        last_consumed_rtk_sequence_ = sample.sequence;
        return;
      }
    }

    const gtsam::Pose3 estimate = smoother_->calculateEstimate<gtsam::Pose3>(nearest->key);
    const gtsam::Point3 predicted_antenna = estimate.transformFrom(base_to_antenna_);
    last_rtk_innovation_m_ = std::hypot(
      predicted_antenna.x() - sample.antenna_in_map.x(),
      predicted_antenna.y() - sample.antenna_in_map.y());
    if (last_rtk_innovation_m_ > rtk_max_innovation_m_) {
      ++rejected_innovation_count_;
      last_consumed_rtk_sequence_ = sample.sequence;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting fixed RTK XY innovation %.3f m above %.3f m",
        last_rtk_innovation_m_, rtk_max_innovation_m_);
      return;
    }

    gtsam::Vector2 sigmas;
    sigmas << sample.sigma_x_m, sample.sigma_y_m;
    const auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(rtk_huber_delta_), gaussian);
    gtsam::NonlinearFactorGraph factors;
    factors.add(boost::make_shared<HorizontalAntennaFactor>(
        nearest->key, sample.antenna_in_map, base_to_antenna_, robust));

    const auto started = std::chrono::steady_clock::now();
    smoother_->update(factors);
    last_optimization_ms_ = millisecondsSince(started);
    nearest->has_rtk_factor = true;
    last_consumed_rtk_sequence_ = sample.sequence;
    last_accepted_rtk_antenna_ = sample.antenna_in_map;
    ++accepted_rtk_count_;
    refreshOptimizedCorrection();
    appendAcceptedRtkPose(sample, smoother_->calculateEstimate<gtsam::Pose3>(nearest->key));
  }

  void refreshOptimizedCorrection()
  {
    if (!smoother_ || keyframes_.empty()) {
      return;
    }
    latest_optimized_map_from_base_ =
      smoother_->calculateEstimate<gtsam::Pose3>(latest_key_);
    target_map_from_odom_ = latest_optimized_map_from_base_.compose(
      keyframes_.back().odom_from_base.inverse());
    updateMarginalCovariance();
  }

  void updateMarginalCovariance()
  {
    try {
      latest_pose_covariance_ = smoother_->marginalCovariance(latest_key_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not calculate fused-pose covariance: %s", error.what());
    }
  }

  void pruneKeyframeIndex(const double current_stamp_sec)
  {
    const double oldest_allowed = current_stamp_sec - fixed_lag_sec_ + 1.0;
    while (keyframes_.size() > 1U && keyframes_.front().stamp_sec < oldest_allowed) {
      keyframes_.pop_front();
    }
  }

  void updateAppliedCorrection(const double stamp_sec)
  {
    if (!applied_correction_stamp_sec_.has_value()) {
      applied_map_from_odom_ = target_map_from_odom_;
      applied_correction_stamp_sec_ = stamp_sec;
      return;
    }
    const double dt = std::clamp(
      stamp_sec - *applied_correction_stamp_sec_, 0.0, 0.25);
    applied_correction_stamp_sec_ = stamp_sec;
    if (dt <= 0.0) {
      return;
    }

    gtsam::Vector3 translation_delta =
      target_map_from_odom_.translation() - applied_map_from_odom_.translation();
    const double maximum_translation = correction_translation_rate_mps_ * dt;
    if (translation_delta.norm() > maximum_translation) {
      translation_delta *= maximum_translation / translation_delta.norm();
    }
    const gtsam::Point3 translation =
      applied_map_from_odom_.translation() + translation_delta;

    gtsam::Vector3 rotation_delta = gtsam::Rot3::Logmap(
      applied_map_from_odom_.rotation().between(target_map_from_odom_.rotation()));
    const double maximum_rotation = correction_rotation_rate_radps_ * dt;
    if (rotation_delta.norm() > maximum_rotation) {
      rotation_delta *= maximum_rotation / rotation_delta.norm();
    }
    const gtsam::Rot3 rotation = applied_map_from_odom_.rotation().compose(
      gtsam::Rot3::Expmap(rotation_delta));
    applied_map_from_odom_ = gtsam::Pose3(rotation, translation);
  }

  void publishCurrentEstimate(const OdomSample & sample)
  {
    updateAppliedCorrection(sample.stamp_sec);
    const gtsam::Pose3 map_from_base = applied_map_from_odom_.compose(
      sample.odom_from_base);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = sample.stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose = poseMessage(map_from_base);
    odometry.twist = sample.twist;
    fillRosCovariance(odometry.pose.covariance);
    fused_odom_publisher_->publish(odometry);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose;
    fused_pose_publisher_->publish(pose);

    if (!last_path_stamp_sec_.has_value() ||
      sample.stamp_sec - *last_path_stamp_sec_ >= path_sample_period_sec_)
    {
      appendPathPose(fused_path_, sample.stamp, map_from_base);
      appendPathPose(
        local_path_, sample.stamp,
        initial_map_from_odom_.compose(sample.odom_from_base));
      last_path_stamp_sec_ = sample.stamp_sec;
      fused_path_publisher_->publish(fused_path_);
      local_path_publisher_->publish(local_path_);
    }

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = sample.stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = odom_frame_;
      const auto correction_pose = poseMessage(applied_map_from_odom_);
      transform.transform.translation.x = correction_pose.position.x;
      transform.transform.translation.y = correction_pose.position.y;
      transform.transform.translation.z = correction_pose.position.z;
      transform.transform.rotation = correction_pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void appendAcceptedRtkPose(
    const FixedRtkSample & sample, const gtsam::Pose3 & optimized_pose)
  {
    const gtsam::Point3 rotated_lever = optimized_pose.rotation().rotate(base_to_antenna_);
    const gtsam::Point3 base_position(
      sample.antenna_in_map.x() - rotated_lever.x(),
      sample.antenna_in_map.y() - rotated_lever.y(),
      optimized_pose.z());
    appendPathPose(
      rtk_path_, rclcpp::Time(
        static_cast<int64_t>(sample.stamp_sec * 1.0e9)),
      gtsam::Pose3(optimized_pose.rotation(), base_position));
    rtk_path_publisher_->publish(rtk_path_);
  }

  void appendPathPose(
    nav_msgs::msg::Path & path,
    const builtin_interfaces::msg::Time & stamp,
    const gtsam::Pose3 & pose)
  {
    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.stamp = stamp;
    stamped_pose.header.frame_id = map_frame_;
    stamped_pose.pose = poseMessage(pose);
    path.header.stamp = stamp;
    path.poses.push_back(stamped_pose);
    if (path.poses.size() > static_cast<std::size_t>(path_max_poses_)) {
      path.poses.erase(
        path.poses.begin(),
        path.poses.begin() + static_cast<std::ptrdiff_t>(
          path.poses.size() - static_cast<std::size_t>(path_max_poses_)));
    }
  }

  void fillRosCovariance(std::array<double, 36> & covariance) const
  {
    covariance.fill(0.0);
    static constexpr std::array<int, 6> ros_to_gtsam{3, 4, 5, 0, 1, 2};
    if (latest_pose_covariance_.rows() != 6 || latest_pose_covariance_.cols() != 6) {
      return;
    }
    for (std::size_t row = 0; row < 6U; ++row) {
      for (std::size_t column = 0; column < 6U; ++column) {
        covariance[row * 6U + column] = latest_pose_covariance_(
          ros_to_gtsam[row], ros_to_gtsam[column]);
      }
    }
  }

  void publishStateFlags()
  {
    ready_publisher_->publish(std_msgs::msg::Bool().set__data(initialized_));
    fixed_active_publisher_->publish(
      std_msgs::msg::Bool().set__data(fixedRecentlyActive()));
  }

  void publishDiagnostics()
  {
    publishStateFlags();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "agribot/fastlivo_rtk_fusion";
    status.hardware_id = "RDK-X5";
    if (!initialized_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = initialization_status_;
    } else if (!fixedRecentlyActive()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "FAST-LIVO2 propagation; no recent RTK quality-4 position";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "FAST-LIVO2 with fixed RTK XY and gravity constraints";
    }
    status.values.push_back(diagnosticValue("initialized", initialized_ ? "true" : "false"));
    status.values.push_back(diagnosticValue("latest_fix_quality", std::to_string(latest_quality_)));
    status.values.push_back(diagnosticValue(
        "consecutive_fixed", std::to_string(consecutive_fixed_count_)));
    status.values.push_back(diagnosticValue(
        "fixed_rtk_factors", std::to_string(accepted_rtk_count_)));
    status.values.push_back(diagnosticValue(
        "rejected_nonfixed", std::to_string(rejected_nonfixed_count_)));
    status.values.push_back(diagnosticValue(
        "rejected_innovation", std::to_string(rejected_innovation_count_)));
    status.values.push_back(diagnosticValue(
        "gravity_factors", std::to_string(accepted_gravity_count_)));
    status.values.push_back(diagnosticValue(
        "missing_gravity", std::to_string(rejected_gravity_count_)));
    status.values.push_back(diagnosticValue(
        "active_keyframes", std::to_string(keyframes_.size())));
    status.values.push_back(diagnosticValue(
        "last_rtk_innovation_m", std::to_string(last_rtk_innovation_m_)));
    status.values.push_back(diagnosticValue(
        "last_optimization_ms", std::to_string(last_optimization_ms_)));
    status.values.push_back(diagnosticValue(
        "georeference_horizontal_rmse_m", std::to_string(georeference_horizontal_rmse_m_)));
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  static double millisecondsSince(
    const std::chrono::steady_clock::time_point & started)
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  }

  std::string odom_topic_;
  std::string fix_topic_;
  std::string quality_topic_;
  std::string imu_topic_;
  std::string seed_pose_topic_;
  std::string fused_odom_topic_;
  std::string fused_pose_topic_;
  std::string fused_path_topic_;
  std::string local_path_topic_;
  std::string rtk_path_topic_;
  std::string ready_topic_;
  std::string fixed_active_topic_;
  std::string diagnostics_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string georeference_file_;
  std::string map_file_;
  std::string initialization_status_{"waiting for a map pose seed or fixed RTK"};

  bool publish_tf_{true};
  bool auto_initialize_from_fixed_rtk_{true};
  int required_fix_quality_{4};
  int required_consecutive_fixed_{3};
  int path_max_poses_{6000};
  double initial_map_from_odom_yaw_rad_{0.0};
  double initial_base_z_m_{0.0};
  double quality_timeout_sec_{0.30};
  double rtk_sync_tolerance_sec_{0.15};
  double imu_sync_tolerance_sec_{0.08};
  double seed_sync_tolerance_sec_{0.30};
  double fixed_lag_sec_{60.0};
  double keyframe_period_sec_{0.50};
  double keyframe_distance_m_{0.30};
  double keyframe_angle_rad_{0.08726646259971647};
  double rtk_factor_min_distance_m_{0.30};
  double rtk_max_innovation_m_{1.50};
  double rtk_horizontal_sigma_floor_m_{0.10};
  double rtk_huber_delta_{1.345};
  double gravity_sigma_rad_{0.017453292519943295};
  double gravity_huber_delta_{1.345};
  double odom_rotation_sigma_rad_{0.008726646259971648};
  double odom_yaw_sigma_rad_{0.017453292519943295};
  double odom_horizontal_sigma_m_{0.03};
  double odom_vertical_sigma_m_{0.05};
  double seed_rotation_sigma_rad_{0.08726646259971647};
  double seed_position_sigma_m_{0.20};
  double automatic_yaw_sigma_rad_{0.7853981633974483};
  double automatic_position_sigma_m_{1.0};
  double initial_roll_pitch_sigma_rad_{0.008726646259971648};
  double initial_z_sigma_m_{0.10};
  double correction_translation_rate_mps_{0.50};
  double correction_rotation_rate_radps_{0.20};
  double fixed_recent_timeout_sec_{0.50};
  double path_sample_period_sec_{0.10};
  double georeference_horizontal_rmse_m_{0.0};
  double last_rtk_innovation_m_{0.0};
  double last_optimization_ms_{0.0};

  int latest_quality_{0};
  int consecutive_fixed_count_{0};
  std::uint64_t rtk_sequence_{0U};
  std::uint64_t last_consumed_rtk_sequence_{0U};
  std::uint64_t next_key_index_{0U};
  std::size_t accepted_rtk_count_{0U};
  std::size_t rejected_nonfixed_count_{0U};
  std::size_t rejected_innovation_count_{0U};
  std::size_t accepted_gravity_count_{0U};
  std::size_t rejected_gravity_count_{0U};
  bool initialized_{false};

  gtsam::Point3 base_to_antenna_{0.1425, 0.2952585, 0.78476};
  gtsam::Rot3 base_from_imu_;
  gtsam::Pose3 map_from_enu_;
  gtsam::Pose3 target_map_from_odom_;
  gtsam::Pose3 applied_map_from_odom_;
  gtsam::Pose3 initial_map_from_odom_;
  gtsam::Pose3 latest_optimized_map_from_base_;
  gtsam::Key latest_key_{0U};
  gtsam::Matrix latest_pose_covariance_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel gravity_noise_;

  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::deque<GravitySample> gravity_buffer_;
  std::deque<Keyframe> keyframes_;
  std::optional<OdomSample> latest_odom_;
  std::optional<FixedRtkSample> latest_fixed_rtk_;
  std::optional<gtsam::Point3> last_accepted_rtk_antenna_;
  std::optional<gtsam::Pose3> pending_seed_map_from_base_;
  std::optional<double> pending_seed_stamp_sec_;
  std::optional<double> latest_quality_receipt_sec_;
  std::optional<double> last_fixed_fix_receipt_sec_;
  std::optional<double> applied_correction_stamp_sec_;
  std::optional<double> last_path_stamp_sec_;

  nav_msgs::msg::Path fused_path_;
  nav_msgs::msg::Path local_path_;
  nav_msgs::msg::Path rtk_path_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr quality_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    seed_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    fused_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fused_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rtk_path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fixed_active_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace agribot_hardware_bringup::fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<agribot_hardware_bringup::fusion::FastLivoRtkFusionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fastlivo_rtk_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
