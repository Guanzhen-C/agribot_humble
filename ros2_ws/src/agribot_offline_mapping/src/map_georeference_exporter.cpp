#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "agribot_hardware_bringup/map_georeference.hpp"
#include "agribot_offline_mapping/georeference_fit.hpp"

namespace agribot_offline_mapping
{
namespace
{

using agribot_hardware_bringup::navsat::MapGeoreference;

Eigen::Isometry3d messagePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & orientation = pose.orientation;
  Eigen::Quaterniond quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("odometry pose has an invalid quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("odometry pose contains a non-finite value");
  }
  return result;
}

double poseYaw(const Eigen::Isometry3d & pose)
{
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

Eigen::Vector3d vector3Parameter(
  rclcpp::Node & node, const std::string & name,
  const std::vector<double> & default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, default_value);
  if (values.size() != 3U ||
    !std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);}))
  {
    throw std::runtime_error(name + " must contain three finite values");
  }
  return {values[0], values[1], values[2]};
}

std::string utcTimestamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm value{};
  gmtime_r(&now, &value);
  std::ostringstream stream;
  stream << std::put_time(&value, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

}  // namespace

class MapGeoreferenceExporter final : public rclcpp::Node
{
public:
  MapGeoreferenceExporter()
  : Node("map_georeference_exporter")
  {
    optimized_path_topic_ = declare_parameter<std::string>(
      "optimized_path_topic", "/lio_sam/mapping/path");
    rtk_odometry_topic_ = declare_parameter<std::string>(
      "rtk_odometry_topic", "/lio_sam/odometry/gps");
    rtk_heading_topic_ = declare_parameter<std::string>(
      "rtk_heading_topic", "/lio_sam/odometry/heading");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/lio_sam/rtk_reference");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    enu_frame_ = declare_parameter<std::string>("enu_frame", "enu");
    antenna_frame_ = declare_parameter<std::string>(
      "antenna_frame", "rtk_master_antenna");
    lidar_to_antenna_ = vector3Parameter(
      *this, "lidar_to_antenna_m", {-0.336484515, 0.291027414, 0.554620722});
    const Eigen::Vector3d lidar_to_base_rpy = vector3Parameter(
      *this, "lidar_to_base_rpy", {0.0, 0.0, 0.0});
    lidar_to_base_.linear() =
      (Eigen::AngleAxisd(lidar_to_base_rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(lidar_to_base_rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(lidar_to_base_rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    output_file_ = declare_parameter<std::string>(
      "output_file", "/tmp/agribot_map_georeference.yaml");
    map_pcd_file_ = declare_parameter<std::string>(
      "map_pcd_file", "/tmp/agribot_map.pcd");
    map_id_ = declare_parameter<std::string>("map_id", "");
    source_bag_ = declare_parameter<std::string>("source_bag", "");
    calibration_version_ = declare_parameter<std::string>(
      "calibration_version", "lio_sam_rtk_v1");
    maximum_sync_offset_sec_ = declare_parameter<double>("maximum_sync_offset_sec", 0.20);
    maximum_heading_sync_offset_sec_ = declare_parameter<double>(
      "maximum_heading_sync_offset_sec", 0.60);
    const int minimum_samples = declare_parameter<int>("minimum_samples", 20);
    minimum_trajectory_span_m_ = declare_parameter<double>(
      "minimum_trajectory_span_m", 5.0);
    robust_minimum_inlier_m_ = declare_parameter<double>(
      "robust_minimum_inlier_m", 0.20);
    robust_mad_multiplier_ = declare_parameter<double>("robust_mad_multiplier", 3.0);
    maximum_horizontal_rmse_m_ = declare_parameter<double>(
      "maximum_horizontal_rmse_m", 0.20);
    maximum_yaw_rmse_deg_ = declare_parameter<double>("maximum_yaw_rmse_deg", 2.0);
    require_yaw_validation_ = declare_parameter<bool>("require_yaw_validation", true);
    const int maximum_stored_samples =
      declare_parameter<int>("maximum_stored_samples", 200000);

    if (output_file_.empty() || map_pcd_file_.empty() || maximum_sync_offset_sec_ <= 0.0 ||
      maximum_heading_sync_offset_sec_ <= 0.0 ||
      minimum_samples < 2 || minimum_trajectory_span_m_ <= 0.0 ||
      robust_minimum_inlier_m_ <= 0.0 || robust_mad_multiplier_ <= 0.0 ||
      maximum_horizontal_rmse_m_ <= 0.0 || maximum_yaw_rmse_deg_ <= 0.0 ||
      maximum_stored_samples < minimum_samples)
    {
      throw std::runtime_error("invalid map georeference exporter parameters");
    }
    minimum_samples_ = static_cast<std::size_t>(minimum_samples);
    maximum_stored_samples_ = static_cast<std::size_t>(maximum_stored_samples);

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/lio_sam/map_georeference_status", latched_qos);
    reference_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      reference_topic_, latched_qos,
      std::bind(&MapGeoreferenceExporter::handleReference, this, std::placeholders::_1));
    rtk_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      rtk_odometry_topic_, 200,
      std::bind(&MapGeoreferenceExporter::handleRtkOdometry, this, std::placeholders::_1));
    heading_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      rtk_heading_topic_, 50,
      std::bind(&MapGeoreferenceExporter::handleRtkHeading, this, std::placeholders::_1));
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      optimized_path_topic_, 10,
      std::bind(&MapGeoreferenceExporter::handleOptimizedPath, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save",
      std::bind(
        &MapGeoreferenceExporter::handleSave, this,
        std::placeholders::_1, std::placeholders::_2));
    setStatus("waiting for the optimized LIO-SAM path and independent RTK histories");
  }

private:
  struct TimedRtkPosition
  {
    rclcpp::Time stamp;
    Eigen::Vector3d enu_position{Eigen::Vector3d::Zero()};
  };

  struct TimedRtkHeading
  {
    rclcpp::Time stamp;
    double enu_yaw{0.0};
  };

  void handleReference(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (!std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    reference_ = *message;
  }

  void handleRtkOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != enu_frame_ || message->child_frame_id != antenna_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring RTK odometry with frames %s -> %s; expected %s -> %s",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        enu_frame_.c_str(), antenna_frame_.c_str());
      return;
    }
    TimedRtkPosition sample;
    sample.stamp = rclcpp::Time(message->header.stamp);
    sample.enu_position = Eigen::Vector3d(
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z);
    if (!sample.enu_position.allFinite()) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid RTK antenna position");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rtk_history_.push_back(sample);
    while (rtk_history_.size() > maximum_stored_samples_) {
      rtk_history_.pop_front();
    }
  }

  void handleRtkHeading(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (message->header.frame_id != enu_frame_) {
      return;
    }
    TimedRtkHeading sample;
    sample.stamp = rclcpp::Time(message->header.stamp);
    try {
      geometry_msgs::msg::Pose pose = message->pose.pose;
      sample.enu_yaw = poseYaw(messagePose(pose));
    } catch (const std::exception &) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    heading_history_.push_back(sample);
    while (heading_history_.size() > maximum_stored_samples_) {
      heading_history_.pop_front();
    }
  }

  void handleOptimizedPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring LIO-SAM optimized path in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    optimized_path_ = *message;
    setStatusLocked(
      "received optimized LIO-SAM path with " +
      std::to_string(message->poses.size()) + " key poses");
  }

  std::vector<GeoreferenceSample> synchronizedFinalSamples(
    const nav_msgs::msg::Path & path,
    const std::deque<TimedRtkPosition> & rtk_history,
    const std::deque<TimedRtkHeading> & heading_history) const
  {
    if (path.poses.empty() || rtk_history.empty()) {
      return {};
    }
    std::vector<GeoreferenceSample> samples;
    samples.reserve(std::min(path.poses.size(), rtk_history.size()));
    std::size_t rtk_index = 0U;
    std::size_t heading_index = 0U;
    std::optional<rclcpp::Time> last_used_stamp;
    for (const auto & stamped_pose : path.poses) {
      const rclcpp::Time pose_stamp(stamped_pose.header.stamp);
      while (rtk_index + 1U < rtk_history.size() &&
        std::abs((rtk_history[rtk_index + 1U].stamp - pose_stamp).seconds()) <
        std::abs((rtk_history[rtk_index].stamp - pose_stamp).seconds()))
      {
        ++rtk_index;
      }
      const auto & rtk = rtk_history[rtk_index];
      if (std::abs((rtk.stamp - pose_stamp).seconds()) > maximum_sync_offset_sec_ ||
        (last_used_stamp.has_value() && rtk.stamp == *last_used_stamp))
      {
        continue;
      }
      try {
        const Eigen::Isometry3d map_to_lidar = messagePose(stamped_pose.pose);
        GeoreferenceSample sample;
        sample.enu_position = rtk.enu_position;
        sample.map_position = map_to_lidar * lidar_to_antenna_;
        while (heading_index + 1U < heading_history.size() &&
          std::abs((heading_history[heading_index + 1U].stamp - pose_stamp).seconds()) <
          std::abs((heading_history[heading_index].stamp - pose_stamp).seconds()))
        {
          ++heading_index;
        }
        if (!heading_history.empty() &&
          std::abs((heading_history[heading_index].stamp - pose_stamp).seconds()) <=
          maximum_heading_sync_offset_sec_)
        {
          sample.enu_yaw = heading_history[heading_index].enu_yaw;
          sample.map_yaw = poseYaw(map_to_lidar * lidar_to_base_);
          sample.has_yaw = true;
        }
        samples.push_back(sample);
        last_used_stamp = rtk.stamp;
      } catch (const std::exception &) {
        continue;
      }
    }
    return samples;
  }

  void handleSave(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      const MapGeoreference georeference = buildGeoreference();
      agribot_hardware_bringup::navsat::writeMapGeoreference(
        output_file_, georeference);
      std::ostringstream message;
      message << "saved " << output_file_ << " from " << georeference.sample_count
              << " inliers; horizontal RMSE=" << std::fixed << std::setprecision(3)
              << georeference.horizontal_rmse_m << " m, horizontal validation="
              << (georeference.horizontal_rmse_m <= maximum_horizontal_rmse_m_ ?
        "passed" : "warning") << ", yaw RMSE="
              << georeference.yaw_rmse_deg << " deg, yaw validation="
              << (georeference.yaw_validation_passed ? "passed" : "warning");
      response->success = true;
      response->message = message.str();
      setStatus(response->message);
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
      setStatus("georeference save rejected: " + response->message);
      RCLCPP_ERROR(get_logger(), "Georeference save rejected: %s", error.what());
    }
  }

  MapGeoreference buildGeoreference()
  {
    nav_msgs::msg::Path optimized_path;
    std::deque<TimedRtkPosition> rtk_history;
    std::deque<TimedRtkHeading> heading_history;
    sensor_msgs::msg::NavSatFix reference;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!reference_.has_value()) {
        throw std::runtime_error("RTK ENU reference has not been received");
      }
      if (!optimized_path_.has_value()) {
        throw std::runtime_error("final optimized LIO-SAM path has not been received");
      }
      optimized_path = *optimized_path_;
      rtk_history = rtk_history_;
      heading_history = heading_history_;
      reference = *reference_;
    }
    const std::vector<GeoreferenceSample> samples =
      synchronizedFinalSamples(optimized_path, rtk_history, heading_history);
    const GeoreferenceFit fit = fitGeoreference(
      samples, minimum_samples_, robust_minimum_inlier_m_, robust_mad_multiplier_);
    const double yaw_rmse_deg = fit.yaw_rmse_rad * 180.0 / M_PI;
    if (fit.trajectory_span_m < minimum_trajectory_span_m_) {
      throw std::runtime_error("calibration trajectory span is below the configured minimum");
    }
    if (fit.horizontal_rmse_m > maximum_horizontal_rmse_m_) {
      RCLCPP_WARN(
        get_logger(),
        "Horizontal validation warning: RMSE %.3f m exceeds %.3f m; saving the "
        "map<-ENU transform with its measured calibration quality",
        fit.horizontal_rmse_m, maximum_horizontal_rmse_m_);
    }
    const bool yaw_validation_passed =
      std::isfinite(yaw_rmse_deg) && yaw_rmse_deg <= maximum_yaw_rmse_deg_;
    if (require_yaw_validation_ && !yaw_validation_passed) {
      throw std::runtime_error("yaw georeference RMSE exceeds the acceptance limit");
    }
    if (!yaw_validation_passed) {
      RCLCPP_WARN(
        get_logger(),
        "Yaw validation warning: RMSE %.3f deg exceeds %.3f deg; saving the "
        "position-trajectory map<-ENU transform for NDT/GICP-refined initialization",
        yaw_rmse_deg, maximum_yaw_rmse_deg_);
    }
    const std::filesystem::path map_path(map_pcd_file_);
    if (!std::filesystem::is_regular_file(map_path)) {
      throw std::runtime_error("final PCD map does not exist: " + map_path.string());
    }

    MapGeoreference result;
    result.map_id = map_id_.empty() ? map_path.stem().string() : map_id_;
    result.map_pcd = std::filesystem::absolute(map_path).lexically_normal().string();
    result.map_fingerprint =
      agribot_hardware_bringup::navsat::fingerprintFile(map_path);
    result.reference_latitude_deg = reference.latitude;
    result.reference_longitude_deg = reference.longitude;
    result.reference_altitude_m = reference.altitude;
    result.map_from_enu_xyz = {
      fit.map_from_enu.translation().x(), fit.map_from_enu.translation().y(),
      fit.map_from_enu.translation().z()};
    result.map_from_enu_rpy = {
      0.0, 0.0,
      std::atan2(fit.map_from_enu.linear()(1, 0), fit.map_from_enu.linear()(0, 0))};
    result.horizontal_rmse_m = fit.horizontal_rmse_m;
    result.yaw_rmse_deg = yaw_rmse_deg;
    result.yaw_validation_passed = yaw_validation_passed;
    result.sample_count = fit.inlier_indices.size();
    result.source_bag = source_bag_;
    result.calibration_version = calibration_version_;
    result.created_at_utc = utcTimestamp();
    std::ostringstream hash_input;
    hash_input << std::setprecision(17) << result.map_id << result.map_fingerprint
               << result.reference_latitude_deg << result.reference_longitude_deg
               << result.reference_altitude_m;
    for (const double value : result.map_from_enu_xyz) {
      hash_input << value;
    }
    for (const double value : result.map_from_enu_rpy) {
      hash_input << value;
    }
    hash_input << result.horizontal_rmse_m << result.yaw_rmse_deg
               << result.yaw_validation_passed
               << result.sample_count << result.source_bag << result.calibration_version;
    result.calibration_hash =
      agribot_hardware_bringup::navsat::fnv1a64Text(hash_input.str());
    return result;
  }

  void setStatus(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    setStatusLocked(value);
  }

  void setStatusLocked(const std::string & value)
  {
    if (value == last_status_) {
      return;
    }
    last_status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    if (status_publisher_) {
      status_publisher_->publish(message);
    }
  }

  std::string optimized_path_topic_;
  std::string rtk_odometry_topic_;
  std::string rtk_heading_topic_;
  std::string reference_topic_;
  std::string map_frame_;
  std::string enu_frame_;
  std::string antenna_frame_;
  Eigen::Vector3d lidar_to_antenna_{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d lidar_to_base_{Eigen::Isometry3d::Identity()};
  std::string output_file_;
  std::string map_pcd_file_;
  std::string map_id_;
  std::string source_bag_;
  std::string calibration_version_;
  double maximum_sync_offset_sec_{0.20};
  double maximum_heading_sync_offset_sec_{0.60};
  std::size_t minimum_samples_{20U};
  double minimum_trajectory_span_m_{5.0};
  double robust_minimum_inlier_m_{0.20};
  double robust_mad_multiplier_{3.0};
  double maximum_horizontal_rmse_m_{0.20};
  double maximum_yaw_rmse_deg_{2.0};
  bool require_yaw_validation_{true};
  std::size_t maximum_stored_samples_{200000U};
  std::mutex mutex_;
  std::deque<TimedRtkPosition> rtk_history_;
  std::deque<TimedRtkHeading> heading_history_;
  std::optional<nav_msgs::msg::Path> optimized_path_;
  std::optional<sensor_msgs::msg::NavSatFix> reference_;
  std::string last_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr reference_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rtk_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    heading_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

}  // namespace agribot_offline_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agribot_offline_mapping::MapGeoreferenceExporter>());
  rclcpp::shutdown();
  return 0;
}
