#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "agribot_hardware_bringup/map_georeference.hpp"

namespace agribot_hardware_bringup
{
namespace
{

namespace navsat = agribot_hardware_bringup::navsat;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    throw std::runtime_error("pose quaternion is invalid");
  }
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Eigen::Isometry3d poseIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("odometry quaternion is invalid");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return result;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::runtime_error("cannot calculate median from no RTK samples");
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2U == 0U) {
    result = 0.5 * (result + *std::max_element(values.begin(), values.begin() + middle));
  }
  return result;
}

Eigen::Vector3d vector3Parameter(
  rclcpp::Node & node,
  const std::string & name,
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

}  // namespace

class RtkMapInitializer final : public rclcpp::Node
{
public:
  RtkMapInitializer()
  : Node("rtk_map_initializer")
  {
    georeference_file_ = declare_parameter<std::string>("georeference_file", "");
    map_file_ = declare_parameter<std::string>("map_file", "");
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/rtk/fix");
    quality_topic_ = declare_parameter<std::string>("quality_topic", "/rtk/fix_quality");
    heading_topic_ = declare_parameter<std::string>(
      "heading_topic", "/rtk/heading_with_covariance");
    heading_solution_topic_ = declare_parameter<std::string>(
      "heading_solution_topic", "/rtk/heading_solution");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/fastlio/odometry");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    localizer_status_topic_ = declare_parameter<std::string>(
      "localizer_status_topic", "/localization/status");
    localizer_ready_topic_ = declare_parameter<std::string>(
      "localizer_ready_topic", "/localization/ready");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/localization/rtk_initializer_status");
    seed_ready_topic_ = declare_parameter<std::string>(
      "seed_ready_topic", "/localization/rtk_seed_ready");
    seed_transform_topic_ = declare_parameter<std::string>(
      "seed_transform_topic", "/localization/rtk_map_to_odom_seed");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    required_fix_quality_ = declare_parameter<int>("required_fix_quality", 4);
    allowed_heading_solutions_ = declare_parameter<std::vector<std::string>>(
      "allowed_heading_solutions", {"L1_INT", "NARROW_INT"});
    const int required_sample_count = declare_parameter<int>("required_sample_count", 10);
    const int minimum_inlier_count = declare_parameter<int>("minimum_inlier_count", 7);
    heading_timeout_sec_ = declare_parameter<double>("heading_timeout_sec", 1.5);
    maximum_position_deviation_m_ = declare_parameter<double>(
      "maximum_position_deviation_m", 0.15);
    maximum_heading_deviation_deg_ = declare_parameter<double>(
      "maximum_heading_deviation_deg", 3.0);
    maximum_heading_std_deg_ = declare_parameter<double>("maximum_heading_std_deg", 3.0);
    default_horizontal_std_m_ = declare_parameter<double>("default_horizontal_std_m", 0.03);
    maximum_odometry_age_sec_ = declare_parameter<double>("maximum_odometry_age_sec", 0.50);
    maximum_georeference_horizontal_rmse_m_ = declare_parameter<double>(
      "maximum_georeference_horizontal_rmse_m", 0.20);
    maximum_georeference_yaw_rmse_deg_ = declare_parameter<double>(
      "maximum_georeference_yaw_rmse_deg", 2.0);
    allow_unvalidated_georeference_yaw_ = declare_parameter<bool>(
      "allow_unvalidated_georeference_yaw", false);
    base_to_antenna_ = vector3Parameter(
      *this, "base_to_master_antenna_m", {-0.0884, 0.1480, 0.24476});

    if (georeference_file_.empty() || map_file_.empty() || required_fix_quality_ < 1 ||
      allowed_heading_solutions_.empty() || required_sample_count < 2 ||
      minimum_inlier_count < 2 || minimum_inlier_count > required_sample_count ||
      heading_timeout_sec_ <= 0.0 || maximum_position_deviation_m_ <= 0.0 ||
      maximum_heading_deviation_deg_ <= 0.0 || maximum_heading_std_deg_ <= 0.0 ||
      default_horizontal_std_m_ <= 0.0 || maximum_odometry_age_sec_ <= 0.0 ||
      maximum_georeference_horizontal_rmse_m_ <= 0.0 ||
      maximum_georeference_yaw_rmse_deg_ <= 0.0)
    {
      throw std::runtime_error("invalid RTK map initializer parameters");
    }
    required_sample_count_ = static_cast<std::size_t>(required_sample_count);
    minimum_inlier_count_ = static_cast<std::size_t>(minimum_inlier_count);

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    initial_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic_, 10);
    seed_transform_publisher_ =
      create_publisher<geometry_msgs::msg::TransformStamped>(seed_transform_topic_, latched_qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);
    seed_ready_publisher_ = create_publisher<std_msgs::msg::Bool>(seed_ready_topic_, latched_qos);

    loadAndVerifyMap();
    publishSeedReady(false);

    quality_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      quality_topic_, 20,
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        latest_quality_ = static_cast<int>(message->data);
      });
    solution_subscription_ = create_subscription<std_msgs::msg::String>(
      heading_solution_topic_, 20,
      [this](const std_msgs::msg::String::SharedPtr message) {
        latest_heading_solution_ = message->data;
        latest_solution_receipt_ = now();
      });
    heading_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      heading_topic_, 20,
      std::bind(&RtkMapInitializer::handleHeading, this, std::placeholders::_1));
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, 20,
      std::bind(&RtkMapInitializer::handleFix, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 100,
      std::bind(&RtkMapInitializer::handleOdometry, this, std::placeholders::_1));
    localizer_status_subscription_ = create_subscription<std_msgs::msg::String>(
      localizer_status_topic_, latched_qos,
      [this](const std_msgs::msg::String::SharedPtr) {
        localizer_available_ = true;
        tryPublishSeed();
      });
    localizer_ready_subscription_ = create_subscription<std_msgs::msg::Bool>(
      localizer_ready_topic_, latched_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) {
          setStatus("RTK seed refined and accepted by NDT/GICP localizer");
        }
      });
  }

private:
  struct RtkSample
  {
    rclcpp::Time stamp;
    Eigen::Vector3d base_position_enu{Eigen::Vector3d::Zero()};
    double base_yaw_enu{0.0};
    double horizontal_variance{0.0};
    double yaw_variance{0.0};
  };

  void loadAndVerifyMap()
  {
    try {
      georeference_ = navsat::loadMapGeoreference(georeference_file_);
      const std::filesystem::path map_path(map_file_);
      if (!std::filesystem::is_regular_file(map_path)) {
        throw std::runtime_error("PCD map is unavailable: " + map_path.string());
      }
      if (map_path.stem().string() != georeference_->map_id) {
        throw std::runtime_error("PCD map name does not match georeference map ID");
      }
      if (navsat::fingerprintFile(map_path) != georeference_->map_fingerprint) {
        throw std::runtime_error("PCD map fingerprint does not match georeference metadata");
      }
      if (georeference_->horizontal_rmse_m > maximum_georeference_horizontal_rmse_m_) {
        throw std::runtime_error(
                "map georeference horizontal calibration does not meet runtime limits");
      }
      if (georeference_->yaw_validation_passed &&
        georeference_->yaw_rmse_deg > maximum_georeference_yaw_rmse_deg_)
      {
        throw std::runtime_error(
                "validated map georeference yaw does not meet runtime limits");
      }
      if (!georeference_->yaw_validation_passed &&
        !allow_unvalidated_georeference_yaw_)
      {
        throw std::runtime_error(
                "position-only map georeference yaw is not allowed by runtime configuration");
      }
      map_from_enu_ = navsat::mapFromEnuTransform(*georeference_);
      local_cartesian_.emplace(
        georeference_->reference_latitude_deg,
        georeference_->reference_longitude_deg,
        georeference_->reference_altitude_m);
      map_valid_ = true;
      if (georeference_->yaw_validation_passed) {
        setStatus("map georeference verified; waiting for fixed RTK samples");
      } else {
        setStatus(
          "position-trajectory map georeference verified; RTK yaw will be a coarse "
          "NDT/GICP prior");
        RCLCPP_WARN(
          get_logger(),
          "Using position-trajectory map georeference with yaw RMSE %.3f deg; "
          "localization readiness remains false until NDT/GICP accepts the pose",
          georeference_->yaw_rmse_deg);
      }
    } catch (const std::exception & error) {
      map_valid_ = false;
      setStatus("RTK initialization disabled: " + std::string(error.what()));
      RCLCPP_ERROR(get_logger(), "%s", last_status_.c_str());
    }
  }

  void handleHeading(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    try {
      const double variance = message->pose.covariance[35];
      if (!std::isfinite(variance) || variance <= 0.0) {
        throw std::runtime_error("heading variance is invalid");
      }
      latest_heading_yaw_ = yawFromQuaternion(message->pose.pose.orientation);
      latest_heading_variance_ = variance;
      latest_heading_stamp_ = rclcpp::Time(message->header.stamp);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring RTK heading: %s", error.what());
    }
  }

  bool headingSolutionAccepted() const
  {
    if (!latest_heading_solution_.has_value() ||
      latest_heading_solution_->rfind("SOL_COMPUTED,", 0U) != 0U)
    {
      return false;
    }
    const std::string type = latest_heading_solution_->substr(
      latest_heading_solution_->find(',') + 1U);
    return std::find(
      allowed_heading_solutions_.begin(), allowed_heading_solutions_.end(), type) !=
           allowed_heading_solutions_.end();
  }

  void handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (!map_valid_ || seed_published_) {
      return;
    }
    if (message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude) || !latest_quality_.has_value() ||
      *latest_quality_ != required_fix_quality_)
    {
      waitingStatus("waiting for RTK quality 4 fixed position");
      return;
    }
    if (!latest_heading_yaw_.has_value() || !latest_heading_stamp_.has_value() ||
      !latest_heading_variance_.has_value() || !headingSolutionAccepted())
    {
      waitingStatus("waiting for integer-fixed dual-antenna heading");
      return;
    }
    const rclcpp::Time fix_stamp(message->header.stamp);
    if (std::abs((fix_stamp - *latest_heading_stamp_).seconds()) > heading_timeout_sec_ ||
      !latest_solution_receipt_.has_value() ||
      (now() - *latest_solution_receipt_).seconds() > heading_timeout_sec_)
    {
      waitingStatus("waiting for a fresh RTK heading synchronized to position");
      return;
    }
    const double maximum_heading_std_rad = maximum_heading_std_deg_ * M_PI / 180.0;
    if (*latest_heading_variance_ > maximum_heading_std_rad * maximum_heading_std_rad) {
      waitingStatus("waiting for RTK heading covariance within the initialization limit");
      return;
    }

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_->Forward(
      message->latitude, message->longitude, message->altitude,
      east, north, up);
    const Eigen::Matrix3d enu_from_base =
      Eigen::AngleAxisd(*latest_heading_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    RtkSample sample{
      fix_stamp,
      Eigen::Vector3d(east, north, up) - enu_from_base * base_to_antenna_,
      *latest_heading_yaw_,
      std::max(
        1.0e-6,
        message->position_covariance[0] > 0.0 && message->position_covariance[4] > 0.0 ?
        0.5 * (message->position_covariance[0] + message->position_covariance[4]) :
        default_horizontal_std_m_ * default_horizontal_std_m_),
      *latest_heading_variance_};
    samples_.push_back(sample);
    while (samples_.size() > required_sample_count_) {
      samples_.pop_front();
    }
    setStatus(
      "collecting fixed RTK initialization samples: " +
      std::to_string(samples_.size()) + "/" + std::to_string(required_sample_count_));
    tryPublishSeed();
  }

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring localization odometry with frames %s -> %s; expected %s -> %s",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }
    try {
      latest_odom_to_base_ = poseIsometry(message->pose.pose);
      latest_odom_stamp_ = rclcpp::Time(message->header.stamp);
      tryPublishSeed();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring localization odometry: %s", error.what());
    }
  }

  std::optional<RtkSample> robustAverage() const
  {
    if (samples_.size() < required_sample_count_) {
      return std::nullopt;
    }
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    double sine_sum = 0.0;
    double cosine_sum = 0.0;
    for (const RtkSample & sample : samples_) {
      x.push_back(sample.base_position_enu.x());
      y.push_back(sample.base_position_enu.y());
      z.push_back(sample.base_position_enu.z());
      sine_sum += std::sin(sample.base_yaw_enu);
      cosine_sum += std::cos(sample.base_yaw_enu);
    }
    const Eigen::Vector3d position_median(median(x), median(y), median(z));
    const double yaw_center = std::atan2(sine_sum, cosine_sum);

    std::vector<const RtkSample *> inliers;
    const double maximum_heading_deviation_rad = maximum_heading_deviation_deg_ * M_PI / 180.0;
    for (const RtkSample & sample : samples_) {
      if ((sample.base_position_enu.head<2>() - position_median.head<2>()).norm() <=
        maximum_position_deviation_m_ &&
        std::abs(wrapAngle(sample.base_yaw_enu - yaw_center)) <= maximum_heading_deviation_rad)
      {
        inliers.push_back(&sample);
      }
    }
    if (inliers.size() < minimum_inlier_count_) {
      return std::nullopt;
    }

    RtkSample result = *inliers.back();
    result.base_position_enu.setZero();
    sine_sum = 0.0;
    cosine_sum = 0.0;
    result.horizontal_variance = 0.0;
    result.yaw_variance = 0.0;
    for (const RtkSample * sample : inliers) {
      result.base_position_enu += sample->base_position_enu;
      sine_sum += std::sin(sample->base_yaw_enu);
      cosine_sum += std::cos(sample->base_yaw_enu);
      result.horizontal_variance += sample->horizontal_variance;
      result.yaw_variance += sample->yaw_variance;
    }
    const double inverse_count = 1.0 / static_cast<double>(inliers.size());
    result.base_position_enu *= inverse_count;
    result.base_yaw_enu = std::atan2(sine_sum, cosine_sum);
    result.horizontal_variance *= inverse_count * inverse_count;
    result.yaw_variance *= inverse_count * inverse_count;
    return result;
  }

  void tryPublishSeed()
  {
    if (!map_valid_ || seed_published_ || !localizer_available_ ||
      !latest_odom_to_base_.has_value())
    {
      return;
    }
    const auto averaged = robustAverage();
    if (!averaged.has_value()) {
      if (samples_.size() >= required_sample_count_) {
        setStatus("fixed RTK samples are not mutually consistent; keeping chassis inhibited");
      }
      return;
    }
    if (!latest_odom_stamp_.has_value() ||
      std::abs((*latest_odom_stamp_ - averaged->stamp).seconds()) > maximum_odometry_age_sec_)
    {
      setStatus("waiting for localization odometry synchronized to the RTK seed");
      return;
    }

    Eigen::Isometry3d enu_to_base = Eigen::Isometry3d::Identity();
    enu_to_base.translation() = averaged->base_position_enu;
    enu_to_base.linear() =
      Eigen::AngleAxisd(averaged->base_yaw_enu, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Isometry3d map_to_base = map_from_enu_ * enu_to_base;
    const Eigen::Isometry3d map_to_odom_seed = map_to_base * latest_odom_to_base_->inverse();
    const double map_yaw = std::atan2(map_to_base.linear()(1, 0), map_to_base.linear()(0, 0));

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp = averaged->stamp;
    initial_pose.header.frame_id = map_frame_;
    initial_pose.pose.pose.position.x = map_to_base.translation().x();
    initial_pose.pose.pose.position.y = map_to_base.translation().y();
    initial_pose.pose.pose.position.z = map_to_base.translation().z();
    initial_pose.pose.pose.orientation.z = std::sin(map_yaw / 2.0);
    initial_pose.pose.pose.orientation.w = std::cos(map_yaw / 2.0);
    const double horizontal_variance = averaged->horizontal_variance +
      georeference_->horizontal_rmse_m * georeference_->horizontal_rmse_m;
    const double yaw_rmse_rad = georeference_->yaw_rmse_deg * M_PI / 180.0;
    initial_pose.pose.covariance[0] = horizontal_variance;
    initial_pose.pose.covariance[7] = horizontal_variance;
    initial_pose.pose.covariance[14] = 0.25;
    initial_pose.pose.covariance[21] = 1.0e6;
    initial_pose.pose.covariance[28] = 1.0e6;
    initial_pose.pose.covariance[35] =
      averaged->yaw_variance + yaw_rmse_rad * yaw_rmse_rad;

    geometry_msgs::msg::TransformStamped seed_transform;
    seed_transform.header = initial_pose.header;
    seed_transform.header.frame_id = map_frame_;
    seed_transform.child_frame_id = odom_frame_;
    seed_transform.transform.translation.x = map_to_odom_seed.translation().x();
    seed_transform.transform.translation.y = map_to_odom_seed.translation().y();
    seed_transform.transform.translation.z = map_to_odom_seed.translation().z();
    const Eigen::Quaterniond seed_quaternion(map_to_odom_seed.linear());
    seed_transform.transform.rotation.x = seed_quaternion.x();
    seed_transform.transform.rotation.y = seed_quaternion.y();
    seed_transform.transform.rotation.z = seed_quaternion.z();
    seed_transform.transform.rotation.w = seed_quaternion.w();

    seed_transform_publisher_->publish(seed_transform);
    initial_pose_publisher_->publish(initial_pose);
    seed_published_ = true;
    publishSeedReady(true);
    setStatus("RTK map seed published once; waiting for NDT/GICP acceptance");
    RCLCPP_INFO(
      get_logger(), "RTK map seed: x=%.3f y=%.3f yaw=%.2f deg",
      map_to_base.translation().x(), map_to_base.translation().y(),
      map_yaw * 180.0 / M_PI);
  }

  void waitingStatus(const std::string & value)
  {
    setStatus(value + "; chassis remains inhibited");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", value.c_str());
  }

  void publishSeedReady(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    seed_ready_publisher_->publish(message);
  }

  void setStatus(const std::string & value)
  {
    if (value == last_status_) {
      return;
    }
    last_status_ = value;
    std_msgs::msg::String message;
    message.data = value;
    status_publisher_->publish(message);
  }

  std::string georeference_file_;
  std::string map_file_;
  std::string fix_topic_;
  std::string quality_topic_;
  std::string heading_topic_;
  std::string heading_solution_topic_;
  std::string odometry_topic_;
  std::string initial_pose_topic_;
  std::string localizer_status_topic_;
  std::string localizer_ready_topic_;
  std::string status_topic_;
  std::string seed_ready_topic_;
  std::string seed_transform_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  int required_fix_quality_{4};
  std::vector<std::string> allowed_heading_solutions_;
  std::size_t required_sample_count_{10U};
  std::size_t minimum_inlier_count_{7U};
  double heading_timeout_sec_{1.5};
  double maximum_position_deviation_m_{0.15};
  double maximum_heading_deviation_deg_{3.0};
  double maximum_heading_std_deg_{3.0};
  double default_horizontal_std_m_{0.03};
  double maximum_odometry_age_sec_{0.50};
  double maximum_georeference_horizontal_rmse_m_{0.20};
  double maximum_georeference_yaw_rmse_deg_{2.0};
  bool allow_unvalidated_georeference_yaw_{false};
  Eigen::Vector3d base_to_antenna_{Eigen::Vector3d::Zero()};
  bool map_valid_{false};
  bool localizer_available_{false};
  bool seed_published_{false};
  std::optional<navsat::MapGeoreference> georeference_;
  std::optional<GeographicLib::LocalCartesian> local_cartesian_;
  Eigen::Isometry3d map_from_enu_{Eigen::Isometry3d::Identity()};
  std::optional<int> latest_quality_;
  std::optional<double> latest_heading_yaw_;
  std::optional<double> latest_heading_variance_;
  std::optional<rclcpp::Time> latest_heading_stamp_;
  std::optional<std::string> latest_heading_solution_;
  std::optional<rclcpp::Time> latest_solution_receipt_;
  std::optional<Eigen::Isometry3d> latest_odom_to_base_;
  std::optional<rclcpp::Time> latest_odom_stamp_;
  std::deque<RtkSample> samples_;
  std::string last_status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
    seed_transform_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr seed_ready_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr quality_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    heading_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solution_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localizer_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localizer_ready_subscription_;
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agribot_hardware_bringup::RtkMapInitializer>());
  rclcpp::shutdown();
  return 0;
}
