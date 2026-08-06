#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "agribot_offline_mapping/rtk_heading_policy.hpp"
#include "agribot_offline_mapping/rtk_lever_arm.hpp"

namespace agribot_offline_mapping
{
namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!std::isfinite(norm) || norm < 1.0e-9) {
    throw std::runtime_error("RTK heading quaternion is invalid");
  }
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class RtkOdometryAdapter final : public rclcpp::Node
{
public:
  RtkOdometryAdapter()
  : Node("rtk_odometry_adapter")
  {
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/rtk/fix");
    quality_topic_ = declare_parameter<std::string>("quality_topic", "/rtk/fix_quality");
    heading_topic_ = declare_parameter<std::string>(
      "heading_topic", "/rtk/heading_with_covariance");
    heading_solution_topic_ = declare_parameter<std::string>(
      "heading_solution_topic", "/rtk/heading_solution");
    position_output_topic_ = declare_parameter<std::string>(
      "position_output_topic", "/lio_sam/odometry/gps");
    antenna_output_topic_ = declare_parameter<std::string>(
      "antenna_output_topic", "/lio_sam/odometry/rtk_antenna");
    heading_output_topic_ = declare_parameter<std::string>(
      "heading_output_topic", "/lio_sam/odometry/heading");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/lio_sam/rtk_reference");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/lio_sam/rtk_adapter_status");
    enu_frame_ = declare_parameter<std::string>("enu_frame", "enu");
    antenna_frame_ = declare_parameter<std::string>(
      "antenna_frame", "rtk_master_antenna");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
    const auto antenna_to_lidar = declare_parameter<std::vector<double>>(
      "antenna_to_lidar_flu_m", {0.3375, -0.2952585, -0.05176});
    if (antenna_to_lidar.size() != 3U ||
      !std::all_of(
        antenna_to_lidar.begin(), antenna_to_lidar.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::runtime_error("antenna_to_lidar_flu_m must contain three finite values");
    }
    antenna_to_lidar_flu_ = Eigen::Vector3d(
      antenna_to_lidar[0], antenna_to_lidar[1], antenna_to_lidar[2]);
    maximum_lever_heading_age_sec_ = declare_parameter<double>(
      "maximum_lever_heading_age_sec", 1.5);
    required_fix_quality_ = declare_parameter<int>("required_fix_quality", 4);
    heading_policy_.fixed_solutions = declare_parameter<std::vector<std::string>>(
      "fixed_heading_solutions", {"L1_INT", "NARROW_INT"});
    heading_policy_.float_solutions = declare_parameter<std::vector<std::string>>(
      "float_heading_solutions", {"L1_FLOAT", "NARROW_FLOAT"});
    heading_policy_.fixed_std_floor_deg = declare_parameter<double>(
      "fixed_heading_std_floor_deg", 1.0);
    heading_policy_.float_std_floor_deg = declare_parameter<double>(
      "float_heading_std_floor_deg", 5.0);
    heading_solution_timeout_sec_ = declare_parameter<double>(
      "heading_solution_timeout_sec", 1.5);
    default_horizontal_std_m_ = declare_parameter<double>("default_horizontal_std_m", 0.03);
    default_vertical_std_m_ = declare_parameter<double>("default_vertical_std_m", 0.06);
    auto_reference_from_first_fix_ = declare_parameter<bool>(
      "auto_reference_from_first_fix", true);
    reference_latitude_deg_ = declare_parameter<double>("reference_latitude_deg", 0.0);
    reference_longitude_deg_ = declare_parameter<double>("reference_longitude_deg", 0.0);
    reference_altitude_m_ = declare_parameter<double>("reference_altitude_m", 0.0);

    if (required_fix_quality_ < 1 || heading_policy_.fixed_solutions.empty() ||
      heading_policy_.float_solutions.empty() || heading_solution_timeout_sec_ <= 0.0 ||
      heading_policy_.fixed_std_floor_deg <= 0.0 ||
      heading_policy_.float_std_floor_deg <= 0.0 || default_horizontal_std_m_ <= 0.0 ||
      default_vertical_std_m_ <= 0.0 || maximum_lever_heading_age_sec_ <= 0.0 ||
      position_output_topic_ == antenna_output_topic_)
    {
      throw std::runtime_error("invalid RTK adapter quality or covariance parameters");
    }
    if (!auto_reference_from_first_fix_) {
      initializeReference(
        reference_latitude_deg_, reference_longitude_deg_, reference_altitude_m_);
    }

    position_publisher_ = create_publisher<nav_msgs::msg::Odometry>(position_output_topic_, 20);
    antenna_publisher_ = create_publisher<nav_msgs::msg::Odometry>(antenna_output_topic_, 20);
    heading_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(heading_output_topic_, 20);
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    reference_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      reference_topic_, latched_qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, latched_qos);
    if (pending_reference_message_.has_value()) {
      reference_publisher_->publish(*pending_reference_message_);
      pending_reference_message_.reset();
    }
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
      std::bind(&RtkOdometryAdapter::handleHeading, this, std::placeholders::_1));
    fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, 20,
      std::bind(&RtkOdometryAdapter::handleFix, this, std::placeholders::_1));
    setStatus("waiting for independent RTK fixed positions and dual-antenna headings");
  }

private:
  void initializeReference(double latitude_deg, double longitude_deg, double altitude_m)
  {
    if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) ||
      !std::isfinite(altitude_m) || std::abs(latitude_deg) > 90.0 ||
      std::abs(longitude_deg) > 180.0)
    {
      throw std::runtime_error("invalid RTK local Cartesian reference");
    }
    reference_latitude_deg_ = latitude_deg;
    reference_longitude_deg_ = longitude_deg;
    reference_altitude_m_ = altitude_m;
    local_cartesian_.emplace(latitude_deg, longitude_deg, altitude_m);
    sensor_msgs::msg::NavSatFix reference;
    reference.header.stamp = now();
    reference.header.frame_id = enu_frame_;
    reference.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    reference.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    reference.latitude = latitude_deg;
    reference.longitude = longitude_deg;
    reference.altitude = altitude_m;
    if (reference_publisher_) {
      reference_publisher_->publish(reference);
    } else {
      pending_reference_message_ = reference;
    }
    RCLCPP_INFO(
      get_logger(), "RTK ENU reference fixed at %.9f, %.9f, %.3f m",
      latitude_deg, longitude_deg, altitude_m);
  }

  void handleHeading(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    try {
      if (!latest_heading_solution_.has_value() || !latest_solution_receipt_.has_value() ||
        std::abs((now() - *latest_solution_receipt_).seconds()) > heading_solution_timeout_sec_)
      {
        rejectHeading("RTK heading solution status is unavailable or stale");
        return;
      }
      const auto variance = effectiveHeadingVariance(
        *latest_heading_solution_, message->pose.covariance[35], heading_policy_);
      if (!variance.has_value()) {
        rejectHeading("RTK heading solution is invalid");
        return;
      }
      const double yaw = yawFromQuaternion(message->pose.pose.orientation);
      geometry_msgs::msg::PoseWithCovarianceStamped output = *message;
      output.header.frame_id = enu_frame_;
      output.pose.pose.position.x = 0.0;
      output.pose.pose.position.y = 0.0;
      output.pose.pose.position.z = 0.0;
      output.pose.pose.orientation.x = 0.0;
      output.pose.pose.orientation.y = 0.0;
      output.pose.pose.orientation.z = std::sin(yaw / 2.0);
      output.pose.pose.orientation.w = std::cos(yaw / 2.0);
      output.pose.covariance.fill(0.0);
      for (const std::size_t index : {0U, 7U, 14U, 21U, 28U}) {
        output.pose.covariance[index] = 1.0e6;
      }
      output.pose.covariance[35] = *variance;
      heading_publisher_->publish(output);
      latest_lever_heading_yaw_ = yaw;
      latest_lever_heading_variance_ = *variance;
      latest_lever_heading_stamp_ = rclcpp::Time(message->header.stamp);
      setStatus("publishing independent RTK position and quality-weighted heading");
    } catch (const std::exception & error) {
      rejectHeading(std::string("ignoring RTK heading: ") + error.what());
    }
  }

  void handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude))
    {
      rejectPosition("RTK position has no valid fix");
      return;
    }
    if (!latest_quality_.has_value() || *latest_quality_ != required_fix_quality_) {
      rejectPosition("RTK position is not quality 4 fixed");
      return;
    }
    if (!local_cartesian_.has_value()) {
      if (!auto_reference_from_first_fix_) {
        rejectPosition("RTK ENU reference is unavailable");
        return;
      }
      initializeReference(message->latitude, message->longitude, message->altitude);
    }

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_->Forward(
      message->latitude, message->longitude, message->altitude, east, north, up);

    nav_msgs::msg::Odometry antenna_output;
    antenna_output.header = message->header;
    antenna_output.header.frame_id = enu_frame_;
    antenna_output.child_frame_id = antenna_frame_;
    antenna_output.pose.pose.position.x = east;
    antenna_output.pose.pose.position.y = north;
    antenna_output.pose.pose.position.z = up;
    antenna_output.pose.pose.orientation.w = 1.0;
    antenna_output.pose.covariance.fill(0.0);
    if (message->position_covariance_type !=
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
      message->position_covariance[0] > 0.0 && message->position_covariance[4] > 0.0)
    {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          antenna_output.pose.covariance[static_cast<std::size_t>(row * 6 + column)] =
            message->position_covariance[static_cast<std::size_t>(row * 3 + column)];
        }
      }
    } else {
      antenna_output.pose.covariance[0] =
        default_horizontal_std_m_ * default_horizontal_std_m_;
      antenna_output.pose.covariance[7] =
        default_horizontal_std_m_ * default_horizontal_std_m_;
      antenna_output.pose.covariance[14] =
        default_vertical_std_m_ * default_vertical_std_m_;
    }
    antenna_output.pose.covariance[21] = 1.0e6;
    antenna_output.pose.covariance[28] = 1.0e6;
    antenna_output.pose.covariance[35] = 1.0e6;
    antenna_publisher_->publish(antenna_output);

    const rclcpp::Time fix_stamp(message->header.stamp);
    if (!latest_lever_heading_yaw_.has_value() ||
      !latest_lever_heading_variance_.has_value() ||
      !latest_lever_heading_stamp_.has_value() ||
      std::abs((fix_stamp - *latest_lever_heading_stamp_).seconds()) >
      maximum_lever_heading_age_sec_)
    {
      rejectPosition("RTK fixed position has no fresh heading for lidar lever compensation");
      return;
    }

    nav_msgs::msg::Odometry output = antenna_output;
    output.child_frame_id = lidar_frame_;
    const Eigen::Vector3d lidar_position = antennaToSensorPosition(
      {east, north, up}, *latest_lever_heading_yaw_, antenna_to_lidar_flu_);
    output.pose.pose.position.x = lidar_position.x();
    output.pose.pose.position.y = lidar_position.y();
    output.pose.pose.position.z = lidar_position.z();
    output.pose.pose.orientation.z = std::sin(*latest_lever_heading_yaw_ / 2.0);
    output.pose.pose.orientation.w = std::cos(*latest_lever_heading_yaw_ / 2.0);

    const Eigen::Vector2d yaw_jacobian = leverArmYawJacobian(
      *latest_lever_heading_yaw_, antenna_to_lidar_flu_);
    const Eigen::Matrix2d added_position_covariance =
      yaw_jacobian * *latest_lever_heading_variance_ * yaw_jacobian.transpose();
    output.pose.covariance[0] += added_position_covariance(0, 0);
    output.pose.covariance[1] += added_position_covariance(0, 1);
    output.pose.covariance[6] += added_position_covariance(1, 0);
    output.pose.covariance[7] += added_position_covariance(1, 1);
    output.pose.covariance[35] = *latest_lever_heading_variance_;
    position_publisher_->publish(output);
    setStatus("publishing lever-compensated RTK lidar position");
  }

  void rejectPosition(const std::string & reason)
  {
    setStatus(reason);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", reason.c_str());
  }

  void rejectHeading(const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", reason.c_str());
  }

  void setStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    if (status_publisher_) {
      status_publisher_->publish(message);
    }
  }

  std::string fix_topic_;
  std::string quality_topic_;
  std::string heading_topic_;
  std::string heading_solution_topic_;
  std::string position_output_topic_;
  std::string antenna_output_topic_;
  std::string heading_output_topic_;
  std::string reference_topic_;
  std::string status_topic_;
  std::string enu_frame_;
  std::string antenna_frame_;
  std::string lidar_frame_;
  int required_fix_quality_{4};
  HeadingNoisePolicy heading_policy_;
  double heading_solution_timeout_sec_{1.5};
  double default_horizontal_std_m_{0.03};
  double default_vertical_std_m_{0.06};
  Eigen::Vector3d antenna_to_lidar_flu_{Eigen::Vector3d::Zero()};
  double maximum_lever_heading_age_sec_{1.5};
  bool auto_reference_from_first_fix_{true};
  double reference_latitude_deg_{0.0};
  double reference_longitude_deg_{0.0};
  double reference_altitude_m_{0.0};
  std::optional<GeographicLib::LocalCartesian> local_cartesian_;
  std::optional<int> latest_quality_;
  std::optional<std::string> latest_heading_solution_;
  std::optional<rclcpp::Time> latest_solution_receipt_;
  std::optional<double> latest_lever_heading_yaw_;
  std::optional<double> latest_lever_heading_variance_;
  std::optional<rclcpp::Time> latest_lever_heading_stamp_;
  std::optional<sensor_msgs::msg::NavSatFix> pending_reference_message_;
  std::string last_status_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr antenna_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr quality_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    heading_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr solution_subscription_;
};

}  // namespace agribot_offline_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agribot_offline_mapping::RtkOdometryAdapter>());
  rclcpp::shutdown();
  return 0;
}
