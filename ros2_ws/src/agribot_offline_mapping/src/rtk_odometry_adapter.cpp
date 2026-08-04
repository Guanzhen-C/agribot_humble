#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace agribot_offline_mapping
{
namespace
{

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

Eigen::Matrix3d yawRotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
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
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/lio_sam/odometry/gps");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/lio_sam/rtk_reference");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/lio_sam/rtk_adapter_status");
    enu_frame_ = declare_parameter<std::string>("enu_frame", "enu");
    target_frame_ = declare_parameter<std::string>("target_frame", "lidar_link");
    required_fix_quality_ = declare_parameter<int>("required_fix_quality", 4);
    allowed_heading_solutions_ = declare_parameter<std::vector<std::string>>(
      "allowed_heading_solutions", {"L1_INT", "NARROW_INT"});
    heading_timeout_sec_ = declare_parameter<double>("heading_timeout_sec", 1.5);
    maximum_heading_std_deg_ = declare_parameter<double>("maximum_heading_std_deg", 3.0);
    default_horizontal_std_m_ = declare_parameter<double>("default_horizontal_std_m", 0.03);
    default_vertical_std_m_ = declare_parameter<double>("default_vertical_std_m", 0.06);
    auto_reference_from_first_fix_ = declare_parameter<bool>(
      "auto_reference_from_first_fix", true);
    reference_latitude_deg_ = declare_parameter<double>("reference_latitude_deg", 0.0);
    reference_longitude_deg_ = declare_parameter<double>("reference_longitude_deg", 0.0);
    reference_altitude_m_ = declare_parameter<double>("reference_altitude_m", 0.0);
    base_to_antenna_ = vector3Parameter(
      *this, "base_to_master_antenna_m", {-0.0884, 0.1480, 0.24476});
    base_to_target_ = vector3Parameter(
      *this, "base_to_target_m", {0.48, 0.0, 0.233});

    if (required_fix_quality_ < 1 || allowed_heading_solutions_.empty() ||
      heading_timeout_sec_ <= 0.0 || maximum_heading_std_deg_ <= 0.0 ||
      default_horizontal_std_m_ <= 0.0 || default_vertical_std_m_ <= 0.0)
    {
      throw std::runtime_error("invalid RTK adapter quality or covariance parameters");
    }
    if (!auto_reference_from_first_fix_) {
      initializeReference(
        reference_latitude_deg_, reference_longitude_deg_, reference_altitude_m_);
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
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
    setStatus("waiting for RTK fixed position and integer-fixed dual-antenna heading");
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
      const double variance = message->pose.covariance[35];
      if (!std::isfinite(variance) || variance <= 0.0) {
        throw std::runtime_error("RTK heading covariance is invalid");
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
    if (!latest_heading_solution_.has_value()) {
      return false;
    }
    const std::string & status = *latest_heading_solution_;
    if (status.rfind("SOL_COMPUTED,", 0U) != 0U) {
      return false;
    }
    const std::string type = status.substr(status.find(',') + 1U);
    return std::find(
      allowed_heading_solutions_.begin(), allowed_heading_solutions_.end(), type) !=
           allowed_heading_solutions_.end();
  }

  void handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
      !std::isfinite(message->altitude))
    {
      reject("RTK position has no valid fix");
      return;
    }
    if (!latest_quality_.has_value() || *latest_quality_ != required_fix_quality_) {
      reject("RTK position is not quality 4 fixed");
      return;
    }
    if (!latest_heading_yaw_.has_value() || !latest_heading_stamp_.has_value() ||
      !latest_heading_variance_.has_value() || !headingSolutionAccepted())
    {
      reject("RTK heading is not integer fixed");
      return;
    }

    const rclcpp::Time fix_stamp(message->header.stamp);
    if (std::abs((fix_stamp - *latest_heading_stamp_).seconds()) > heading_timeout_sec_ ||
      !latest_solution_receipt_.has_value() ||
      (now() - *latest_solution_receipt_).seconds() > heading_timeout_sec_)
    {
      reject("RTK heading is stale relative to the position fix");
      return;
    }
    const double maximum_heading_std_rad = maximum_heading_std_deg_ * M_PI / 180.0;
    if (*latest_heading_variance_ > maximum_heading_std_rad * maximum_heading_std_rad) {
      reject("RTK heading covariance exceeds the mapping limit");
      return;
    }

    if (!local_cartesian_.has_value()) {
      if (!auto_reference_from_first_fix_) {
        reject("RTK ENU reference is unavailable");
        return;
      }
      initializeReference(message->latitude, message->longitude, message->altitude);
      if (pending_reference_message_.has_value()) {
        reference_publisher_->publish(*pending_reference_message_);
        pending_reference_message_.reset();
      }
    }

    double antenna_east = 0.0;
    double antenna_north = 0.0;
    double antenna_up = 0.0;
    local_cartesian_->Forward(
      message->latitude, message->longitude, message->altitude,
      antenna_east, antenna_north, antenna_up);
    const Eigen::Vector3d antenna_enu(antenna_east, antenna_north, antenna_up);
    const Eigen::Matrix3d enu_from_base = yawRotation(*latest_heading_yaw_);
    const Eigen::Vector3d target_enu =
      antenna_enu + enu_from_base * (base_to_target_ - base_to_antenna_);

    nav_msgs::msg::Odometry output;
    output.header = message->header;
    output.header.frame_id = enu_frame_;
    output.child_frame_id = target_frame_;
    output.pose.pose.position.x = target_enu.x();
    output.pose.pose.position.y = target_enu.y();
    output.pose.pose.position.z = target_enu.z();
    output.pose.pose.orientation.z = std::sin(*latest_heading_yaw_ / 2.0);
    output.pose.pose.orientation.w = std::cos(*latest_heading_yaw_ / 2.0);

    Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Zero();
    if (message->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
      message->position_covariance[0] > 0.0 && message->position_covariance[4] > 0.0)
    {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          position_covariance(row, column) =
            message->position_covariance[static_cast<std::size_t>(row * 3 + column)];
        }
      }
    } else {
      position_covariance.diagonal() = Eigen::Vector3d(
        default_horizontal_std_m_ * default_horizontal_std_m_,
        default_horizontal_std_m_ * default_horizontal_std_m_,
        default_vertical_std_m_ * default_vertical_std_m_);
    }
    const Eigen::Vector3d target_delta = base_to_target_ - base_to_antenna_;
    const double sine = std::sin(*latest_heading_yaw_);
    const double cosine = std::cos(*latest_heading_yaw_);
    const Eigen::Vector3d yaw_jacobian(
      -sine * target_delta.x() - cosine * target_delta.y(),
      cosine * target_delta.x() - sine * target_delta.y(), 0.0);
    position_covariance +=
      yaw_jacobian * (*latest_heading_variance_) * yaw_jacobian.transpose();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        output.pose.covariance[static_cast<std::size_t>(row * 6 + column)] =
          position_covariance(row, column);
      }
    }
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = *latest_heading_variance_;
    publisher_->publish(output);
    setStatus("publishing fixed RTK target pose in the stored ENU frame");
  }

  void reject(const std::string & reason)
  {
    setStatus(reason);
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
  std::string output_topic_;
  std::string reference_topic_;
  std::string status_topic_;
  std::string enu_frame_;
  std::string target_frame_;
  int required_fix_quality_{4};
  std::vector<std::string> allowed_heading_solutions_;
  double heading_timeout_sec_{1.5};
  double maximum_heading_std_deg_{3.0};
  double default_horizontal_std_m_{0.03};
  double default_vertical_std_m_{0.06};
  bool auto_reference_from_first_fix_{true};
  double reference_latitude_deg_{0.0};
  double reference_longitude_deg_{0.0};
  double reference_altitude_m_{0.0};
  Eigen::Vector3d base_to_antenna_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d base_to_target_{Eigen::Vector3d::Zero()};
  std::optional<GeographicLib::LocalCartesian> local_cartesian_;
  std::optional<int> latest_quality_;
  std::optional<double> latest_heading_yaw_;
  std::optional<double> latest_heading_variance_;
  std::optional<rclcpp::Time> latest_heading_stamp_;
  std::optional<std::string> latest_heading_solution_;
  std::optional<rclcpp::Time> latest_solution_receipt_;
  std::optional<sensor_msgs::msg::NavSatFix> pending_reference_message_;
  std::string last_status_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
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
