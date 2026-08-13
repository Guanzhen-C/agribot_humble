#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "agribot_hardware_bringup/georeference_test_conversions.hpp"
#include "agribot_hardware_bringup/map_georeference.hpp"

namespace agribot_hardware_bringup
{
namespace
{

namespace navsat = agribot_hardware_bringup::navsat;
using namespace std::chrono_literals;

Eigen::Vector3d vector3Parameter(
  rclcpp::Node & node,
  const std::string & name,
  const std::vector<double> & default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, default_value);
  if (values.size() != 3U ||
    !std::all_of(values.begin(), values.end(), [](const double value) {
      return std::isfinite(value);
    }))
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
    throw std::runtime_error("input orientation quaternion is invalid");
  }
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(yaw / 2.0);
  result.w = std::cos(yaw / 2.0);
  return result;
}

Eigen::Isometry3d poseIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("input orientation quaternion is invalid");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  result.linear() = quaternion.normalized().toRotationMatrix();
  return result;
}

}  // namespace

class GeoreferenceTestBridge final : public rclcpp::Node
{
public:
  GeoreferenceTestBridge()
  : Node("georeference_test_bridge")
  {
    georeference_file_ = declare_parameter<std::string>("georeference_file", "");
    map_file_ = declare_parameter<std::string>("map_file", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    clicked_point_topic_ = declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    clicked_geopoint_topic_ = declare_parameter<std::string>(
      "clicked_geopoint_topic", "/georeference_test/clicked_geopoint");
    rtk_input_topic_ = declare_parameter<std::string>(
      "rtk_input_topic", "/georeference_test/rtk_input");
    initial_pose_topic_ = declare_parameter<std::string>(
      "initial_pose_topic", "/georeference_test/initialpose");
    seed_pose_topic_ = declare_parameter<std::string>(
      "seed_pose_topic", "/georeference_test/rtk_seed_pose");
    refined_pose_input_topic_ = declare_parameter<std::string>(
      "refined_pose_input_topic", "/localization_pose");
    refined_pose_topic_ = declare_parameter<std::string>(
      "refined_pose_topic", "/georeference_test/refined_pose");
    refined_geopose_topic_ = declare_parameter<std::string>(
      "refined_geopose_topic", "/georeference_test/refined_geopose");
    localizer_status_topic_ = declare_parameter<std::string>(
      "localizer_status_topic", "/localization/status");
    result_topic_ = declare_parameter<std::string>(
      "result_topic", "/georeference_test/result");
    input_horizontal_std_m_ = declare_parameter<double>("input_horizontal_std_m", 0.05);
    input_heading_std_deg_ = declare_parameter<double>("input_heading_std_deg", 2.0);
    base_to_antenna_ = vector3Parameter(
      *this, "base_to_master_antenna_m", {0.1425, 0.2952585, 0.78476});

    if (georeference_file_.empty() || map_file_.empty() ||
      input_horizontal_std_m_ <= 0.0 || input_heading_std_deg_ <= 0.0)
    {
      throw std::runtime_error("invalid georeference test parameters");
    }
    loadAndVerifyGeoreference();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    clicked_geopoint_publisher_ =
      create_publisher<geographic_msgs::msg::GeoPointStamped>(
      clicked_geopoint_topic_, latched_qos);
    clicked_map_point_publisher_ =
      create_publisher<geometry_msgs::msg::PointStamped>(
      "/georeference_test/clicked_map_point", latched_qos);
    initial_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic_, 10);
    seed_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      seed_pose_topic_, latched_qos);
    refined_pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      refined_pose_topic_, latched_qos);
    refined_geopose_publisher_ =
      create_publisher<geographic_msgs::msg::GeoPoseStamped>(
      refined_geopose_topic_, latched_qos);
    result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic_, latched_qos);

    clicked_point_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic_, 10,
      std::bind(&GeoreferenceTestBridge::handleClickedPoint, this, std::placeholders::_1));
    rtk_input_subscription_ = create_subscription<geographic_msgs::msg::GeoPoseStamped>(
      rtk_input_topic_, 10,
      std::bind(&GeoreferenceTestBridge::handleRtkInput, this, std::placeholders::_1));
    refined_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      refined_pose_input_topic_, 10,
      std::bind(&GeoreferenceTestBridge::handleRefinedPose, this, std::placeholders::_1));
    localizer_status_subscription_ = create_subscription<std_msgs::msg::String>(
      localizer_status_topic_, latched_qos,
      std::bind(&GeoreferenceTestBridge::handleLocalizerStatus, this, std::placeholders::_1));
    seed_dispatch_timer_ = create_wall_timer(
      100ms, std::bind(&GeoreferenceTestBridge::tryDispatchInitialPose, this));

    RCLCPP_INFO(
      get_logger(),
      "Georeference test ready: click RViz Publish Point for map-to-WGS84; publish GeoPoseStamped "
      "on %s for RTK-seeded NDT/GICP localization",
      rtk_input_topic_.c_str());
  }

private:
  void loadAndVerifyGeoreference()
  {
    georeference_ = navsat::loadMapGeoreference(georeference_file_);
    const std::filesystem::path map_path(map_file_);
    if (!std::filesystem::is_regular_file(map_path)) {
      throw std::runtime_error("PCD map is unavailable: " + map_path.string());
    }
    if (map_path.stem().string() != georeference_.map_id) {
      throw std::runtime_error("PCD map name does not match georeference map ID");
    }
    if (navsat::fingerprintFile(map_path) != georeference_.map_fingerprint) {
      throw std::runtime_error("PCD map fingerprint does not match georeference metadata");
    }
    map_from_enu_ = navsat::mapFromEnuTransform(georeference_);
    enu_from_map_ = map_from_enu_.inverse();
    local_cartesian_.Reset(
      georeference_.reference_latitude_deg,
      georeference_.reference_longitude_deg,
      georeference_.reference_altitude_m);
  }

  geographic_msgs::msg::GeoPoint mapPointToGeopoint(const Eigen::Vector3d & point_map) const
  {
    const Eigen::Vector3d point_enu = enu_from_map_ * point_map;
    geographic_msgs::msg::GeoPoint result;
    local_cartesian_.Reverse(
      point_enu.x(), point_enu.y(), point_enu.z(),
      result.latitude, result.longitude, result.altitude);
    return result;
  }

  void handleClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring clicked point in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const Eigen::Vector3d point_map(message->point.x, message->point.y, message->point.z);
    if (!point_map.allFinite()) {
      RCLCPP_WARN(get_logger(), "Ignoring clicked point containing a non-finite coordinate");
      return;
    }

    geographic_msgs::msg::GeoPointStamped output;
    output.header = message->header;
    output.header.frame_id = "wgs84";
    output.position = mapPointToGeopoint(point_map);
    clicked_geopoint_publisher_->publish(output);

    geometry_msgs::msg::PointStamped selected = *message;
    selected.header.frame_id = map_frame_;
    clicked_map_point_publisher_->publish(selected);
    RCLCPP_INFO(
      get_logger(),
      "Clicked map point (%.3f, %.3f) -> latitude=%.9f longitude=%.9f "
      "derived_altitude=%.3f m",
      point_map.x(), point_map.y(), output.position.latitude,
      output.position.longitude, output.position.altitude);
  }

  void handleRtkInput(const geographic_msgs::msg::GeoPoseStamped::SharedPtr message)
  {
    const auto & input = message->pose;
    if (!std::isfinite(input.position.latitude) ||
      !std::isfinite(input.position.longitude) ||
      std::abs(input.position.latitude) > 90.0 ||
      std::abs(input.position.longitude) > 180.0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid RTK latitude or longitude");
      return;
    }

    try {
      const double yaw_enu = yawFromQuaternion(input.orientation);
      const double altitude = std::isfinite(input.position.altitude) ?
        input.position.altitude : georeference_.reference_altitude_m;
      double east = 0.0;
      double north = 0.0;
      double up = 0.0;
      local_cartesian_.Forward(
        input.position.latitude, input.position.longitude, altitude,
        east, north, up);
      const Eigen::Isometry3d enu_to_base = navsat::baseEnuPoseFromAntennaMeasurement(
        {east, north, up}, yaw_enu, base_to_antenna_);
      Eigen::Isometry3d map_to_base = map_from_enu_ * enu_to_base;
      const double yaw_map = std::atan2(map_to_base.linear()(1, 0), map_to_base.linear()(0, 0));
      map_to_base.translation().z() = 0.0;
      map_to_base.linear() =
        Eigen::AngleAxisd(yaw_map, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      geometry_msgs::msg::PoseWithCovarianceStamped seed;
      seed.header.stamp = now();
      seed.header.frame_id = map_frame_;
      seed.pose.pose.position.x = map_to_base.translation().x();
      seed.pose.pose.position.y = map_to_base.translation().y();
      seed.pose.pose.position.z = 0.0;
      seed.pose.pose.orientation = yawQuaternion(yaw_map);
      const double horizontal_variance =
        input_horizontal_std_m_ * input_horizontal_std_m_ +
        georeference_.horizontal_rmse_m * georeference_.horizontal_rmse_m;
      const double heading_std_rad = input_heading_std_deg_ * M_PI / 180.0;
      seed.pose.covariance[0] = horizontal_variance;
      seed.pose.covariance[7] = horizontal_variance;
      seed.pose.covariance[14] = 0.25;
      seed.pose.covariance[21] = 1.0e6;
      seed.pose.covariance[28] = 1.0e6;
      const double georeference_yaw_std_rad =
        georeference_.yaw_rmse_deg * M_PI / 180.0;
      seed.pose.covariance[35] =
        heading_std_rad * heading_std_rad +
        georeference_yaw_std_rad * georeference_yaw_std_rad;

      last_seed_pose_ = map_to_base;
      pending_initial_pose_ = seed;
      awaiting_localizer_start_ = false;
      awaiting_refinement_ = false;
      result_published_ = false;
      seed_pose_publisher_->publish(seed);
      tryDispatchInitialPose();

      RCLCPP_INFO(
        get_logger(),
        "RTK input latitude=%.9f longitude=%.9f heading=%.3f deg -> "
        "rear-axle map seed x=%.3f y=%.3f yaw=%.3f deg; waiting for NDT/GICP",
        input.position.latitude, input.position.longitude,
        navsat::enuYawToGnssHeadingDegrees(yaw_enu),
        map_to_base.translation().x(), map_to_base.translation().y(),
        yaw_map * 180.0 / M_PI);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid RTK pose input: %s", error.what());
    }
  }

  void tryDispatchInitialPose()
  {
    if (!pending_initial_pose_.has_value() ||
      initial_pose_publisher_->get_subscription_count() == 0U)
    {
      return;
    }
    pending_initial_pose_->header.stamp = now();
    initial_pose_publisher_->publish(*pending_initial_pose_);
    pending_initial_pose_.reset();
    awaiting_localizer_start_ = true;
    RCLCPP_INFO(
      get_logger(), "RTK map seed delivered to the NDT/GICP localizer");
  }

  void handleLocalizerStatus(const std_msgs::msg::String::SharedPtr message)
  {
    if (awaiting_localizer_start_ &&
      message->data.find("initial pose received") != std::string::npos)
    {
      awaiting_localizer_start_ = false;
    }
    if (last_seed_pose_.has_value() && !awaiting_localizer_start_ &&
      message->data.find("initial localization accepted") != std::string::npos)
    {
      awaiting_refinement_ = true;
    }
  }

  void handleRefinedPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!last_seed_pose_.has_value() || !awaiting_refinement_ || result_published_) {
      return;
    }
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      return;
    }

    try {
      const Eigen::Isometry3d map_to_base = poseIsometry(message->pose.pose);
      const double yaw_map = std::atan2(
        map_to_base.linear()(1, 0), map_to_base.linear()(0, 0));
      const double map_from_enu_yaw = std::atan2(
        map_from_enu_.linear()(1, 0), map_from_enu_.linear()(0, 0));
      const double yaw_enu = navsat::wrapAngleRadians(yaw_map - map_from_enu_yaw);
      const Eigen::Vector3d antenna_map =
        navsat::antennaMapPositionFromBasePose(map_to_base, base_to_antenna_);
      const auto antenna_geopoint = mapPointToGeopoint(antenna_map);

      geometry_msgs::msg::PoseWithCovarianceStamped refined = *message;
      refined.header.frame_id = map_frame_;
      refined_pose_publisher_->publish(refined);

      geographic_msgs::msg::GeoPoseStamped refined_geo;
      refined_geo.header = message->header;
      refined_geo.header.frame_id = "wgs84";
      refined_geo.pose.position = antenna_geopoint;
      refined_geo.pose.orientation = yawQuaternion(yaw_enu);
      refined_geopose_publisher_->publish(refined_geo);

      const Eigen::Vector3d translation_correction =
        map_to_base.translation() - last_seed_pose_->translation();
      const double yaw_correction = navsat::wrapAngleRadians(
        yaw_map - std::atan2(
          last_seed_pose_->linear()(1, 0), last_seed_pose_->linear()(0, 0)));
      std::ostringstream result;
      result.setf(std::ios::fixed);
      result.precision(9);
      result << "NDT/GICP accepted: map x=" << map_to_base.translation().x()
             << " y=" << map_to_base.translation().y();
      result.precision(3);
      result << " yaw_deg=" << yaw_map * 180.0 / M_PI;
      result.precision(9);
      result << " master_lat=" << antenna_geopoint.latitude
             << " master_lon=" << antenna_geopoint.longitude;
      result.precision(3);
      result << " heading_deg=" << navsat::enuYawToGnssHeadingDegrees(yaw_enu)
             << " seed_correction_m=" << translation_correction.head<2>().norm()
             << " seed_correction_yaw_deg=" << yaw_correction * 180.0 / M_PI;
      std_msgs::msg::String result_message;
      result_message.data = result.str();
      result_publisher_->publish(result_message);
      RCLCPP_INFO(get_logger(), "%s", result_message.data.c_str());
      result_published_ = true;
      awaiting_refinement_ = false;
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid refined localization pose: %s", error.what());
    }
  }

  std::string georeference_file_;
  std::string map_file_;
  std::string map_frame_;
  std::string clicked_point_topic_;
  std::string clicked_geopoint_topic_;
  std::string rtk_input_topic_;
  std::string initial_pose_topic_;
  std::string seed_pose_topic_;
  std::string refined_pose_input_topic_;
  std::string refined_pose_topic_;
  std::string refined_geopose_topic_;
  std::string localizer_status_topic_;
  std::string result_topic_;
  double input_horizontal_std_m_{0.05};
  double input_heading_std_deg_{2.0};
  Eigen::Vector3d base_to_antenna_{Eigen::Vector3d::Zero()};
  navsat::MapGeoreference georeference_;
  GeographicLib::LocalCartesian local_cartesian_;
  Eigen::Isometry3d map_from_enu_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d enu_from_map_{Eigen::Isometry3d::Identity()};
  std::optional<Eigen::Isometry3d> last_seed_pose_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_initial_pose_;
  bool awaiting_localizer_start_{false};
  bool awaiting_refinement_{false};
  bool result_published_{false};

  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr
    clicked_geopoint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    clicked_map_point_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    seed_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    refined_pose_publisher_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr
    refined_geopose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    clicked_point_subscription_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr
    rtk_input_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    refined_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    localizer_status_subscription_;
  rclcpp::TimerBase::SharedPtr seed_dispatch_timer_;
};

}  // namespace agribot_hardware_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<agribot_hardware_bringup::GeoreferenceTestBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("georeference_test_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
