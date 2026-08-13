#ifndef AGRIBOT_HARDWARE_BRINGUP__GEOREFERENCE_TEST_CONVERSIONS_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__GEOREFERENCE_TEST_CONVERSIONS_HPP_

#include <cmath>

#include <Eigen/Geometry>

namespace agribot_hardware_bringup::navsat
{

inline double wrapAngleRadians(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double normalizeHeadingDegrees(const double heading_deg)
{
  double result = std::fmod(heading_deg, 360.0);
  if (result < 0.0) {
    result += 360.0;
  }
  return result;
}

// GNSS heading is clockwise from true north; ROS ENU yaw is counter-clockwise from east.
inline double gnssHeadingDegreesToEnuYaw(const double heading_deg)
{
  return wrapAngleRadians(M_PI_2 - heading_deg * M_PI / 180.0);
}

inline double enuYawToGnssHeadingDegrees(const double yaw_rad)
{
  return normalizeHeadingDegrees(90.0 - yaw_rad * 180.0 / M_PI);
}

inline Eigen::Isometry3d planarPose(
  const Eigen::Vector3d & position,
  const double yaw_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position;
  pose.linear() =
    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

inline Eigen::Isometry3d baseEnuPoseFromAntennaMeasurement(
  const Eigen::Vector3d & antenna_position_enu,
  const double base_yaw_enu,
  const Eigen::Vector3d & base_to_antenna)
{
  const Eigen::Matrix3d enu_from_base =
    Eigen::AngleAxisd(base_yaw_enu, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return planarPose(
    antenna_position_enu - enu_from_base * base_to_antenna,
    base_yaw_enu);
}

inline Eigen::Vector3d antennaMapPositionFromBasePose(
  const Eigen::Isometry3d & map_to_base,
  const Eigen::Vector3d & base_to_antenna)
{
  return map_to_base * base_to_antenna;
}

}  // namespace agribot_hardware_bringup::navsat

#endif  // AGRIBOT_HARDWARE_BRINGUP__GEOREFERENCE_TEST_CONVERSIONS_HPP_
