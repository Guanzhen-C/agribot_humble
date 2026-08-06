#ifndef AGRIBOT_OFFLINE_MAPPING__RTK_LEVER_ARM_HPP_
#define AGRIBOT_OFFLINE_MAPPING__RTK_LEVER_ARM_HPP_

#include <cmath>

#include <Eigen/Core>

namespace agribot_offline_mapping
{

inline Eigen::Vector3d antennaToSensorPosition(
  const Eigen::Vector3d & antenna_position_enu,
  double vehicle_yaw_enu,
  const Eigen::Vector3d & antenna_to_sensor_flu)
{
  const double cosine = std::cos(vehicle_yaw_enu);
  const double sine = std::sin(vehicle_yaw_enu);
  Eigen::Matrix3d enu_from_vehicle = Eigen::Matrix3d::Identity();
  enu_from_vehicle(0, 0) = cosine;
  enu_from_vehicle(0, 1) = -sine;
  enu_from_vehicle(1, 0) = sine;
  enu_from_vehicle(1, 1) = cosine;
  return antenna_position_enu + enu_from_vehicle * antenna_to_sensor_flu;
}

inline Eigen::Vector2d leverArmYawJacobian(
  double vehicle_yaw_enu,
  const Eigen::Vector3d & antenna_to_sensor_flu)
{
  const double cosine = std::cos(vehicle_yaw_enu);
  const double sine = std::sin(vehicle_yaw_enu);
  return {
    -sine * antenna_to_sensor_flu.x() - cosine * antenna_to_sensor_flu.y(),
    cosine * antenna_to_sensor_flu.x() - sine * antenna_to_sensor_flu.y()};
}

}  // namespace agribot_offline_mapping

#endif  // AGRIBOT_OFFLINE_MAPPING__RTK_LEVER_ARM_HPP_
