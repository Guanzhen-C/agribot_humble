#ifndef AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_

#include <cmath>

#include <Eigen/Dense>

namespace agribot_hardware_bringup::navsat {

inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d &flu) {
    return {flu.x(), -flu.y(), -flu.z()};
}

inline Eigen::Matrix3d mapEnuFromNed(double extra_yaw_rad) {
    const double c = std::cos(extra_yaw_rad);
    const double s = std::sin(extra_yaw_rad);
    Eigen::Matrix3d transform;
    transform << -s, c, 0.0,
                  c, s, 0.0,
                0.0, 0.0, -1.0;
    return transform;
}

inline Eigen::Matrix3d nedFrdToMapEnuFluMatrix(
    const Eigen::Matrix3d &ned_from_frd,
    double extra_yaw_rad) {

    const Eigen::Matrix3d frd_from_flu =
        Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
    return mapEnuFromNed(extra_yaw_rad) * ned_from_frd * frd_from_flu;
}

inline Eigen::Vector3d imuMapPositionToBaseMapPosition(
    const Eigen::Vector3d &imu_position_map,
    const Eigen::Matrix3d &map_from_base_flu,
    const Eigen::Vector3d &base_to_imu_flu) {

    return imu_position_map - map_from_base_flu * base_to_imu_flu;
}

inline Eigen::Vector3d imuMapVelocityToBaseFluVelocity(
    const Eigen::Vector3d &imu_velocity_map,
    const Eigen::Matrix3d &map_from_base_flu,
    const Eigen::Vector3d &angular_velocity_flu,
    const Eigen::Vector3d &base_to_imu_flu) {

    const Eigen::Vector3d imu_velocity_flu =
        map_from_base_flu.transpose() * imu_velocity_map;
    return imu_velocity_flu - angular_velocity_flu.cross(base_to_imu_flu);
}

}  // namespace agribot_hardware_bringup::navsat

#endif  // AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_
