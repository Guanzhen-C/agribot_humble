#ifndef AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_

#include <cmath>
#include <optional>

#include <Eigen/Dense>

namespace agribot_hardware_bringup::navsat {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

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

inline Eigen::Vector3d mapEnuStandardDeviationToNed(
    const Eigen::Vector3d &standard_deviation_map,
    double extra_yaw_rad) {

    const Eigen::Matrix3d covariance_map =
        standard_deviation_map.array().square().matrix().asDiagonal();
    const Eigen::Matrix3d ned_from_map =
        mapEnuFromNed(extra_yaw_rad).transpose();
    const Eigen::Vector3d variance_ned =
        (ned_from_map * covariance_map * ned_from_map.transpose())
            .diagonal()
            .cwiseMax(0.0);
    return variance_ned.array().sqrt();
}

inline bool shouldUseRtkHeading(
    double fix_time,
    double heading_time,
    const std::optional<double> &last_used_heading_time,
    double timeout_sec) {

    if (!std::isfinite(fix_time) || !std::isfinite(heading_time) ||
        !std::isfinite(timeout_sec) || timeout_sec < 0.0 ||
        std::abs(fix_time - heading_time) > timeout_sec) {
        return false;
    }
    return !last_used_heading_time.has_value() ||
           heading_time > *last_used_heading_time + 1e-6;
}

inline Eigen::Vector3d baseMapPositionToSensorMapPosition(
    const Eigen::Vector3d &base_position_map,
    const Eigen::Matrix3d &map_from_base_flu,
    const Eigen::Vector3d &base_to_sensor_flu) {

    return base_position_map + map_from_base_flu * base_to_sensor_flu;
}

inline Eigen::Vector3d imuMapPositionToBaseMapPosition(
    const Eigen::Vector3d &imu_position_map,
    const Eigen::Matrix3d &map_from_base_flu,
    const Eigen::Vector3d &base_to_imu_flu) {

    return imu_position_map - map_from_base_flu * base_to_imu_flu;
}

inline Matrix6d imuPoseCovarianceToBaseMapPoseCovariance(
    const Matrix6d &imu_pose_covariance_ned,
    const Eigen::Matrix3d &map_from_ned,
    const Eigen::Matrix3d &ned_from_frd,
    const Eigen::Vector3d &base_to_imu_flu) {

    const Eigen::Vector3d base_to_imu_frd = fluToFrd(base_to_imu_flu);
    const Eigen::Vector3d base_to_imu_ned =
        ned_from_frd * base_to_imu_frd;
    Eigen::Matrix3d lever_skew;
    lever_skew << 0.0, -base_to_imu_ned.z(), base_to_imu_ned.y(),
                  base_to_imu_ned.z(), 0.0, -base_to_imu_ned.x(),
                  -base_to_imu_ned.y(), base_to_imu_ned.x(), 0.0;

    // KF-GINS defines position error as estimate minus truth, while its
    // Phi-angle attitude correction is truth minus estimate. Map both to the
    // reported pose's truth-minus-estimate perturbation before propagation.
    Matrix6d jacobian = Matrix6d::Zero();
    jacobian.block<3, 3>(0, 0) = -map_from_ned;
    jacobian.block<3, 3>(0, 3) = map_from_ned * lever_skew;
    jacobian.block<3, 3>(3, 3) = map_from_ned;
    return jacobian * imu_pose_covariance_ned * jacobian.transpose();
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

inline Matrix6d independentImuTwistCovarianceToBaseFlu(
    const Eigen::Matrix3d &imu_velocity_covariance_flu,
    const Eigen::Matrix3d &angular_velocity_covariance_flu,
    const Eigen::Vector3d &base_to_imu_flu) {

    Eigen::Matrix3d lever_skew;
    lever_skew << 0.0, -base_to_imu_flu.z(), base_to_imu_flu.y(),
                  base_to_imu_flu.z(), 0.0, -base_to_imu_flu.x(),
                  -base_to_imu_flu.y(), base_to_imu_flu.x(), 0.0;
    Matrix6d covariance = Matrix6d::Zero();
    covariance.block<3, 3>(0, 0) =
        imu_velocity_covariance_flu +
        lever_skew * angular_velocity_covariance_flu *
            lever_skew.transpose();
    covariance.block<3, 3>(0, 3) =
        lever_skew * angular_velocity_covariance_flu;
    covariance.block<3, 3>(3, 0) =
        covariance.block<3, 3>(0, 3).transpose();
    covariance.block<3, 3>(3, 3) = angular_velocity_covariance_flu;
    return covariance;
}

}  // namespace agribot_hardware_bringup::navsat

#endif  // AGRIBOT_HARDWARE_BRINGUP__NAVSAT_FRAME_CONVERSIONS_HPP_
