#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "agribot_hardware_bringup/navsat_frame_conversions.hpp"

namespace navsat = agribot_hardware_bringup::navsat;

namespace {

Eigen::Matrix3d yawRotation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void expectVectorNear(
    const Eigen::Vector3d &actual,
    const Eigen::Vector3d &expected,
    double tolerance = 1e-9) {

    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(actual[index], expected[index], tolerance);
    }
}

}  // namespace

TEST(NavSatFrameConversions, ConvertsRosFluLeverArmToKfGinsFrd) {
    const Eigen::Vector3d lever_flu(-0.2309, 0.1480, 0.10176);
    expectVectorNear(
        navsat::fluToFrd(lever_flu),
        Eigen::Vector3d(-0.2309, -0.1480, -0.10176));
}

TEST(NavSatFrameConversions, ConvertsNedHeadingsToRosBaseOrientation) {
    const Eigen::Matrix3d north = navsat::nedFrdToMapEnuFluMatrix(
        yawRotation(0.0), 0.0);
    const Eigen::Matrix3d east = navsat::nedFrdToMapEnuFluMatrix(
        yawRotation(M_PI / 2.0), 0.0);

    expectVectorNear(
        north * Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY());
    expectVectorNear(
        east * Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitX());
}

TEST(NavSatFrameConversions, ShiftsImuPoseToRearAxleCenter) {
    const Eigen::Vector3d base_to_imu(0.1425, 0.0, 0.143);
    const Eigen::Vector3d imu_position(4.1425, 2.0, 1.143);

    expectVectorNear(
        navsat::imuMapPositionToBaseMapPosition(
            imu_position, Eigen::Matrix3d::Identity(), base_to_imu),
        Eigen::Vector3d(4.0, 2.0, 1.0));

    expectVectorNear(
        navsat::imuMapPositionToBaseMapPosition(
            imu_position, yawRotation(M_PI / 2.0), base_to_imu),
        imu_position - Eigen::Vector3d(0.0, 0.1425, 0.143));
}

TEST(NavSatFrameConversions, ShiftsImuVelocityToRearAxleInBaseFrame) {
    const Eigen::Vector3d base_to_imu(0.1425, 0.0, 0.143);
    const Eigen::Vector3d angular_velocity(0.0, 0.0, 1.0);
    const Eigen::Vector3d imu_velocity_map(1.0, 0.1425, 0.0);

    expectVectorNear(
        navsat::imuMapVelocityToBaseFluVelocity(
            imu_velocity_map,
            Eigen::Matrix3d::Identity(),
            angular_velocity,
            base_to_imu),
        Eigen::Vector3d(1.0, 0.0, 0.0));
}
