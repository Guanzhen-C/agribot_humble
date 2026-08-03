#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "agribot_hardware_bringup/navsat_frame_conversions.hpp"

namespace navsat = agribot_hardware_bringup::navsat;

namespace {

Eigen::Matrix3d yawRotation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d rotationFromVector(const Eigen::Vector3d &rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < 1e-15) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Vector3d rotationVector(const Eigen::Matrix3d &rotation) {
    const Eigen::AngleAxisd angle_axis(rotation);
    if (std::abs(angle_axis.angle()) < 1e-15) {
        return Eigen::Vector3d::Zero();
    }
    return angle_axis.angle() * angle_axis.axis();
}

void expectVectorNear(
    const Eigen::Vector3d &actual,
    const Eigen::Vector3d &expected,
    double tolerance = 1e-9) {

    for (int index = 0; index < 3; ++index) {
        EXPECT_NEAR(actual[index], expected[index], tolerance);
    }
}

template<typename DerivedA, typename DerivedB>
void expectMatrixNear(
    const Eigen::MatrixBase<DerivedA> &actual,
    const Eigen::MatrixBase<DerivedB> &expected,
    double tolerance = 1e-9) {

    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    for (Eigen::Index row = 0; row < actual.rows(); ++row) {
        for (Eigen::Index col = 0; col < actual.cols(); ++col) {
            EXPECT_NEAR(actual(row, col), expected(row, col), tolerance)
                << "at (" << row << ", " << col << ")";
        }
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

TEST(NavSatFrameConversions, RotatesMapPositionUncertaintyIntoNed) {
    const Eigen::Vector3d standard_deviation_map(1.0, 2.0, 3.0);

    expectVectorNear(
        navsat::mapEnuStandardDeviationToNed(
            standard_deviation_map, 0.0),
        Eigen::Vector3d(2.0, 1.0, 3.0));
    expectVectorNear(
        navsat::mapEnuStandardDeviationToNed(
            standard_deviation_map, M_PI / 2.0),
        Eigen::Vector3d(1.0, 2.0, 3.0));
}

TEST(NavSatFrameConversions, ConsumesEachFreshRtkHeadingOnlyOnce) {
    EXPECT_TRUE(navsat::shouldUseRtkHeading(
        100.18, 100.0, std::nullopt, 0.25));
    EXPECT_FALSE(navsat::shouldUseRtkHeading(
        100.20, 100.0, 100.0, 0.25));
    EXPECT_FALSE(navsat::shouldUseRtkHeading(
        101.30, 101.0, 100.0, 0.25));
    EXPECT_TRUE(navsat::shouldUseRtkHeading(
        101.18, 101.0, 100.0, 0.25));
}

TEST(NavSatFrameConversions, PlacesSensorFromRearAxlePose) {
    const Eigen::Vector3d base_position(3.0, 4.0, 0.0);
    const Eigen::Vector3d base_to_sensor(-0.0884, 0.1480, 0.24476);

    expectVectorNear(
        navsat::baseMapPositionToSensorMapPosition(
            base_position, yawRotation(M_PI / 2.0), base_to_sensor),
        base_position + Eigen::Vector3d(-0.1480, -0.0884, 0.24476));
}

TEST(NavSatFrameConversions, RecoversRearAxleThroughCompleteThreeDimensionalLeverChain) {
    const Eigen::Vector3d base_position(7.0, -2.0, 0.4);
    const Eigen::Vector3d base_to_imu(0.1425, 0.0, 0.143);
    const Eigen::Vector3d imu_to_antenna(-0.2309, 0.1480, 0.10176);
    const Eigen::Matrix3d map_from_base =
        (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    const Eigen::Vector3d antenna_position =
        navsat::baseMapPositionToSensorMapPosition(
            base_position,
            map_from_base,
            base_to_imu + imu_to_antenna);
    const Eigen::Vector3d imu_position =
        antenna_position - map_from_base * imu_to_antenna;

    expectVectorNear(
        navsat::imuMapPositionToBaseMapPosition(
            imu_position, map_from_base, base_to_imu),
        base_position);
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

TEST(NavSatFrameConversions, PropagatesIndependentTwistCovarianceToRearAxle) {
    const Eigen::Matrix3d velocity_covariance =
        Eigen::Vector3d(0.04, 0.09, 0.16).asDiagonal();
    const Eigen::Matrix3d angular_covariance =
        Eigen::Vector3d(0.01, 0.02, 0.03).asDiagonal();
    const navsat::Matrix6d covariance =
        navsat::independentImuTwistCovarianceToBaseFlu(
            velocity_covariance,
            angular_covariance,
            Eigen::Vector3d(0.1425, 0.0, 0.143));

    EXPECT_LT((covariance - covariance.transpose()).norm(), 1e-12);
    EXPECT_GT(
        covariance.selfadjointView<Eigen::Lower>()
            .eigenvalues()
            .minCoeff(),
        0.0);
    expectMatrixNear(
        covariance.block<3, 3>(3, 3), angular_covariance);
}

TEST(NavSatFrameConversions, PropagatesImuPoseCovarianceToRearAxle) {
    navsat::Matrix6d imu_covariance = navsat::Matrix6d::Zero();
    imu_covariance(0, 0) = 1.0;
    imu_covariance(1, 1) = 2.0;
    imu_covariance(2, 2) = 3.0;
    imu_covariance(3, 3) = 4.0;
    imu_covariance(4, 4) = 5.0;
    imu_covariance(5, 5) = 6.0;

    const navsat::Matrix6d base_covariance =
        navsat::imuPoseCovarianceToBaseMapPoseCovariance(
            imu_covariance,
            Eigen::Matrix3d::Identity(),
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(1.0, 0.0, 0.0));

    EXPECT_NEAR(base_covariance(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(base_covariance(1, 1), 8.0, 1e-9);
    EXPECT_NEAR(base_covariance(2, 2), 8.0, 1e-9);
    EXPECT_NEAR(base_covariance(1, 5), -6.0, 1e-9);
    EXPECT_NEAR(base_covariance(2, 4), 5.0, 1e-9);
    EXPECT_NEAR(base_covariance(5, 1), -6.0, 1e-9);
    EXPECT_NEAR(base_covariance(4, 2), 5.0, 1e-9);
}

TEST(NavSatFrameConversions, PropagatesCorrelatedCovarianceUsingFiniteDifferenceJacobian) {
    const double map_to_ned_yaw = 0.37;
    const double c = std::cos(map_to_ned_yaw);
    const double s = std::sin(map_to_ned_yaw);
    Eigen::Matrix3d map_from_ned;
    map_from_ned << -s, c, 0.0,
                     c, s, 0.0,
                   0.0, 0.0, -1.0;
    const Eigen::Matrix3d ned_from_frd =
        (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3d frd_from_flu =
        Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
    const Eigen::Vector3d base_to_imu_flu(0.1425, -0.027, 0.143);
    const Eigen::Vector3d base_to_imu_frd = frd_from_flu * base_to_imu_flu;
    const Eigen::Vector3d estimated_imu_position_ned(12.0, -3.0, 1.2);
    const Eigen::Matrix3d estimated_map_from_base =
        map_from_ned * ned_from_frd * frd_from_flu;
    const Eigen::Vector3d estimated_base_position_map =
        map_from_ned *
        (estimated_imu_position_ned - ned_from_frd * base_to_imu_frd);

    auto output_pose_error = [&](const Eigen::Matrix<double, 6, 1> &error_state) {
        // KF-GINS position error is estimate-minus-truth, while Phi-angle is
        // truth-minus-estimate and acts on the left in the NED frame.
        const Eigen::Vector3d true_imu_position_ned =
            estimated_imu_position_ned - error_state.head<3>();
        const Eigen::Matrix3d true_ned_from_frd =
            rotationFromVector(error_state.tail<3>()) * ned_from_frd;
        const Eigen::Vector3d true_base_position_map =
            map_from_ned *
            (true_imu_position_ned - true_ned_from_frd * base_to_imu_frd);
        const Eigen::Matrix3d true_map_from_base =
            map_from_ned * true_ned_from_frd * frd_from_flu;

        Eigen::Matrix<double, 6, 1> pose_error;
        pose_error.head<3>() =
            true_base_position_map - estimated_base_position_map;
        pose_error.tail<3>() = rotationVector(
            true_map_from_base * estimated_map_from_base.transpose());
        return pose_error;
    };

    navsat::Matrix6d finite_difference_jacobian;
    constexpr double epsilon = 1e-6;
    for (int col = 0; col < 6; ++col) {
        Eigen::Matrix<double, 6, 1> positive =
            Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> negative =
            Eigen::Matrix<double, 6, 1>::Zero();
        positive[col] = epsilon;
        negative[col] = -epsilon;
        finite_difference_jacobian.col(col) =
            (output_pose_error(positive) - output_pose_error(negative)) /
            (2.0 * epsilon);
    }

    navsat::Matrix6d covariance_factor = navsat::Matrix6d::Zero();
    covariance_factor.diagonal() << 0.40, 0.50, 0.60, 0.07, 0.08, 0.09;
    covariance_factor(3, 0) = 0.030;
    covariance_factor(4, 0) = -0.015;
    covariance_factor(4, 1) = 0.025;
    covariance_factor(5, 1) = -0.020;
    covariance_factor(5, 2) = 0.035;
    const navsat::Matrix6d imu_covariance =
        covariance_factor * covariance_factor.transpose();

    const navsat::Matrix6d expected_base_covariance =
        finite_difference_jacobian * imu_covariance *
        finite_difference_jacobian.transpose();
    const navsat::Matrix6d actual_base_covariance =
        navsat::imuPoseCovarianceToBaseMapPoseCovariance(
            imu_covariance,
            map_from_ned,
            ned_from_frd,
            base_to_imu_flu);

    expectMatrixNear(
        actual_base_covariance, expected_base_covariance, 1e-9);
    EXPECT_LT(
        (actual_base_covariance - actual_base_covariance.transpose()).norm(),
        1e-12);
    EXPECT_GT(
        actual_base_covariance.selfadjointView<Eigen::Lower>()
            .eigenvalues()
            .minCoeff(),
        0.0);
}
