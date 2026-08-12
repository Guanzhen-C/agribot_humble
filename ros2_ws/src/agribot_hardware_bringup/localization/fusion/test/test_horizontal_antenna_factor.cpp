#include <cmath>

#include <gtest/gtest.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "agribot_hardware_bringup/horizontal_antenna_factor.hpp"

namespace agribot_hardware_bringup::fusion
{

TEST(HorizontalAntennaFactor, AppliesRotatedLeverArmAndIgnoresHeight)
{
  const gtsam::Pose3 pose(
    gtsam::Rot3::Rz(M_PI_2), gtsam::Point3(1.0, 2.0, 0.4));
  const gtsam::Point3 lever_arm(0.2, 0.3, 0.5);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
  const HorizontalAntennaFactor factor(
    gtsam::Symbol('x', 0), gtsam::Point3(0.7, 2.2, 99.0), lever_arm, noise);

  const gtsam::Vector error = factor.evaluateError(pose);
  ASSERT_EQ(error.size(), 2);
  EXPECT_NEAR(error.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(error.y(), 0.0, 1.0e-9);
}

TEST(HorizontalAntennaFactor, AnalyticalJacobianMatchesNumericalDerivative)
{
  const gtsam::Pose3 pose(
    gtsam::Rot3::RzRyRx(0.02, -0.03, 0.7),
    gtsam::Point3(4.0, -1.0, 0.2));
  const gtsam::Point3 lever_arm(0.1425, 0.2952585, 0.78476);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
  const HorizontalAntennaFactor factor(
    gtsam::Symbol('x', 3), gtsam::Point3(4.1, -0.8, 10.0), lever_arm, noise);

  gtsam::Matrix analytical;
  factor.evaluateError(pose, analytical);
  const gtsam::Matrix numerical = gtsam::numericalDerivative11<
    gtsam::Vector, gtsam::Pose3>(
    [&factor](const gtsam::Pose3 & value) {return factor.evaluateError(value);},
    pose, 1.0e-6);

  EXPECT_TRUE(gtsam::assert_equal(numerical, analytical, 1.0e-6));
}

TEST(GravityAttitudeFactor, ConstrainsRollPitchWithoutConstrainingYaw)
{
  const gtsam::Rot3 measured_attitude = gtsam::Rot3::RzRyRx(0.08, -0.05, 0.4);
  const gtsam::Unit3 up_in_base(
    measured_attitude.unrotate(gtsam::Vector3(0.0, 0.0, 1.0)));
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.01);
  const gtsam::Pose3AttitudeFactor factor(
    gtsam::Symbol('x', 4), gtsam::Unit3(0.0, 0.0, 1.0), noise, up_in_base);

  const gtsam::Pose3 same_tilt_different_yaw(
    gtsam::Rot3::RzRyRx(0.08, -0.05, -1.2), gtsam::Point3(2.0, 3.0, 4.0));
  const gtsam::Vector yaw_only_error = factor.evaluateError(same_tilt_different_yaw);
  EXPECT_NEAR(yaw_only_error.norm(), 0.0, 1.0e-9);

  const gtsam::Pose3 wrong_roll(
    gtsam::Rot3::RzRyRx(0.20, -0.05, -1.2), gtsam::Point3(2.0, 3.0, 4.0));
  EXPECT_GT(factor.evaluateError(wrong_roll).norm(), 0.05);
}

}  // namespace agribot_hardware_bringup::fusion
