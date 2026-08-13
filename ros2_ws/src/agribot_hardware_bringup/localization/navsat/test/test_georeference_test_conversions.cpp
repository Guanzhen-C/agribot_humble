#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "agribot_hardware_bringup/georeference_test_conversions.hpp"

namespace navsat = agribot_hardware_bringup::navsat;

TEST(GeoreferenceTestConversions, ConvertsClockwiseNorthHeadingToRosEnuYaw)
{
  EXPECT_NEAR(navsat::gnssHeadingDegreesToEnuYaw(0.0), M_PI_2, 1.0e-12);
  EXPECT_NEAR(navsat::gnssHeadingDegreesToEnuYaw(90.0), 0.0, 1.0e-12);
  EXPECT_NEAR(navsat::gnssHeadingDegreesToEnuYaw(180.0), -M_PI_2, 1.0e-12);
  EXPECT_NEAR(
    navsat::enuYawToGnssHeadingDegrees(
      navsat::gnssHeadingDegreesToEnuYaw(271.25)),
    271.25, 1.0e-12);
}

TEST(GeoreferenceTestConversions, RemovesMasterAntennaLeverArmFromRtkMeasurement)
{
  const Eigen::Vector3d base_to_antenna(0.1425, 0.2952585, 0.78476);
  const double yaw = M_PI_2;
  const Eigen::Vector3d base_position(12.0, -4.0, 0.5);
  const Eigen::Matrix3d enu_from_base =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d antenna_position =
    base_position + enu_from_base * base_to_antenna;

  const Eigen::Isometry3d enu_to_base =
    navsat::baseEnuPoseFromAntennaMeasurement(
    antenna_position, yaw, base_to_antenna);
  EXPECT_NEAR((enu_to_base.translation() - base_position).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(
    std::atan2(enu_to_base.linear()(1, 0), enu_to_base.linear()(0, 0)),
    yaw, 1.0e-12);
  EXPECT_NEAR(
    (navsat::antennaMapPositionFromBasePose(enu_to_base, base_to_antenna) -
    antenna_position).norm(),
    0.0, 1.0e-12);
}
