#include <cmath>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "agribot_offline_mapping/rtk_lever_arm.hpp"

namespace
{

using agribot_offline_mapping::antennaToSensorPosition;
using agribot_offline_mapping::leverArmYawJacobian;

TEST(RtkLeverArm, ConvertsLeftMasterAntennaToLidarAtCardinalHeadings)
{
  const Eigen::Vector3d antenna(10.0, 20.0, 3.0);
  const Eigen::Vector3d antenna_to_lidar(0.3375, -0.2952585, -0.05176);
  EXPECT_TRUE(
    antennaToSensorPosition(antenna, 0.0, antenna_to_lidar).isApprox(
      Eigen::Vector3d(10.3375, 19.7047415, 2.94824), 1.0e-12));
  EXPECT_TRUE(
    antennaToSensorPosition(antenna, M_PI_2, antenna_to_lidar).isApprox(
      Eigen::Vector3d(10.2952585, 20.3375, 2.94824), 1.0e-12));
}

TEST(RtkLeverArm, YawJacobianMatchesFiniteDifference)
{
  const Eigen::Vector3d antenna = Eigen::Vector3d::Zero();
  const Eigen::Vector3d lever(0.3375, -0.2952585, -0.05176);
  const double yaw = 0.73;
  const double epsilon = 1.0e-7;
  const Eigen::Vector2d numerical =
    (antennaToSensorPosition(antenna, yaw + epsilon, lever) -
    antennaToSensorPosition(antenna, yaw - epsilon, lever)).head<2>() /
    (2.0 * epsilon);
  EXPECT_TRUE(leverArmYawJacobian(yaw, lever).isApprox(numerical, 1.0e-8));
}

}  // namespace
