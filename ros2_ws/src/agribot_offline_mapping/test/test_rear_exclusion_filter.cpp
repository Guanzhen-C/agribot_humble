#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "agribot_offline_mapping/rear_exclusion_filter.hpp"

namespace agribot_offline_mapping
{
namespace
{

TEST(RearExclusionFilter, UsesRearAxleCenteredBounds)
{
  const RearExclusionRegion region{true, -4.0, -0.1275, 0.60};
  const Eigen::Isometry3d base_from_lidar = transformFromXyzRpy(
    Eigen::Vector3d(0.48, 0.0, 0.233), Eigen::Vector3d::Zero());

  EXPECT_TRUE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(-2.28, 0.0, 0.0), base_from_lidar, region));
  EXPECT_FALSE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(-2.28, 0.61, 0.0), base_from_lidar, region));
  EXPECT_FALSE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(-4.49, 0.0, 0.0), base_from_lidar, region));
  EXPECT_FALSE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(-0.60, 0.0, 0.0), base_from_lidar, region));
}

TEST(RearExclusionFilter, CanBeDisabled)
{
  const RearExclusionRegion region{false, -4.0, -0.1275, 0.60};

  EXPECT_FALSE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Isometry3d::Identity(), region));
}

TEST(RearExclusionFilter, AppliesSensorRotation)
{
  const RearExclusionRegion region{true, -4.0, -0.1275, 0.60};
  const Eigen::Isometry3d base_from_lidar = transformFromXyzRpy(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 1.5707963267948966));

  EXPECT_TRUE(
    shouldExcludeRearPoint(
      Eigen::Vector3d(0.0, 1.0, 0.0), base_from_lidar, region));
}

}  // namespace
}  // namespace agribot_offline_mapping
