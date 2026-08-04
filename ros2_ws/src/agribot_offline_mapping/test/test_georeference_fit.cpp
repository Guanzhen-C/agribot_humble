#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "agribot_offline_mapping/georeference_fit.hpp"

namespace
{

using agribot_offline_mapping::GeoreferenceSample;

TEST(GeoreferenceFit, RecoversPlanarTransformAndRejectsOutlier)
{
  constexpr double yaw = 0.37;
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d translation(12.0, -4.0, 1.2);
  std::vector<GeoreferenceSample> samples;
  for (int index = 0; index < 40; ++index) {
    const Eigen::Vector3d enu(
      0.35 * index, 1.5 * std::sin(0.2 * index), 0.1 * std::cos(0.1 * index));
    GeoreferenceSample sample;
    sample.enu_position = enu;
    sample.map_position = rotation * enu + translation;
    sample.map_position.x() += 0.01 * std::sin(index);
    sample.enu_yaw = 0.1;
    sample.map_yaw = 0.1 + yaw;
    sample.has_yaw = true;
    samples.push_back(sample);
  }
  samples[10].map_position.x() += 4.0;
  samples[10].map_position.y() -= 3.0;

  const auto result = agribot_offline_mapping::fitGeoreference(
    samples, 20U, 0.10, 3.0);
  EXPECT_LT(result.horizontal_rmse_m, 0.02);
  // The position fit includes centimetre-scale noise, so its recovered yaw is
  // expected to differ from the exact heading observations by a small amount.
  EXPECT_LT(result.yaw_rmse_rad, 1.0e-4);
  EXPECT_GT(result.trajectory_span_m, 10.0);
  EXPECT_EQ(result.inlier_indices.size(), 39U);
  EXPECT_NEAR(result.map_from_enu.translation().x(), translation.x(), 0.02);
  EXPECT_NEAR(result.map_from_enu.translation().y(), translation.y(), 0.02);
  EXPECT_NEAR(result.map_from_enu.translation().z(), translation.z(), 1.0e-6);
  EXPECT_NEAR(
    std::atan2(result.map_from_enu.linear()(1, 0), result.map_from_enu.linear()(0, 0)),
    yaw, 0.002);
}

TEST(GeoreferenceFit, RejectsInsufficientSamples)
{
  std::vector<GeoreferenceSample> samples(3);
  EXPECT_THROW(
    agribot_offline_mapping::fitGeoreference(samples, 10U, 0.10, 3.0),
    std::runtime_error);
}

}  // namespace
