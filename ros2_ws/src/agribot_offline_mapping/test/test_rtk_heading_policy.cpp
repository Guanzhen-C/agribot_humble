#include <cmath>

#include <gtest/gtest.h>

#include "agribot_offline_mapping/rtk_heading_policy.hpp"

namespace
{

using agribot_offline_mapping::HeadingNoisePolicy;
using agribot_offline_mapping::effectiveHeadingVariance;

TEST(RtkHeadingPolicy, UsesFixedAndFloatFloors)
{
  const HeadingNoisePolicy policy;
  constexpr double pi = 3.14159265358979323846;
  const auto fixed = effectiveHeadingVariance("SOL_COMPUTED,L1_INT", 1.0e-8, policy);
  const auto floating = effectiveHeadingVariance("SOL_COMPUTED,L1_FLOAT", 1.0e-8, policy);
  ASSERT_TRUE(fixed.has_value());
  ASSERT_TRUE(floating.has_value());
  EXPECT_NEAR(*fixed, std::pow(pi / 180.0, 2), 1.0e-12);
  EXPECT_NEAR(*floating, std::pow(5.0 * pi / 180.0, 2), 1.0e-12);
}

TEST(RtkHeadingPolicy, PreservesLargerReceiverUncertainty)
{
  const HeadingNoisePolicy policy;
  const auto result = effectiveHeadingVariance("SOL_COMPUTED,NARROW_INT", 0.04, policy);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 0.04);
}

TEST(RtkHeadingPolicy, RejectsInvalidSolutionsWithoutAffectingPositionPolicy)
{
  const HeadingNoisePolicy policy;
  EXPECT_FALSE(effectiveHeadingVariance("INSUFFICIENT_OBS,NONE", 0.01, policy).has_value());
  EXPECT_FALSE(effectiveHeadingVariance("SOL_COMPUTED,NONE", 0.01, policy).has_value());
}

}  // namespace
