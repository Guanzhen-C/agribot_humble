#include <gtest/gtest.h>
#include "tracked_slip_policy.hpp"

using agribot_vehicle_description::tracked_lateral_compliance;

TEST(TrackedSlipPolicy, IdleAndStraightRetainLowCompliance)
{
  EXPECT_DOUBLE_EQ(tracked_lateral_compliance(0, 0, 0, 150, .1, .01, .02, .08), .1);
  EXPECT_DOUBLE_EQ(tracked_lateral_compliance(.2, .2, .2, 150, .1, .01, .02, .08), .1);
  EXPECT_DOUBLE_EQ(tracked_lateral_compliance(.2, .201, .2, 150, .1, .01, .02, .08), .1);
}

TEST(TrackedSlipPolicy, TurningRecoversControlledSkidInBothDirections)
{
  const double a = tracked_lateral_compliance(-.09, .09, .09, 150, .1, .01, .02, .08);
  const double b = tracked_lateral_compliance(.09, -.09, -.09, 150, .1, .01, .02, .08);
  EXPECT_NEAR(a * .09 / 150, .01, 1e-12);
  EXPECT_DOUBLE_EQ(a, b);
}

TEST(TrackedSlipPolicy, TransitionIsBoundedAndContinuous)
{
  const double low = tracked_lateral_compliance(.1, .12, .1, 150, .1, .01, .02, .08);
  const double mid = tracked_lateral_compliance(.1, .15, .1, 150, .1, .01, .02, .08);
  const double high = tracked_lateral_compliance(.1, .18, .1, 150, .1, .01, .02, .08);
  EXPECT_NEAR(low, .1, 1e-12);
  EXPECT_NEAR(mid, .5 * (.1 + 15), 1e-12);
  EXPECT_NEAR(high, 15, 1e-12);
}
