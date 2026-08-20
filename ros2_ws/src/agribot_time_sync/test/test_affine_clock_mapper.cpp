#include <gtest/gtest.h>

#include "agribot_time_sync/affine_clock_mapper.hpp"

TEST(AffineClockMapper, RecoversClockScaleAndLowDelayBoundary)
{
  agribot_time_sync::ClockMapperConfig config;
  config.window_size = 5000;
  config.min_samples = 100;
  config.fit_interval_samples = 10;
  config.min_span_sec = 5.0;
  config.max_scale_error_ppm = 200.0;
  config.transport_delay_sec = 0.006;
  config.lower_delay_quantile = 0.0;
  agribot_time_sync::AffineClockMapper mapper(config);

  agribot_time_sync::ClockMapperResult result;
  constexpr double device_rate_error = 50.0e-6;
  for (int index = 0; index < 5000; ++index) {
    const double actual_time = 1000.0 + 0.01 * index;
    const double device_time = 0.01 * index * (1.0 + device_rate_error);
    const double jitter = 0.001 * static_cast<double>(index % 7);
    result = mapper.observe(device_time, actual_time + 0.006 + jitter);
  }

  EXPECT_TRUE(result.synchronized);
  EXPECT_NEAR(result.scale_error_ppm, -50.0, 0.5);
  EXPECT_NEAR(result.stamp_sec, 1049.99, 2.0e-4);
  EXPECT_GE(result.estimated_delay_sec, 0.006);
}

TEST(AffineClockMapper, ResetsAfterDeviceClockRestart)
{
  agribot_time_sync::ClockMapperConfig config;
  config.min_samples = 3;
  config.min_span_sec = 0.01;
  agribot_time_sync::AffineClockMapper mapper(config);
  mapper.observe(10.0, 100.0);
  mapper.observe(10.1, 100.1);
  mapper.observe(10.2, 100.2);
  const auto result = mapper.observe(0.1, 100.3);
  EXPECT_EQ(result.reset_count, 1U);
  EXPECT_FALSE(result.synchronized);
}

TEST(WrappingCounter32, DistinguishesWrapFromRestart)
{
  agribot_time_sync::WrappingCounter32 counter;
  bool reset = false;
  EXPECT_EQ(counter.unwrap(0xFFFFFFF0U, reset), 0xFFFFFFF0ULL);
  EXPECT_FALSE(reset);
  EXPECT_EQ(counter.unwrap(0x00000010U, reset), 0x100000010ULL);
  EXPECT_FALSE(reset);
  counter.reset();
  EXPECT_EQ(counter.unwrap(1000U, reset), 1000U);
  EXPECT_EQ(counter.unwrap(100U, reset), 100U);
  EXPECT_TRUE(reset);
}
