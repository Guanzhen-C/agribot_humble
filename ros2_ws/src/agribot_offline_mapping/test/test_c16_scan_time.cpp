#include <gtest/gtest.h>

#include <stdexcept>

#include "agribot_offline_mapping/c16_scan_time.hpp"

namespace agribot_offline_mapping
{
namespace
{

TEST(C16ScanTime, ConvertsEndStampedCloudToScanStart)
{
  EXPECT_NEAR(normalizedScanStartOffset(0.000003, 0.100078, true), -0.100075, 1.0e-12);
}

TEST(C16ScanTime, PreservesStartStampedCloud)
{
  EXPECT_NEAR(normalizedScanStartOffset(0.000003, 0.100078, false), 0.000003, 1.0e-12);
}

TEST(C16ScanTime, RejectsAnInvalidRange)
{
  EXPECT_THROW(normalizedScanStartOffset(0.1, 0.0, true), std::invalid_argument);
}

}  // namespace
}  // namespace agribot_offline_mapping
