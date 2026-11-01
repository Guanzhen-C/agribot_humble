#pragma once

namespace agribot_differential_fastlivo_mppi
{
constexpr unsigned int kScannedRayCategory = 0x2;
// Preserve SimpleTrackedVehiclePlugin's native robot / belt / side bits.
constexpr unsigned int kScannedVehicleCategory = 0x10000000;
constexpr unsigned int kScannedRayMask = 0x0fffffff & ~0x2;
constexpr unsigned int kScannedPhysicalMask = kScannedVehicleCategory;
}  // namespace agribot_differential_fastlivo_mppi
