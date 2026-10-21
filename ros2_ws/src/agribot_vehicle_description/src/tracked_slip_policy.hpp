#pragma once

#include <algorithm>
#include <cmath>

namespace agribot_vehicle_description
{
inline double tracked_lateral_compliance(
  double left_speed, double right_speed, double wheel_speed, double normal_force,
  double straight_compliance, double turning_slip, double turn_start, double turn_full)
{
  const double blend = std::clamp(
    (std::abs(right_speed - left_speed) - turn_start) / (turn_full - turn_start), 0.0, 1.0);
  const double turning_compliance = turning_slip * normal_force /
    std::max(std::abs(wheel_speed), 0.01);
  return (1.0 - blend) * straight_compliance + blend * turning_compliance;
}
}  // namespace agribot_vehicle_description
