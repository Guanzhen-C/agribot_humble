#pragma once

#include <algorithm>
#include <cmath>

namespace agribot_differential_fastlivo_mppi
{
struct TrackCommand
{
  double linear{0.0};
  double angular{0.0};
};

inline TrackCommand limitedTrackCommand(
  TrackCommand previous, TrackCommand target, double dt,
  double linear_acceleration, double angular_acceleration,
  double half_spacing, double steering_efficiency, double max_belt_speed)
{
  const double outer = std::abs(target.linear) +
    std::abs(target.angular) * half_spacing / steering_efficiency;
  const double belt_scale = outer > max_belt_speed ? max_belt_speed / outer : 1.0;
  target.linear *= belt_scale;
  target.angular *= belt_scale;
  dt = std::clamp(dt, 0.0, 0.02);
  const double dv = target.linear - previous.linear;
  const double dw = target.angular - previous.angular;
  double fraction = 1.0;
  if (std::abs(dv) > 1e-12) {
    fraction = std::min(fraction, linear_acceleration * dt / std::abs(dv));
  }
  if (std::abs(dw) > 1e-12) {
    fraction = std::min(fraction, angular_acceleration * dt / std::abs(dw));
  }
  // Common interpolation preserves both rate bounds and the convex belt-speed
  // envelope, including an abrupt switch between driving and in-place turning.
  return {previous.linear + fraction * dv, previous.angular + fraction * dw};
}
}  // namespace agribot_differential_fastlivo_mppi
