#pragma once
#include <algorithm>
#include <cmath>

namespace agribot_hitch {
inline double harrow_noise(double x, double y)
{
  const double value = std::sin(x*12.9898+y*78.233)*43758.5453;
  return value-std::floor(value);
}
inline double harrow_bed_height(double across, double noise)
{
  const double edge = std::clamp(std::min(across, 1-across)*14., 0., 1.);
  return .004+edge*(.009+.022*noise);
}
inline double harrow_clod_height(double noise) {return .016+.027*noise;}
inline double harrow_edge_alpha(double across)
{
  const double t = std::clamp(std::min(across, 1-across)*16., 0., 1.);
  return t*t*(3.-2.*t);
}
inline bool harrow_roller_crushes(double x, double y, double half_width)
{
  return x > -.32 && x < .26 && std::abs(y) < half_width;
}
inline double harrow_throw_speed(double speed)
{
  return std::clamp(.65+.75*std::abs(speed), .65, 1.8);
}
}
