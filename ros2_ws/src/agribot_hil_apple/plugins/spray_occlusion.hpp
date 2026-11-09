#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <ignition/math/Vector3.hh>

namespace agribot_hil
{
struct SprayDepthGrid
{
  ignition::math::Vector3d origin, direction, u, v;
  unsigned size{9};
  double tangent{std::tan(.28)};
  std::vector<double> ranges;

  bool Occluded(const ignition::math::Vector3d & point, double padding) const
  {
    if (size < 2 || ranges.size() != size*size) {return true;}
    const auto delta = point-origin;
    const double forward = delta.Dot(direction);
    if (forward <= 0.) {return true;}
    const double x = (delta.Dot(u)/forward/tangent+1.)*.5*(size-1);
    const double y = (delta.Dot(v)/forward/tangent+1.)*.5*(size-1);
    if (x < 0. || y < 0. || x > size-1 || y > size-1) {return true;}
    const auto ix = std::min(size-2, static_cast<unsigned>(x));
    const auto iy = std::min(size-2, static_cast<unsigned>(y));
    // Use the nearest surrounding sample, not interpolation across a wall edge.
    const double limit = std::min({ranges[iy*size+ix], ranges[iy*size+ix+1],
                                  ranges[(iy+1)*size+ix], ranges[(iy+1)*size+ix+1]});
    return delta.Length()+padding >= limit;
  }
};
}  // namespace agribot_hil
