#include <cassert>
#include "spray_occlusion.hpp"

int main()
{
  agribot_hil::SprayDepthGrid grid;
  grid.direction = {1, 0, 0}; grid.u = {0, 1, 0}; grid.v = {0, 0, 1};
  grid.ranges.assign(81, 4.5);
  assert(!grid.Occluded({1., 0., 0.}, .05));
  assert(grid.Occluded({4.48, 0., 0.}, .05));
  grid.ranges.assign(81, 1.);
  assert(!grid.Occluded({.8, 0., 0.}, .05));
  assert(grid.Occluded({.98, 0., 0.}, .05));
  assert(grid.Occluded({1.2, 0., 0.}, .05));
  // Never blend a near wall sample with its clear neighbour to create a leak.
  grid.ranges.assign(81, 4.5); grid.ranges[40] = .7;
  assert(grid.Occluded({.8, .01, .01}, .05));
  assert(grid.Occluded({-.1, 0., 0.}, .05));
  assert(grid.Occluded({1., 1., 0.}, .05));
  grid.ranges.clear();
  assert(grid.Occluded({.5, 0., 0.}, .05));
}
