#include "../plugins/harrow_surface.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main()
{
  for (int x = -100; x <= 100; ++x) {
    for (int y = -100; y <= 100; ++y) {
      const double noise = agribot_hitch::harrow_noise(x*.023, y*.041);
      assert(noise >= 0 && noise < 1);
      const double height = agribot_hitch::harrow_bed_height((x+100)/200., noise);
      assert(height >= .004 && height <= .035);
      assert(.010+agribot_hitch::harrow_clod_height(noise) < .054);
    }
  }
  assert(agribot_hitch::harrow_bed_height(0, 1) == .004);
  assert(agribot_hitch::harrow_bed_height(1, 1) == .004);
  assert(agribot_hitch::harrow_edge_alpha(0) == 0.);
  assert(agribot_hitch::harrow_edge_alpha(.5) == 1.);
  assert(agribot_hitch::harrow_edge_alpha(1) == 0.);
  assert(agribot_hitch::harrow_roller_crushes(0, 1.2, 1.27));
  assert(!agribot_hitch::harrow_roller_crushes(.4, 0, 1.27));
  assert(!agribot_hitch::harrow_roller_crushes(-1., 0, 1.27));
  assert(!agribot_hitch::harrow_roller_crushes(0, 1.4, 1.27));
  assert(agribot_hitch::harrow_throw_speed(0.) < agribot_hitch::harrow_throw_speed(.5));
  assert(agribot_hitch::harrow_throw_speed(10.) <= 1.8);
  std::puts("Harrow shallow bed, feathered edges and small clods PASS");
}
