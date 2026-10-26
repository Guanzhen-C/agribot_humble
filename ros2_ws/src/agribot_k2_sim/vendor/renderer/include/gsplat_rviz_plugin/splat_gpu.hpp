// Derived from RVizSplat; Apache-2.0, see renderer/LICENSE.
#pragma once
#include <cstddef>
namespace gsplat_rviz_plugin {
struct SplatGPU {
 float center[3]; float alpha;
 float covA[3]; float _pad0;
 float covB[3]; float _pad1;
 float sh[1][4];
};
static_assert(sizeof(SplatGPU)==64,"Portable DC-only Gaussian record");
}
