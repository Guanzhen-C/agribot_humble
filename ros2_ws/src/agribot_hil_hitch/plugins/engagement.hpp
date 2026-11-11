#ifndef AGRIBOT_HITCH_ENGAGEMENT_HPP
#define AGRIBOT_HITCH_ENGAGEMENT_HPP
#include <cmath>
namespace agribot_hitch {
inline bool engaged(double speed, double lowest, double x, double y,
                    double min_x, double max_x, double min_y, double max_y)
{
    return std::isfinite(speed) && std::isfinite(lowest) && std::isfinite(x) && std::isfinite(y) &&
        std::abs(speed) > .03 && lowest >= -.15 && lowest <= .055 &&
        x >= min_x && x <= max_x && y >= min_y && y <= max_y;
}
}
#endif
