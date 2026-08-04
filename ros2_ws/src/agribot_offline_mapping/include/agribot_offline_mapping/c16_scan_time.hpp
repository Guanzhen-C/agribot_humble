#ifndef AGRIBOT_OFFLINE_MAPPING__C16_SCAN_TIME_HPP_
#define AGRIBOT_OFFLINE_MAPPING__C16_SCAN_TIME_HPP_

#include <cmath>
#include <stdexcept>

namespace agribot_offline_mapping
{

inline double normalizedScanStartOffset(
  double minimum_point_time,
  double maximum_point_time,
  bool input_stamp_is_scan_end)
{
  if (!std::isfinite(minimum_point_time) || !std::isfinite(maximum_point_time) ||
    maximum_point_time < minimum_point_time)
  {
    throw std::invalid_argument("invalid C16 point-time range");
  }
  return minimum_point_time - (input_stamp_is_scan_end ? maximum_point_time : 0.0);
}

}  // namespace agribot_offline_mapping

#endif  // AGRIBOT_OFFLINE_MAPPING__C16_SCAN_TIME_HPP_
