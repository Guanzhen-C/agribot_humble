#ifndef AGRIBOT_OFFLINE_MAPPING__REAR_EXCLUSION_FILTER_HPP_
#define AGRIBOT_OFFLINE_MAPPING__REAR_EXCLUSION_FILTER_HPP_

#include <cmath>

#include <Eigen/Geometry>

namespace agribot_offline_mapping
{

struct RearExclusionRegion
{
  bool enabled{true};
  double minimum_x{-4.0};
  double maximum_x{-0.1275};
  double half_width{0.60};

  bool valid() const
  {
    return std::isfinite(minimum_x) && std::isfinite(maximum_x) &&
           std::isfinite(half_width) && minimum_x < maximum_x &&
           maximum_x <= 0.0 && half_width > 0.0;
  }

  bool contains(const Eigen::Vector3d & point_in_base) const
  {
    return enabled && point_in_base.x() >= minimum_x &&
           point_in_base.x() <= maximum_x &&
           std::abs(point_in_base.y()) <= half_width;
  }
};

inline Eigen::Isometry3d transformFromXyzRpy(
  const Eigen::Vector3d & translation,
  const Eigen::Vector3d & roll_pitch_yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  transform.linear() =
    (Eigen::AngleAxisd(roll_pitch_yaw.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(roll_pitch_yaw.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll_pitch_yaw.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

inline bool shouldExcludeRearPoint(
  const Eigen::Vector3d & point_in_lidar,
  const Eigen::Isometry3d & base_from_lidar,
  const RearExclusionRegion & region)
{
  return region.contains(base_from_lidar * point_in_lidar);
}

}  // namespace agribot_offline_mapping

#endif  // AGRIBOT_OFFLINE_MAPPING__REAR_EXCLUSION_FILTER_HPP_
