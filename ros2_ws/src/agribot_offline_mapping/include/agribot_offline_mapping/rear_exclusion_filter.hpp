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

struct AxisAlignedExclusionBox
{
  bool enabled{true};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d half_extent{Eigen::Vector3d::Constant(0.05)};

  bool valid() const
  {
    return center.allFinite() && half_extent.allFinite() &&
           (half_extent.array() > 0.0).all();
  }

  bool contains(const Eigen::Vector3d & point_in_base) const
  {
    return enabled &&
           ((point_in_base - center).array().abs() <= half_extent.array()).all();
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

inline bool shouldExcludeSelfPoint(
  const Eigen::Vector3d & point_in_lidar,
  const Eigen::Isometry3d & base_from_lidar,
  const RearExclusionRegion & rear_region,
  const AxisAlignedExclusionBox & left_antenna,
  const AxisAlignedExclusionBox & right_antenna)
{
  const Eigen::Vector3d point_in_base = base_from_lidar * point_in_lidar;
  return rear_region.contains(point_in_base) ||
         left_antenna.contains(point_in_base) ||
         right_antenna.contains(point_in_base);
}

}  // namespace agribot_offline_mapping

#endif  // AGRIBOT_OFFLINE_MAPPING__REAR_EXCLUSION_FILTER_HPP_
