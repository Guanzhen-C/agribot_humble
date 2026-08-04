#ifndef AGRIBOT_OFFLINE_MAPPING__GEOREFERENCE_FIT_HPP_
#define AGRIBOT_OFFLINE_MAPPING__GEOREFERENCE_FIT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace agribot_offline_mapping
{

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct GeoreferenceSample
{
  Eigen::Vector3d enu_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d map_position{Eigen::Vector3d::Zero()};
  double enu_yaw{0.0};
  double map_yaw{0.0};
  bool has_yaw{false};
};

struct GeoreferenceFit
{
  Eigen::Isometry3d map_from_enu{Eigen::Isometry3d::Identity()};
  double horizontal_rmse_m{std::numeric_limits<double>::infinity()};
  double yaw_rmse_rad{std::numeric_limits<double>::infinity()};
  double trajectory_span_m{0.0};
  std::vector<std::size_t> inlier_indices;
};

inline double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::runtime_error("cannot calculate the median of an empty set");
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2U == 0U) {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    result = 0.5 * (result + *lower);
  }
  return result;
}

inline Eigen::Isometry3d fitPlanarTransform(
  const std::vector<GeoreferenceSample> & samples,
  const std::vector<std::size_t> & indices)
{
  if (indices.size() < 2U) {
    throw std::runtime_error("at least two samples are required for planar fitting");
  }
  Eigen::Vector2d enu_centroid = Eigen::Vector2d::Zero();
  Eigen::Vector2d map_centroid = Eigen::Vector2d::Zero();
  double z_translation = 0.0;
  for (const std::size_t index : indices) {
    enu_centroid += samples.at(index).enu_position.head<2>();
    map_centroid += samples.at(index).map_position.head<2>();
    z_translation +=
      samples.at(index).map_position.z() - samples.at(index).enu_position.z();
  }
  const double inverse_count = 1.0 / static_cast<double>(indices.size());
  enu_centroid *= inverse_count;
  map_centroid *= inverse_count;
  z_translation *= inverse_count;

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const std::size_t index : indices) {
    covariance +=
      (samples.at(index).enu_position.head<2>() - enu_centroid) *
      (samples.at(index).map_position.head<2>() - map_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix2d> decomposition(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d rotation = decomposition.matrixV() * decomposition.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix2d reflection = Eigen::Matrix2d::Identity();
    reflection(1, 1) = -1.0;
    rotation = decomposition.matrixV() * reflection * decomposition.matrixU().transpose();
  }

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear().topLeftCorner<2, 2>() = rotation;
  result.translation().head<2>() = map_centroid - rotation * enu_centroid;
  result.translation().z() = z_translation;
  return result;
}

inline GeoreferenceFit fitGeoreference(
  const std::vector<GeoreferenceSample> & samples,
  std::size_t minimum_samples,
  double minimum_inlier_distance_m,
  double mad_multiplier,
  int robust_iterations = 3)
{
  if (minimum_samples < 2U || samples.size() < minimum_samples ||
    minimum_inlier_distance_m <= 0.0 || mad_multiplier <= 0.0 || robust_iterations < 1)
  {
    throw std::runtime_error("invalid georeference fitting inputs");
  }
  std::vector<std::size_t> inliers(samples.size());
  std::iota(inliers.begin(), inliers.end(), 0U);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  for (int iteration = 0; iteration < robust_iterations; ++iteration) {
    transform = fitPlanarTransform(samples, inliers);
    std::vector<double> residuals;
    residuals.reserve(inliers.size());
    for (const std::size_t index : inliers) {
      const Eigen::Vector2d predicted =
        (transform * samples[index].enu_position).head<2>();
      residuals.push_back((samples[index].map_position.head<2>() - predicted).norm());
    }
    const double residual_median = median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (const double residual : residuals) {
      deviations.push_back(std::abs(residual - residual_median));
    }
    const double robust_sigma = 1.4826 * median(deviations);
    const double threshold = std::max(
      minimum_inlier_distance_m, residual_median + mad_multiplier * robust_sigma);
    std::vector<std::size_t> next_inliers;
    for (std::size_t offset = 0; offset < inliers.size(); ++offset) {
      if (residuals[offset] <= threshold) {
        next_inliers.push_back(inliers[offset]);
      }
    }
    if (next_inliers.size() < minimum_samples) {
      throw std::runtime_error("too few georeference inliers after robust rejection");
    }
    if (next_inliers == inliers) {
      break;
    }
    inliers = std::move(next_inliers);
  }
  transform = fitPlanarTransform(samples, inliers);

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const std::size_t index : inliers) {
    centroid += samples[index].enu_position.head<2>();
  }
  centroid /= static_cast<double>(inliers.size());

  double squared_position_error = 0.0;
  double squared_yaw_error = 0.0;
  std::size_t yaw_count = 0U;
  double maximum_radius = 0.0;
  const double transform_yaw = std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
  for (const std::size_t index : inliers) {
    const Eigen::Vector2d predicted =
      (transform * samples[index].enu_position).head<2>();
    squared_position_error +=
      (samples[index].map_position.head<2>() - predicted).squaredNorm();
    maximum_radius = std::max(
      maximum_radius, (samples[index].enu_position.head<2>() - centroid).norm());
    if (samples[index].has_yaw) {
      const double error = wrapAngle(
        samples[index].map_yaw - samples[index].enu_yaw - transform_yaw);
      squared_yaw_error += error * error;
      ++yaw_count;
    }
  }

  GeoreferenceFit result;
  result.map_from_enu = transform;
  result.horizontal_rmse_m = std::sqrt(
    squared_position_error / static_cast<double>(inliers.size()));
  result.yaw_rmse_rad = yaw_count == 0U ? std::numeric_limits<double>::infinity() :
    std::sqrt(squared_yaw_error / static_cast<double>(yaw_count));
  result.trajectory_span_m = 2.0 * maximum_radius;
  result.inlier_indices = std::move(inliers);
  return result;
}

}  // namespace agribot_offline_mapping

#endif  // AGRIBOT_OFFLINE_MAPPING__GEOREFERENCE_FIT_HPP_
