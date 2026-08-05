#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace agribot_offline_mapping
{

struct HeadingNoisePolicy
{
  std::vector<std::string> fixed_solutions{"L1_INT", "NARROW_INT"};
  std::vector<std::string> float_solutions{"L1_FLOAT", "NARROW_FLOAT"};
  double fixed_std_floor_deg{1.0};
  double float_std_floor_deg{5.0};
};

inline std::optional<std::string> computedSolutionType(const std::string & solution)
{
  constexpr char prefix[] = "SOL_COMPUTED,";
  if (solution.rfind(prefix, 0U) != 0U || solution.size() <= sizeof(prefix) - 1U) {
    return std::nullopt;
  }
  return solution.substr(sizeof(prefix) - 1U);
}

inline std::optional<double> effectiveHeadingVariance(
  const std::string & solution, double reported_variance,
  const HeadingNoisePolicy & policy)
{
  if (!std::isfinite(reported_variance) || reported_variance <= 0.0 ||
    !std::isfinite(policy.fixed_std_floor_deg) || policy.fixed_std_floor_deg <= 0.0 ||
    !std::isfinite(policy.float_std_floor_deg) || policy.float_std_floor_deg <= 0.0)
  {
    return std::nullopt;
  }
  const auto type = computedSolutionType(solution);
  if (!type.has_value()) {
    return std::nullopt;
  }
  double floor_deg = 0.0;
  if (std::find(policy.fixed_solutions.begin(), policy.fixed_solutions.end(), *type) !=
    policy.fixed_solutions.end())
  {
    floor_deg = policy.fixed_std_floor_deg;
  } else if (
    std::find(policy.float_solutions.begin(), policy.float_solutions.end(), *type) !=
    policy.float_solutions.end())
  {
    floor_deg = policy.float_std_floor_deg;
  } else {
    return std::nullopt;
  }
  constexpr double pi = 3.14159265358979323846;
  const double floor_rad = floor_deg * pi / 180.0;
  return std::max(reported_variance, floor_rad * floor_rad);
}

}  // namespace agribot_offline_mapping
