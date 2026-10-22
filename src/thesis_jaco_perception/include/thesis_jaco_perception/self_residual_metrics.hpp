#ifndef THESIS_JACO_PERCEPTION__SELF_RESIDUAL_METRICS_HPP_
#define THESIS_JACO_PERCEPTION__SELF_RESIDUAL_METRICS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "thesis_jaco_perception/self_filter_geometry.hpp"

namespace thesis_jaco_perception
{

enum class ClearanceBand
{
  kNonPositive,
  kZeroToOneCm,
  kOneToTwoCm,
  kTwoToFourCm,
  kFourToEightCm,
  kOverEightCm,
};

struct ResidualAssociation
{
  std::size_t capsule_index{};
  double clearance{};
};

struct ClearanceStatistics
{
  std::size_t count{};
  double minimum{std::numeric_limits<double>::quiet_NaN()};
  double median{std::numeric_limits<double>::quiet_NaN()};
  double p90{std::numeric_limits<double>::quiet_NaN()};
  double maximum{std::numeric_limits<double>::quiet_NaN()};
};

inline ClearanceBand clearance_band(const double clearance)
{
  if (clearance <= 0.0) {
    return ClearanceBand::kNonPositive;
  }
  if (clearance <= 0.01) {
    return ClearanceBand::kZeroToOneCm;
  }
  if (clearance <= 0.02) {
    return ClearanceBand::kOneToTwoCm;
  }
  if (clearance <= 0.04) {
    return ClearanceBand::kTwoToFourCm;
  }
  if (clearance <= 0.08) {
    return ClearanceBand::kFourToEightCm;
  }
  return ClearanceBand::kOverEightCm;
}

inline ResidualAssociation nearest_capsule_residual(
  const Point3 & point, const std::vector<Capsule3> & capsules)
{
  validate_point(point, "external point");
  if (capsules.empty()) {
    throw std::invalid_argument("capsules must not be empty");
  }
  ResidualAssociation result{};
  result.clearance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < capsules.size(); ++index) {
    validate_capsule(capsules[index]);
    const double clearance = point_segment_distance(
      point, capsules[index].start, capsules[index].end) - capsules[index].radius;
    if (clearance < result.clearance) {
      result = {index, clearance};
    }
  }
  return result;
}

inline double percentile(const std::vector<double> & sorted_values, const double fraction)
{
  if (sorted_values.empty() || fraction < 0.0 || fraction > 1.0) {
    throw std::invalid_argument("invalid percentile input");
  }
  const double position = fraction * static_cast<double>(sorted_values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double interpolation = position - static_cast<double>(lower);
  return sorted_values[lower] + interpolation * (sorted_values[upper] - sorted_values[lower]);
}

inline ClearanceStatistics clearance_statistics(std::vector<double> values)
{
  if (values.empty()) {
    return {};
  }
  std::sort(values.begin(), values.end());
  return {
    values.size(), values.front(), percentile(values, 0.5), percentile(values, 0.9), values.back()};
}

}  // namespace thesis_jaco_perception

#endif  // THESIS_JACO_PERCEPTION__SELF_RESIDUAL_METRICS_HPP_
