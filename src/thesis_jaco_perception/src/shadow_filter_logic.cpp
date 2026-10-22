#include "thesis_jaco_perception/shadow_filter_logic.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace thesis_jaco_perception
{
std::string shadow_configuration_error(const ShadowFilterConfiguration & configuration)
{
  if (!configuration.enabled) {
    return "DISABLED";
  }
  if (configuration.expected_frame != "world") {
    return "expected_frame must be world";
  }
  if (configuration.camera_optical_frame.empty()) {
    return "camera_optical_frame is required";
  }
  const std::size_t count = configuration.capsule_start_frames.size();
  if (count == 0U || configuration.capsule_end_frames.size() != count ||
    configuration.capsule_inner_radii_m.size() != count)
  {
    return "capsule frame and radius lists must be non-empty and equal-sized";
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (configuration.capsule_start_frames[index].empty() ||
      configuration.capsule_end_frames[index].empty() ||
      !std::isfinite(configuration.capsule_inner_radii_m[index]) ||
      configuration.capsule_inner_radii_m[index] <= 0.0)
    {
      return "capsule frames must be non-empty and inner radii positive";
    }
  }
  if (!std::isfinite(configuration.epsilon_total_m) ||
    configuration.epsilon_total_m < 0.0)
  {
    return "epsilon_total_m must be finite and non-negative";
  }
  if (!std::isfinite(configuration.max_cloud_age_sec) ||
    configuration.max_cloud_age_sec <= 0.0)
  {
    return "max_cloud_age_sec must be finite and positive";
  }
  if (!std::isfinite(configuration.tf_timeout_sec) ||
    configuration.tf_timeout_sec <= 0.0 || configuration.tf_timeout_sec > 0.1)
  {
    return "tf_timeout_sec must be finite, positive and no greater than 0.1";
  }
  return "";
}

std::string cloud_header_error(
  const CloudHeaderInfo & header, std::int64_t now_nanoseconds,
  double max_cloud_age_sec)
{
  if (header.frame_id != "world") {
    return "cloud frame_id must be world";
  }
  if (header.stamp_nanoseconds <= 0) {
    return "cloud stamp must be non-zero";
  }
  if (now_nanoseconds <= 0 || !std::isfinite(max_cloud_age_sec) ||
    max_cloud_age_sec <= 0.0)
  {
    return "clock or max_cloud_age_sec is invalid";
  }
  const long double age = static_cast<long double>(now_nanoseconds - header.stamp_nanoseconds) /
    1.0e9L;
  if (age < 0.0L) {
    return "cloud stamp is in the future";
  }
  if (age > static_cast<long double>(max_cloud_age_sec)) {
    return "cloud stamp is stale";
  }
  return "";
}

PointPartitions partition_observed_points(
  const Point3 & camera_origin, const std::vector<Capsule3> & capsules,
  const std::vector<Point3> & points, double epsilon_total_m)
{
  validate_point(camera_origin, "camera origin");
  validate_epsilon_total(epsilon_total_m);
  for (const auto & capsule : capsules) {
    validate_capsule(capsule);
  }
  PointPartitions partitions;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!is_valid_point(points[index])) {
      continue;
    }
    switch (classify_observed_point(
        camera_origin, points[index], capsules, epsilon_total_m))
    {
      case PointClassification::SELF_CONFIRMED:
        partitions.self_confirmed_indices.push_back(index);
        break;
      case PointClassification::EXTERNAL_CONFIRMED:
        partitions.external_confirmed_indices.push_back(index);
        break;
      case PointClassification::OCCLUDED_UNKNOWN:
        partitions.occluded_unknown_indices.push_back(index);
        break;
    }
  }
  return partitions;
}

}  // namespace thesis_jaco_perception
