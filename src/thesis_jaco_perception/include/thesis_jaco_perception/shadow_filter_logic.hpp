#ifndef THESIS_JACO_PERCEPTION__SHADOW_FILTER_LOGIC_HPP_
#define THESIS_JACO_PERCEPTION__SHADOW_FILTER_LOGIC_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "thesis_jaco_perception/self_filter_geometry.hpp"

namespace thesis_jaco_perception
{

struct ShadowFilterConfiguration
{
  bool enabled{false};
  std::string camera_optical_frame;
  std::string expected_frame{"world"};
  std::vector<std::string> capsule_start_frames;
  std::vector<std::string> capsule_end_frames;
  std::vector<double> capsule_inner_radii_m;
  double epsilon_total_m{-1.0};
  double max_cloud_age_sec{0.5};
  double tf_timeout_sec{0.05};
};

struct CloudHeaderInfo
{
  std::string frame_id;
  std::int64_t stamp_nanoseconds{0};
};

struct PointPartitions
{
  std::vector<std::size_t> self_confirmed_indices;
  std::vector<std::size_t> external_confirmed_indices;
  std::vector<std::size_t> occluded_unknown_indices;
};

// Return a readable reason instead of accepting physical defaults implicitly.
std::string shadow_configuration_error(const ShadowFilterConfiguration & configuration);
std::string cloud_header_error(
  const CloudHeaderInfo & header, std::int64_t now_nanoseconds,
  double max_cloud_age_sec);
PointPartitions partition_observed_points(
  const Point3 & camera_origin, const std::vector<Capsule3> & capsules,
  const std::vector<Point3> & points, double epsilon_total_m);

}  // namespace thesis_jaco_perception

#endif  // THESIS_JACO_PERCEPTION__SHADOW_FILTER_LOGIC_HPP_
