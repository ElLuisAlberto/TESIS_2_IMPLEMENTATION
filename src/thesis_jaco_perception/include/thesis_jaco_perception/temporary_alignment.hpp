#ifndef THESIS_JACO_PERCEPTION__TEMPORARY_ALIGNMENT_HPP_
#define THESIS_JACO_PERCEPTION__TEMPORARY_ALIGNMENT_HPP_

#include <array>
#include <string>
#include <vector>
#include "thesis_jaco_perception/self_filter_geometry.hpp"

namespace thesis_jaco_perception
{
struct TemporaryAlignmentResult {
  bool accepted{false}; bool session_only{true}; bool not_for_safety{true};
  std::string reason; std::array<double, 6> pose{};
  double initial_cost{0.0}; double final_cost{0.0}; std::size_t points_used{0};
};
TemporaryAlignmentResult align_temporary_cloud(
  const std::vector<Point3> & camera_points, const std::vector<Capsule3> & world_capsules,
  const std::array<double, 6> & initial_pose);
}  // namespace thesis_jaco_perception
#endif
