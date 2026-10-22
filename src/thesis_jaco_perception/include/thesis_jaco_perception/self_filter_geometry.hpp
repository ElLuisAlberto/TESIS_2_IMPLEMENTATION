#ifndef THESIS_JACO_PERCEPTION__SELF_FILTER_GEOMETRY_HPP_
#define THESIS_JACO_PERCEPTION__SELF_FILTER_GEOMETRY_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace thesis_jaco_perception
{

struct Point3
{
  double x;
  double y;
  double z;
};

struct Ray3
{
  Point3 origin;
  Point3 direction;
};

struct Capsule3
{
  Point3 start;
  Point3 end;
  double radius;
};

struct RayCapsuleIntersection
{
  double distance;
  Point3 point;
  std::size_t capsule_index;
};

enum class PointClassification
{
  SELF_CONFIRMED,
  EXTERNAL_CONFIRMED,
  OCCLUDED_UNKNOWN,
};

// All validation and geometric functions throw std::invalid_argument for
// non-finite inputs, invalid radii/tolerances, null rays, or degenerate axes.
bool is_valid_point(const Point3 & point);
void validate_point(const Point3 & point, const char * name);
void validate_ray(const Ray3 & ray);
void validate_capsule(const Capsule3 & capsule);
void validate_epsilon_total(double epsilon_total);

double point_segment_distance(
  const Point3 & point, const Point3 & start, const Point3 & end);

std::optional<RayCapsuleIntersection> first_ray_capsule_intersection(
  const Ray3 & ray, const Capsule3 & capsule);

std::optional<RayCapsuleIntersection> first_ray_capsules_intersection(
  const Ray3 & ray, const std::vector<Capsule3> & capsules);

PointClassification classify_observed_point(
  const Point3 & camera_origin,
  const Point3 & observed_point,
  const std::vector<Capsule3> & capsules,
  double epsilon_total);

}  // namespace thesis_jaco_perception

#endif  // THESIS_JACO_PERCEPTION__SELF_FILTER_GEOMETRY_HPP_
