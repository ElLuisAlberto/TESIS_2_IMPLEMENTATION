#include "thesis_jaco_perception/self_filter_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace thesis_jaco_perception
{
namespace
{
constexpr double kAxisEpsilon = 1.0e-12;
constexpr double kDiscriminantEpsilon = 1.0e-12;

Point3 subtract(const Point3 & first, const Point3 & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Point3 add_scaled(const Point3 & point, const Point3 & direction, double scale)
{
  return {
    point.x + direction.x * scale,
    point.y + direction.y * scale,
    point.z + direction.z * scale,
  };
}

double dot(const Point3 & first, const Point3 & second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

double squared_norm(const Point3 & vector)
{
  return dot(vector, vector);
}

Point3 normalized(const Point3 & vector)
{
  const double length = std::sqrt(squared_norm(vector));
  return {vector.x / length, vector.y / length, vector.z / length};
}

void consider_sphere_intersections(
  const Ray3 & ray,
  const Point3 & center,
  double radius,
  double & best_distance)
{
  const Point3 offset = subtract(ray.origin, center);
  const double half_b = dot(ray.direction, offset);
  double discriminant = half_b * half_b - (squared_norm(offset) - radius * radius);
  if (discriminant < -kDiscriminantEpsilon) {
    return;
  }
  discriminant = std::max(0.0, discriminant);
  const double root = std::sqrt(discriminant);
  const double first = -half_b - root;
  const double second = -half_b + root;
  if (first >= 0.0) {
    best_distance = std::min(best_distance, first);
  }
  if (second >= 0.0) {
    best_distance = std::min(best_distance, second);
  }
}
}  // namespace

bool is_valid_point(const Point3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void validate_point(const Point3 & point, const char * name)
{
  if (!is_valid_point(point)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validate_ray(const Ray3 & ray)
{
  validate_point(ray.origin, "ray origin");
  validate_point(ray.direction, "ray direction");
  if (squared_norm(ray.direction) <= kAxisEpsilon * kAxisEpsilon) {
    throw std::invalid_argument("ray direction must be non-zero");
  }
}

void validate_capsule(const Capsule3 & capsule)
{
  validate_point(capsule.start, "capsule start");
  validate_point(capsule.end, "capsule end");
  if (!std::isfinite(capsule.radius) || capsule.radius <= 0.0) {
    throw std::invalid_argument("capsule radius must be finite and positive");
  }
  if (squared_norm(subtract(capsule.end, capsule.start)) <=
    kAxisEpsilon * kAxisEpsilon)
  {
    throw std::invalid_argument("capsule axis must be non-degenerate");
  }
}

void validate_epsilon_total(double epsilon_total)
{
  if (!std::isfinite(epsilon_total) || epsilon_total < 0.0) {
    throw std::invalid_argument("epsilon_total must be finite and non-negative");
  }
}

double point_segment_distance(
  const Point3 & point, const Point3 & start, const Point3 & end)
{
  validate_point(point, "point");
  validate_point(start, "segment start");
  validate_point(end, "segment end");
  const Point3 axis = subtract(end, start);
  const double axis_squared = squared_norm(axis);
  if (axis_squared <= kAxisEpsilon * kAxisEpsilon) {
    throw std::invalid_argument("segment must be non-degenerate");
  }
  const double projection = std::clamp(
    dot(subtract(point, start), axis) / axis_squared, 0.0, 1.0);
  return std::sqrt(squared_norm(subtract(point, add_scaled(start, axis, projection))));
}

std::optional<RayCapsuleIntersection> first_ray_capsule_intersection(
  const Ray3 & input_ray, const Capsule3 & capsule)
{
  validate_ray(input_ray);
  validate_capsule(capsule);
  const Ray3 ray{input_ray.origin, normalized(input_ray.direction)};
  const Point3 axis = subtract(capsule.end, capsule.start);
  const Point3 offset = subtract(ray.origin, capsule.start);
  const double axis_squared = squared_norm(axis);
  const double axis_ray = dot(axis, ray.direction);
  const double axis_offset = dot(axis, offset);
  const double ray_offset = dot(ray.direction, offset);
  const double offset_squared = squared_norm(offset);
  const double quadratic_a = axis_squared - axis_ray * axis_ray;
  const double quadratic_b = axis_squared * ray_offset - axis_offset * axis_ray;
  const double quadratic_c = axis_squared * offset_squared - axis_offset * axis_offset -
    capsule.radius * capsule.radius * axis_squared;
  double best_distance = std::numeric_limits<double>::infinity();

  if (quadratic_a > kAxisEpsilon) {
    double discriminant = quadratic_b * quadratic_b - quadratic_a * quadratic_c;
    if (discriminant >= -kDiscriminantEpsilon) {
      discriminant = std::max(0.0, discriminant);
      const double root = std::sqrt(discriminant);
      for (const double distance : {
          (-quadratic_b - root) / quadratic_a,
          (-quadratic_b + root) / quadratic_a})
      {
        const double axial_coordinate = axis_offset + distance * axis_ray;
        if (distance >= 0.0 && axial_coordinate >= 0.0 &&
          axial_coordinate <= axis_squared)
        {
          best_distance = std::min(best_distance, distance);
        }
      }
    }
  }

  consider_sphere_intersections(ray, capsule.start, capsule.radius, best_distance);
  consider_sphere_intersections(ray, capsule.end, capsule.radius, best_distance);
  if (!std::isfinite(best_distance)) {
    return std::nullopt;
  }
  return RayCapsuleIntersection{
    best_distance, add_scaled(ray.origin, ray.direction, best_distance), 0U};
}

std::optional<RayCapsuleIntersection> first_ray_capsules_intersection(
  const Ray3 & ray, const std::vector<Capsule3> & capsules)
{
  validate_ray(ray);
  std::optional<RayCapsuleIntersection> closest;
  for (std::size_t index = 0; index < capsules.size(); ++index) {
    const auto intersection = first_ray_capsule_intersection(ray, capsules[index]);
    if (!intersection.has_value()) {
      continue;
    }
    if (!closest.has_value() || intersection->distance < closest->distance) {
      closest = intersection;
      closest->capsule_index = index;
    }
  }
  return closest;
}

PointClassification classify_observed_point(
  const Point3 & camera_origin,
  const Point3 & observed_point,
  const std::vector<Capsule3> & capsules,
  double epsilon_total)
{
  validate_point(camera_origin, "camera origin");
  validate_point(observed_point, "observed point");
  validate_epsilon_total(epsilon_total);
  const Point3 observed_direction = subtract(observed_point, camera_origin);
  const double observed_distance = std::sqrt(squared_norm(observed_direction));
  if (observed_distance <= kAxisEpsilon) {
    throw std::invalid_argument("observed point must differ from camera origin");
  }
  const auto model_intersection = first_ray_capsules_intersection(
    {camera_origin, observed_direction}, capsules);
  if (!model_intersection.has_value() ||
    observed_distance < model_intersection->distance - epsilon_total)
  {
    return PointClassification::EXTERNAL_CONFIRMED;
  }
  if (std::abs(observed_distance - model_intersection->distance) <= epsilon_total) {
    return PointClassification::SELF_CONFIRMED;
  }
  return PointClassification::OCCLUDED_UNKNOWN;
}

}  // namespace thesis_jaco_perception
