#include "thesis_jaco_perception/self_filter_geometry.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace geometry = thesis_jaco_perception;

namespace
{
const geometry::Point3 kCamera{0.0, 0.0, 0.0};
const geometry::Capsule3 kVisibleCapsule{{2.0, 0.0, -1.0}, {2.0, 0.0, 1.0}, 0.5};

TEST(SelfFilterGeometry, ClassifiesPointInFrontOfVisibleCapsuleAsExternal)
{
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {1.0, 0.0, 0.0}, {kVisibleCapsule}, 0.01),
    geometry::PointClassification::EXTERNAL_CONFIRMED);
}

TEST(SelfFilterGeometry, ClassifiesExpectedCapsuleSurfaceAsSelf)
{
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {1.5, 0.0, 0.0}, {kVisibleCapsule}, 0.01),
    geometry::PointClassification::SELF_CONFIRMED);
}

TEST(SelfFilterGeometry, ClassifiesPointBehindVisibleCapsuleAsOccludedUnknown)
{
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {3.0, 0.0, 0.0}, {kVisibleCapsule}, 0.01),
    geometry::PointClassification::OCCLUDED_UNKNOWN);
}

TEST(SelfFilterGeometry, RayMissingAllCapsulesClassifiesPointAsExternal)
{
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {0.0, 2.0, 0.0}, {kVisibleCapsule}, 0.0),
    geometry::PointClassification::EXTERNAL_CONFIRMED);
}

TEST(SelfFilterGeometry, RejectsInvalidGeometryAndTolerance)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(geometry::validate_capsule({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(geometry::validate_capsule({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0}),
    std::invalid_argument);
  EXPECT_THROW(geometry::validate_ray({kCamera, {0.0, 0.0, 0.0}}), std::invalid_argument);
  EXPECT_THROW(geometry::validate_epsilon_total(-0.01), std::invalid_argument);
  EXPECT_THROW(geometry::validate_epsilon_total(nan), std::invalid_argument);
  EXPECT_THROW(geometry::validate_point({nan, 0.0, 0.0}, "point"), std::invalid_argument);
}

TEST(SelfFilterGeometry, SelectsTheFirstIntersectionAcrossCapsules)
{
  const geometry::Capsule3 farther{{4.0, 0.0, -1.0}, {4.0, 0.0, 1.0}, 0.5};
  const auto result = geometry::first_ray_capsules_intersection(
    {kCamera, {1.0, 0.0, 0.0}}, {farther, kVisibleCapsule});
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->capsule_index, 1U);
  EXPECT_NEAR(result->distance, 1.5, 1.0e-12);
}

TEST(SelfFilterGeometry, HandlesTangencyAndEndCapIntersections)
{
  const auto tangent = geometry::first_ray_capsule_intersection(
    {{0.0, 0.5, 0.0}, {1.0, 0.0, 0.0}}, kVisibleCapsule);
  ASSERT_TRUE(tangent.has_value());
  EXPECT_NEAR(tangent->distance, 2.0, 1.0e-12);

  const geometry::Capsule3 end_cap{{2.0, 0.0, 0.0}, {2.0, 0.0, 1.0}, 0.5};
  const auto result = geometry::first_ray_capsule_intersection(
    {kCamera, {1.0, 0.0, 0.0}}, end_cap);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->distance, 1.5, 1.0e-12);
}

TEST(SelfFilterGeometry, CalculatesPointSegmentDistance)
{
  EXPECT_NEAR(
    geometry::point_segment_distance({1.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}),
    1.0, 1.0e-12);
}
}  // namespace
