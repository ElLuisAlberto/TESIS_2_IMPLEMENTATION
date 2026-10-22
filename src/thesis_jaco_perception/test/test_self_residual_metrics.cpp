#include "thesis_jaco_perception/self_filter_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace geometry = thesis_jaco_perception;

namespace
{

constexpr std::size_t kCapsuleCount = 6U;

enum class CoverageLabel {kNoResidual, kLocalCoverageCandidate, kExcessiveOrAmbiguousExpansion};

struct CoverageStatistics
{
  std::size_t count{};
  double p50{std::numeric_limits<double>::quiet_NaN()};
  double p90{std::numeric_limits<double>::quiet_NaN()};
  double p95{std::numeric_limits<double>::quiet_NaN()};
  double p99{std::numeric_limits<double>::quiet_NaN()};
  double maximum{std::numeric_limits<double>::quiet_NaN()};
};

double percentile(const std::vector<double> & sorted, const double fraction)
{
  const double position = fraction * static_cast<double>(sorted.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  return sorted[lower] + (position - static_cast<double>(lower)) *
    (sorted[upper] - sorted[lower]);
}

CoverageStatistics coverage_statistics(std::vector<double> values)
{
  if (values.empty()) {
    return {};
  }
  std::sort(values.begin(), values.end());
  return {values.size(), percentile(values, 0.5), percentile(values, 0.9),
    percentile(values, 0.95), percentile(values, 0.99), values.back()};
}

CoverageLabel coverage_label(const CoverageStatistics & statistics, const double radius)
{
  if (statistics.count == 0U) {
    return CoverageLabel::kNoResidual;
  }
  const double increment_p95 = statistics.p95 - radius;
  const double p99_spread = statistics.p99 - statistics.p95;
  if (statistics.count >= 20U && increment_p95 > 0.0 && increment_p95 <= 0.025 &&
    p99_spread <= 0.020)
  {
    return CoverageLabel::kLocalCoverageCandidate;
  }
  return CoverageLabel::kExcessiveOrAmbiguousExpansion;
}

bool parse_capsule_radii_csv(
  const std::string & input, std::array<double, kCapsuleCount> & radii)
{
  std::array<double, kCapsuleCount> parsed{};
  std::size_t index = 0U;
  std::size_t begin = 0U;
  while (begin <= input.size()) {
    const auto end = input.find(',', begin);
    const std::string token = input.substr(begin, end == std::string::npos ?
      std::string::npos : end - begin);
    if (token.empty() || index >= kCapsuleCount) {
      return false;
    }
    char * parsed_end = nullptr;
    const double value = std::strtod(token.c_str(), &parsed_end);
    if (parsed_end == token.c_str() || *parsed_end != '\0' || !std::isfinite(value) || value <= 0.0) {
      return false;
    }
    parsed[index++] = value;
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  if (index != kCapsuleCount) {
    return false;
  }
  radii = parsed;
  return true;
}

const geometry::Point3 kCamera{-1.0, 0.0, 0.0};
const std::vector<geometry::Capsule3> kCapsules = {
  {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.10},
};

geometry::Ray3 ray_to(const geometry::Point3 & point)
{
  return {kCamera, {point.x - kCamera.x, point.y - kCamera.y, point.z - kCamera.z}};
}

TEST(SelfResidualMetrics, FindsKnownFirstRayIntersection)
{
  const geometry::Point3 point{0.20, 0.0, 0.0};
  const auto intersection = geometry::first_ray_capsules_intersection(ray_to(point), kCapsules);
  ASSERT_TRUE(intersection.has_value());
  EXPECT_EQ(intersection->capsule_index, 0U);
  EXPECT_NEAR(intersection->distance, 0.90, 1e-12);
  EXPECT_NEAR(std::sqrt(1.20 * 1.20) - intersection->distance, 0.30, 1e-12);
}

TEST(SelfResidualMetrics, MatchesAllThreeClassificationCases)
{
  constexpr double epsilon = 0.07;
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {-0.20, 0.0, 0.0}, kCapsules, epsilon),
    geometry::PointClassification::EXTERNAL_CONFIRMED);
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {-0.08, 0.0, 0.0}, kCapsules, epsilon),
    geometry::PointClassification::SELF_CONFIRMED);
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, {0.20, 0.0, 0.0}, kCapsules, epsilon),
    geometry::PointClassification::OCCLUDED_UNKNOWN);
}

TEST(SelfResidualMetrics, ReportsRayWithoutIntersectionAsExternal)
{
  const geometry::Point3 point{-1.0, 1.0, 0.0};
  EXPECT_FALSE(geometry::first_ray_capsules_intersection(ray_to(point), kCapsules).has_value());
  EXPECT_EQ(
    geometry::classify_observed_point(kCamera, point, kCapsules, 0.07),
    geometry::PointClassification::EXTERNAL_CONFIRMED);
}

TEST(SelfResidualMetrics, ParsesSixValidCapsuleRadii)
{
  std::array<double, kCapsuleCount> radii{};
  ASSERT_TRUE(parse_capsule_radii_csv("0.105,0.105,0.100,0.090,0.100,0.105", radii));
  EXPECT_DOUBLE_EQ(radii[0], 0.105);
  EXPECT_DOUBLE_EQ(radii[3], 0.090);
  EXPECT_DOUBLE_EQ(radii[5], 0.105);
}

TEST(SelfResidualMetrics, RejectsInvalidCapsuleRadiiCsv)
{
  std::array<double, kCapsuleCount> radii{};
  EXPECT_FALSE(parse_capsule_radii_csv("0.1,0.1,0.1", radii));
  EXPECT_FALSE(parse_capsule_radii_csv("0.1,0.1,0.1,0.1,0.1,0", radii));
  EXPECT_FALSE(parse_capsule_radii_csv("0.1,0.1,0.1,0.1,0.1,nan", radii));
}

TEST(SelfResidualMetrics, CalculatesKnownRequiredRadiusFromFiniteSegment)
{
  const geometry::Point3 point{0.5, 0.2, 0.0};
  const double axis_distance = geometry::point_segment_distance(
    point, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(axis_distance, 0.2, 1e-12);
  EXPECT_NEAR(axis_distance - 0.1, 0.1, 1e-12);
  EXPECT_NEAR(axis_distance + 0.003, 0.203, 1e-12);
}

TEST(SelfResidualMetrics, CalculatesRequiredRadiusPercentiles)
{
  const auto statistics = coverage_statistics({0.10, 0.11, 0.12, 0.13, 0.14});
  EXPECT_EQ(statistics.count, 5U);
  EXPECT_NEAR(statistics.p50, 0.12, 1e-12);
  EXPECT_NEAR(statistics.p90, 0.136, 1e-12);
  EXPECT_NEAR(statistics.p95, 0.138, 1e-12);
  EXPECT_NEAR(statistics.p99, 0.1396, 1e-12);
  EXPECT_NEAR(statistics.maximum, 0.14, 1e-12);
}

TEST(SelfResidualMetrics, LabelsLocalCoverageCandidateOnlyWithinStrictLimit)
{
  std::vector<double> required(20U, 0.120);
  const auto statistics = coverage_statistics(required);
  EXPECT_EQ(coverage_label(statistics, 0.100), CoverageLabel::kLocalCoverageCandidate);
}

TEST(SelfResidualMetrics, LabelsExcessiveOrAmbiguousExpansion)
{
  EXPECT_EQ(
    coverage_label(coverage_statistics(std::vector<double>(20U, 0.126)), 0.100),
    CoverageLabel::kExcessiveOrAmbiguousExpansion);
  EXPECT_EQ(
    coverage_label(coverage_statistics(std::vector<double>(19U, 0.120)), 0.100),
    CoverageLabel::kExcessiveOrAmbiguousExpansion);
  std::vector<double> broad_required(19U, 0.120);
  broad_required.push_back(0.150);
  EXPECT_EQ(
    coverage_label(coverage_statistics(broad_required), 0.100),
    CoverageLabel::kExcessiveOrAmbiguousExpansion);
}

}  // namespace
