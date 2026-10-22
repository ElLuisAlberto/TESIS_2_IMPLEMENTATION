#include <cmath>
#include <sstream>

#include <gtest/gtest.h>

#include "thesis_jaco_perception/multipose_alignment.hpp"

using namespace thesis_jaco_perception;

namespace
{

std::vector<Capsule3> caps(double s)
{
  return {{{0 + s, 0, 0}, {0 + s, 0, 1}, .07},
    {{.4 + s, 0, 0}, {.7 + s, .2, .6}, .07},
    {{.8 + s, .2, 0}, {1 + s, .5, .7}, .07},
    {{1.2 + s, -.2, 0}, {1.5 + s, .1, .8}, .07},
    {{1.6 + s, .2, 0}, {1.9 + s, .5, .6}, .07},
    {{2 + s, -.2, 0}, {2.2 + s, .2, .7}, .07}};
}

MultiPoseCapture capture(double s, std::array<double, 6> t)
{
  MultiPoseCapture x;
  x.world_capsules = caps(s);
  for (auto & c : x.world_capsules) {
    for (int i = 0; i < 12; ++i) {
      const double f = i / 11.0;
      x.optical_cloud.push_back({
        c.start.x + (c.end.x - c.start.x) * f + c.radius - t[0],
        c.start.y + (c.end.y - c.start.y) * f - t[1],
        c.start.z + (c.end.z - c.start.z) * f - t[2]});
    }
  }
  return x;
}

}  // namespace

TEST(MultiPoseAlignment, RemainsFailClosedWhileRecoveryIsExperimental)
{
  const std::array<double, 6> truth{.04, -.03, .02, 0, 0, 0};
  std::vector<MultiPoseCapture> captures{
    capture(0, truth), capture(.18, truth), capture(-.14, truth)};
  MultiPoseAlignmentEstimator estimator;
  const auto result = estimator.estimate(captures, {}, {});

  std::ostringstream diagnostic;
  diagnostic << "experimental recovery is not validated; truth=("
             << truth[0] << ", " << truth[1] << ", " << truth[2]
             << ") estimated=(" << result.pose[0] << ", " << result.pose[1]
             << ", " << result.pose[2] << ") axis_error=("
             << result.pose[0] - truth[0] << ", "
             << result.pose[1] - truth[1] << ", "
             << result.pose[2] - truth[2] << ")";
  SCOPED_TRACE(diagnostic.str());

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "EXPERIMENTAL_NOT_VALIDATED");
  for (const double component : result.pose) {
    EXPECT_TRUE(std::isfinite(component));
  }
  ASSERT_EQ(result.rounds.size(), 3U);
  for (const auto & associations : result.rounds) {
    EXPECT_FALSE(associations.empty());
  }
}
