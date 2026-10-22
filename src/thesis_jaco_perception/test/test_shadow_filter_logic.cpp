#include "thesis_jaco_perception/shadow_filter_logic.hpp"

#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace geometry = thesis_jaco_perception;

namespace
{
geometry::ShadowFilterConfiguration valid_configuration()
{
  geometry::ShadowFilterConfiguration configuration;
  configuration.enabled = true;
  configuration.camera_optical_frame = "camera_optical_frame";
  configuration.capsule_start_frames = {"capsule_start"};
  configuration.capsule_end_frames = {"capsule_end"};
  configuration.capsule_inner_radii_m = {0.5};
  configuration.epsilon_total_m = 0.01;
  return configuration;
}

TEST(ShadowFilterLogic, PartitionsSyntheticCloudAgainstSyntheticTfModel)
{
  const geometry::Capsule3 capsule{{2.0, 0.0, -1.0}, {2.0, 0.0, 1.0}, 0.5};
  const auto result = geometry::partition_observed_points(
    {0.0, 0.0, 0.0}, {capsule},
    {{1.0, 0.0, 0.0}, {1.5, 0.0, 0.0}, {3.0, 0.0, 0.0}}, 0.01);
  ASSERT_EQ(result.external_confirmed_indices.size(), 1U);
  EXPECT_EQ(result.external_confirmed_indices[0], 0U);
  ASSERT_EQ(result.self_confirmed_indices.size(), 1U);
  EXPECT_EQ(result.self_confirmed_indices[0], 1U);
  ASSERT_EQ(result.occluded_unknown_indices.size(), 1U);
  EXPECT_EQ(result.occluded_unknown_indices[0], 2U);
}

TEST(ShadowFilterLogic, RejectsCloudOutsideWorldAndInvalidStamps)
{
  constexpr std::int64_t kNow = 10000000000LL;
  EXPECT_FALSE(geometry::cloud_header_error({"world", 9400000000LL}, kNow, 0.5).empty());
  EXPECT_FALSE(geometry::cloud_header_error({"world", 0LL}, kNow, 0.5).empty());
  EXPECT_FALSE(geometry::cloud_header_error({"world", 10000000001LL}, kNow, 0.5).empty());
  EXPECT_FALSE(geometry::cloud_header_error({"camera", kNow}, kNow, 0.5).empty());
  EXPECT_TRUE(geometry::cloud_header_error({"world", kNow}, kNow, 0.5).empty());
}

TEST(ShadowFilterLogic, RejectsMissingOrInconsistentPhysicalConfiguration)
{
  auto configuration = valid_configuration();
  configuration.capsule_end_frames.clear();
  EXPECT_FALSE(geometry::shadow_configuration_error(configuration).empty());

  configuration = valid_configuration();
  configuration.capsule_inner_radii_m[0] = 0.0;
  EXPECT_FALSE(geometry::shadow_configuration_error(configuration).empty());

  configuration = valid_configuration();
  configuration.epsilon_total_m = -1.0;
  EXPECT_FALSE(geometry::shadow_configuration_error(configuration).empty());

  configuration = valid_configuration();
  configuration.epsilon_total_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(geometry::shadow_configuration_error(configuration).empty());
}

TEST(ShadowFilterLogic, DisabledConfigurationCannotPromoteExternalPoints)
{
  auto configuration = valid_configuration();
  configuration.enabled = false;
  EXPECT_EQ(geometry::shadow_configuration_error(configuration), "DISABLED");
}

TEST(ShadowFilterLogic, RejectsTfTimeoutOutsideBriefBound)
{
  auto configuration = valid_configuration();
  configuration.tf_timeout_sec = 0.11;
  EXPECT_FALSE(geometry::shadow_configuration_error(configuration).empty());
}
}  // namespace
