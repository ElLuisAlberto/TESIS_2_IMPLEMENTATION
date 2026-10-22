#include <gtest/gtest.h>

#include "thesis_jaco_perception/calibration_candidate_logic.hpp"

namespace thesis_jaco_perception
{
namespace
{
CalibrationSample sample()
{
  CalibrationSample value;
  value.qw = 1.0;
  value.reprojection_error_px = 0.2;
  return value;
}

std::vector<CalibrationSample> valid_samples()
{
  return std::vector<CalibrationSample>(20U, sample());
}

TEST(CalibrationCandidateLogic, RejectsInsufficientSamples)
{
  CalibrationPolicy policy;
  const auto candidate = evaluate_calibration_samples({sample()}, policy);
  EXPECT_FALSE(candidate.accepted);
  EXPECT_EQ(candidate.reason, "INSUFFICIENT_SAMPLES");
}

TEST(CalibrationCandidateLogic, RejectsHighReprojectionError)
{
  auto samples = valid_samples();
  samples.front().reprojection_error_px = 1.1;
  const auto candidate = evaluate_calibration_samples(samples, CalibrationPolicy{});
  EXPECT_FALSE(candidate.accepted);
  EXPECT_EQ(candidate.reason, "REPROJECTION_ERROR_HIGH");
}

TEST(CalibrationCandidateLogic, RejectsDispersionAndFinalPersistenceWithoutAcceptance)
{
  auto samples = valid_samples();
  samples.front().tx = 0.01;
  const auto rejected = evaluate_calibration_samples(samples, CalibrationPolicy{});
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "TRANSLATION_DISPERSION_HIGH");
  std::string reason;
  EXPECT_FALSE(persistence_is_authorized(rejected, false, reason));
  EXPECT_EQ(reason, "CANDIDATE_NOT_ACCEPTED");
}

TEST(CalibrationCandidateLogic, AcceptsOnlySessionCandidate)
{
  const auto candidate = evaluate_calibration_samples(valid_samples(), CalibrationPolicy{});
  ASSERT_TRUE(candidate.accepted);
  EXPECT_TRUE(candidate.session_only);
  std::string reason;
  EXPECT_FALSE(persistence_is_authorized(candidate, false, reason));
  EXPECT_EQ(reason, "EXPLICIT_ACCEPTANCE_REQUIRED");
}
}  // namespace
}  // namespace thesis_jaco_perception
