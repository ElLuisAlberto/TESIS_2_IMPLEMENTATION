#ifndef THESIS_JACO_PERCEPTION__CALIBRATION_CANDIDATE_LOGIC_HPP_
#define THESIS_JACO_PERCEPTION__CALIBRATION_CANDIDATE_LOGIC_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace thesis_jaco_perception
{
struct CalibrationSample
{
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  double reprojection_error_px{0.0};
};

struct CalibrationPolicy
{
  std::size_t min_samples{20U};
  double max_reprojection_error_px{1.0};
  double max_translation_dispersion_m{0.003};
  double max_rotation_dispersion_deg{0.5};
};

struct CalibrationCandidate
{
  bool accepted{false};
  bool session_only{true};
  std::string reason;
  std::size_t sample_count{0U};
  CalibrationSample pose;
  double max_reprojection_error_px{0.0};
  double max_translation_dispersion_m{0.0};
  double max_rotation_dispersion_deg{0.0};
};

CalibrationCandidate evaluate_calibration_samples(
  const std::vector<CalibrationSample> & samples,
  const CalibrationPolicy & policy);

bool persistence_is_authorized(
  const CalibrationCandidate & candidate,
  bool explicit_operator_acceptance,
  std::string & reason);
}  // namespace thesis_jaco_perception

#endif  // THESIS_JACO_PERCEPTION__CALIBRATION_CANDIDATE_LOGIC_HPP_
