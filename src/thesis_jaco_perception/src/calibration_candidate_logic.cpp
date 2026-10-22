#include "thesis_jaco_perception/calibration_candidate_logic.hpp"

#include <algorithm>
#include <cmath>

namespace thesis_jaco_perception
{
namespace
{
double norm(const CalibrationSample & sample)
{
  return std::sqrt(sample.qx * sample.qx + sample.qy * sample.qy +
    sample.qz * sample.qz + sample.qw * sample.qw);
}

double rotation_distance_deg(const CalibrationSample & a, const CalibrationSample & b)
{
  const double dot = std::clamp(
    std::abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw), 0.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  return 2.0 * std::acos(dot) * 180.0 / kPi;
}
}  // namespace

CalibrationCandidate evaluate_calibration_samples(
  const std::vector<CalibrationSample> & samples,
  const CalibrationPolicy & policy)
{
  CalibrationCandidate result;
  result.sample_count = samples.size();
  if (samples.size() < policy.min_samples) {
    result.reason = "INSUFFICIENT_SAMPLES";
    return result;
  }

  CalibrationSample mean;
  mean.qw = 0.0;
  for (const auto & sample : samples) {
    if (!std::isfinite(sample.reprojection_error_px) || norm(sample) < 1e-9) {
      result.reason = "INVALID_POSE";
      return result;
    }
    if (sample.reprojection_error_px > policy.max_reprojection_error_px) {
      result.reason = "REPROJECTION_ERROR_HIGH";
      return result;
    }
    mean.tx += sample.tx;
    mean.ty += sample.ty;
    mean.tz += sample.tz;
    mean.qx += sample.qx;
    mean.qy += sample.qy;
    mean.qz += sample.qz;
    mean.qw += sample.qw;
    result.max_reprojection_error_px = std::max(
      result.max_reprojection_error_px, sample.reprojection_error_px);
  }
  const double count = static_cast<double>(samples.size());
  mean.tx /= count;
  mean.ty /= count;
  mean.tz /= count;
  mean.qx /= count;
  mean.qy /= count;
  mean.qz /= count;
  mean.qw /= count;
  const double mean_norm = norm(mean);
  mean.qx /= mean_norm;
  mean.qy /= mean_norm;
  mean.qz /= mean_norm;
  mean.qw /= mean_norm;

  for (const auto & sample : samples) {
    const double translation = std::sqrt(
      std::pow(sample.tx - mean.tx, 2.0) + std::pow(sample.ty - mean.ty, 2.0) +
      std::pow(sample.tz - mean.tz, 2.0));
    result.max_translation_dispersion_m = std::max(result.max_translation_dispersion_m, translation);
    result.max_rotation_dispersion_deg = std::max(
      result.max_rotation_dispersion_deg, rotation_distance_deg(sample, mean));
  }
  result.pose = mean;
  if (result.max_translation_dispersion_m > policy.max_translation_dispersion_m) {
    result.reason = "TRANSLATION_DISPERSION_HIGH";
    return result;
  }
  if (result.max_rotation_dispersion_deg > policy.max_rotation_dispersion_deg) {
    result.reason = "ROTATION_DISPERSION_HIGH";
    return result;
  }
  result.accepted = true;
  result.reason = "CANDIDATE_SESSION_ONLY";
  return result;
}

bool persistence_is_authorized(
  const CalibrationCandidate & candidate,
  bool explicit_operator_acceptance,
  std::string & reason)
{
  if (!candidate.accepted) {
    reason = "CANDIDATE_NOT_ACCEPTED";
    return false;
  }
  if (!explicit_operator_acceptance) {
    reason = "EXPLICIT_ACCEPTANCE_REQUIRED";
    return false;
  }
  reason = "PERSISTENCE_MAY_BE_REQUESTED_IN_A_FUTURE_SESSION";
  return true;
}
}  // namespace thesis_jaco_perception
