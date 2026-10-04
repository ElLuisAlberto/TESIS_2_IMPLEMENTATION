#ifndef THESIS_HARDWARE_BRIDGE__BRIDGE_LOGIC_HPP_
#define THESIS_HARDWARE_BRIDGE__BRIDGE_LOGIC_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace thesis_hardware_bridge
{

constexpr std::size_t kJointCount = 6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

using JointVector = std::array<double, kJointCount>;

template<typename Operation, typename BetweenAttempts>
bool retry_operation(
  int maximum_attempts,
  Operation operation,
  BetweenAttempts between_attempts)
{
  if (maximum_attempts < 1) {
    throw std::invalid_argument("maximum attempts must be positive");
  }
  for (int attempt = 1; attempt <= maximum_attempts; ++attempt) {
    if (operation(attempt)) {
      return true;
    }
    if (attempt < maximum_attempts) {
      between_attempts();
    }
  }
  return false;
}

constexpr std::array<const char *, kJointCount> kJointNames = {
  "j2n6s300_joint_1",
  "j2n6s300_joint_2",
  "j2n6s300_joint_3",
  "j2n6s300_joint_4",
  "j2n6s300_joint_5",
  "j2n6s300_joint_6",
};

constexpr JointVector kVelocityLimits = {
  18.0 * kDegreesToRadians,
  18.0 * kDegreesToRadians,
  18.0 * kDegreesToRadians,
  24.0 * kDegreesToRadians,
  24.0 * kDegreesToRadians,
  24.0 * kDegreesToRadians,
};

constexpr JointVector kLowerLimits = {
  -2.0 * kPi,
  47.0 * kDegreesToRadians,
  19.0 * kDegreesToRadians,
  -2.0 * kPi,
  -2.0 * kPi,
  -2.0 * kPi,
};

constexpr JointVector kUpperLimits = {
  2.0 * kPi,
  313.0 * kDegreesToRadians,
  341.0 * kDegreesToRadians,
  2.0 * kPi,
  2.0 * kPi,
  2.0 * kPi,
};

inline bool is_continuous(std::size_t index)
{
  return index == 0 || index == 3 || index == 4 || index == 5;
}

inline bool finite_vector(const JointVector & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

inline double shortest_delta(
  std::size_t index, double target, double current)
{
  if (!std::isfinite(target) || !std::isfinite(current)) {
    throw std::invalid_argument("joint values must be finite");
  }
  const double delta = target - current;
  if (!is_continuous(index)) {
    return delta;
  }
  return std::atan2(std::sin(delta), std::cos(delta));
}

inline bool inside_operational_limits(const JointVector & values)
{
  if (!finite_vector(values)) {
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (values[index] < kLowerLimits[index] ||
      values[index] > kUpperLimits[index])
    {
      return false;
    }
  }
  return true;
}

inline JointVector normalized_target(
  const JointVector & current, const JointVector & target)
{
  if (!finite_vector(current) || !finite_vector(target)) {
    throw std::invalid_argument("joint vectors must be finite");
  }
  JointVector normalized{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    normalized[index] =
      current[index] + shortest_delta(index, target[index], current[index]);
  }
  return normalized;
}

inline JointVector nominal_velocity(
  const JointVector & current,
  const JointVector & target,
  double duration_sec)
{
  if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
    throw std::invalid_argument("duration must be positive and finite");
  }
  if (!inside_operational_limits(target)) {
    throw std::invalid_argument("target violates operational limits");
  }
  JointVector velocity{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    velocity[index] =
      shortest_delta(index, target[index], current[index]) / duration_sec;
    if (std::abs(velocity[index]) > kVelocityLimits[index] + 1.0e-9) {
      throw std::invalid_argument("target violates velocity limits");
    }
  }
  return velocity;
}

inline JointVector track_target_velocity(
  const JointVector & current,
  const JointVector & target,
  const JointVector & requested_velocity,
  double speed_scale,
  double control_period_sec,
  double tolerance_rad)
{
  if (!finite_vector(current) || !finite_vector(target) ||
    !finite_vector(requested_velocity))
  {
    throw std::invalid_argument("joint vectors must be finite");
  }
  if (!std::isfinite(speed_scale) || speed_scale < 0.0 || speed_scale > 1.0) {
    throw std::invalid_argument("speed scale must be in [0, 1]");
  }
  if (!std::isfinite(control_period_sec) || control_period_sec <= 0.0) {
    throw std::invalid_argument("control period must be positive");
  }
  if (!std::isfinite(tolerance_rad) || tolerance_rad <= 0.0) {
    throw std::invalid_argument("tolerance must be positive");
  }

  JointVector command{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double error =
      shortest_delta(index, target[index], current[index]);
    if (std::abs(error) <= tolerance_rad || speed_scale <= 0.0) {
      command[index] = 0.0;
      continue;
    }
    const double requested_magnitude = std::abs(requested_velocity[index]);
    const double maximum =
      std::min(kVelocityLimits[index], requested_magnitude) * speed_scale;
    const double stop_limited = std::abs(error) / control_period_sec;
    const double magnitude = std::min(maximum, stop_limited);
    command[index] = std::copysign(magnitude, error);
  }
  return command;
}

inline bool target_reached(
  const JointVector & current,
  const JointVector & target,
  double tolerance_rad)
{
  if (!finite_vector(current) || !finite_vector(target) ||
    !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0)
  {
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (std::abs(shortest_delta(index, target[index], current[index])) >
      tolerance_rad)
    {
      return false;
    }
  }
  return true;
}

inline JointVector estimate_velocity(
  const JointVector & previous,
  const JointVector & current,
  double elapsed_sec)
{
  if (!finite_vector(previous) || !finite_vector(current)) {
    throw std::invalid_argument("joint vectors must be finite");
  }
  if (!std::isfinite(elapsed_sec) || elapsed_sec <= 0.0) {
    throw std::invalid_argument("sample period must be positive");
  }
  JointVector velocity{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    velocity[index] =
      shortest_delta(index, current[index], previous[index]) / elapsed_sec;
  }
  return velocity;
}

inline double decode_velocity_degrees(double raw_degrees_per_second)
{
  if (!std::isfinite(raw_degrees_per_second)) {
    throw std::invalid_argument("raw velocity must be finite");
  }
  double corrected = 2.0 * raw_degrees_per_second;
  if (corrected > 180.0) {
    corrected -= 360.0;
  }
  return corrected;
}

inline double effective_speed_scale(
  const std::string & state, double requested_scale)
{
  if (!std::isfinite(requested_scale)) {
    throw std::invalid_argument("speed scale must be finite");
  }
  if (state == "STOP") {
    return 0.0;
  }
  if (state == "ALLOW" || state == "WARNING") {
    return 1.0;
  }
  if (state == "REDUCTION" &&
    requested_scale > 0.0 && requested_scale <= 1.0)
  {
    return requested_scale;
  }
  throw std::invalid_argument("invalid execution control");
}

}  // namespace thesis_hardware_bridge

#endif  // THESIS_HARDWARE_BRIDGE__BRIDGE_LOGIC_HPP_
