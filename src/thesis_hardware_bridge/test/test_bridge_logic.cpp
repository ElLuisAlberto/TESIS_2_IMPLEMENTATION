#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "thesis_hardware_bridge/bridge_logic.hpp"

namespace bridge = thesis_hardware_bridge;

TEST(BridgeLogic, ShortestDeltaWrapsContinuousJoints)
{
  const double current = 179.0 * bridge::kDegreesToRadians;
  const double target = -179.0 * bridge::kDegreesToRadians;
  EXPECT_NEAR(
    bridge::shortest_delta(0, target, current),
    2.0 * bridge::kDegreesToRadians, 1.0e-12);
  EXPECT_NEAR(
    bridge::shortest_delta(1, target, current),
    -358.0 * bridge::kDegreesToRadians, 1.0e-12);
}

TEST(BridgeLogic, RejectsTargetThatExceedsVelocityInvariant)
{
  bridge::JointVector current = {
    0.0, bridge::kPi, bridge::kPi, 0.0, 0.0, 0.0};
  bridge::JointVector target = current;
  target[0] += 20.0 * bridge::kDegreesToRadians;
  EXPECT_THROW(
    bridge::nominal_velocity(current, target, 1.0),
    std::invalid_argument);
}

TEST(BridgeLogic, ReductionScalesPhysicalVelocity)
{
  bridge::JointVector current = {
    0.0, bridge::kPi, bridge::kPi, 0.0, 0.0, 0.0};
  bridge::JointVector target = current;
  target[0] += 10.0 * bridge::kDegreesToRadians;
  const auto nominal = bridge::nominal_velocity(current, target, 1.0);
  const auto command = bridge::track_target_velocity(
    current, target, nominal, 0.5, 0.01,
    4.0,
    0.1 * bridge::kDegreesToRadians);
  EXPECT_NEAR(
    command[0], 5.0 * bridge::kDegreesToRadians, 1.0e-12);
}

TEST(BridgeLogic, PositionTrackingBrakesBeforeTheTarget)
{
  bridge::JointVector current = {
    0.0, bridge::kPi, bridge::kPi, 0.0, 0.0, 0.0};
  bridge::JointVector target = current;
  bridge::JointVector requested{};
  target[0] += 1.0 * bridge::kDegreesToRadians;
  requested[0] = 10.0 * bridge::kDegreesToRadians;

  const auto command = bridge::track_target_velocity(
    current, target, requested, 1.0, 0.05, 4.0,
    0.1 * bridge::kDegreesToRadians);

  EXPECT_NEAR(
    command[0], 4.0 * bridge::kDegreesToRadians, 1.0e-12);
}

TEST(BridgeLogic, PositionTrackingConvergesWithoutCrossingTheTarget)
{
  bridge::JointVector current = {
    0.0, bridge::kPi, bridge::kPi, 0.0, 0.0, 0.0};
  bridge::JointVector target = current;
  bridge::JointVector requested{};
  target[0] += 5.0 * bridge::kDegreesToRadians;
  requested[0] = 10.0 * bridge::kDegreesToRadians;

  double previous_error = target[0] - current[0];
  for (int sample = 0; sample < 100; ++sample) {
    const auto command = bridge::track_target_velocity(
      current, target, requested, 1.0, 0.05, 4.0,
      0.1 * bridge::kDegreesToRadians);
    current[0] += command[0] * 0.05;
    const double error = target[0] - current[0];
    EXPECT_GE(error, -1.0e-12);
    EXPECT_LE(std::abs(error), std::abs(previous_error) + 1.0e-12);
    previous_error = error;
  }

  EXPECT_LE(
    std::abs(previous_error), 0.1 * bridge::kDegreesToRadians);
}

TEST(BridgeLogic, StopAlwaysForcesZeroScale)
{
  EXPECT_DOUBLE_EQ(bridge::effective_speed_scale("STOP", 1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    bridge::effective_speed_scale("WARNING", 0.2), 1.0);
  EXPECT_DOUBLE_EQ(
    bridge::effective_speed_scale("REDUCTION", 0.35), 0.35);
}

TEST(BridgeLogic, KinovaVelocityFeedbackIsDecoded)
{
  EXPECT_DOUBLE_EQ(bridge::decode_velocity_degrees(5.0), 10.0);
  EXPECT_DOUBLE_EQ(bridge::decode_velocity_degrees(177.5), -5.0);
}

TEST(BridgeLogic, ContinuousCommandsMustBeFresh)
{
  EXPECT_TRUE(bridge::message_age_is_fresh(0.04, 0.15));
  EXPECT_TRUE(bridge::message_age_is_fresh(-0.05, 0.15));
  EXPECT_FALSE(bridge::message_age_is_fresh(0.151, 0.15));
  EXPECT_FALSE(bridge::message_age_is_fresh(-0.051, 0.15));
  EXPECT_FALSE(bridge::message_age_is_fresh(
    std::numeric_limits<double>::infinity(), 0.15));
}

TEST(BridgeLogic, TargetToleranceUsesWrappedDistance)
{
  bridge::JointVector current = {
    179.9 * bridge::kDegreesToRadians,
    bridge::kPi, bridge::kPi, 0.0, 0.0, 0.0};
  bridge::JointVector target = current;
  target[0] = -179.9 * bridge::kDegreesToRadians;
  EXPECT_TRUE(bridge::target_reached(
    current, target, 0.3 * bridge::kDegreesToRadians));
}

TEST(BridgeLogic, EstimatedVelocityUsesWrappedJointDistance)
{
  bridge::JointVector previous{};
  bridge::JointVector current{};
  previous[0] = 359.0 * bridge::kDegreesToRadians;
  current[0] = 1.0 * bridge::kDegreesToRadians;
  current[1] = 2.0 * bridge::kDegreesToRadians;

  const auto velocity = bridge::estimate_velocity(
    previous, current, 0.02);

  EXPECT_NEAR(
    velocity[0], 100.0 * bridge::kDegreesToRadians, 1.0e-12);
  EXPECT_NEAR(
    velocity[1], 100.0 * bridge::kDegreesToRadians, 1.0e-12);
  EXPECT_THROW(
    bridge::estimate_velocity(previous, current, 0.0),
    std::invalid_argument);
}

TEST(BridgeLogic, RetryOperationRecoversFromTransientFailure)
{
  int calls = 0;
  int waits = 0;
  const bool succeeded = bridge::retry_operation(
    5,
    [&calls](int attempt) {
      ++calls;
      return attempt == 3;
    },
    [&waits]() {++waits;});

  EXPECT_TRUE(succeeded);
  EXPECT_EQ(calls, 3);
  EXPECT_EQ(waits, 2);
}

TEST(BridgeLogic, RetryOperationStopsAfterConfiguredAttempts)
{
  int calls = 0;
  int waits = 0;
  const bool succeeded = bridge::retry_operation(
    4,
    [&calls](int) {
      ++calls;
      return false;
    },
    [&waits]() {++waits;});

  EXPECT_FALSE(succeeded);
  EXPECT_EQ(calls, 4);
  EXPECT_EQ(waits, 3);
  EXPECT_THROW(
    bridge::retry_operation(0, [](int) {return true;}, []() {}),
    std::invalid_argument);
}
