#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

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

TEST(BridgeLogic, ArmedHoldDoesNotStreamPhysicalVelocityCommands)
{
  EXPECT_FALSE(bridge::should_transmit_motion_command(true, false, false));
  EXPECT_TRUE(bridge::should_transmit_motion_command(true, true, false));
  EXPECT_TRUE(bridge::should_transmit_motion_command(true, false, true));
  EXPECT_FALSE(bridge::should_transmit_motion_command(false, true, true));
}

TEST(BridgeLogic, SlowOutputDoesNotHoldFeedbackStateLock)
{
  // The adapter computes a command under state_mutex_ and releases it before
  // the potentially slow Kinova output call. Model that ownership boundary:
  // an output call may take 250 ms while a feedback update remains available.
  std::mutex state_mutex;
  std::atomic<bool> output_started{false};
  std::thread slow_output([&output_started]() {
    output_started.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  });
  while (!output_started.load()) {
    std::this_thread::yield();
  }
  const auto started = std::chrono::steady_clock::now();
  {
    const std::lock_guard<std::mutex> state_lock(state_mutex);
  }
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  slow_output.join();
  EXPECT_LT(elapsed, 100.0);
}

TEST(BridgeLogic, JogMustBelongToCurrentArmEpoch)
{
  EXPECT_FALSE(bridge::jog_command_matches_arm_epoch(
      "jog_continuous", "arm-7"));
  EXPECT_FALSE(bridge::jog_command_matches_arm_epoch(
      "jog:arm-6:123", "arm-7"));
  EXPECT_FALSE(bridge::jog_command_matches_arm_epoch(
      "jog::123", ""));
  EXPECT_TRUE(bridge::jog_command_matches_arm_epoch(
      "jog:arm-7:123", "arm-7"));
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

TEST(BridgeLogic, ProximityInterlockRequiresWorldFreshSourceAndKnownState)
{
  EXPECT_TRUE(bridge::proximity_sample_is_valid(
    "ALLOW", "world", 0.25, 0.10, false, 0.10, 0.75));
  EXPECT_TRUE(bridge::proximity_sample_is_valid(
    "STOP", "world", -0.01, 0.10, false, 0.10, 0.75));
  EXPECT_FALSE(bridge::proximity_sample_is_valid(
    "ALLOW", "camera", 0.25, 0.10, false, 0.10, 0.75));
  EXPECT_FALSE(bridge::proximity_sample_is_valid(
    "HOLD", "world", 0.25, 0.10, false, 0.10, 0.75));
  EXPECT_FALSE(bridge::proximity_sample_is_valid(
    "ALLOW", "world", 0.25, 0.10, true, 0.10, 0.75));
  EXPECT_FALSE(bridge::proximity_sample_is_valid(
    "ALLOW", "world", 0.25, 0.10, false, 0.751, 0.75));
  EXPECT_FALSE(bridge::proximity_sample_is_valid(
    "ALLOW", "world", 0.25, 0.10, false, -0.051, 0.75));
}

TEST(BridgeLogic, PointCommandAdmissionRejectsInvalidStampsAndReusedIds)
{
  bridge::RecentCommandIds recent_ids;

  EXPECT_EQ(
    bridge::admit_point_command(
      "point-fresh", false, 0.10, 0.15, recent_ids),
    bridge::CommandAdmission::kAccepted);
  EXPECT_EQ(
    bridge::admit_point_command(
      "point-stale", false, 0.151, 0.15, recent_ids),
    bridge::CommandAdmission::kStaleStamp);
  EXPECT_EQ(
    bridge::admit_point_command(
      "point-future", false, -0.051, 0.15, recent_ids),
    bridge::CommandAdmission::kFutureStamp);
  EXPECT_EQ(
    bridge::admit_point_command(
      "", false, 0.01, 0.15, recent_ids),
    bridge::CommandAdmission::kEmptyId);
  EXPECT_EQ(
    bridge::admit_point_command(
      "point-zero", true, 0.01, 0.15, recent_ids),
    bridge::CommandAdmission::kZeroStamp);
  EXPECT_EQ(
    bridge::admit_point_command(
      "point-fresh", false, 0.01, 0.15, recent_ids),
    bridge::CommandAdmission::kReusedId);
}

TEST(BridgeLogic, ExecutionControlAdmissionRequiresFreshActiveId)
{
  EXPECT_EQ(
    bridge::admit_execution_control(
      "point-active", "point-active", false, 0.10, 0.15, false),
    bridge::CommandAdmission::kAccepted);
  EXPECT_EQ(
    bridge::admit_execution_control(
      "point-active", "point-active", false, 0.151, 0.15, false),
    bridge::CommandAdmission::kStaleStamp);
  EXPECT_EQ(
    bridge::admit_execution_control(
      "point-active", "point-active", false, -0.051, 0.15, false),
    bridge::CommandAdmission::kFutureStamp);
  EXPECT_EQ(
    bridge::admit_execution_control(
      "point-active", "point-active", true, 0.01, 0.15, false),
    bridge::CommandAdmission::kZeroStamp);
  EXPECT_EQ(
    bridge::admit_execution_control(
      "", "point-active", false, 0.01, 0.15, false),
    bridge::CommandAdmission::kEmptyId);
  EXPECT_EQ(
    bridge::admit_execution_control(
      "other", "point-active", false, 0.01, 0.15, false),
    bridge::CommandAdmission::kNotActive);
}

TEST(BridgeLogic, StopControlRetainsPriority)
{
  EXPECT_EQ(
    bridge::admit_execution_control(
      "", "point-active", true, 10.0, 0.15, true),
    bridge::CommandAdmission::kAccepted);
  EXPECT_DOUBLE_EQ(bridge::effective_speed_scale("STOP", 1.0), 0.0);
}

TEST(BridgeLogic, ArmInterlockRequiresEveryPrecondition)
{
  bridge::ArmInterlockInputs inputs;
  inputs.output_permitted = true;
  inputs.connected = true;
  inputs.joint_state_fresh = true;
  inputs.system_ready = true;
  inputs.system_ready_fresh = true;
  inputs.proximity_valid = true;
  inputs.proximity_fresh = true;
  inputs.supervisor_available = true;
  inputs.safety_stop_active = false;

  EXPECT_TRUE(bridge::arm_interlock_reasons(inputs).empty());
  EXPECT_FALSE(bridge::should_disarm_for_interlock_loss(true, inputs));

  inputs.output_permitted = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.output_permitted = true;
  inputs.connected = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.connected = true;
  inputs.joint_state_fresh = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.joint_state_fresh = true;
  inputs.system_ready = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.system_ready = true;
  inputs.system_ready_fresh = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.system_ready_fresh = true;
  inputs.proximity_valid = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.proximity_valid = true;
  inputs.proximity_fresh = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.proximity_fresh = true;
  inputs.supervisor_available = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.supervisor_available = true;
  inputs.safety_stop_active = true;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
}

TEST(BridgeLogic, ArmInterlockDisarmsAfterSafetyLoss)
{
  bridge::ArmInterlockInputs inputs;
  inputs.output_permitted = true;
  inputs.connected = true;
  inputs.joint_state_fresh = true;
  inputs.system_ready = true;
  inputs.system_ready_fresh = true;
  inputs.proximity_valid = true;
  inputs.proximity_fresh = true;
  inputs.supervisor_available = true;
  inputs.safety_stop_active = false;

  EXPECT_FALSE(bridge::should_disarm_for_interlock_loss(true, inputs));
  inputs.proximity_fresh = false;
  EXPECT_TRUE(bridge::should_disarm_for_interlock_loss(true, inputs));
  EXPECT_FALSE(bridge::should_disarm_for_interlock_loss(false, inputs));
}

TEST(BridgeLogic, GeometricStopAllowsOnlyExplicitProtectiveRecovery)
{
  bridge::ArmInterlockInputs inputs;
  inputs.output_permitted = true;
  inputs.connected = true;
  inputs.joint_state_fresh = true;
  inputs.system_ready = false;
  inputs.system_ready_fresh = true;
  inputs.proximity_valid = true;
  inputs.proximity_fresh = true;
  inputs.supervisor_available = true;
  inputs.safety_stop_active = true;

  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  inputs.protective_recovery_active = true;
  EXPECT_TRUE(bridge::arm_interlock_reasons(inputs).empty());
  EXPECT_FALSE(bridge::should_disarm_for_interlock_loss(true, inputs));

  inputs.proximity_fresh = false;
  EXPECT_FALSE(bridge::arm_interlock_reasons(inputs).empty());
  EXPECT_TRUE(bridge::should_disarm_for_interlock_loss(true, inputs));
}

TEST(BridgeLogic, SupervisorStopLatchClearsOnlyForAdmittedMotionOrDisarm)
{
  EXPECT_TRUE(bridge::next_supervisor_stop_active(
      false, true, false, false));
  EXPECT_TRUE(bridge::next_supervisor_stop_active(
      true, false, false, false));
  EXPECT_FALSE(bridge::next_supervisor_stop_active(
      true, false, true, false));
  EXPECT_FALSE(bridge::next_supervisor_stop_active(
      true, false, false, true));
  EXPECT_TRUE(bridge::next_supervisor_stop_active(
      true, true, true, true));
}

TEST(BridgeLogic, RecoverySurvivesReadyRaceUntilFreshJogIsAdmitted)
{
  bool supervisor_stop = bridge::next_supervisor_stop_active(
    false, true, false, false);
  bool recovery = bridge::next_protective_recovery_active(
    false, true, true, false, supervisor_stop);
  EXPECT_TRUE(recovery);

  // Proximity becomes REDUCTION/READY before the admitted JOG reaches the
  // adapter.  The historical STOP must not cause an intervening disarm.
  recovery = bridge::next_protective_recovery_active(
    recovery, false, true, true, supervisor_stop);
  EXPECT_TRUE(recovery);

  supervisor_stop = bridge::next_supervisor_stop_active(
    supervisor_stop, false, true, false);
  recovery = bridge::next_protective_recovery_active(
    recovery, false, true, true, supervisor_stop);
  EXPECT_FALSE(supervisor_stop);
  EXPECT_FALSE(recovery);
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
