"""Deterministic tests for short-horizon joint prediction."""

import math

import pytest

from thesis_core.joint_model import JOINT_NAMES, JOINT_VELOCITY_LIMITS
from thesis_core.joint_prediction import (
    is_state_fresh,
    predict_joint_samples,
)


INITIAL = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def assert_velocity_limits(prediction):
    for first, second in zip(
        prediction.positions, prediction.positions[1:]
    ):
        for name, start, end in zip(JOINT_NAMES, first, second):
            velocity = abs(end - start) / prediction.sample_period
            assert velocity <= JOINT_VELOCITY_LIMITS[name] + 1.0e-12


def test_single_joint_has_analytic_solution_and_exact_endpoints():
    target = list(INITIAL)
    target[0] += 0.10
    result = predict_joint_samples(INITIAL, target, 1.0, 1.0, 21)
    assert result.positions[0] == INITIAL
    assert result.sample_times[0] == 0.0
    assert result.sample_times[-1] == 1.0
    assert math.isclose(result.sample_period, 0.05, abs_tol=1.0e-12)
    assert math.isclose(result.positions[10][0], 0.05, abs_tol=1.0e-12)
    assert math.isclose(result.positions[-1][0], 0.10, abs_tol=1.0e-12)
    assert_velocity_limits(result)


def test_two_joints_move_simultaneously_with_common_time_base():
    target = list(INITIAL)
    target[0] += 0.20
    target[3] += 0.30
    result = predict_joint_samples(INITIAL, target, 1.0, 1.0, 21)
    assert math.isclose(result.positions[10][0], 0.10, abs_tol=1.0e-12)
    assert math.isclose(result.positions[10][3], 0.15, abs_tol=1.0e-12)
    assert result.positions[-1] == pytest.approx(target)
    assert_velocity_limits(result)


def test_trajectory_finishing_early_holds_target_until_horizon():
    target = list(INITIAL)
    target[0] += 0.10
    result = predict_joint_samples(INITIAL, target, 0.5, 1.0, 21)
    assert result.positions[10] == pytest.approx(target)
    assert all(
        sample == pytest.approx(target) for sample in result.positions[10:]
    )
    assert_velocity_limits(result)


def test_duration_longer_than_horizon_returns_one_second_state():
    target = list(INITIAL)
    target[0] += 0.20
    result = predict_joint_samples(INITIAL, target, 2.0, 1.0, 21)
    assert math.isclose(result.positions[-1][0], 0.10, abs_tol=1.0e-12)
    assert_velocity_limits(result)


def test_immediate_direction_reversal_changes_first_future_sample():
    positive = list(INITIAL)
    negative = list(INITIAL)
    positive[0] += 0.20
    negative[0] -= 0.20
    forward = predict_joint_samples(INITIAL, positive, 1.0, 1.0, 21)
    reverse = predict_joint_samples(INITIAL, negative, 1.0, 1.0, 21)
    assert forward.positions[1][0] > INITIAL[0]
    assert reverse.positions[1][0] < INITIAL[0]


def test_continuous_joint_crosses_pi_by_shortest_positive_turn():
    current = (math.radians(170.0), math.pi, math.pi, 0.0, 0.0, 0.0)
    target = (math.radians(-170.0), math.pi, math.pi, 0.0, 0.0, 0.0)
    result = predict_joint_samples(current, target, 2.0, 2.0, 41)
    assert math.isclose(
        result.positions[-1][0], math.radians(190.0), abs_tol=1.0e-12
    )
    assert all(
        second[0] >= first[0]
        for first, second in zip(result.positions, result.positions[1:])
    )
    assert_velocity_limits(result)


def test_stale_state_is_rejected_by_freshness_predicate():
    assert is_state_fresh(10.0, 10.5, 0.5)
    assert not is_state_fresh(10.0, 10.500001, 0.5)
    assert not is_state_fresh(11.0, 10.0, 0.5)


def test_rest_keeps_every_sample_at_measured_state():
    result = predict_joint_samples(INITIAL, INITIAL, 1.0, 1.0, 21)
    assert all(sample == INITIAL for sample in result.positions)
    assert result.effective_duration == 1.0


def test_fast_request_extends_effective_duration_to_velocity_limit():
    target = list(INITIAL)
    target[0] += math.radians(36.0)
    result = predict_joint_samples(INITIAL, target, 1.0, 1.0, 21)
    assert math.isclose(result.effective_duration, 2.0, abs_tol=1.0e-12)
    assert math.isclose(
        result.positions[-1][0], math.radians(18.0), abs_tol=1.0e-12
    )
    assert_velocity_limits(result)


@pytest.mark.parametrize(
    'duration,horizon,samples',
    [(0.0, 1.0, 21), (1.0, 0.0, 21), (1.0, 1.0, 1)],
)
def test_invalid_temporal_configuration_is_rejected(
    duration, horizon, samples
):
    with pytest.raises(ValueError):
        predict_joint_samples(INITIAL, INITIAL, duration, horizon, samples)
