"""Regression tests for the canonical JACO2 joint model."""

import math

import pytest

from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    normalize_target,
    saturate_target_by_velocity,
    validate_duration,
    validate_joint_names,
    validate_joint_positions,
)


ZERO = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def target_with_velocity(index, degrees_per_second, duration=1.0):
    target = list(ZERO)
    target[index] += math.radians(degrees_per_second) * duration
    return tuple(target)


def test_canonical_joint_order():
    assert JOINT_NAMES == tuple(
        f'j2n6s300_joint_{index}' for index in range(1, 7)
    )
    assert validate_joint_names(JOINT_NAMES) == JOINT_NAMES


def test_j1_positive_36_is_limited_to_positive_18_degrees_per_second():
    result = saturate_target_by_velocity(
        ZERO, target_with_velocity(0, 36.0), 1.0
    )
    assert result.was_limited
    assert result.limiting_joint == JOINT_NAMES[0]
    assert math.isclose(
        result.positions[0] - ZERO[0], math.radians(18.0), abs_tol=1e-12
    )


def test_j1_negative_36_is_limited_to_negative_18_degrees_per_second():
    result = saturate_target_by_velocity(
        ZERO, target_with_velocity(0, -36.0), 1.0
    )
    assert result.was_limited
    assert math.isclose(
        result.positions[0] - ZERO[0], -math.radians(18.0), abs_tol=1e-12
    )


def test_j4_positive_48_is_limited_to_positive_24_degrees_per_second():
    result = saturate_target_by_velocity(
        ZERO, target_with_velocity(3, 48.0), 1.0
    )
    assert result.was_limited
    assert result.limiting_joint == JOINT_NAMES[3]
    assert math.isclose(
        result.positions[3] - ZERO[3], math.radians(24.0), abs_tol=1e-12
    )


def test_continuous_joint_uses_positive_twenty_degree_shortest_turn():
    start = (math.radians(170.0), math.pi, math.pi, 0.0, 0.0, 0.0)
    target = (math.radians(-170.0), math.pi, math.pi, 0.0, 0.0, 0.0)
    normalized = normalize_target(start, target)
    assert math.isclose(
        normalized[0] - start[0], math.radians(20.0), abs_tol=1e-12
    )


def test_non_finite_position_is_rejected():
    invalid = list(ZERO)
    invalid[2] = math.nan
    with pytest.raises(ValueError, match='non-finite'):
        validate_joint_positions(invalid)


def test_wrong_joint_order_is_rejected():
    invalid = list(JOINT_NAMES)
    invalid[0], invalid[1] = invalid[1], invalid[0]
    with pytest.raises(ValueError, match='canonical order'):
        validate_joint_names(invalid)


def test_wrong_position_count_is_rejected():
    with pytest.raises(ValueError, match='six'):
        validate_joint_positions(ZERO[:-1])


@pytest.mark.parametrize('duration', [0.0, -1.0, math.nan])
def test_non_positive_or_non_finite_duration_is_rejected(duration):
    with pytest.raises(ValueError, match='positive and finite'):
        validate_duration(duration)


def test_target_inside_limit_is_unchanged():
    target = target_with_velocity(0, 10.0)
    result = saturate_target_by_velocity(ZERO, target, 1.0)
    assert not result.was_limited
    assert result.positions == pytest.approx(target)


def test_operational_limits_are_exactly_18_and_24_degrees_per_second():
    expected = [18.0, 18.0, 18.0, 24.0, 24.0, 24.0]
    actual = [
        math.degrees(JOINT_VELOCITY_LIMITS[name]) for name in JOINT_NAMES
    ]
    assert actual == pytest.approx(expected)
