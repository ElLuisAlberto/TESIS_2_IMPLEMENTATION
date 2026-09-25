"""Tests for deterministic short JOG references."""

import math

import pytest

from thesis_simulation.jog_reference import (
    advance_jog_reference,
    build_jog_reference,
)


def test_scale_one_preserves_nominal_velocity():
    positions, velocities = build_jog_reference(
        (0.0,) * 6,
        (math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        1.0,
        0.1,
    )
    assert positions[0] == pytest.approx(math.pi / 100.0)
    assert velocities[0] == pytest.approx(math.pi / 10.0)


def test_half_scale_preserves_half_nominal_velocity():
    positions, velocities = build_jog_reference(
        (0.0,) * 6,
        (math.pi / 20.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        1.0,
        0.1,
    )
    assert positions[0] == pytest.approx(math.pi / 200.0)
    assert velocities[0] == pytest.approx(math.pi / 20.0)


def test_zero_motion_produces_zero_slope():
    current = (0.2, -0.1, 0.3, 0.0, 0.4, -0.2)
    positions, velocities = build_jog_reference(
        current,
        current,
        1.0,
        0.1,
    )
    assert positions == pytest.approx(current)
    assert velocities == pytest.approx((0.0,) * 6)


@pytest.mark.parametrize(
    'current,target,horizon,period',
    [
        ((0.0,), (0.0, 1.0), 1.0, 0.1),
        ((float('nan'),), (0.0,), 1.0, 0.1),
        ((0.0,), (float('inf'),), 1.0, 0.1),
        ((0.0,), (0.0,), 0.0, 0.1),
        ((0.0,), (0.0,), 1.0, -0.1),
    ],
)
def test_invalid_input_is_rejected(current, target, horizon, period):
    with pytest.raises(ValueError):
        build_jog_reference(current, target, horizon, period)


def test_rolling_reference_does_not_inherit_measurement_lag():
    """A repeated reference advances even if the measurement is delayed."""
    current = (0.0,) * 6
    horizon_target = (math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    first, velocity = advance_jog_reference(
        current, horizon_target, 1.0, 0.1, None, None, 0.25
    )
    second, velocity = advance_jog_reference(
        current, horizon_target, 1.0, 0.1, first, velocity, 0.25
    )
    assert first[0] == pytest.approx(math.pi / 100.0)
    assert second[0] == pytest.approx(math.pi / 50.0)


def test_rolling_reference_is_bounded_by_maximum_lead():
    """The rolling target cannot drift beyond the configured safe lead."""
    current = (0.0,) * 6
    horizon_target = (math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    previous = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    previous_velocity = (math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    target, _ = advance_jog_reference(
        current,
        horizon_target,
        1.0,
        0.1,
        previous,
        previous_velocity,
        0.25,
    )
    assert target[0] == pytest.approx(math.pi / 40.0)


def test_direction_reversal_reanchors_to_measurement():
    """A reversal takes effect immediately from the measured position."""
    current = (0.0,) * 6
    target, velocity = advance_jog_reference(
        current,
        (-math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        1.0,
        0.1,
        (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        (math.pi / 10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        0.25,
    )
    assert target[0] == pytest.approx(-math.pi / 100.0)
    assert velocity[0] == pytest.approx(-math.pi / 10.0)
