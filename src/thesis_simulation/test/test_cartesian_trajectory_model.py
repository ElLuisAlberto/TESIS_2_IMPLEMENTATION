"""Regression tests for rolling Cartesian trajectory timing."""

import math

import pytest

from thesis_simulation.cartesian_trajectory_model import (
    advance_progress,
    minimum_joint_path_duration,
    rolling_target_fraction,
)


def test_minimum_duration_checks_every_uniform_segment():
    path = (
        (0.0, math.pi, math.pi, 0.0, 0.0, 0.0),
        (math.radians(9.0), math.pi, math.pi, 0.0, 0.0, 0.0),
        (math.radians(18.0), math.pi, math.pi, 0.0, 0.0, 0.0),
    )
    assert minimum_joint_path_duration(path) == pytest.approx(1.0)


def test_supervised_progress_stops_and_scales():
    assert advance_progress(0.25, 0.0, 1.0, 4.0) == pytest.approx(0.25)
    assert advance_progress(0.25, 0.5, 1.0, 4.0) == pytest.approx(0.375)
    assert advance_progress(0.95, 1.0, 1.0, 4.0) == pytest.approx(1.0)


def test_rolling_fraction_looks_one_horizon_ahead():
    assert rolling_target_fraction(0.20, 4.0, 1.0) == pytest.approx(0.45)
    assert rolling_target_fraction(0.90, 4.0, 1.0) == pytest.approx(1.0)


@pytest.mark.parametrize(
    'progress,scale,elapsed,duration',
    [
        (-0.1, 1.0, 0.1, 1.0),
        (0.0, 1.1, 0.1, 1.0),
        (0.0, 1.0, -0.1, 1.0),
        (0.0, 1.0, 0.1, 0.0),
    ],
)
def test_invalid_progress_inputs_are_rejected(
    progress, scale, elapsed, duration
):
    with pytest.raises(ValueError):
        advance_progress(progress, scale, elapsed, duration)
