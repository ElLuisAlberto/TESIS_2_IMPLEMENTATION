"""Tests for minimum capsule clearance over a future horizon."""

import pytest

from thesis_core.clearance_geometry import minimum_configuration_clearance
from thesis_core.horizon_clearance import evaluate_capsule_horizon


NAMES = tuple(f'segment_{index}' for index in range(6))
RADII = (0.10,) * 6
TIMES = tuple(index * 0.05 for index in range(21))


def samples():
    result = []
    for sample_index in range(21):
        x_value = 0.01 * sample_index
        result.append(tuple(
            (
                (x_value, 0.0, 0.0),
                (x_value + 1.0, 0.0, 0.0),
            )
            for segment in range(6)
        ))
    return tuple(result)


def evaluate(center, velocity=(0.0, 0.0, 0.0)):
    return evaluate_capsule_horizon(
        samples(),
        TIMES,
        RADII,
        NAMES,
        center,
        velocity,
        0.10,
    )


def test_six_by_twenty_one_combinations_are_evaluated():
    result = evaluate((0.5, 1.0, 0.0))
    assert result.sample_count == 21
    assert result.evaluated_combinations == 126


def test_static_obstacle_preserves_its_center():
    result = evaluate((0.5, 1.0, 0.0))
    assert result.minimum.obstacle_center == pytest.approx((0.5, 1.0, 0.0))


def test_known_obstacle_velocity_updates_minimum_witness():
    result = evaluate((0.5, 1.5, 0.0), (0.0, -1.0, 0.0))
    expected = (0.5, 1.5 - result.minimum.sample_time, 0.0)
    assert result.minimum.obstacle_center == pytest.approx(expected)


def test_approach_reduces_minimum_clearance():
    static = evaluate((0.5, 1.5, 0.0))
    approaching = evaluate((0.5, 1.5, 0.0), (0.0, -1.0, 0.0))
    assert approaching.minimum.clearance < static.minimum.clearance


def test_moving_away_does_not_create_false_reduction():
    static = evaluate((0.5, 1.5, 0.0))
    moving_away = evaluate((0.5, 1.5, 0.0), (0.0, 1.0, 0.0))
    assert moving_away.minimum.clearance >= static.minimum.clearance
    assert moving_away.minimum.sample_index == 0
    assert moving_away.minimum.sample_time == 0.0


def test_minimum_sample_and_time_are_coherent():
    result = evaluate((1.15, 1.0, 0.0))
    assert result.minimum.sample_time == pytest.approx(
        TIMES[result.minimum.sample_index]
    )


def test_fast_horizon_matches_canonical_geometry_for_every_sample():
    center = (0.5, 1.5, 0.0)
    velocity = (0.0, -1.0, 0.0)
    result = evaluate(center, velocity)
    canonical = tuple(
        minimum_configuration_clearance(
            segment_sample,
            NAMES,
            RADII,
            tuple(
                center[axis] + velocity[axis] * sample_time
                for axis in range(3)
            ),
            0.10,
            sample_index,
            sample_time,
        )
        for sample_index, (sample_time, segment_sample) in enumerate(
            zip(TIMES, samples())
        )
    )
    expected = min(canonical, key=lambda item: item.clearance)

    assert result.sample_clearances == pytest.approx(
        tuple(item.clearance for item in canonical), abs=1.0e-12
    )
    assert result.minimum.clearance == pytest.approx(
        expected.clearance, abs=1.0e-12
    )
    assert result.minimum.segment_name == expected.segment_name
    assert result.minimum.sample_index == expected.sample_index
    assert result.minimum.closest_robot_point == pytest.approx(
        expected.closest_robot_point, abs=1.0e-12
    )
