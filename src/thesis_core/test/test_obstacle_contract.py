"""Contract validation and frame-transform tests for obstacle estimates."""

import math

import pytest

from thesis_core.obstacle_contract import (
    rotate_vector,
    transform_obstacle,
    validate_obstacle,
)


def test_accepts_fresh_finite_obstacle():
    assert validate_obstacle(
        (1, 2, 3), (0, 0, 0), 0.1, 0.02, 10.0, 10.2
    ) == ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), 0.1, 0.02)


@pytest.mark.parametrize('stamp', (8.0, 10.2))
def test_rejects_stale_or_future_estimates(stamp):
    with pytest.raises(ValueError):
        validate_obstacle((0, 0, 0), (0, 0, 0), 0.1, 0.0, stamp, 10.0)


@pytest.mark.parametrize(
    'center,velocity,radius,uncertainty',
    [((math.nan, 0, 0), (0, 0, 0), 0.1, 0),
     ((0, 0, 0), (math.inf, 0, 0), 0.1, 0),
     ((0, 0, 0), (0, 0, 0), 0.0, 0),
     ((0, 0, 0), (0, 0, 0), 0.1, -0.01)],
)
def test_rejects_invalid_geometry(center, velocity, radius, uncertainty):
    with pytest.raises(ValueError):
        validate_obstacle(
            center, velocity, radius, uncertainty, 1.0, 1.0
        )


def test_transform_rotates_center_and_velocity_and_adds_translation():
    half = math.sqrt(0.5)
    center, velocity = transform_obstacle(
        (1, 0, 0), (1, 0, 0), (2, 3, 4), (0, 0, half, half)
    )
    assert center == pytest.approx((2, 4, 4))
    assert velocity == pytest.approx((0, 1, 0))


def test_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        rotate_vector((1, 0, 0), (0, 0, 0, 0))
