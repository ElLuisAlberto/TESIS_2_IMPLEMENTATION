"""Unit tests for deterministic capsule-sphere clearance."""

import math

import pytest

from thesis_core.clearance_geometry import (
    closest_point_on_segment,
    segment_sphere_clearance,
)


def clearance(center):
    return segment_sphere_clearance(
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        0.20,
        center,
        0.30,
    )


def test_sphere_in_front_of_segment_center():
    result = clearance((1.0, 1.0, 0.0))
    assert result.closest_robot_point == pytest.approx((1.0, 0.0, 0.0))
    assert result.clearance == pytest.approx(0.50)


def test_sphere_near_segment_endpoint():
    result = clearance((3.0, 0.0, 0.0))
    assert result.closest_robot_point == pytest.approx((2.0, 0.0, 0.0))
    assert result.clearance == pytest.approx(0.50)


def test_overlapping_sphere_has_negative_clearance():
    assert clearance((1.0, 0.10, 0.0)).clearance == pytest.approx(-0.40)


def test_far_sphere_has_large_positive_clearance():
    assert clearance((1.0, 10.0, 0.0)).clearance == pytest.approx(9.50)


def test_degenerate_segment_uses_its_endpoint():
    point = closest_point_on_segment(
        (2.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    assert point == (1.0, 0.0, 0.0)


@pytest.mark.parametrize(
    'center,radius',
    (((math.nan, 0.0, 0.0), 0.1), ((0.0, 0.0, 0.0), math.inf)),
)
def test_non_finite_geometry_is_rejected(center, radius):
    with pytest.raises(ValueError):
        segment_sphere_clearance(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            0.1,
            center,
            radius,
        )
