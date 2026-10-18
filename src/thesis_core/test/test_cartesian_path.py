import math

import pytest

from thesis_core.cartesian_path import (
    cartesian_distance,
    interpolate_joint_path,
    interpolate_transform,
    path_segment_count,
    synchronized_duration,
)
from thesis_core.jaco_kinematics import (
    rotation_matrix_to_rpy,
    transform_matrix,
    translation,
)


def test_transform_interpolation_preserves_endpoints_and_midpoint():
    start = transform_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    target = transform_matrix(
        (0.20, -0.10, 0.30),
        (0.0, 0.0, math.radians(90.0)),
    )
    midpoint = interpolate_transform(start, target, 0.5)
    assert translation(interpolate_transform(start, target, 0.0)) == pytest.approx(
        translation(start)
    )
    assert translation(interpolate_transform(start, target, 1.0)) == pytest.approx(
        translation(target)
    )
    assert translation(midpoint) == pytest.approx((0.10, -0.05, 0.15))
    assert rotation_matrix_to_rpy(midpoint)[2] == pytest.approx(
        math.radians(45.0)
    )


def test_distance_and_duration_synchronize_translation_and_rotation():
    start = transform_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    target = transform_matrix(
        (0.12, 0.0, 0.0),
        (0.0, 0.0, math.radians(20.0)),
    )
    distance, angle = cartesian_distance(start, target)
    duration = synchronized_duration(
        distance,
        angle,
        0.04,
        math.radians(10.0),
    )
    assert distance == pytest.approx(0.12)
    assert angle == pytest.approx(math.radians(20.0))
    assert duration == pytest.approx(3.0)


def test_segment_count_respects_linear_and_angular_resolution():
    assert path_segment_count(0.12, math.radians(20.0)) == 12
    assert path_segment_count(0.01, math.radians(45.0)) == 23


def test_joint_path_interpolation_uses_uniform_progress():
    path = ((0.0, 1.0), (1.0, 2.0), (2.0, 4.0))
    assert interpolate_joint_path(path, 0.25) == pytest.approx((0.5, 1.5))
    assert interpolate_joint_path(path, 0.75) == pytest.approx((1.5, 3.0))
