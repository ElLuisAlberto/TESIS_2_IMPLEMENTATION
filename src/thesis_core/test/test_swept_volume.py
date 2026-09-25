"""Tests for deterministic swept-capsule construction."""

import math

import pytest

from thesis_core.jaco_kinematics import CAPSULE_RADII
from thesis_core.swept_volume import (
    build_swept_volume,
    maximum_endpoint_travel,
    sample_capsule_axis,
)


Q0 = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def trajectory(joint_deltas, count=21):
    return tuple(tuple(
        value + joint_deltas[joint] * index / (count - 1)
        for joint, value in enumerate(Q0)
    ) for index in range(count))


def test_six_capsules_use_inflated_canonical_radii():
    volume = build_swept_volume((Q0,), 0.02, 0.04)
    assert len(volume.current_capsules) == 6
    assert tuple(
        capsule.radius for capsule in volume.current_capsules
    ) == pytest.approx(tuple(radius + 0.02 for radius in CAPSULE_RADII))


def test_axis_samples_include_endpoints_and_bound_spacing():
    points = sample_capsule_axis((0.0, 0.0, 0.0), (0.0, 0.0, 0.101), 0.04)
    assert points[0] == (0.0, 0.0, 0.0)
    assert points[-1] == (0.0, 0.0, 0.101)
    assert all(
        math.dist(first, second) <= 0.04 + 1.0e-12
        for first, second in zip(points, points[1:])
    )


def test_rest_volume_equals_current_inflated_capsules():
    volume = build_swept_volume((Q0,) * 21, 0.02, 0.04)
    for current, predicted in zip(
        volume.current_capsules, volume.predicted_points
    ):
        assert predicted == current.points
    assert maximum_endpoint_travel(volume) == 0.0


@pytest.mark.parametrize('delta', (0.20, -0.20))
def test_joint_one_positive_and_negative_create_swept_motion(delta):
    volume = build_swept_volume(
        trajectory((delta, 0.0, 0.0, 0.0, 0.0, 0.0)),
        0.02,
        0.04,
    )
    assert maximum_endpoint_travel(volume) > 0.01


@pytest.mark.parametrize('delta', (0.20, -0.20))
def test_joint_two_positive_and_negative_create_swept_motion(delta):
    volume = build_swept_volume(
        trajectory((0.0, delta, 0.0, 0.0, 0.0, 0.0)),
        0.02,
        0.04,
    )
    assert maximum_endpoint_travel(volume) > 0.01


def test_two_joint_motion_differs_from_each_individual_motion():
    joint_one = build_swept_volume(
        trajectory((0.20, 0.0, 0.0, 0.0, 0.0, 0.0)), 0.02, 0.04
    )
    joint_two = build_swept_volume(
        trajectory((0.0, -0.20, 0.0, 0.0, 0.0, 0.0)), 0.02, 0.04
    )
    combined = build_swept_volume(
        trajectory((0.20, -0.20, 0.0, 0.0, 0.0, 0.0)), 0.02, 0.04
    )
    assert combined.endpoint_traces != joint_one.endpoint_traces
    assert combined.endpoint_traces != joint_two.endpoint_traces


def test_direction_change_rebuilds_without_residual_points():
    positive = trajectory((0.20, 0.0, 0.0, 0.0, 0.0, 0.0))
    negative = trajectory((-0.20, 0.0, 0.0, 0.0, 0.0, 0.0))
    forward = build_swept_volume(positive, 0.02, 0.04)
    reverse = build_swept_volume(negative, 0.02, 0.04)
    rebuilt = build_swept_volume(negative, 0.02, 0.04)
    assert forward.endpoint_traces != reverse.endpoint_traces
    assert rebuilt == reverse


def test_half_scale_shortens_future_geometric_travel():
    full = build_swept_volume(
        trajectory((0.20, 0.0, 0.0, 0.0, 0.0, 0.0)), 0.02, 0.04
    )
    half = build_swept_volume(
        trajectory((0.10, 0.0, 0.0, 0.0, 0.0, 0.0)), 0.02, 0.04
    )
    assert maximum_endpoint_travel(half) < maximum_endpoint_travel(full)


@pytest.mark.parametrize(
    'margin,spacing',
    ((-0.01, 0.04), (0.02, 0.0), (math.nan, 0.04)),
)
def test_invalid_geometry_configuration_is_rejected(margin, spacing):
    with pytest.raises(ValueError):
        build_swept_volume((Q0,), margin, spacing)
