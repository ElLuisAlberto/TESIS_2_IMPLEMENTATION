"""Regression tests for continuous Cartesian velocity integration."""

import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from thesis_core.cartesian_kinematics import (
    solve_inverse_kinematics_transform,
)
from thesis_core.jaco_kinematics import end_effector_transform
from thesis_simulation.cartesian_jog_model import (
    apply_twist_deadzone,
    bounded_integration_step,
    integrate_world_twist,
    limit_twist_acceleration,
    validate_world_twist,
)
from thesis_simulation.cartesian_jog_adapter import CartesianJogAdapter


IDENTITY = (
    (1.0, 0.0, 0.0, 0.4),
    (0.0, 1.0, 0.0, -0.2),
    (0.0, 0.0, 1.0, 0.8),
    (0.0, 0.0, 0.0, 1.0),
)


def test_integrates_world_translation_without_changing_orientation():
    target = integrate_world_twist(
        IDENTITY, (0.04, -0.02, 0.01), (0.0, 0.0, 0.0), 0.5
    )
    assert target[0][3] == pytest.approx(0.42)
    assert target[1][3] == pytest.approx(-0.21)
    assert target[2][3] == pytest.approx(0.805)
    assert tuple(tuple(row[:3]) for row in target[:3]) == tuple(
        row[:3] for row in IDENTITY[:3]
    )


def test_integrates_world_yaw_with_rodrigues_rotation():
    target = integrate_world_twist(
        IDENTITY,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2.0),
        1.0,
    )
    assert target[0][0] == pytest.approx(0.0, abs=1.0e-9)
    assert target[0][1] == pytest.approx(-1.0)
    assert target[1][0] == pytest.approx(1.0)
    assert target[1][1] == pytest.approx(0.0, abs=1.0e-9)


def test_rejects_twist_above_independent_hard_limits():
    with pytest.raises(ValueError, match='linear speed'):
        validate_world_twist(
            (0.16, 0.0, 0.0), (0.0, 0.0, 0.0), 0.15, 0.6
        )
    with pytest.raises(ValueError, match='angular speed'):
        validate_world_twist(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.7), 0.15, 0.6
        )


def test_rejects_invalid_horizon_and_non_finite_values():
    with pytest.raises(ValueError, match='horizon'):
        integrate_world_twist(
            IDENTITY, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        )
    with pytest.raises(ValueError, match='finite'):
        validate_world_twist(
            (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0), 0.15, 0.6
        )


def test_high_velocity_shortens_only_the_local_ik_step():
    step = bounded_integration_step(
        (0.15, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.10,
        maximum_linear_step_m=0.005,
    )
    assert step == pytest.approx(1.0 / 30.0)


def test_deadzone_and_acceleration_limit_cartesian_input():
    linear, angular = apply_twist_deadzone(
        (0.0005, -0.002, 0.0),
        (math.radians(0.2), math.radians(-2.0), 0.0),
        0.001,
        math.radians(0.5),
    )
    assert linear == (0.0, -0.002, 0.0)
    assert angular == (0.0, math.radians(-2.0), 0.0)

    limited_linear, limited_angular = limit_twist_acceleration(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.04, 0.0, 0.0),
        (0.0, 0.0, math.radians(10.0)),
        0.05,
        0.20,
        math.radians(60.0),
    )
    assert limited_linear == pytest.approx((0.01, 0.0, 0.0))
    assert limited_angular == pytest.approx(
        (0.0, 0.0, math.radians(3.0))
    )


def test_hardware_cartesian_jog_requires_fresh_arm_epoch():
    adapter = SimpleNamespace(
        require_hardware_arm_epoch=True,
        hardware_armed=True,
        arm_epoch='arm-4',
        armed_at_ns=2_000_000_000,
        jog_command_id=None,
        stamp_nanoseconds=CartesianJogAdapter.stamp_nanoseconds,
    )

    assert not CartesianJogAdapter.admit_new_hardware_stream(
        adapter, Time(sec=2, nanosec=0)
    )
    assert CartesianJogAdapter.admit_new_hardware_stream(
        adapter, Time(sec=2, nanosec=1)
    )
    assert adapter.jog_command_id == 'jog:arm-4:2000000001'


@pytest.mark.parametrize(
    'linear,angular',
    (
        ((0.04, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.04, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.04), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (math.radians(10.0), 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, math.radians(10.0), 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(10.0))),
    ),
)
def test_recommended_pose_supports_each_local_jog_axis(linear, angular):
    joints = tuple(math.radians(value) for value in (
        -45.0, 160.0, 200.0, 0.0, 15.0, 0.0
    ))
    current = end_effector_transform(joints)
    target = integrate_world_twist(current, linear, angular, 0.10)
    result = solve_inverse_kinematics_transform(
        joints,
        target,
        position_tolerance_m=0.0002,
        orientation_tolerance_rad=math.radians(0.1),
    )
    assert result.success
