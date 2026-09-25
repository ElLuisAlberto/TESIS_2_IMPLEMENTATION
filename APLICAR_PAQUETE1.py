#!/usr/bin/env python3
"""Apply Package 1 joint-model centralization to the exact audited sources."""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict


EXPECTED_HASHES: Dict[str, str] = {
    'src/thesis_core/thesis_core/safety_supervisor_node.py':
        'fa7dea61245d142658bccd2b4c25dea6d0fafc71035e379f20a080a6d1e5336d',
    'src/thesis_core/thesis_core/execution_reference.py':
        'dd4565c26bd6ba3c22ec8d572252abbe5ab92117cd1b5697d0569e155545aa61',
    'src/thesis_core/thesis_core/horizon_preview.py':
        'c18b5b9022f0b2c401eabc9ef132e387619715255915628d8db182200d44fcae',
    'src/thesis_simulation/thesis_simulation/simulation_command_adapter.py':
        'ea1335d72afb9d06dbc301cae7c812536e063a016f9417438e218fff3e7fdb65',
    'src/thesis_ui/thesis_ui/joint_control_gui.py':
        '6163ce7520c7a98d80c0b3ffdd080bdc7a1f6c0d0bc3ac5c80f33adaabe0ae09',
    'src/thesis_ui/package.xml':
        '6971be43f3cd6b517d9e9acd756255acfbb43b6327a2ff34b0394fa2ddd42a28',
}


JOINT_MODEL = '''"""Canonical JACO2 joint model and operational motion limits."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final, Mapping, Sequence, Tuple, cast


JointVector = Tuple[float, float, float, float, float, float]

JOINT_NAMES: Final[Tuple[str, ...]] = tuple(
    f'j2n6s300_joint_{index}' for index in range(1, 7)
)
JOINT_COUNT: Final[int] = len(JOINT_NAMES)

# Application-space position bounds.  The URDF retains the manufacturer's
# physical model; these values are the canonical limits used by thesis nodes.
JOINT_POSITION_LIMITS: Final[Mapping[str, Tuple[float, float]]] = {
    JOINT_NAMES[0]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[1]: (math.radians(47.0), math.radians(313.0)),
    JOINT_NAMES[2]: (math.radians(19.0), math.radians(341.0)),
    JOINT_NAMES[3]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[4]: (-2.0 * math.pi, 2.0 * math.pi),
    JOINT_NAMES[5]: (-2.0 * math.pi, 2.0 * math.pi),
}

CONTINUOUS_JOINT_INDEXES: Final[Tuple[int, ...]] = (0, 3, 4, 5)
CONTINUOUS_JOINTS: Final[frozenset[str]] = frozenset(
    JOINT_NAMES[index] for index in CONTINUOUS_JOINT_INDEXES
)

# Deliberately conservative operational limits for this thesis.  The JACO2
# URDF keeps the nominal physical limits (36/48 deg/s), while every command
# path in the application is bounded here to 18/24 deg/s.
JOINT_VELOCITY_LIMITS: Final[Mapping[str, float]] = {
    JOINT_NAMES[0]: math.radians(18.0),
    JOINT_NAMES[1]: math.radians(18.0),
    JOINT_NAMES[2]: math.radians(18.0),
    JOINT_NAMES[3]: math.radians(24.0),
    JOINT_NAMES[4]: math.radians(24.0),
    JOINT_NAMES[5]: math.radians(24.0),
}


@dataclass(frozen=True)
class SaturationResult:
    """Result of applying operational velocity and position bounds."""

    positions: JointVector
    requested_velocities: JointVector
    applied_velocities: JointVector
    limiting_joint: str
    requested_velocity: float
    limited_velocity: float
    was_limited: bool


def validate_joint_names(names: Sequence[str]) -> Tuple[str, ...]:
    """Return canonical names or raise for any order/size mismatch."""
    result = tuple(names)
    if result != JOINT_NAMES:
        raise ValueError('expected six arm joints in canonical order')
    return result


def validate_joint_positions(
    values: Sequence[float],
    name: str = 'positions',
) -> JointVector:
    """Return one finite six-joint vector."""
    if len(values) != JOINT_COUNT:
        raise ValueError(f'{name} must contain six joint positions')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} contains a non-finite position')
    return cast(JointVector, result)


def validate_duration(duration: float) -> float:
    """Return a finite positive duration."""
    result = float(duration)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError('duration must be positive and finite')
    return result


def validate_position_limits(values: Sequence[float]) -> JointVector:
    """Return positions or raise for the first application-limit violation."""
    result = validate_joint_positions(values)
    for joint_name, position in zip(JOINT_NAMES, result):
        lower, upper = JOINT_POSITION_LIMITS[joint_name]
        if not lower <= position <= upper:
            raise ValueError(
                f'{joint_name}={position:.6f} rad outside '
                f'[{lower:.6f}, {upper:.6f}] rad'
            )
    return result


def shortest_joint_delta(
    joint_name: str,
    target_position: float,
    current_position: float,
) -> float:
    """Return target-current with the shortest continuous-joint turn."""
    if joint_name not in JOINT_POSITION_LIMITS:
        raise ValueError(f'unknown joint: {joint_name}')
    if (
        not math.isfinite(target_position)
        or not math.isfinite(current_position)
    ):
        raise ValueError('joint positions must be finite')
    delta = float(target_position) - float(current_position)
    if joint_name in CONTINUOUS_JOINTS:
        return math.atan2(math.sin(delta), math.cos(delta))
    return delta


def normalize_target(
    start: Sequence[float],
    target: Sequence[float],
) -> JointVector:
    """Represent a target using the shortest continuous-joint turns."""
    start_vector = validate_joint_positions(start, 'start')
    target_vector = validate_joint_positions(target, 'target')
    return cast(JointVector, tuple(
        current + shortest_joint_delta(joint_name, requested, current)
        for joint_name, current, requested in zip(
            JOINT_NAMES, start_vector, target_vector
        )
    ))


def saturate_target_by_velocity(
    current: Sequence[float],
    requested: Sequence[float],
    duration: float,
) -> SaturationResult:
    """Bound a target by canonical velocity and position limits."""
    current_vector = validate_joint_positions(current, 'current')
    requested_vector = validate_joint_positions(requested, 'requested')
    duration = validate_duration(duration)

    bounded = []
    requested_velocities = []
    applied_velocities = []
    ratios = []
    was_limited = False

    for joint_name, current_value, requested_value in zip(
        JOINT_NAMES, current_vector, requested_vector
    ):
        delta = shortest_joint_delta(
            joint_name, requested_value, current_value
        )
        requested_velocity = delta / duration
        velocity_limit = JOINT_VELOCITY_LIMITS[joint_name]
        applied_velocity = min(
            max(requested_velocity, -velocity_limit), velocity_limit
        )
        bounded_value = current_value + applied_velocity * duration
        lower, upper = JOINT_POSITION_LIMITS[joint_name]
        bounded_value = min(max(bounded_value, lower), upper)
        applied_velocity = (bounded_value - current_value) / duration

        limited = not math.isclose(
            applied_velocity, requested_velocity, rel_tol=0.0, abs_tol=1.0e-12
        )
        was_limited = was_limited or limited
        bounded.append(bounded_value)
        requested_velocities.append(requested_velocity)
        applied_velocities.append(applied_velocity)
        ratios.append(abs(requested_velocity) / velocity_limit)

    limiting_index = max(range(JOINT_COUNT), key=ratios.__getitem__)
    return SaturationResult(
        positions=cast(JointVector, tuple(bounded)),
        requested_velocities=cast(
            JointVector, tuple(requested_velocities)
        ),
        applied_velocities=cast(JointVector, tuple(applied_velocities)),
        limiting_joint=JOINT_NAMES[limiting_index],
        requested_velocity=requested_velocities[limiting_index],
        limited_velocity=applied_velocities[limiting_index],
        was_limited=was_limited,
    )
'''


TEST_JOINT_MODEL = '''"""Regression tests for the canonical JACO2 joint model."""

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
'''


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{label}: expected one match, found {count}')
    return text.replace(old, new, 1)


def transform_safety(text: str) -> str:
    old_import = '''from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    minimum_sphere_clearance,
)

EXPECTED_ARM_JOINTS = [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
]

JOINT_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[1]: (
        47.0 * math.pi / 180.0,
        313.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[2]: (
        19.0 * math.pi / 180.0,
        341.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[3]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[4]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[5]: (-2.0 * math.pi, 2.0 * math.pi),
}

JOINT_VELOCITY_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[1]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[2]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[3]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[4]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[5]: 48.0 * math.pi / 180.0,
}

CONTINUOUS_JOINTS = {
    EXPECTED_ARM_JOINTS[0],
    EXPECTED_ARM_JOINTS[3],
    EXPECTED_ARM_JOINTS[4],
    EXPECTED_ARM_JOINTS[5],
}
'''
    new_import = '''from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    minimum_sphere_clearance,
)
from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    saturate_target_by_velocity,
    shortest_joint_delta,
)
'''
    text = replace_once(text, old_import, new_import, 'safety imports')
    text = text.replace('EXPECTED_ARM_JOINTS', 'JOINT_NAMES')
    text = text.replace('JOINT_LIMITS', 'JOINT_POSITION_LIMITS')
    text = replace_once(
        text,
        'if list(msg.joint_names) != JOINT_NAMES:',
        'if tuple(msg.joint_names) != JOINT_NAMES:',
        'candidate canonical joint order',
    )
    velocity_parameter = '''        self.declare_parameter(
            'velocity_scale',
            0.5,
        )

'''
    text = replace_once(
        text, velocity_parameter, '', 'velocity_scale declaration'
    )
    old_jog_bounds = '''        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_supervised_jog(msg, current, horizon)
            return

        bounded_target = []
        for name, current_value, requested_value in zip(
            JOINT_NAMES,
            current,
            msg.positions,
        ):
            delta = self.shortest_joint_delta(
                name,
                float(requested_value),
                current_value,
            )
            maximum_delta = JOINT_VELOCITY_LIMITS[name] * horizon
            delta = min(max(delta, -maximum_delta), maximum_delta)
            bounded_target.append(current_value + delta)
        bounded_target = tuple(bounded_target)
'''
    new_jog_bounds = '''        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                horizon,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f'JOG REJECTED: {exc}'
            )
            return
        bounded_target = saturation.positions

        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_supervised_jog(msg, bounded_target, horizon)
            return
'''
    text = replace_once(
        text, old_jog_bounds, new_jog_bounds, 'jog saturation'
    )
    old_reason = '''            f'd_seguro={clearance:.3f}m; margen={margin:.3f}m; '
            f'escala={speed_scale:.3f}'
'''
    new_reason = '''            f'd_seguro={clearance:.3f}m; margen={margin:.3f}m; '
            f'escala={speed_scale:.3f}; '
            f'joint_lim={saturation.limiting_joint}; '
            f'v_req={saturation.requested_velocity:.4f}rad/s; '
            f'v_lim={saturation.limited_velocity:.4f}rad/s'
'''
    text = replace_once(text, old_reason, new_reason, 'jog diagnostics')
    old_shortest = '''    def shortest_joint_delta(
        self,
        joint_name,
        target_position,
        current_position,
    ):
        delta = target_position - current_position

        if joint_name in CONTINUOUS_JOINTS:
            return math.atan2(
                math.sin(delta),
                math.cos(delta),
            )

        return delta
'''
    new_shortest = '''    def shortest_joint_delta(
        self,
        joint_name,
        target_position,
        current_position,
    ):
        """Compatibility wrapper around the canonical joint model."""
        return shortest_joint_delta(
            joint_name,
            target_position,
            current_position,
        )
'''
    text = replace_once(
        text, old_shortest, new_shortest, 'shortest delta wrapper'
    )
    old_scale = '''        velocity_scale = float(
            self.get_parameter(
                'velocity_scale'
            ).value
        )

'''
    text = replace_once(text, old_scale, '', 'velocity_scale read')
    old_allowed = '''            allowed_velocity = (
                JOINT_VELOCITY_LIMITS[joint_name]
                * velocity_scale
            )
'''
    new_allowed = '''            allowed_velocity = JOINT_VELOCITY_LIMITS[joint_name]
'''
    text = replace_once(
        text, old_allowed, new_allowed, 'velocity validation limit'
    )
    text = replace_once(
        text,
        'if requested_velocity > allowed_velocity:',
        'if requested_velocity > allowed_velocity + 1.0e-9:',
        'velocity comparison tolerance',
    )
    insertion_anchor = '''    def state_for_clearance(self, clearance):
'''
    bounded_method = '''    def bound_candidate_velocity(
        self,
        msg,
        duration_sec,
    ):
        """Return a velocity-bounded copy before predictive evaluation."""
        if not all(name in self.current_positions for name in JOINT_NAMES):
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: current joint state unavailable',
            )
            return None
        current = tuple(
            self.current_positions[name] for name in JOINT_NAMES
        )
        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                duration_sec,
            )
        except ValueError as exc:
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: {exc}',
            )
            return None

        bounded = self.copy_command_with_duration(msg, duration_sec)
        bounded.positions = list(saturation.positions)
        if saturation.was_limited:
            self.get_logger().warning(
                f'VELOCITY SATURATED {msg.command_id}: '
                f'joint={saturation.limiting_joint}, '
                f'requested={saturation.requested_velocity:.4f}rad/s, '
                f'limited={saturation.limited_velocity:.4f}rad/s'
            )
        return bounded, saturation

'''
    text = replace_once(
        text,
        insertion_anchor,
        bounded_method + insertion_anchor,
        'candidate bound method insertion',
    )
    old_policy = '''        supervised_msg = self.apply_proximity_policy(
            msg,
            duration_sec,
        )
'''
    new_policy = '''        bounded_result = self.bound_candidate_velocity(
            msg,
            duration_sec,
        )
        if bounded_result is None:
            return
        bounded_msg, saturation = bounded_result

        supervised_msg = self.apply_proximity_policy(
            bounded_msg,
            duration_sec,
        )
'''
    text = replace_once(
        text, old_policy, new_policy, 'candidate saturation order'
    )
    old_log_tail = '''            f'duration={supervised_duration_sec:.2f} s | '
            f'input_latency={input_latency_ms:.3f} ms'
'''
    new_log_tail = '''            f'duration={supervised_duration_sec:.2f} s | '
            f'joint_lim={saturation.limiting_joint} | '
            f'v_req={saturation.requested_velocity:.4f} rad/s | '
            f'v_lim={saturation.limited_velocity:.4f} rad/s | '
            f'input_latency={input_latency_ms:.3f} ms'
'''
    text = replace_once(
        text, old_log_tail, new_log_tail, 'candidate diagnostics'
    )
    return text


def transform_execution_reference(text: str) -> str:
    old = '''import math


JOINT_COUNT = 6
CONTINUOUS_JOINT_INDEXES = (0, 3, 4, 5)


def _validate_vector(values, name):
    if len(values) != JOINT_COUNT:
        raise ValueError(f'{name} must contain six joint positions')
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{name} contains a non-finite position')


def normalize_target(start, target):
    """Return the target using the shortest continuous-joint turn."""
    _validate_vector(start, 'start')
    _validate_vector(target, 'target')

    normalized = list(target)
    for index in CONTINUOUS_JOINT_INDEXES:
        delta = math.atan2(
            math.sin(target[index] - start[index]),
            math.cos(target[index] - start[index]),
        )
        normalized[index] = start[index] + delta
    return tuple(normalized)
'''
    new = '''import math

from thesis_core.joint_model import normalize_target
from thesis_core.joint_model import validate_joint_positions


def _validate_vector(values, name):
    validate_joint_positions(values, name)
'''
    return replace_once(text, old, new, 'execution reference model')


def transform_horizon(text: str) -> str:
    old_import = '''from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments
'''
    new_import = '''from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments
from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    normalize_target,
)
'''
    text = replace_once(text, old_import, new_import, 'horizon imports')
    text = replace_once(
        text,
        "JOINTS = tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))\n",
        '',
        'horizon local joints',
    )
    text = text.replace('JOINTS', 'JOINT_NAMES')
    old_motion = '''    delta = [b - a for a, b in zip(current, target)]
    for index in (0, 3, 4, 5):
        delta[index] = math.atan2(
            math.sin(delta[index]),
            math.cos(delta[index]),
        )
    limits = [math.radians(value) for value in (18, 18, 18, 24, 24, 24)]
    effective_duration = max(
        duration,
        *(abs(delta_value) / limit
          for delta_value, limit in zip(delta, limits)),
    )
'''
    new_motion = '''    normalized_target = normalize_target(current, target)
    delta = [
        target_value - current_value
        for current_value, target_value in zip(current, normalized_target)
    ]
    effective_duration = max(
        duration,
        *(
            abs(delta_value) / JOINT_VELOCITY_LIMITS[joint_name]
            for joint_name, delta_value in zip(JOINT_NAMES, delta)
        ),
    )
'''
    return replace_once(text, old_motion, new_motion, 'horizon limits')


def transform_adapter(text: str) -> str:
    text = replace_once(
        text,
        'from thesis_core.execution_reference import normalize_target\n\n\n'
        "JOINTS = tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))\n",
        'from thesis_core.joint_model import (\n'
        '    JOINT_NAMES,\n'
        '    normalize_target,\n'
        '    saturate_target_by_velocity,\n'
        ')\n',
        'adapter joint model import',
    )
    text = text.replace('JOINTS', 'JOINT_NAMES')
    old_jog = '''        try:
            horizon_target = normalize_target(
                current,
                tuple(msg.positions),
            )
        except ValueError:
            return

        fraction = min(1.0, control_period / horizon)
'''
    new_jog = '''        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                horizon,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f'JOG ADAPTER REJECTED: {exc}'
            )
            return
        if saturation.was_limited:
            self.get_logger().error(
                'JOG ADAPTER REJECTED: supervisor velocity invariant '
                f'violated by {saturation.limiting_joint}; '
                f'requested={saturation.requested_velocity:.4f}rad/s, '
                f'limit={saturation.limited_velocity:.4f}rad/s'
            )
            return
        horizon_target = saturation.positions

        fraction = min(1.0, control_period / horizon)
'''
    text = replace_once(text, old_jog, new_jog, 'adapter jog invariant')
    old_command = '''        try:
            target = normalize_target(start, tuple(msg.positions))
        except ValueError as exc:
            self.reject(msg, 'REJECTED', str(exc))
            return

        metadata = self.initial_metadata(msg, start, target, duration)
'''
    new_command = '''        try:
            saturation = saturate_target_by_velocity(
                start,
                msg.positions,
                duration,
            )
        except ValueError as exc:
            self.reject(msg, 'REJECTED', str(exc))
            return
        if saturation.was_limited:
            self.reject(
                msg,
                'REJECTED',
                'invariante de velocidad del supervisor incumplida: '
                f'{saturation.limiting_joint}, '
                f'solicitada={saturation.requested_velocity:.4f}rad/s, '
                f'límite={saturation.limited_velocity:.4f}rad/s',
            )
            return
        target = saturation.positions

        metadata = self.initial_metadata(msg, start, target, duration)
'''
    return replace_once(
        text,
        old_command,
        new_command,
        'adapter candidate invariant',
    )


def transform_ui(text: str) -> str:
    old_block = '''import tf2_ros


JOINT_NAMES = [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
]

JOINT_LABELS = [
'''
    new_block = '''import tf2_ros

from thesis_core.joint_model import (
    CONTINUOUS_JOINT_INDEXES,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    saturate_target_by_velocity,
)


JOINT_LABELS = [
'''
    text = replace_once(text, old_block, new_block, 'ui joint names import')
    old_limits = '''JOINT_LIMITS_DEG = [
    (-360.0, 360.0),
    (47.0, 313.0),
    (19.0, 341.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
]

INITIAL_POSE_DEG = [0.0, 180.0, 180.0, 0.0, 0.0, 0.0]
TEST_POSE_DEG = [math.degrees(0.20), 180.0, 180.0, 0.0, 0.0, 0.0]

# These values match the current supervisor configuration:
# nominal J1-J3 = 36 deg/s, nominal J4-J6 = 48 deg/s,
# with velocity_scale = 0.5.
ALLOWED_SPEED_DEG = [18.0, 18.0, 18.0, 24.0, 24.0, 24.0]
MAX_JOG_SPEED_DEG = [36.0, 36.0, 36.0, 48.0, 48.0, 48.0]
CONTINUOUS_JOINT_INDEXES = {0, 3, 4, 5}
'''
    new_limits = '''JOINT_LIMITS_DEG = tuple(
    tuple(math.degrees(value) for value in JOINT_POSITION_LIMITS[name])
    for name in JOINT_NAMES
)

INITIAL_POSE_DEG = [0.0, 180.0, 180.0, 0.0, 0.0, 0.0]
TEST_POSE_DEG = [math.degrees(0.20), 180.0, 180.0, 0.0, 0.0, 0.0]

ALLOWED_SPEED_DEG = tuple(
    math.degrees(JOINT_VELOCITY_LIMITS[name]) for name in JOINT_NAMES
)
'''
    text = replace_once(text, old_limits, new_limits, 'ui limits')
    text = replace_once(
        text,
        'speed_input.setRange(1.0, MAX_JOG_SPEED_DEG[index])',
        'speed_input.setRange(1.0, ALLOWED_SPEED_DEG[index])',
        'ui jog maximum',
    )
    text = replace_once(
        text,
        "'Estimación informativa con velocity_scale = 0.5. '\n"
        "            'El safety_supervisor conserva la decisión final.'",
        "'Estimación con límites operativos comunes de 18/24 °/s. '\n"
        "            'El safety_supervisor conserva la decisión final.'",
        'ui preview note',
    )
    old_candidate = '''        msg.joint_names = list(JOINT_NAMES)
        msg.positions = list(positions)

        seconds = int(duration_sec)
'''
    new_candidate = '''        msg.joint_names = list(JOINT_NAMES)
        current = tuple(
            self.current_positions[name] for name in JOINT_NAMES
        )
        saturation = saturate_target_by_velocity(
            current,
            positions,
            duration_sec,
        )
        msg.positions = list(saturation.positions)

        seconds = int(duration_sec)
'''
    text = replace_once(
        text,
        old_candidate,
        new_candidate,
        'ui candidate first saturation',
    )
    return text


def transform_ui_package(text: str) -> str:
    return replace_once(
        text,
        '  <exec_depend>sensor_msgs</exec_depend>\n'
        '  <exec_depend>thesis_interfaces</exec_depend>\n',
        '  <exec_depend>sensor_msgs</exec_depend>\n'
        '  <exec_depend>thesis_core</exec_depend>\n'
        '  <exec_depend>thesis_interfaces</exec_depend>\n',
        'ui thesis_core dependency',
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--check', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    if not (root / 'src').is_dir():
        print(f'ERROR: no src directory under {root}', file=sys.stderr)
        return 2

    new_paths = [
        root / 'src/thesis_core/thesis_core/joint_model.py',
        root / 'src/thesis_core/test/test_joint_model.py',
    ]
    if any(path.exists() for path in new_paths):
        print('ERROR: a Package 1 new file already exists', file=sys.stderr)
        return 3

    for relative, expected in EXPECTED_HASHES.items():
        path = root / relative
        if not path.is_file():
            print(f'ERROR: missing {relative}', file=sys.stderr)
            return 4
        actual = sha256(path)
        if actual != expected:
            print(
                f'ERROR: hash mismatch for {relative}\n'
                f'expected={expected}\nactual={actual}',
                file=sys.stderr,
            )
            return 5

    transforms = {
        'src/thesis_core/thesis_core/safety_supervisor_node.py':
            transform_safety,
        'src/thesis_core/thesis_core/execution_reference.py':
            transform_execution_reference,
        'src/thesis_core/thesis_core/horizon_preview.py': transform_horizon,
        'src/thesis_simulation/thesis_simulation/simulation_command_adapter.py':
            transform_adapter,
        'src/thesis_ui/thesis_ui/joint_control_gui.py': transform_ui,
        'src/thesis_ui/package.xml': transform_ui_package,
    }
    changed: Dict[str, str] = {}
    try:
        for relative, transform in transforms.items():
            original = (root / relative).read_text(encoding='utf-8')
            result = transform(original)
            if result == original:
                raise RuntimeError(f'{relative}: transform made no change')
            changed[relative] = result
    except RuntimeError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 6

    if args.check:
        print('CHECK_OK=1')
        for relative in changed:
            print(f'WOULD_MODIFY={relative}')
        for path in new_paths:
            print(f'WOULD_CREATE={path.relative_to(root)}')
        return 0

    stamp = datetime.now().strftime('%Y%m%dT%H%M%S')
    backup = root.parent / f'RESPALDO_TESIS_PAQUETE1_{stamp}'
    for relative in EXPECTED_HASHES:
        source = root / relative
        destination = backup / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

    for path in new_paths:
        path.parent.mkdir(parents=True, exist_ok=True)

    for relative, result in changed.items():
        (root / relative).write_text(result, encoding='utf-8')
    new_paths[0].write_text(JOINT_MODEL, encoding='utf-8')
    new_paths[1].write_text(TEST_JOINT_MODEL, encoding='utf-8')

    print('APLICACION_OK=1')
    print(f'RESPALDO={backup}')
    for relative in changed:
        print(f'MODIFICADO={relative}')
    for path in new_paths:
        print(f'CREADO={path.relative_to(root)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
