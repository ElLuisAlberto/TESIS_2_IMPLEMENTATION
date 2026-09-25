"""Canonical JACO2 joint model and operational motion limits."""

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
