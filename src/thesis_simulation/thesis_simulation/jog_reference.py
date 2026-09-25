"""Deterministic generation of short rolling JOG references."""

from __future__ import annotations

import math
from typing import Sequence, Tuple


JointVector = Tuple[float, ...]


def build_jog_reference(
    current: Sequence[float],
    horizon_target: Sequence[float],
    horizon_sec: float,
    control_period_sec: float,
) -> Tuple[JointVector, JointVector]:
    """
    Return the short target and its constant horizon velocity.

    The target preserves the package equation
    q_control = q_current + (control_period / horizon) * delta_q.
    The velocity is explicitly included in JointTrajectoryPoint so that a
    spline controller does not restart a zero-slope segment every cycle.
    """
    current_values = tuple(float(value) for value in current)
    target_values = tuple(float(value) for value in horizon_target)
    horizon = float(horizon_sec)
    control_period = float(control_period_sec)

    if not current_values or len(current_values) != len(target_values):
        raise ValueError('current and horizon_target must have equal size')
    if not all(math.isfinite(value) for value in current_values):
        raise ValueError('current must contain only finite values')
    if not all(math.isfinite(value) for value in target_values):
        raise ValueError('horizon_target must contain only finite values')
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError('horizon_sec must be finite and positive')
    if not math.isfinite(control_period) or control_period <= 0.0:
        raise ValueError('control_period_sec must be finite and positive')

    fraction = min(1.0, control_period / horizon)
    delta = tuple(
        target - actual
        for actual, target in zip(current_values, target_values)
    )
    positions = tuple(
        actual + fraction * displacement
        for actual, displacement in zip(current_values, delta)
    )
    velocities = tuple(displacement / horizon for displacement in delta)
    return positions, velocities


def advance_jog_reference(
    current: Sequence[float],
    horizon_target: Sequence[float],
    horizon_sec: float,
    control_period_sec: float,
    previous_target: Sequence[float] | None,
    previous_velocity: Sequence[float] | None,
    maximum_lead_sec: float,
) -> Tuple[JointVector, JointVector]:
    """
    Advance a continuous rolling reference without inheriting tracking lag.

    The first reference is anchored to the measured state. Later references
    advance from the previously commanded target while the direction remains
    compatible. A reversal resets the anchor immediately to the measurement.
    The commanded lead is bounded by velocity times maximum_lead_sec.
    """
    measured_target, velocity = build_jog_reference(
        current,
        horizon_target,
        horizon_sec,
        control_period_sec,
    )
    current_values = tuple(float(value) for value in current)
    target_values = tuple(float(value) for value in horizon_target)
    period = float(control_period_sec)
    maximum_lead = float(maximum_lead_sec)
    if not math.isfinite(maximum_lead) or maximum_lead < period:
        raise ValueError(
            'maximum_lead_sec must be finite and at least one control period'
        )

    use_previous = previous_target is not None and previous_velocity is not None
    if use_previous:
        previous_positions = tuple(float(value) for value in previous_target)
        previous_velocities = tuple(
            float(value) for value in previous_velocity
        )
        if (
            len(previous_positions) != len(current_values)
            or len(previous_velocities) != len(current_values)
            or not all(math.isfinite(value) for value in previous_positions)
            or not all(math.isfinite(value) for value in previous_velocities)
        ):
            raise ValueError('previous JOG reference is invalid')
        direction_reversed = any(
            old * new < -1.0e-12
            for old, new in zip(previous_velocities, velocity)
        )
        use_previous = not direction_reversed

    if not use_previous:
        return measured_target, velocity

    rolling_positions = []
    for actual, goal, old_target, joint_velocity in zip(
        current_values,
        target_values,
        previous_positions,
        velocity,
    ):
        if abs(joint_velocity) <= 1.0e-12:
            rolling_positions.append(actual)
            continue
        candidate = old_target + joint_velocity * period
        lead = abs(joint_velocity) * maximum_lead
        candidate = min(max(candidate, actual - lead), actual + lead)
        if joint_velocity > 0.0:
            candidate = min(candidate, goal)
        else:
            candidate = max(candidate, goal)
        rolling_positions.append(candidate)
    return tuple(rolling_positions), velocity
