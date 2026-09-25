"""Deterministic short-horizon joint-space prediction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple, cast

from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    JointVector,
    normalize_target,
    validate_duration,
    validate_joint_positions,
)


JointSamples = Tuple[JointVector, ...]


@dataclass(frozen=True)
class JointPrediction:
    """Joint samples and their exact temporal discretization."""

    positions: JointSamples
    sample_times: Tuple[float, ...]
    sample_period: float
    horizon: float
    effective_duration: float


def _validate_positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be positive and finite')
    return result


def _validate_sample_count(samples: int) -> int:
    if isinstance(samples, bool) or not isinstance(samples, int):
        raise ValueError('samples must be an integer')
    if samples < 2:
        raise ValueError('samples must be at least two')
    return samples


def _validate_configuration_limits(values: JointVector, name: str) -> None:
    for joint_name, value in zip(JOINT_NAMES, values):
        lower, upper = JOINT_POSITION_LIMITS[joint_name]
        if not lower <= value <= upper:
            raise ValueError(
                f'{name}: {joint_name}={value:.6f} rad outside '
                f'[{lower:.6f}, {upper:.6f}] rad'
            )


def predict_joint_samples(
    current: Sequence[float],
    target: Sequence[float],
    duration: float,
    horizon: float,
    samples: int,
) -> JointPrediction:
    """Predict a velocity-bounded piecewise-linear joint trajectory."""
    current_vector = validate_joint_positions(current, 'current')
    target_vector = validate_joint_positions(target, 'target')
    duration = validate_duration(duration)
    horizon = _validate_positive_finite(horizon, 'horizon')
    sample_count = _validate_sample_count(samples)
    _validate_configuration_limits(current_vector, 'current')
    _validate_configuration_limits(target_vector, 'target')

    normalized_target = normalize_target(current_vector, target_vector)
    _validate_configuration_limits(normalized_target, 'normalized target')
    deltas = tuple(
        target_value - current_value
        for current_value, target_value in zip(
            current_vector, normalized_target
        )
    )
    effective_duration = max(
        duration,
        *(
            abs(delta) / JOINT_VELOCITY_LIMITS[joint_name]
            for joint_name, delta in zip(JOINT_NAMES, deltas)
        ),
    )
    sample_period = horizon / float(sample_count - 1)
    sample_times = tuple(
        index * sample_period for index in range(sample_count)
    )
    predicted = []
    for sample_time in sample_times:
        alpha = min(sample_time / effective_duration, 1.0)
        configuration = cast(JointVector, tuple(
            current_value + alpha * delta
            for current_value, delta in zip(current_vector, deltas)
        ))
        _validate_configuration_limits(configuration, 'predicted sample')
        predicted.append(configuration)

    return JointPrediction(
        positions=tuple(predicted),
        sample_times=sample_times,
        sample_period=sample_period,
        horizon=horizon,
        effective_duration=effective_duration,
    )


def is_state_fresh(
    received_monotonic: float,
    now_monotonic: float,
    max_state_age_sec: float,
) -> bool:
    """Return whether a received state is valid at the evaluation instant."""
    values = (received_monotonic, now_monotonic, max_state_age_sec)
    if not all(math.isfinite(value) for value in values):
        return False
    age = now_monotonic - received_monotonic
    return max_state_age_sec > 0.0 and 0.0 <= age <= max_state_age_sec
