"""Deterministic preventive decision policy independent from ROS 2."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence


NO_EVENT_TIME = -1.0


@dataclass(frozen=True)
class ScaleSearchResult:
    """Maximum scale found by a deterministic binary search."""

    scale: float
    clearance: float
    feasible: bool
    iterations: int


@dataclass(frozen=True)
class SafetyPolicyInput:
    """Complete evidence consumed by the preventive policy."""

    current_clearance: float
    nominal_clearance: float
    supervised_clearance: float
    protective_margin: float
    stop_distance: float
    warning_distance: float
    minimum_scale: float
    candidate_scale: float
    nominal_ttc: float = NO_EVENT_TIME
    previous_scale: float = 1.0
    joint_state_valid: bool = True
    proximity_valid: bool = True
    prediction_valid: bool = True
    withdrawal_safe: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    """Deterministic state, safe scale and machine-readable reason."""

    state: str
    speed_scale: float
    reason_code: str


def _finite(name: str, value: float) -> float:
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f'{name} must be finite')
    return numeric


def _valid_event_time(value: float) -> bool:
    return value == NO_EVENT_TIME or (
        math.isfinite(value) and value >= 0.0
    )


def maximum_safe_scale(
    clearance_at_scale: Callable[[float], float],
    protective_margin: float,
    minimum_scale: float = 0.05,
    iterations: int = 10,
) -> ScaleSearchResult:
    """Return the largest sampled scale whose clearance meets the margin."""
    margin = _finite('protective_margin', protective_margin)
    minimum = _finite('minimum_scale', minimum_scale)
    if margin < 0.0:
        raise ValueError('protective_margin must be non-negative')
    if not 0.0 < minimum <= 1.0:
        raise ValueError('minimum_scale must be in (0, 1]')
    if iterations <= 0:
        raise ValueError('iterations must be positive')

    zero_clearance = _finite(
        'clearance_at_scale(0)', clearance_at_scale(0.0)
    )
    if zero_clearance < margin:
        return ScaleSearchResult(0.0, zero_clearance, False, iterations)

    full_clearance = _finite(
        'clearance_at_scale(1)', clearance_at_scale(1.0)
    )
    if full_clearance >= margin:
        return ScaleSearchResult(1.0, full_clearance, True, iterations)

    low = 0.0
    high = 1.0
    low_clearance = zero_clearance
    for _ in range(iterations):
        candidate = 0.5 * (low + high)
        clearance = _finite(
            'clearance_at_scale(candidate)',
            clearance_at_scale(candidate),
        )
        if clearance >= margin:
            low = candidate
            low_clearance = clearance
        else:
            high = candidate

    feasible = low >= minimum
    return ScaleSearchResult(low, low_clearance, feasible, iterations)


def decide_preventive_state(data: SafetyPolicyInput) -> SafetyDecision:
    """Apply the ordered fail-safe ALLOW/WARNING/REDUCTION/STOP policy."""
    if not data.joint_state_valid:
        return SafetyDecision('STOP', 0.0, 'STATE_UNAVAILABLE_OR_STALE')
    if not data.proximity_valid:
        return SafetyDecision('STOP', 0.0, 'PROXIMITY_UNAVAILABLE_OR_STALE')
    if not data.prediction_valid:
        return SafetyDecision('STOP', 0.0, 'PREDICTION_ERROR')

    try:
        current = _finite('current_clearance', data.current_clearance)
        nominal = _finite('nominal_clearance', data.nominal_clearance)
        supervised = _finite(
            'supervised_clearance', data.supervised_clearance
        )
        margin = _finite('protective_margin', data.protective_margin)
        stop = _finite('stop_distance', data.stop_distance)
        warning = _finite('warning_distance', data.warning_distance)
        minimum = _finite('minimum_scale', data.minimum_scale)
        candidate = _finite('candidate_scale', data.candidate_scale)
        previous = _finite('previous_scale', data.previous_scale)
    except (TypeError, ValueError):
        return SafetyDecision('STOP', 0.0, 'NUMERIC_ERROR')

    if not _valid_event_time(float(data.nominal_ttc)):
        return SafetyDecision('STOP', 0.0, 'NUMERIC_ERROR')
    if not 0.0 <= margin < stop < warning:
        return SafetyDecision('STOP', 0.0, 'INVALID_THRESHOLDS')
    if not 0.0 < minimum <= 1.0:
        return SafetyDecision('STOP', 0.0, 'INVALID_THRESHOLDS')
    if not 0.0 <= candidate <= 1.0 or not 0.0 <= previous <= 1.0:
        return SafetyDecision('STOP', 0.0, 'NUMERIC_ERROR')

    if current <= stop:
        if data.withdrawal_safe and candidate >= minimum:
            return SafetyDecision(
                'REDUCTION', candidate, 'PROTECTIVE_WITHDRAWAL'
            )
        return SafetyDecision('STOP', 0.0, 'CURRENT_CLEARANCE_STOP')

    if nominal < margin:
        if candidate < minimum:
            return SafetyDecision('STOP', 0.0, 'NO_SAFE_SCALE')
        if supervised < margin:
            return SafetyDecision('STOP', 0.0, 'PREDICTION_ERROR')
        return SafetyDecision(
            'REDUCTION', candidate, 'PREDICTIVE_INTRUSION'
        )

    if current <= warning:
        return SafetyDecision('WARNING', 1.0, 'CURRENT_WARNING')

    return SafetyDecision('ALLOW', 1.0, 'CLEAR')


def withdrawal_is_non_approaching(
    current_clearance: float,
    nominal_clearance: float,
    tolerance: float = 1.0e-9,
) -> bool:
    """Return true only if a candidate does not reduce current clearance."""
    current = _finite('current_clearance', current_clearance)
    nominal = _finite('nominal_clearance', nominal_clearance)
    epsilon = _finite('tolerance', tolerance)
    if epsilon < 0.0:
        raise ValueError('tolerance must be non-negative')
    return nominal >= current - epsilon


def withdrawal_path_is_safe(
    clearances: Sequence[float],
    minimum_progress: float = 0.002,
    monotonic_tolerance: float = 0.001,
) -> bool:
    """Accept only a path that never approaches and finishes farther away."""
    values = tuple(float(value) for value in clearances)
    progress = _finite('minimum_progress', minimum_progress)
    tolerance = _finite('monotonic_tolerance', monotonic_tolerance)
    if len(values) < 2 or progress <= 0.0 or tolerance < 0.0:
        return False
    if not all(math.isfinite(value) for value in values):
        return False
    if any(
        following < previous - tolerance
        for previous, following in zip(values, values[1:])
    ):
        return False
    return values[-1] >= values[0] + progress
