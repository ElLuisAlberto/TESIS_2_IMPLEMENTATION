"""Deterministic distance and crossing-time evaluation over a horizon."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from thesis_core.clearance_geometry import (
    Segment3,
    SegmentClearance,
    finite_point,
    minimum_configuration_clearance,
)
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    SEGMENT_NAMES,
    capsule_segments,
)


KINEMATIC_TF_TOLERANCE_M = 0.005
MODEL_UNCERTAINTY_M = 0.010
SAMPLING_UNCERTAINTY_M = 0.005
LATENCY_UNCERTAINTY_M = 0.005
DEFAULT_PROTECTIVE_MARGIN_M = (
    MODEL_UNCERTAINTY_M
    + SAMPLING_UNCERTAINTY_M
    + LATENCY_UNCERTAINTY_M
)
NO_EVENT_TIME = -1.0


@dataclass(frozen=True)
class HorizonClearance:
    """Global minimum, measured state and threshold-crossing times."""

    current: SegmentClearance
    minimum: SegmentClearance
    sample_count: int
    evaluated_combinations: int
    protective_margin: float
    minimum_protective_clearance: float
    first_protective_entry_time: Optional[float]
    first_collision_time: Optional[float]
    sample_clearances: Tuple[float, ...]


def total_protective_margin(
    model_uncertainty: float,
    sampling_uncertainty: float,
    latency_uncertainty: float,
) -> float:
    """Validate and add the three uncertainty contributions."""
    components = tuple(float(value) for value in (
        model_uncertainty,
        sampling_uncertainty,
        latency_uncertainty,
    ))
    if not all(math.isfinite(value) and value >= 0.0 for value in components):
        raise ValueError('uncertainty components must be finite and non-negative')
    total = sum(components)
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError('total protective margin must be finite and positive')
    return total


def event_time_or_invalid(value: Optional[float]) -> float:
    """Represent an absent future event with the documented negative value."""
    return NO_EVENT_TIME if value is None else float(value)


def _sample_times(values: Sequence[float], count: int) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != count or count == 0:
        raise ValueError('sample_times must match the non-empty trajectory')
    if not all(math.isfinite(value) and value >= 0.0 for value in result):
        raise ValueError('sample_times must be finite and non-negative')
    if not math.isclose(result[0], 0.0, abs_tol=1.0e-12):
        raise ValueError('the first sample time must be zero')
    if any(end <= start for start, end in zip(result, result[1:])):
        raise ValueError('sample_times must be strictly increasing')
    return result


def interpolated_threshold_crossing_time(
    sample_times: Sequence[float],
    clearances: Sequence[float],
    threshold: float,
) -> Optional[float]:
    """Return the first downward threshold crossing using linear interpolation."""
    values = tuple(float(value) for value in clearances)
    times = _sample_times(sample_times, len(values))
    limit = float(threshold)
    if not math.isfinite(limit):
        raise ValueError('threshold must be finite')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('clearances must be finite')
    if values[0] <= limit:
        return times[0]
    for index in range(len(values) - 1):
        first = values[index]
        second = values[index + 1]
        if first > limit and second <= limit:
            denominator = first - second
            if denominator <= 0.0 or not math.isfinite(denominator):
                return times[index + 1]
            fraction = (first - limit) / denominator
            fraction = min(max(fraction, 0.0), 1.0)
            return times[index] + fraction * (times[index + 1] - times[index])
    return None


def evaluate_capsule_horizon(
    capsule_samples: Sequence[Sequence[Segment3]],
    sample_times: Sequence[float],
    capsule_radii: Sequence[float],
    segment_names: Sequence[str],
    obstacle_center: Sequence[float],
    obstacle_velocity: Sequence[float],
    obstacle_radius: float,
    protective_margin: float = DEFAULT_PROTECTIVE_MARGIN_M,
) -> HorizonClearance:
    """Evaluate every capsule/sample pair and both safety thresholds."""
    samples = tuple(tuple(sample) for sample in capsule_samples)
    times = _sample_times(sample_times, len(samples))
    center = finite_point(obstacle_center, 'obstacle center')
    velocity = finite_point(obstacle_velocity, 'obstacle velocity')
    names = tuple(segment_names)
    radii = tuple(capsule_radii)
    margin = float(protective_margin)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError('protective margin must be finite and non-negative')
    results = []
    for sample_index, (sample_time, segments) in enumerate(
        zip(times, samples)
    ):
        predicted_center = tuple(
            center[axis] + velocity[axis] * sample_time
            for axis in range(3)
        )
        results.append(minimum_configuration_clearance(
            segments,
            names,
            radii,
            predicted_center,
            obstacle_radius,
            sample_index,
            sample_time,
        ))
    clearances = tuple(result.clearance for result in results)
    minimum = min(results, key=lambda result: result.clearance)
    return HorizonClearance(
        current=results[0],
        minimum=minimum,
        sample_count=len(samples),
        evaluated_combinations=len(samples) * len(radii),
        protective_margin=margin,
        minimum_protective_clearance=minimum.clearance - margin,
        first_protective_entry_time=interpolated_threshold_crossing_time(
            times,
            clearances,
            margin,
        ),
        first_collision_time=interpolated_threshold_crossing_time(
            times,
            clearances,
            0.0,
        ),
        sample_clearances=clearances,
    )


def evaluate_joint_horizon(
    joint_samples: Sequence[Sequence[float]],
    sample_times: Sequence[float],
    obstacle_center: Sequence[float],
    obstacle_velocity: Sequence[float],
    obstacle_radius: float,
    protective_margin: float = DEFAULT_PROTECTIVE_MARGIN_M,
) -> HorizonClearance:
    """Convert joint samples into capsule axes and evaluate the horizon."""
    samples = tuple(tuple(float(value) for value in sample)
                    for sample in joint_samples)
    if any(len(sample) != 6 for sample in samples):
        raise ValueError('every joint sample must contain six positions')
    capsule_samples = tuple(capsule_segments(sample) for sample in samples)
    return evaluate_capsule_horizon(
        capsule_samples,
        sample_times,
        CAPSULE_RADII,
        SEGMENT_NAMES,
        obstacle_center,
        obstacle_velocity,
        obstacle_radius,
        protective_margin,
    )
