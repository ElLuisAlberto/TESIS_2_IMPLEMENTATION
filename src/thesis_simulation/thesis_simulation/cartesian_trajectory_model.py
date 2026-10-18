"""Pure timing helpers for rolling Cartesian trajectory execution."""

import math

from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    shortest_joint_delta,
)


def minimum_joint_path_duration(path):
    """Return the duration required by every uniformly spaced path segment."""
    configurations = tuple(tuple(float(value) for value in item) for item in path)
    if len(configurations) < 2:
        raise ValueError('joint path requires at least two configurations')
    if any(len(item) != len(JOINT_NAMES) for item in configurations):
        raise ValueError('every joint path configuration must contain six values')
    if not all(
        math.isfinite(value)
        for configuration in configurations
        for value in configuration
    ):
        raise ValueError('joint path contains a non-finite value')

    segment_count = len(configurations) - 1
    required = 0.0
    for start, target in zip(configurations, configurations[1:]):
        for name, current, requested in zip(JOINT_NAMES, start, target):
            segment_duration = abs(
                shortest_joint_delta(name, requested, current)
            ) / JOINT_VELOCITY_LIMITS[name]
            required = max(required, segment_duration * segment_count)
    return required


def advance_progress(progress, speed_scale, elapsed_sec, duration_sec):
    """Integrate supervised progress without allowing reversal or overshoot."""
    values = tuple(
        float(value)
        for value in (progress, speed_scale, elapsed_sec, duration_sec)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('trajectory timing values must be finite')
    progress, speed_scale, elapsed_sec, duration_sec = values
    if not 0.0 <= progress <= 1.0:
        raise ValueError('progress must belong to [0, 1]')
    if not 0.0 <= speed_scale <= 1.0:
        raise ValueError('speed_scale must belong to [0, 1]')
    if elapsed_sec < 0.0 or duration_sec <= 0.0:
        raise ValueError('elapsed time and duration are invalid')
    return min(1.0, progress + speed_scale * elapsed_sec / duration_sec)


def rolling_target_fraction(progress, duration_sec, horizon_sec):
    """Return the nominal path fraction at the end of a short horizon."""
    values = tuple(float(value) for value in (progress, duration_sec, horizon_sec))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('rolling target values must be finite')
    progress, duration_sec, horizon_sec = values
    if not 0.0 <= progress <= 1.0:
        raise ValueError('progress must belong to [0, 1]')
    if duration_sec <= 0.0 or not 0.1 <= horizon_sec <= 2.0:
        raise ValueError('duration or horizon is invalid')
    return min(1.0, progress + horizon_sec / duration_sec)
