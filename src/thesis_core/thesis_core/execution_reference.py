"""Helpers for sampling an accepted joint trajectory reference."""

import math

from thesis_core.joint_model import normalize_target
from thesis_core.joint_model import validate_joint_positions


def _validate_vector(values, name):
    validate_joint_positions(values, name)


def sample_reference(start, target, duration, elapsed, horizon, count):
    """
    Sample an absolute-time reference from the current execution point.

    ``elapsed`` is measured from the reference start time. Samples after the
    requested duration remain at the target, so a late visualizer does not
    extrapolate beyond the command sent to the controller.
    """
    _validate_vector(start, 'start')
    _validate_vector(target, 'target')
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be positive and finite')
    if not math.isfinite(elapsed):
        raise ValueError('elapsed must be finite')
    if not math.isfinite(horizon) or horizon < 0:
        raise ValueError('horizon must be finite and non-negative')
    if count < 2:
        raise ValueError('count must be at least two')

    target = normalize_target(start, target)
    elapsed = max(0.0, elapsed)
    samples = []
    for index in range(count):
        offset = horizon * index / float(count - 1)
        time_from_start = min(duration, elapsed + offset)
        fraction = time_from_start / duration
        samples.append(tuple(
            start_value + fraction * (target_value - start_value)
            for start_value, target_value in zip(start, target)
        ))
    return samples
