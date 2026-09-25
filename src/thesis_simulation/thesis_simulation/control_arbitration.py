"""Pure priority rules for controls pending an action cancellation."""

from __future__ import annotations

import math


STATE_SEVERITY = {
    'ALLOW': 0,
    'WARNING': 1,
    'REDUCTION': 2,
    'STOP': 3,
}


def effective_speed_scale(state: str, speed_scale: float) -> float:
    """Return the controller scale represented by one safety decision."""
    if state not in STATE_SEVERITY:
        raise ValueError(f'unknown safety state: {state}')
    value = float(speed_scale)
    if not math.isfinite(value):
        raise ValueError('speed_scale must be finite')
    if state == 'STOP':
        return 0.0
    if state in {'ALLOW', 'WARNING'}:
        return 1.0
    if not 0.0 < value <= 1.0:
        raise ValueError('REDUCTION speed_scale must be in (0, 1]')
    return value


def is_more_restrictive(
    candidate_state: str,
    candidate_scale: float,
    reference_state: str,
    reference_scale: float,
) -> bool:
    """Return whether candidate must replace a pending control request."""
    candidate_severity = STATE_SEVERITY[candidate_state]
    reference_severity = STATE_SEVERITY[reference_state]
    if candidate_severity != reference_severity:
        return candidate_severity > reference_severity
    return effective_speed_scale(
        candidate_state,
        candidate_scale,
    ) < effective_speed_scale(reference_state, reference_scale) - 1.0e-12
