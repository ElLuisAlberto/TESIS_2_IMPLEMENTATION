"""Pure readiness rules for the physical JACO stack."""

import math


def sample_is_fresh(received_at, now, timeout_sec):
    """Return whether a wall-clock sample is recent."""
    values = (now, timeout_sec)
    if received_at is None or not all(math.isfinite(v) for v in values):
        return False
    age = now - received_at
    return 0.0 <= age <= timeout_sec


def readiness_reasons(
        connected,
        armed,
        require_armed,
        joint_state_fresh,
        proximity_fresh,
        require_proximity):
    """Return every failed physical-stack prerequisite."""
    reasons = []
    if not connected:
        reasons.append('adaptador JACO no conectado')
    if require_armed and not armed:
        reasons.append('salida física no armada')
    if not joint_state_fresh:
        reasons.append('estado articular ausente u obsoleto')
    if require_proximity and not proximity_fresh:
        reasons.append('proximidad ausente u obsoleta')
    return reasons
