"""Small timing contracts used by the system readiness monitor."""

import math


def sample_is_fresh(received_at, now, timeout_sec):
    """Return whether a monotonic-time sample is within its timeout."""
    if received_at is None:
        return False
    if not all(math.isfinite(value) for value in (
            received_at, now, timeout_sec)):
        return False
    age = now - received_at
    return 0.0 <= age <= timeout_sec
