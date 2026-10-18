"""Pure readiness rules for the physical JACO stack."""

import math


CANONICAL_JOINT_NAMES = (
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
)


def sample_is_fresh(received_at, now, timeout_sec):
    """Return whether a wall-clock sample is recent."""
    values = (now, timeout_sec)
    if received_at is None or not all(math.isfinite(v) for v in values):
        return False
    age = now - received_at
    return 0.0 <= age <= timeout_sec


def source_stamp_is_fresh(stamp_sec, now_sec, timeout_sec,
                          future_tolerance_sec=0.05):
    """Reject zero, stale and materially future source stamps."""
    values = (stamp_sec, now_sec, timeout_sec, future_tolerance_sec)
    if not all(math.isfinite(v) for v in values):
        return False
    age = now_sec - stamp_sec
    return (
        stamp_sec > 0.0
        and timeout_sec > 0.0
        and future_tolerance_sec >= 0.0
        and -future_tolerance_sec <= age <= timeout_sec
    )


def joint_state_is_valid(names, positions, stamp_sec, now_sec, timeout_sec):
    """Validate canonical physical feedback and its acquisition stamp."""
    if len(names) != len(positions):
        return False
    by_name = dict(zip(names, positions))
    if not all(name in by_name for name in CANONICAL_JOINT_NAMES):
        return False
    if not all(
            math.isfinite(float(by_name[name]))
            for name in CANONICAL_JOINT_NAMES):
        return False
    return source_stamp_is_fresh(stamp_sec, now_sec, timeout_sec)


def proximity_is_valid(state, reference_frame, minimum_clearance,
                       obstacle_radius, stamp_sec, now_sec, timeout_sec):
    """Validate the physical safety sample without converting its frame."""
    if state not in {'ALLOW', 'WARNING', 'REDUCTION', 'STOP'}:
        return False
    if reference_frame != 'world':
        return False
    if not math.isfinite(minimum_clearance):
        return False
    if not math.isfinite(obstacle_radius) or obstacle_radius < 0.0:
        return False
    return source_stamp_is_fresh(stamp_sec, now_sec, timeout_sec)


def readiness_reasons(
        connected,
        armed,
        require_armed,
        joint_state_fresh,
        proximity_fresh,
        require_proximity,
        proximity_safe=True):
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
    if require_proximity and proximity_fresh and not proximity_safe:
        reasons.append('proximidad en STOP o inválida')
    return reasons
