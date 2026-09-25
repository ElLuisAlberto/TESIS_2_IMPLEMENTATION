"""Fail-closed evaluation of observed Advance 2 scenario evidence."""

from dataclasses import dataclass
from math import isfinite, pi
from typing import Mapping, Sequence, Tuple

from .contracts import CRITICAL_FIELDS, SCENARIOS


@dataclass(frozen=True)
class Evaluation:
    """One deterministic verdict with explicit failure reasons."""

    passed: bool
    reasons: Tuple[str, ...]


def _number(row: Mapping[str, object], key: str) -> float:
    """Read a finite numeric observation; reject absent measurements."""
    value = float(row[key])
    if not isfinite(value):
        raise ValueError(f'{key} no es finito')
    return value


def missing_critical_fields(row: Mapping[str, object]) -> Tuple[str, ...]:
    """Return critical columns absent or empty from a record."""
    return tuple(
        key for key in CRITICAL_FIELDS
        if key not in row or row[key] is None or str(row[key]).strip() == ''
    )


def evaluate(row: Mapping[str, object]) -> Evaluation:
    """Evaluate only measurements that actually establish a scenario."""
    missing = missing_critical_fields(row)
    if missing:
        return Evaluation(False, ('EMPTY:' + ','.join(missing),))
    scenario = str(row['scenario_id'])
    if scenario not in SCENARIOS:
        return Evaluation(False, ('UNKNOWN_SCENARIO',))
    try:
        numeric = {
            key: _number(row, key) for key in (
                'd_actual', 'd_nominal', 'd_supervisada', 'ttc_nominal',
                't_min', 'speed_scale', 'qdot_medida_max',
                'latency_end_to_end_ms', 'observation_count',
                'jog_hold_age_ms', 'horizon_update_ms',
            )
        }
    except (KeyError, TypeError, ValueError, OverflowError):
        return Evaluation(False, ('INVALID_NUMBER',))
    state = str(row['observed']).upper()
    scale = numeric['speed_scale']
    actual = numeric['d_actual']
    nominal = numeric['d_nominal']
    supervised = numeric['d_supervisada']
    ttc = numeric['ttc_nominal']
    reasons = []
    if state not in SCENARIOS[scenario].expected_states:
        reasons.append('UNEXPECTED_STATE')
    expected = '|'.join(sorted(SCENARIOS[scenario].expected_states))
    if str(row['expected']) != expected:
        reasons.append('EXPECTED_CONTRACT_MISMATCH')
    if (numeric['observation_count'] < 1
            or str(row['command_id']) == 'UNAVAILABLE'):
        reasons.append('NO_CORRELATED_OBSERVATION')
    if str(row['reason_code']) == 'UNAVAILABLE':
        reasons.append('NO_DIAGNOSTIC')
    if not 0.0 <= scale <= 1.0:
        reasons.append('INVALID_SCALE')
    if numeric['latency_end_to_end_ms'] < 0.0:
        reasons.append('NO_PIPELINE_LATENCY')
    if numeric['qdot_medida_max'] < 0.0:
        reasons.append('NO_MEASURED_VELOCITY')
    if scenario in {
        'E01', 'E02', 'E03', 'E04', 'E05', 'E07', 'E08', 'E13',
    } and numeric['qdot_medida_max'] < 0.01:
        reasons.append('NO_OBSERVED_MOVEMENT')
    if actual < -1.0 or nominal < -1.0 or supervised < -1.0:
        reasons.append('INVALID_CLEARANCE')
    if ttc < -1.0 or numeric['t_min'] < -1.0:
        reasons.append('INVALID_TIME')
    if scenario in {'E01', 'E02'}:
        if abs(scale - 1.0) > 1e-6:
            reasons.append('FREE_MOTION_SLOWED')
        if nominal < 0.02:
            reasons.append('NOMINAL_HORIZON_INTRUSION')
    if scenario == 'E03':
        if row['d_nominal_decreased'] != 'true':
            reasons.append('NO_DECREASING_NOMINAL_CLEARANCE')
        if ttc < 0.0 or nominal > actual + 1e-6:
            reasons.append('NO_VALID_TTC')
    if scenario in {'E04', 'E05'}:
        if state != 'REDUCTION' or not 0.0 < scale < 1.0:
            reasons.append('REDUCTION_NOT_APPLIED')
        if nominal >= 0.02 or supervised < 0.02 - 1e-6:
            reasons.append('UNPROTECTED_PREDICTION')
        if scenario == 'E05' and row['scale_decreased'] != 'true':
            reasons.append('NO_PROGRESSIVE_REDUCTION')
    if scenario == 'E06':
        if state != 'STOP' or actual > 0.05 + 1e-6 or scale != 0.0:
            reasons.append('CRITICAL_STOP_NOT_APPLIED')
        if str(row['reason_code']) != 'CURRENT_CLEARANCE_STOP':
            reasons.append('WRONG_STOP_REASON')
    if scenario == 'E07':
        if row['scale_recovered'] != 'true':
            reasons.append('NO_OBSERVED_RECOVERY')
    if scenario == 'E08' and not 0.0 <= numeric['horizon_update_ms'] <= 100.0:
        reasons.append('NO_ONE_CYCLE_HORIZON_UPDATE')
    fail_safe_reasons = {
        'E09': 'STATE_UNAVAILABLE_OR_STALE',
        'E10': 'PROXIMITY_UNAVAILABLE_OR_STALE',
    }
    if scenario in fail_safe_reasons:
        if state != 'STOP' or scale != 0.0:
            reasons.append('FAIL_SAFE_NOT_APPLIED')
        if fail_safe_reasons[scenario] not in str(row['reason_code']):
            reasons.append('WRONG_FAIL_SAFE_REASON')
    if scenario == 'E11':
        if state != 'HOLD':
            reasons.append('WATCHDOG_HOLD_NOT_OBSERVED')
        if 'JOG' not in str(row['reason_code']).upper():
            reasons.append('WRONG_WATCHDOG_REASON')
        deadline_ms = 250.0 + numeric['latency_end_to_end_ms']
        if not 0.0 <= numeric['jog_hold_age_ms'] <= deadline_ms:
            reasons.append('WATCHDOG_HOLD_LATE_OR_UNMEASURED')
    if scenario == 'E12':
        if row['expected_segment'] != row['limiting_segment']:
            reasons.append('WRONG_LIMITING_SEGMENT')
    if scenario == 'E13':
        if numeric['qdot_medida_max'] > 24.0 * pi / 180.0 + 0.02:
            reasons.append('VELOCITY_LIMIT_EXCEEDED')
        if str(row['joint_velocity_violation']) == 'true':
            reasons.append('JOINT_VELOCITY_LIMIT_EXCEEDED')
        if str(row['joint_velocity_checked']) != 'true':
            reasons.append('JOINT_VELOCITY_UNMEASURED')
        if str(row['saturation_confirmed']) != 'true':
            reasons.append('SATURATION_UNCONFIRMED')
    if scenario == 'E14':
        if state != 'REJECTED' or row['controller_status'] != 'REJECTED':
            reasons.append('INVALID_TARGET_NOT_REJECTED')
        if str(row['reason_code']) not in {
            'PREDICTION_ERROR', 'VALIDATION_REJECTED',
        }:
            reasons.append('WRONG_REJECTION_REASON')
    if scenario == 'E15':
        if state != 'STOP' or scale != 0.0:
            reasons.append('CONTROLLER_FAILURE_NOT_SAFE')
        if row['controller_status'] not in {'FAILED', 'REJECTED'}:
            reasons.append('NO_CONTROLLER_FAILURE_EVIDENCE')
    return Evaluation(not reasons, tuple(reasons) if reasons else ('PASS',))


def nonincreasing(values: Sequence[float], tolerance: float = 1e-6) -> bool:
    """Return whether a sequence never increases beyond tolerance."""
    return all(b <= a + tolerance for a, b in zip(values, values[1:]))


def nondecreasing(values: Sequence[float], tolerance: float = 1e-6) -> bool:
    """Return whether a sequence never decreases beyond tolerance."""
    return all(b + tolerance >= a for a, b in zip(values, values[1:]))
