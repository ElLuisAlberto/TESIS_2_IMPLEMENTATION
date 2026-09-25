"""Fail-closed experimental verdict regression checks."""

import pytest

from thesis_validation.contracts import SCENARIOS
from thesis_validation.evaluation import evaluate, nondecreasing, nonincreasing


def row(scenario, state, scale, d_actual=0.4, d_nominal=0.3, ttc=0.5):
    """Build one explicit synthetic observation; no ROS claim is implied."""
    return {
        'timestamp': '2026-01-01T00:00:00Z', 'scenario_id': scenario,
        'repetition': 1,
        'expected': '|'.join(sorted(SCENARIOS[scenario].expected_states)),
        'observed': state, 'command_id': 'synthetic-1',
        'd_actual': d_actual, 'd_nominal': d_nominal,
        'd_supervisada': max(d_nominal, 0.02), 'ttc_nominal': ttc,
        't_min': 0.5, 'speed_scale': scale, 'qdot_medida_max': 0.1,
        'latency_end_to_end_ms': 20.0,
        'limiting_segment': 'shoulder_to_upper_arm',
        'reason_code': 'TEST', 'observation_count': 3,
        'd_nominal_decreased': 'true', 'scale_decreased': 'true',
        'scale_recovered': 'true', 'controller_status': 'UNAVAILABLE',
        'expected_segment': 'shoulder_to_upper_arm',
        'jog_hold_age_ms': -1.0, 'horizon_update_ms': -1.0,
        'saturation_confirmed': 'false',
        'joint_velocity_violation': 'false',
        'joint_velocity_checked': 'true',
        'verdict': 'PENDING',
    }


@pytest.mark.parametrize('scenario', ['E01', 'E02'])
def test_free_motion_is_not_slowed(scenario):
    state = 'ALLOW' if scenario == 'E01' else 'WARNING'
    assert evaluate(row(scenario, state, 1.0)).passed
    assert not evaluate(row(scenario, state, 0.5)).passed


@pytest.mark.parametrize('scenario', ['E04', 'E05'])
def test_predictive_invasion_requires_reduction(scenario):
    evidence = row(scenario, 'REDUCTION', 0.5, d_nominal=0.01)
    assert evaluate(evidence).passed
    assert not evaluate(row(scenario, 'ALLOW', 1.0)).passed


def test_critical_clearance_requires_stop():
    evidence = row('E06', 'STOP', 0.0, d_actual=0.05)
    evidence['reason_code'] = 'CURRENT_CLEARANCE_STOP'
    assert evaluate(evidence).passed


@pytest.mark.parametrize('scenario,reason', [
    ('E09', 'STATE_UNAVAILABLE_OR_STALE'),
    ('E10', 'PROXIMITY_UNAVAILABLE_OR_STALE'),
])
def test_stale_input_causes_fail_safe(scenario, reason):
    evidence = row(scenario, 'STOP', 0.0)
    evidence['reason_code'] = reason
    assert evaluate(evidence).passed
    evidence['reason_code'] = 'CURRENT_CLEARANCE_STOP'
    assert not evaluate(evidence).passed


def test_jog_hold_requires_measured_age():
    evidence = row('E11', 'HOLD', 0.0)
    evidence['reason_code'] = 'JOG_WATCHDOG_EXPIRED'
    evidence['jog_hold_age_ms'] = 249.0
    assert evaluate(evidence).passed
    evidence['jog_hold_age_ms'] = -1.0
    assert not evaluate(evidence).passed


def test_controller_failure_requires_status_and_stop():
    evidence = row('E15', 'STOP', 0.0)
    evidence['controller_status'] = 'FAILED'
    assert evaluate(evidence).passed
    evidence['controller_status'] = 'UNAVAILABLE'
    assert not evaluate(evidence).passed


def test_invalid_number_and_missing_observation_fail_closed():
    evidence = row('E01', 'ALLOW', 1.0)
    evidence['d_actual'] = 'nan'
    assert not evaluate(evidence).passed
    evidence['d_actual'] = 0.4
    evidence['command_id'] = 'UNAVAILABLE'
    assert not evaluate(evidence).passed


def test_direction_change_requires_measured_latency():
    evidence = row('E08', 'ALLOW', 1.0)
    assert not evaluate(evidence).passed
    evidence['horizon_update_ms'] = 90.0
    assert evaluate(evidence).passed


def test_saturation_requires_confirmation_and_per_joint_measurement():
    evidence = row('E13', 'ALLOW', 1.0)
    assert not evaluate(evidence).passed
    evidence['saturation_confirmed'] = 'true'
    assert evaluate(evidence).passed
    evidence['joint_velocity_violation'] = 'true'
    assert not evaluate(evidence).passed


def test_progressive_reduction_is_monotonic():
    assert nonincreasing([1.0, 0.8, 0.6, 0.4])
    assert not nonincreasing([1.0, 0.6, 0.7])


def test_recovery_is_monotonic():
    assert nondecreasing([0.2, 0.3, 0.4, 1.0])
    assert not nondecreasing([0.2, 0.5, 0.4])
