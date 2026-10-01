"""Unit tests for the deterministic preventive safety policy."""

import math

import pytest

from thesis_core.safety_policy import (
    SafetyPolicyInput,
    decide_preventive_state,
    maximum_safe_scale,
    withdrawal_is_non_approaching,
    withdrawal_path_is_non_approaching,
    withdrawal_path_is_safe,
)


def policy_input(**updates):
    """Return one nominally safe policy input with selected overrides."""
    values = {
        'current_clearance': 0.40,
        'nominal_clearance': 0.20,
        'supervised_clearance': 0.20,
        'protective_margin': 0.02,
        'stop_distance': 0.05,
        'warning_distance': 0.30,
        'minimum_scale': 0.05,
        'candidate_scale': 1.0,
        'nominal_ttc': -1.0,
        'previous_scale': 1.0,
    }
    values.update(updates)
    return SafetyPolicyInput(**values)


@pytest.mark.parametrize(
    ('updates', 'reason'),
    (
        ({'joint_state_valid': False}, 'STATE_UNAVAILABLE_OR_STALE'),
        ({'proximity_valid': False}, 'PROXIMITY_UNAVAILABLE_OR_STALE'),
        ({'prediction_valid': False}, 'PREDICTION_ERROR'),
        ({'nominal_clearance': math.nan}, 'NUMERIC_ERROR'),
    ),
)
def test_invalid_evidence_is_fail_safe_stop(updates, reason):
    decision = decide_preventive_state(policy_input(**updates))
    assert decision.state == 'STOP'
    assert decision.speed_scale == 0.0
    assert decision.reason_code == reason


@pytest.mark.parametrize('clearance', (0.05, 0.04, -0.01))
def test_current_stop_distance_has_absolute_priority(clearance):
    decision = decide_preventive_state(policy_input(
        current_clearance=clearance,
        nominal_clearance=0.50,
        supervised_clearance=0.50,
    ))
    assert decision.state == 'STOP'
    assert decision.speed_scale == 0.0
    assert decision.reason_code == 'CURRENT_CLEARANCE_STOP'


def test_warning_never_reduces_speed():
    decision = decide_preventive_state(policy_input(
        current_clearance=0.20,
        nominal_clearance=0.10,
        supervised_clearance=0.10,
    ))
    assert decision.state == 'WARNING'
    assert decision.speed_scale == 1.0


def test_clear_current_and_future_is_allow():
    decision = decide_preventive_state(policy_input(
        current_clearance=0.40,
        nominal_clearance=0.10,
        supervised_clearance=0.10,
    ))
    assert decision.state == 'ALLOW'
    assert decision.speed_scale == 1.0


def test_future_intrusion_selects_reduction():
    decision = decide_preventive_state(policy_input(
        nominal_clearance=-0.04,
        supervised_clearance=0.0201,
        candidate_scale=0.42,
        nominal_ttc=0.7,
    ))
    assert decision.state == 'REDUCTION'
    assert decision.speed_scale == pytest.approx(0.42)
    assert decision.reason_code == 'PREDICTIVE_INTRUSION'


def test_missing_minimum_safe_scale_is_stop():
    decision = decide_preventive_state(policy_input(
        nominal_clearance=-0.04,
        supervised_clearance=0.02,
        candidate_scale=0.049,
    ))
    assert decision.state == 'STOP'
    assert decision.reason_code == 'NO_SAFE_SCALE'


def test_unsafe_supervised_result_is_prediction_error():
    decision = decide_preventive_state(policy_input(
        nominal_clearance=-0.04,
        supervised_clearance=0.019,
        candidate_scale=0.50,
    ))
    assert decision.state == 'STOP'
    assert decision.reason_code == 'PREDICTION_ERROR'


def test_binary_search_returns_largest_safe_scale():
    result = maximum_safe_scale(
        lambda scale: 0.12 - 0.20 * scale,
        protective_margin=0.02,
        minimum_scale=0.05,
        iterations=10,
    )
    assert result.feasible
    assert result.scale <= 0.5
    assert 0.5 - result.scale <= 1.0 / 2 ** 10
    assert result.clearance >= 0.02


def test_binary_search_detects_no_safe_scale():
    result = maximum_safe_scale(
        lambda scale: 0.01 - scale,
        protective_margin=0.02,
    )
    assert not result.feasible
    assert result.scale == 0.0


def test_more_danger_never_increases_scale():
    moderate = maximum_safe_scale(
        lambda scale: 0.12 - 0.20 * scale,
        protective_margin=0.02,
    )
    severe = maximum_safe_scale(
        lambda scale: 0.08 - 0.20 * scale,
        protective_margin=0.02,
    )
    assert severe.scale <= moderate.scale


def test_withdrawal_predicate_rejects_additional_approach():
    assert withdrawal_is_non_approaching(0.08, 0.10)
    assert not withdrawal_is_non_approaching(0.08, 0.07)


def test_inside_stop_allows_only_verified_withdrawal():
    decision = decide_preventive_state(policy_input(
        current_clearance=0.02,
        nominal_clearance=0.02,
        supervised_clearance=0.02,
        candidate_scale=0.10,
        withdrawal_safe=True,
    ))
    assert decision.state == 'REDUCTION'
    assert decision.speed_scale == pytest.approx(0.10)
    assert decision.reason_code == 'PROTECTIVE_WITHDRAWAL'


def test_inside_stop_rejects_unverified_withdrawal():
    decision = decide_preventive_state(policy_input(
        current_clearance=0.02,
        nominal_clearance=0.01,
        supervised_clearance=0.01,
        candidate_scale=0.10,
        withdrawal_safe=False,
    ))
    assert decision.state == 'STOP'
    assert decision.speed_scale == 0.0
    assert decision.reason_code == 'CURRENT_CLEARANCE_STOP'


def test_withdrawal_requires_minimum_final_progress():
    assert withdrawal_path_is_safe((0.020, 0.021, 0.023))
    assert not withdrawal_path_is_safe((0.020, 0.021, 0.021))


def test_scaled_withdrawal_allows_monotonic_submillimeter_progress():
    assert withdrawal_path_is_non_approaching((0.020, 0.0202, 0.0204))
    assert not withdrawal_path_is_safe((0.020, 0.0202, 0.0204))


def test_withdrawal_rejects_any_intermediate_approach():
    assert not withdrawal_path_is_safe((0.020, 0.018, 0.030))


def test_non_approaching_path_rejects_an_inward_step():
    assert not withdrawal_path_is_non_approaching(
        (0.020, 0.022, 0.0205),
    )


def test_invalid_evidence_still_overrides_safe_withdrawal():
    decision = decide_preventive_state(policy_input(
        current_clearance=0.02,
        candidate_scale=0.10,
        withdrawal_safe=True,
        proximity_valid=False,
    ))
    assert decision.state == 'STOP'
    assert decision.reason_code == 'PROXIMITY_UNAVAILABLE_OR_STALE'
