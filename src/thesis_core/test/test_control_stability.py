"""Unit tests for deterministic temporal control stabilization."""

import pytest

from thesis_core.control_stability import ControlStabilityFilter


def make_filter():
    """Return the package-A baseline filter."""
    return ControlStabilityFilter(5, 0.10, 0.10)


def test_reduction_is_immediate():
    control = make_filter()
    control.update('jog', 'ALLOW', 1.0, 0.0)
    result = control.update('jog', 'REDUCTION', 0.47, 0.01)
    assert result.state == 'REDUCTION'
    assert result.speed_scale == pytest.approx(0.47)
    assert result.transition_reason == 'RESTRICTION_IMMEDIATE'


def test_stop_is_immediate_and_latched():
    control = make_filter()
    control.update('trajectory', 'ALLOW', 1.0, 0.0)
    result = control.update('trajectory', 'STOP', 0.0, 0.01)
    assert result.state == 'STOP'
    assert result.speed_scale == 0.0
    result = control.update('trajectory', 'STOP', 0.0, 0.02)
    assert result.transition_reason == 'STOP_LATCHED'


def test_five_safe_samples_are_required_before_recovery():
    control = make_filter()
    control.update('jog', 'REDUCTION', 0.5, 0.0)
    for index in range(4):
        result = control.update('jog', 'ALLOW', 1.0, 0.1 * index)
        assert result.speed_scale == pytest.approx(0.5)
    result = control.update('jog', 'ALLOW', 1.0, 0.4)
    assert result.recovery_count == 5
    assert result.speed_scale == pytest.approx(0.6)


def test_fast_messages_do_not_accelerate_recovery_counter():
    control = make_filter()
    control.update('jog', 'REDUCTION', 0.5, 0.0)
    first = control.update('jog', 'ALLOW', 1.0, 1.0)
    second = control.update('jog', 'ALLOW', 1.0, 1.02)
    assert first.recovery_count == 1
    assert second.recovery_count == 1


def test_unsafe_sample_resets_recovery():
    control = make_filter()
    control.update('jog', 'REDUCTION', 0.5, 0.0)
    control.update('jog', 'ALLOW', 1.0, 0.1)
    control.update('jog', 'ALLOW', 1.0, 0.2)
    result = control.update('jog', 'REDUCTION', 0.4, 0.21)
    assert result.speed_scale == pytest.approx(0.4)
    assert result.recovery_count == 0
    assert result.transition_reason == 'RESTRICTION_IMMEDIATE'


def test_recovery_scale_is_monotonic_and_rate_limited():
    control = make_filter()
    control.update('jog', 'REDUCTION', 0.5, 0.0)
    values = []
    for index in range(10):
        result = control.update('jog', 'ALLOW', 1.0, 1.0 + 0.1 * index)
        values.append(result.speed_scale)
    assert values == sorted(values)
    assert max(
        second - first for first, second in zip(values, values[1:])
    ) <= 0.100000001
    assert values[-1] == pytest.approx(1.0)
    assert result.state == 'ALLOW'


def test_command_change_resets_and_applies_new_evidence():
    control = make_filter()
    control.update('first', 'STOP', 0.0, 0.0)
    result = control.update('second', 'ALLOW', 1.0, 0.01)
    assert result.state == 'ALLOW'
    assert result.speed_scale == 1.0
    assert result.recovery_count == 0
    assert result.transition_reason == 'COMMAND_CHANGED_RESET'


def test_warning_recovery_preserves_scale_one_semantics():
    control = make_filter()
    control.update('jog', 'REDUCTION', 0.8, 0.0)
    for index in range(5):
        result = control.update('jog', 'WARNING', 1.0, 0.1 * index)
    assert result.state == 'REDUCTION'
    assert result.speed_scale == pytest.approx(0.9)
    result = control.update('jog', 'WARNING', 1.0, 0.5)
    assert result.state == 'WARNING'
    assert result.speed_scale == 1.0
