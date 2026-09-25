"""Tests for fail-safe arbitration during action cancellation."""

from thesis_simulation.control_arbitration import is_more_restrictive


def test_allow_cannot_overwrite_pending_stop():
    assert not is_more_restrictive('ALLOW', 1.0, 'STOP', 0.0)


def test_warning_cannot_overwrite_pending_stop():
    assert not is_more_restrictive('WARNING', 1.0, 'STOP', 0.0)


def test_stop_overwrites_pending_reduction():
    assert is_more_restrictive('STOP', 0.0, 'REDUCTION', 0.5)


def test_lower_reduction_scale_overwrites_pending_reduction():
    assert is_more_restrictive('REDUCTION', 0.3, 'REDUCTION', 0.5)


def test_higher_reduction_scale_cannot_overwrite_pending_reduction():
    assert not is_more_restrictive('REDUCTION', 0.7, 'REDUCTION', 0.5)
