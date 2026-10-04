"""Verify runtime supervision remains alive while hardware settles."""

import pytest

from thesis_core.safety_supervisor_node import runtime_remaining_time


def test_runtime_remaining_uses_nominal_time_before_deadline():
    """Nominal remaining time is preserved before trajectory completion."""
    assert runtime_remaining_time(4.0, 1.5) == pytest.approx(2.5)


def test_runtime_remaining_keeps_monitoring_after_deadline():
    """A settling goal retains a short monitored horizon after its deadline."""
    assert runtime_remaining_time(4.0, 4.4) == pytest.approx(0.10)


@pytest.mark.parametrize(
    ('duration', 'elapsed'),
    ((0.0, 0.0), (-1.0, 0.0), (1.0, -0.1)),
)
def test_runtime_remaining_rejects_invalid_timing(duration, elapsed):
    """Invalid execution timing cannot bypass fail-closed handling."""
    with pytest.raises(ValueError):
        runtime_remaining_time(duration, elapsed)
