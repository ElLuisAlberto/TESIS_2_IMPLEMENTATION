"""Unit tests for package-B deterministic telemetry helpers."""

import pytest

from thesis_telemetry.telemetry_math import (
    RunningStats,
    ZeroVelocityDetector,
    percentile95,
)


def test_statistics_include_required_values():
    stats = RunningStats()
    for value in (1.0, 2.0, 3.0, 4.0):
        stats.add(value)
    count, mean, stddev, minimum, maximum, p95 = stats.summary()
    assert count == 4
    assert mean == pytest.approx(2.5)
    assert stddev == pytest.approx(1.11803398875)
    assert minimum == 1.0
    assert maximum == 4.0
    assert p95 == pytest.approx(3.85)


def test_percentile_empty_is_documented_sentinel():
    assert percentile95([]) == -1.0


def test_zero_velocity_requires_consecutive_samples():
    detector = ZeroVelocityDetector(0.01, 3)
    assert not detector.update((0.009, 0.0))
    assert not detector.update((0.008, 0.0))
    assert detector.update((0.007, 0.0))


def test_motion_resets_zero_velocity_confirmation():
    detector = ZeroVelocityDetector(0.01, 2)
    assert not detector.update((0.0,))
    assert not detector.update((0.02,))
    assert not detector.update((0.0,))
    assert detector.update((0.0,))
