import math

from thesis_simulation.readiness_contract import sample_is_fresh


def test_sample_is_fresh_within_timeout():
    assert sample_is_fresh(10.0, 10.5, 1.0)


def test_sample_at_timeout_boundary_is_fresh():
    assert sample_is_fresh(10.0, 11.0, 1.0)


def test_missing_old_or_future_sample_is_not_fresh():
    assert not sample_is_fresh(None, 10.0, 1.0)
    assert not sample_is_fresh(8.9, 10.0, 1.0)
    assert not sample_is_fresh(10.1, 10.0, 1.0)


def test_non_finite_timing_values_are_not_fresh():
    assert not sample_is_fresh(math.nan, 10.0, 1.0)
    assert not sample_is_fresh(9.0, math.inf, 1.0)
    assert not sample_is_fresh(9.0, 10.0, math.nan)
