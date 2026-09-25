"""Deterministic tests for Package 5 safety timing metrics."""

import math

import pytest

from thesis_core.horizon_clearance import (
    DEFAULT_PROTECTIVE_MARGIN_M,
    NO_EVENT_TIME,
    event_time_or_invalid,
    evaluate_capsule_horizon,
    interpolated_threshold_crossing_time,
    total_protective_margin,
)


def test_uncertainty_components_sum_to_initial_margin():
    margin = total_protective_margin(0.010, 0.005, 0.005)
    assert margin == pytest.approx(0.020)
    assert margin == pytest.approx(DEFAULT_PROTECTIVE_MARGIN_M)


def test_interpolates_crossing_between_samples():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05),
        (0.04, -0.01),
        0.02,
    )
    assert crossing == pytest.approx(0.02)


def test_start_inside_threshold_returns_zero():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05),
        (0.01, -0.01),
        0.02,
    )
    assert crossing == 0.0


def test_moving_away_has_no_future_event():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05, 0.10),
        (0.10, 0.15, 0.20),
        0.02,
    )
    assert crossing is None
    assert event_time_or_invalid(crossing) == NO_EVENT_TIME


def test_protective_entry_precedes_geometric_collision():
    times = (0.0, 1.0, 2.0, 3.0)
    clearances = (0.30, 0.20, 0.10, 0.0)
    protective = interpolated_threshold_crossing_time(
        times, clearances, 0.15
    )
    collision = interpolated_threshold_crossing_time(
        times, clearances, 0.0
    )
    assert protective == pytest.approx(1.5)
    assert collision == pytest.approx(3.0)
    assert protective < collision


def test_reduction_delays_or_removes_protective_entry():
    times = (0.0, 0.5, 1.0)
    nominal = (0.10, 0.01, -0.05)
    supervised = (0.10, 0.08, 0.06)
    nominal_ttc = interpolated_threshold_crossing_time(
        times, nominal, 0.02
    )
    supervised_ttc = interpolated_threshold_crossing_time(
        times, supervised, 0.02
    )
    assert nominal_ttc is not None
    assert supervised_ttc is None


def test_ttc_decreases_for_a_faster_uniform_approach():
    times = (0.0, 0.5, 1.0)
    slow = interpolated_threshold_crossing_time(
        times, (0.10, 0.04, -0.02), 0.02
    )
    fast = interpolated_threshold_crossing_time(
        times, (0.10, 0.00, -0.10), 0.02
    )
    assert slow is not None
    assert fast is not None
    assert fast < slow


def test_increasing_sample_count_reduces_interpolation_error():
    exact = math.sqrt(0.8)
    errors = []
    for count in (21, 41, 81):
        times = tuple(index / float(count - 1) for index in range(count))
        clearances = tuple(1.0 - value * value for value in times)
        crossing = interpolated_threshold_crossing_time(
            times, clearances, 0.2
        )
        assert crossing is not None
        errors.append(abs(crossing - exact))
    assert errors[2] <= errors[1] <= errors[0]
    assert errors[2] < 1.0e-4


def test_horizon_reports_interpolated_protective_and_collision_times():
    segment = (((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),)
    result = evaluate_capsule_horizon(
        (segment, segment),
        (0.0, 1.0),
        (0.10,),
        ('test_segment',),
        (0.5, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        0.10,
        0.02,
    )
    assert result.first_protective_entry_time == pytest.approx(0.78)
    assert result.first_collision_time == pytest.approx(0.80)
    assert result.minimum_protective_clearance == pytest.approx(-0.22)
