"""Tests for the formal Advance 2 scenario contract."""

from thesis_validation.contracts import CRITICAL_FIELDS, SCENARIOS


def test_all_fifteen_scenarios_are_defined():
    assert tuple(sorted(SCENARIOS)) == tuple(f"E{i:02d}" for i in range(1, 16))


def test_latency_scenario_requires_ten_repetitions():
    for scenario in ("E08", "E11"):
        assert SCENARIOS[scenario].requires_latency
        assert SCENARIOS[scenario].repetitions == 10


def test_csv_contract_has_no_duplicate_columns():
    assert len(CRITICAL_FIELDS) == len(set(CRITICAL_FIELDS))
