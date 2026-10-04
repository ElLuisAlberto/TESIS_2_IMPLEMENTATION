from thesis_hardware.readiness_contract import (
    readiness_reasons,
    sample_is_fresh,
)


def test_sample_freshness_is_fail_closed():
    assert sample_is_fresh(9.8, 10.0, 0.5)
    assert not sample_is_fresh(None, 10.0, 0.5)
    assert not sample_is_fresh(9.0, 10.0, 0.5)
    assert not sample_is_fresh(10.1, 10.0, 0.5)


def test_physical_readiness_requires_every_enabled_input():
    assert readiness_reasons(True, True, True, True, True, True) == []
    reasons = readiness_reasons(
        connected=False,
        armed=False,
        require_armed=True,
        joint_state_fresh=False,
        proximity_fresh=False,
        require_proximity=True,
    )
    assert len(reasons) == 4


def test_read_only_mode_can_ignore_runtime_arm():
    assert readiness_reasons(
        True, False, False, True, True, True
    ) == []
