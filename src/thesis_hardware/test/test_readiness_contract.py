from thesis_hardware.readiness_contract import (
    CANONICAL_JOINT_NAMES,
    joint_state_is_valid,
    proximity_is_valid,
    readiness_reasons,
    sample_is_fresh,
    source_stamp_is_fresh,
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


def test_source_stamps_fail_closed():
    assert source_stamp_is_fresh(9.8, 10.0, 0.5)
    assert not source_stamp_is_fresh(0.0, 10.0, 0.5)
    assert not source_stamp_is_fresh(9.4, 10.0, 0.5)
    assert not source_stamp_is_fresh(10.051, 10.0, 0.5)


def test_joint_state_requires_all_canonical_joints_and_fresh_stamp():
    positions = [0.0] * len(CANONICAL_JOINT_NAMES)
    assert joint_state_is_valid(
        CANONICAL_JOINT_NAMES, positions, 9.8, 10.0, 0.5
    )
    assert not joint_state_is_valid(
        CANONICAL_JOINT_NAMES[:-1], positions[:-1], 9.8, 10.0, 0.5
    )
    invalid = list(positions)
    invalid[2] = float('nan')
    assert not joint_state_is_valid(
        CANONICAL_JOINT_NAMES, invalid, 9.8, 10.0, 0.5
    )


def test_proximity_requires_world_known_state_and_fresh_stamp():
    assert proximity_is_valid(
        'ALLOW', 'world', 0.2, 0.1, 9.8, 10.0, 0.5
    )
    assert proximity_is_valid(
        'STOP', 'world', -0.1, 0.1, 9.8, 10.0, 0.5
    )
    assert not proximity_is_valid(
        'ALLOW', 'camera', 0.2, 0.1, 9.8, 10.0, 0.5
    )
    assert not proximity_is_valid(
        'HOLD', 'world', 0.2, 0.1, 9.8, 10.0, 0.5
    )


def test_stop_proximity_keeps_readiness_not_ready():
    assert readiness_reasons(
        True, False, False, True, True, True, proximity_safe=False
    ) == ['proximidad en STOP o inválida']
