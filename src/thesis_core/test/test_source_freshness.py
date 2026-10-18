"""Fail-closed source-timestamp tests for state and proximity inputs."""

from types import SimpleNamespace

from thesis_core.safety_supervisor_node import (
    SafetySupervisorNode,
    source_stamp_is_fresh,
)


class _Logger:

    def warning(self, _message):
        pass


def _stamp(seconds, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def _node():
    now = SimpleNamespace(nanoseconds=10_000_000_000)
    return SimpleNamespace(
        current_positions={'previous': 1.0},
        last_state_receive_ns=1,
        last_state_arrival_ns=None,
        proximity_status=SimpleNamespace(state='ALLOW'),
        last_proximity_receive_ns=1,
        last_proximity_arrival_ns=None,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        get_parameter=lambda _name: SimpleNamespace(value=0.5),
        get_logger=lambda: _Logger(),
    )


def test_source_stamp_accepts_fresh_and_rejects_old_zero_or_future():
    now_ns = 10_000_000_000
    assert source_stamp_is_fresh(now_ns, _stamp(9, 800_000_000), 0.5)
    assert not source_stamp_is_fresh(now_ns, _stamp(9), 0.5)
    assert not source_stamp_is_fresh(now_ns, _stamp(0), 0.5)
    assert not source_stamp_is_fresh(now_ns, _stamp(10, 60_000_000), 0.5)


def test_stale_joint_state_clears_state_instead_of_refreshing_it():
    node = _node()
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(9)),
        name=['joint'], position=[0.0],
    )

    SafetySupervisorNode.joint_state_callback(node, message)

    assert node.current_positions == {}
    assert node.last_state_receive_ns is None


def test_stale_proximity_clears_state_instead_of_allowing_it():
    node = _node()
    message = SimpleNamespace(stamp=_stamp(9), state='ALLOW')

    SafetySupervisorNode.proximity_status_callback(node, message)

    assert node.proximity_status is None
    assert node.last_proximity_receive_ns is None
