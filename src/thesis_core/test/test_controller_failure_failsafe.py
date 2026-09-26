"""Verify fail-closed handling of controller terminal states."""

from types import SimpleNamespace

from thesis_core.safety_supervisor_node import SafetySupervisorNode


class _Now:

    nanoseconds = 123456789


class _Clock:

    @staticmethod
    def now():
        return _Now()


class _Stability:

    def __init__(self):
        self.resets = []

    def reset(self, reason):
        self.resets.append(reason)


def _fake_node(active_execution=None):
    stops = []
    node = SimpleNamespace(
        active_execution=active_execution,
        last_execution_receive_ns=100,
        last_runtime_command_id=(
            None
            if active_execution is None
            else active_execution.command_id
        ),
        last_runtime_state='ALLOW',
        last_runtime_scale=1.0,
        runtime_recovery_count=0,
        runtime_stability=_Stability(),
        get_clock=lambda: _Clock(),
        publish_runtime_stop=lambda command_id, code, reason: stops.append(
            (command_id, code, reason)
        ),
    )
    return node, stops


def _message(command_id, status, detail=''):
    return SimpleNamespace(
        command_id=command_id,
        status=status,
        detail=detail,
    )


def test_rejected_without_active_execution_publishes_stop():
    node, stops = _fake_node()

    SafetySupervisorNode.execution_callback(
        node,
        _message('e15-r1', 'REJECTED', 'controlador inactivo'),
    )

    assert len(stops) == 1
    assert stops[0][0] == 'e15-r1'
    assert stops[0][1] == 'CONTROLLER_REJECTED'
    assert 'controlador inactivo' in stops[0][2]


def test_failed_active_execution_publishes_stop_and_clears_state():
    active = _message('e15-r2', 'ACCEPTED')
    node, stops = _fake_node(active)

    SafetySupervisorNode.execution_callback(
        node,
        _message('e15-r2', 'FAILED', 'error del controlador'),
    )

    assert len(stops) == 1
    assert stops[0][1] == 'CONTROLLER_FAILED'
    assert node.active_execution is None
    assert node.last_execution_receive_ns is None
    assert node.last_runtime_command_id is None
    assert node.runtime_stability.resets == ['EXECUTION_FAILED']


def test_duplicate_rejection_does_not_stop_accepted_execution():
    active = _message('duplicado', 'ACCEPTED')
    node, stops = _fake_node(active)

    SafetySupervisorNode.execution_callback(
        node,
        _message('duplicado', 'REJECTED', 'trayectoria ya activa'),
    )

    assert stops == []
    assert node.active_execution is active
    assert node.last_runtime_command_id == 'duplicado'
