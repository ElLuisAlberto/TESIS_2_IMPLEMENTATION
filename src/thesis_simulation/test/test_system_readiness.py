"""Regression tests for the system readiness monitor."""

from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from thesis_simulation import system_readiness_node


def test_controller_timeout_retries_and_recovers_ready(monkeypatch):
    """A lost first response must not prevent a later READY result."""
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(
        system_readiness_node.time,
        'monotonic',
        lambda: clock.now,
    )

    first_future = Mock()
    first_future.done.return_value = False
    second_future = Mock()
    second_future.done.return_value = True
    second_future.result.return_value = SimpleNamespace(controller=[
        SimpleNamespace(name='arm_controller', state='active'),
    ])

    client = Mock()
    client.service_is_ready.side_effect = (True, True, False)
    client.call_async.side_effect = (first_future, second_future)

    node = SimpleNamespace(
        controller_active=False,
        controller_error='esperando controller_manager',
        controller_future=None,
        controller_future_started_at=None,
        controller_request_timeout_sec=2.0,
        controller_client=client,
        action_client=Mock(),
        ready_publisher=Mock(),
        status_publisher=Mock(),
        last_status=None,
        get_logger=Mock(),
        _node_graph_status=lambda: [],
        _freshness_status=lambda _now: [],
        _connection_status=lambda: [],
    )
    node.action_client.server_is_ready.return_value = True
    node._update_controller_state = MethodType(
        system_readiness_node.SystemReadinessNode._update_controller_state,
        node,
    )

    system_readiness_node.SystemReadinessNode.evaluate(node)
    assert not node.ready_publisher.publish.call_args.args[0].data

    clock.now = 2.0
    system_readiness_node.SystemReadinessNode.evaluate(node)
    first_future.cancel.assert_called_once_with()
    assert client.call_async.call_count == 2

    clock.now = 2.1
    system_readiness_node.SystemReadinessNode.evaluate(node)

    assert node.controller_active
    assert node.ready_publisher.publish.call_args.args[0].data
    assert node.status_publisher.publish.call_args.args[0].data == 'READY'
