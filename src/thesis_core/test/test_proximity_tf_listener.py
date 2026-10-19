"""Regression tests for concurrent TF reception in the proximity monitor."""

from thesis_core import proximity_monitor_node


def test_dedicated_tf_listener_uses_its_own_node_and_executor(monkeypatch):
    calls = []
    sentinel = object()

    def fake_listener(*args, **kwargs):
        calls.append((args, kwargs))
        return sentinel

    monkeypatch.setattr(
        proximity_monitor_node, 'TransformListener', fake_listener
    )
    buffer = object()

    listener = proximity_monitor_node.create_dedicated_tf_listener(buffer)
    assert listener is sentinel
    assert calls == [((buffer, None), {'spin_thread': True})]
