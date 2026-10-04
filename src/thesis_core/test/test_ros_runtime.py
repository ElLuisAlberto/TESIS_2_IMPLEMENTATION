"""Tests for clean ROS 2 Python node shutdown."""

from unittest.mock import Mock

import pytest
from rclpy.executors import ExternalShutdownException

from thesis_core import ros_runtime


def test_normal_exit_destroys_node_and_shuts_context(monkeypatch):
    """A normal executor return must release both node and context."""
    node = Mock()
    shutdown = Mock()
    monkeypatch.setattr(ros_runtime.rclpy, 'spin', Mock())
    monkeypatch.setattr(ros_runtime.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(ros_runtime.rclpy, 'shutdown', shutdown)

    ros_runtime.spin_node(node)

    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()


def test_external_shutdown_is_not_reported_as_failure(monkeypatch):
    """A launch-requested context shutdown must terminate successfully."""
    node = Mock()
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'spin',
        Mock(side_effect=ExternalShutdownException()),
    )
    monkeypatch.setattr(ros_runtime.rclpy, 'ok', lambda: False)

    ros_runtime.spin_node(node)

    node.destroy_node.assert_called_once_with()


def test_live_context_exception_is_preserved(monkeypatch):
    """An unexpected failure must still propagate while ROS is active."""
    node = Mock()
    shutdown = Mock()
    monkeypatch.setattr(
        ros_runtime.rclpy,
        'spin',
        Mock(side_effect=RuntimeError('unexpected')),
    )
    monkeypatch.setattr(ros_runtime.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(ros_runtime.rclpy, 'shutdown', shutdown)

    with pytest.raises(RuntimeError, match='unexpected'):
        ros_runtime.spin_node(node)

    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
