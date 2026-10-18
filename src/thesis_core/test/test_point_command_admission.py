"""Tests for point-command replay and timestamp admission."""

from types import SimpleNamespace

import pytest

from thesis_core.joint_model import JOINT_NAMES
from thesis_core.safety_supervisor_node import (
    RecentCommandIds,
    SafetySupervisorNode,
    admit_point_command,
)


class _Clock:

    def __init__(self, nanoseconds):
        self._now = SimpleNamespace(nanoseconds=nanoseconds)

    def now(self):
        return self._now


class _Logger:

    def warning(self, _message):
        pass

    def info(self, _message):
        pass


class _Publisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _stamp(seconds, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def _command(command_id, stamp):
    return SimpleNamespace(
        command_id=command_id,
        stamp=stamp,
        joint_names=list(JOINT_NAMES),
        positions=[0.0, 3.141592653589793, 3.141592653589793,
                   0.0, 0.0, 0.0],
        duration=SimpleNamespace(sec=1, nanosec=0),
    )


def _fake_node(now_ns=10_000_000_000):
    publisher = _Publisher()
    decisions = []
    rejections = []
    saturation = SimpleNamespace(
        limiting_joint='', requested_velocity=0.0, limited_velocity=0.0,
    )
    node = SimpleNamespace(
        recent_point_command_ids=RecentCommandIds(),
        pending_point_command_id=None,
        active_execution=None,
        get_clock=lambda: _Clock(now_ns),
        get_parameter=lambda name: SimpleNamespace(
            value=0.15 if name == 'max_jog_message_age_sec' else None,
        ),
        publish_timing=lambda *_args, **_kwargs: None,
        publish_decision=lambda *args: decisions.append(args),
        reject_point_command=lambda *args: rejections.append(args),
        get_logger=lambda: _Logger(),
        validate_velocity=lambda *_args: True,
        bound_candidate_velocity=lambda message, _duration: (
            message, saturation,
        ),
        apply_proximity_policy=lambda message, _duration: message,
        supervised_publisher=publisher,
        last_proximity_decision='ALLOW',
        last_prediction_reason_code='CLEAR',
        last_prediction_speed_scale=1.0,
    )
    return node, publisher, decisions, rejections


def test_fresh_point_command_is_forwarded_once():
    node, publisher, _decisions, rejections = _fake_node()

    SafetySupervisorNode.candidate_command_callback(
        node, _command('point-fresh', _stamp(9, 900_000_000)),
    )

    assert len(publisher.messages) == 1
    assert publisher.messages[0].command_id == 'point-fresh'
    assert node.pending_point_command_id == 'point-fresh'
    assert rejections == []


@pytest.mark.parametrize(
    ('command_id', 'stamp', 'expected_code'),
    (
        ('point-zero', _stamp(0), 'POINT_COMMAND_STAMP_ZERO'),
        ('point-stale', _stamp(9, 800_000_000),
         'POINT_COMMAND_STAMP_STALE'),
        ('point-future', _stamp(10, 60_000_000),
         'POINT_COMMAND_STAMP_FUTURE'),
        ('', _stamp(9, 900_000_000), 'POINT_COMMAND_ID_EMPTY'),
    ),
)
def test_invalid_point_command_is_not_forwarded(
    command_id, stamp, expected_code,
):
    node, publisher, _decisions, rejections = _fake_node()

    SafetySupervisorNode.candidate_command_callback(
        node, _command(command_id, stamp),
    )

    assert publisher.messages == []
    assert len(rejections) == 1
    assert rejections[0][1] == expected_code
    assert expected_code in rejections[0][2]


def test_reused_point_command_is_not_forwarded_twice():
    node, publisher, _decisions, rejections = _fake_node()
    command = _command('point-reused', _stamp(9, 900_000_000))

    SafetySupervisorNode.candidate_command_callback(node, command)
    SafetySupervisorNode.candidate_command_callback(node, command)

    assert len(publisher.messages) == 1
    assert len(rejections) == 1
    assert rejections[0][1] == 'POINT_COMMAND_ID_REUSED'
    assert 'POINT_COMMAND_ID_REUSED' in rejections[0][2]


def test_rejected_active_or_pending_point_publishes_stop():
    stops = []
    decisions = []
    node = SimpleNamespace(
        active_execution=SimpleNamespace(command_id='point-active'),
        pending_point_command_id='point-active',
        publish_decision=lambda *args: decisions.append(args),
        publish_runtime_stop=lambda *args: stops.append(args),
    )

    SafetySupervisorNode.reject_point_command(
        node,
        _command('point-active', _stamp(9, 900_000_000)),
        'POINT_COMMAND_ID_REUSED',
        'duplicate point command',
    )

    assert decisions[0][1] is False
    assert stops == [
        ('point-active', 'POINT_COMMAND_ID_REUSED',
         'duplicate point command'),
    ]


def test_recent_id_window_reserves_rejected_attempts():
    ids = RecentCommandIds()
    accepted, code = admit_point_command(
        10_000_000_000, _stamp(0), 'point-terminal', 0.15, ids,
    )

    assert accepted is False
    assert code == 'POINT_COMMAND_STAMP_ZERO'
    accepted, code = admit_point_command(
        10_000_000_000, _stamp(9, 900_000_000),
        'point-terminal', 0.15, ids,
    )
    assert accepted is False
    assert code == 'POINT_COMMAND_ID_REUSED'
