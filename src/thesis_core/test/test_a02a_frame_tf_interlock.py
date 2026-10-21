"""Direct fail-closed tests for the A-02a frame and time interlock."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from tf2_ros import TransformException

from thesis_core.proximity_monitor_node import ProximityMonitorNode
from thesis_core.safety_supervisor_node import (
    SAFETY_REFERENCE_FRAME,
    SafetySupervisorNode,
    kinematic_tf_position_error,
    newest_synchronized_kinematic_sample,
    proximity_reference_frame_is_valid,
)
from thesis_core.jaco_kinematics import end_effector_transform, translation


class _Logger:

    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)

    def info(self, _message):
        pass


class _Publisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _stamp(seconds, nanoseconds=0):
    return TimeMessage(sec=seconds, nanosec=nanoseconds)


def _supervisor_node():
    now = SimpleNamespace(nanoseconds=10_000_000_000)
    return SimpleNamespace(
        proximity_status=SimpleNamespace(state='ALLOW'),
        last_proximity_receive_ns=1,
        last_proximity_arrival_ns=None,
        last_proximity_rejection_reason=None,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        get_parameter=lambda _name: SimpleNamespace(value=0.5),
        get_logger=lambda: _Logger(),
    )


def _proximity(frame=SAFETY_REFERENCE_FRAME, state='ALLOW'):
    return SimpleNamespace(
        stamp=_stamp(9, 900_000_000),
        reference_frame=frame,
        state=state,
        minimum_clearance=0.20,
        limiting_segment='segment',
    )


def test_only_world_is_an_accepted_proximity_reference_frame():
    assert proximity_reference_frame_is_valid('world')
    assert not proximity_reference_frame_is_valid('')
    assert not proximity_reference_frame_is_valid('map')


def test_fk_tf_position_error_detects_offset_above_tolerance():
    joints = (0.0, 2.8, 3.4, 0.0, 0.2, 0.0)
    model_position = translation(end_effector_transform(joints))
    tf_position = (model_position[0] + 0.006, *model_position[1:])

    assert kinematic_tf_position_error(joints, model_position) == 0.0
    assert kinematic_tf_position_error(joints, tf_position) > 0.005


def test_fk_tf_uses_newest_exact_transform_already_distributed():
    newest_stamp = 10_000_000_000
    previous_stamp = 9_950_000_000
    joints = (0.0, 2.8, 3.4, 0.0, 0.2, 0.0)
    position = translation(end_effector_transform(joints))

    class DelayedTransformBuffer:
        def lookup_transform(self, _target, _source, requested):
            if requested.nanoseconds == newest_stamp:
                raise TransformException('newest TF is still in transit')
            assert requested.nanoseconds == previous_stamp
            return _transform(*position)

    stamp, selected_joints, tf_position = (
        newest_synchronized_kinematic_sample(
            ((previous_stamp, joints), (newest_stamp, joints)),
            DelayedTransformBuffer(),
            newest_stamp,
            0.2,
        )
    )

    assert stamp == previous_stamp
    assert selected_joints == joints
    assert tf_position == pytest.approx(position)


@pytest.mark.parametrize('frame', ('', 'map'))
def test_invalid_proximity_frame_becomes_unavailable(frame):
    node = _supervisor_node()

    SafetySupervisorNode.proximity_status_callback(node, _proximity(frame))

    assert node.proximity_status is None
    assert node.last_proximity_receive_ns is None
    assert node.last_proximity_rejection_reason == 'PROXIMITY_FRAME_INVALID'


def test_valid_world_proximity_is_retained():
    node = _supervisor_node()
    message = _proximity()

    SafetySupervisorNode.proximity_status_callback(node, message)

    assert node.proximity_status is message
    assert node.last_proximity_rejection_reason is None


def test_upstream_stop_precedes_prediction():
    decisions = []
    prediction_called = []
    node = SimpleNamespace(
        proximity_status=_proximity(state='STOP'),
        last_proximity_receive_ns=9_900_000_000,
        last_proximity_decision='ALLOW',
        last_prediction_reason_code='CLEAR',
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)
        ),
        get_parameter=lambda name: SimpleNamespace(
            value=0.5 if name == 'max_proximity_age_sec' else None
        ),
        publish_decision=lambda *args: decisions.append(args),
        predict_candidate=lambda _message: prediction_called.append(True),
        get_logger=lambda: _Logger(),
    )
    command = SimpleNamespace(command_id='upstream-stop')

    result = SafetySupervisorNode.apply_proximity_policy(node, command, 1.0)

    assert result is None
    assert prediction_called == []
    assert node.last_proximity_decision == 'STOP'
    assert decisions[0][2:4] == ('STOP', 'UPSTREAM_PROXIMITY_STOP')


class _TransformBuffer:

    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.calls = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.error is not None:
            raise self.error
        return self.result


def _transform(x=1.0, y=2.0, z=3.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
        )
    )


def test_tf_lookup_uses_the_acquisition_stamp():
    buffer = _TransformBuffer(result=_transform())
    node = SimpleNamespace(reference_frame='world', tf_buffer=buffer)
    stamp = _stamp(7, 123)

    point = ProximityMonitorNode._frame_point(
        node, 'link_1', stamp, Duration(seconds=0.04)
    )

    assert (point.x, point.y, point.z) == (1.0, 2.0, 3.0)
    target, source, requested_time = buffer.calls[0][0]
    assert (target, source) == ('world', 'link_1')
    assert requested_time.nanoseconds == 7_000_000_123
    assert buffer.calls[0][1]['timeout'].nanoseconds == 40_000_000


@pytest.mark.parametrize('error', (
    TransformException('TF absent'),
    TransformException('TF stale'),
    TransformException('TF transform failed'),
))
def test_missing_stale_or_failed_tf_is_not_converted_to_a_point(error):
    node = SimpleNamespace(
        reference_frame='world',
        tf_buffer=_TransformBuffer(error=error),
    )

    with pytest.raises(TransformException):
        ProximityMonitorNode._frame_point(
            node, 'link_1', _stamp(7), Duration(seconds=0.04)
        )


def test_tf_failure_publishes_stop_not_allow():
    publisher = _Publisher()
    node = SimpleNamespace(
        get_parameter=lambda _name: SimpleNamespace(value=True),
        _obstacle=lambda: (
            Point(x=1.0, y=0.0, z=0.0), 0.1, (0.0, 0.0, 0.0), _stamp(7),
        ),
        _publish_obstacle_marker=lambda *_args: None,
        start_frames=['start'],
        end_frames=['end'],
        tf_lookup_timeout_sec=0.04,
        _frame_point=lambda *_args: (_ for _ in ()).throw(
            TransformException('TF absent')
        ),
        missing_frames=set(),
        segment_names=['segment'],
        radii=[0.1],
        reference_frame='world',
        publisher=publisher,
        previous_state=None,
        get_logger=lambda: _Logger(),
        _publish_tf_stop=lambda *args: ProximityMonitorNode._publish_tf_stop(
            node, *args
        ),
    )

    ProximityMonitorNode.evaluate(node)

    assert len(publisher.messages) == 1
    assert publisher.messages[0].state == 'STOP'
    assert publisher.messages[0].limiting_segment == 'TF_UNAVAILABLE'
