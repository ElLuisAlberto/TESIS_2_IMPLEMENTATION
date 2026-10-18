"""Bounded exact-stamp TF lookup tests for the proximity monitor."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from tf2_ros import TransformException

from thesis_core.proximity_monitor_node import ProximityMonitorNode


def _stamp(seconds, nanoseconds=0):
    return TimeMessage(sec=seconds, nanosec=nanoseconds)


def _transform(x=1.0, y=2.0, z=3.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
        )
    )


class _ExactStampBuffer:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def lookup_transform(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


def test_exact_tf_available_immediately_returns_a_point():
    buffer = _ExactStampBuffer(_transform())
    node = SimpleNamespace(reference_frame='world', tf_buffer=buffer)
    source_stamp = _stamp(4, 123)

    point = ProximityMonitorNode._frame_point(
        node, 'link_1', source_stamp, Duration(seconds=0.04)
    )

    assert (point.x, point.y, point.z) == (1.0, 2.0, 3.0)
    args, kwargs = buffer.calls[0]
    assert args[2].nanoseconds == 4_000_000_123
    assert kwargs['timeout'].nanoseconds == 40_000_000


def test_exact_tf_arriving_within_bounded_timeout_is_accepted():
    class _DelayedExactBuffer(_ExactStampBuffer):
        def lookup_transform(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            assert kwargs['timeout'].nanoseconds > 0
            return _transform()

    buffer = _DelayedExactBuffer(None)
    node = SimpleNamespace(reference_frame='world', tf_buffer=buffer)
    source_stamp = _stamp(5)

    ProximityMonitorNode._frame_point(
        node, 'link_1', source_stamp, Duration(seconds=0.04)
    )

    args, _kwargs = buffer.calls[0]
    assert args[2].nanoseconds == 5_000_000_000


def test_missing_exact_tf_does_not_fall_back_to_latest():
    class _LatestOnlyBuffer(_ExactStampBuffer):
        def lookup_transform(self, *args, **kwargs):
            self.calls.append((args, kwargs))
            requested_time = args[2]
            assert requested_time.nanoseconds == 6_000_000_000
            raise TransformException('exact timestamp unavailable; latest exists')

    buffer = _LatestOnlyBuffer(None)
    node = SimpleNamespace(reference_frame='world', tf_buffer=buffer)

    try:
        ProximityMonitorNode._frame_point(
            node, 'link_1', _stamp(6), Duration(seconds=0.04)
        )
    except TransformException:
        pass
    else:
        raise AssertionError('must reject a latest-only transform')


def test_tf_timeout_publishes_the_existing_fail_closed_stop():
    messages = []
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
            TransformException('timed out waiting for exact TF')
        ),
        missing_frames=set(),
        segment_names=['segment'],
        radii=[0.1],
        reference_frame='world',
        publisher=SimpleNamespace(publish=messages.append),
        previous_state=None,
        get_logger=lambda: SimpleNamespace(warning=lambda _msg: None,
                                           info=lambda _msg: None),
        _publish_tf_stop=lambda *args: ProximityMonitorNode._publish_tf_stop(
            node, *args
        ),
    )

    ProximityMonitorNode.evaluate(node)

    assert len(messages) == 1
    assert messages[0].state == 'STOP'
    assert messages[0].limiting_segment == 'TF_UNAVAILABLE'
