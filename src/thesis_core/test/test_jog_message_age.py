"""Tests for continuous-command freshness calculations."""

import math
from types import SimpleNamespace

from thesis_core.safety_supervisor_node import message_age_seconds


def stamp(seconds, nanoseconds=0):
    """Build the timestamp shape used by ROS messages."""
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def test_message_age_uses_ros_nanoseconds():
    assert math.isclose(
        message_age_seconds(2_250_000_000, stamp(2, 50_000_000)),
        0.20,
    )


def test_zero_stamp_is_never_fresh():
    assert math.isinf(message_age_seconds(2_000_000_000, stamp(0)))


def test_future_stamp_returns_negative_age():
    assert math.isclose(
        message_age_seconds(2_000_000_000, stamp(2, 20_000_000)),
        -0.02,
    )
