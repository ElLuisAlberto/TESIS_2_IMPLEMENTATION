"""Generate one physical candidate only after transport is ready."""

import math
import time

from rclpy.duration import Duration
from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import JointCommand


def wait_for_supervisor(publisher, timeout_sec=5.0):
    """Wait for DDS discovery before a candidate is constructed."""
    deadline = time.monotonic() + float(timeout_sec)
    while time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        time.sleep(0.02)
    return publisher.get_subscription_count() > 0


def publish_fresh_candidate(node, publisher, positions, duration_sec,
                            command_id):
    """Stamp and publish a complete candidate with no work in between."""
    if not wait_for_supervisor(publisher):
        raise RuntimeError('safety supervisor subscription is unavailable')
    if len(positions) != len(JOINT_NAMES):
        raise ValueError('the candidate must contain six arm positions')
    if not all(math.isfinite(float(value)) for value in positions):
        raise ValueError('candidate positions must be finite')
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError('candidate duration must be positive and finite')

    # This is deliberately the final operation before publish.  In particular,
    # no service, dialog, discovery wait or calculation occurs after this read.
    now = node.get_clock().now()
    message = JointCommand()
    message.stamp = now.to_msg()
    message.command_id = str(command_id)
    message.joint_names = list(JOINT_NAMES)
    message.positions = [float(value) for value in positions]
    seconds = int(duration_sec)
    message.duration.sec = seconds
    message.duration.nanosec = int((duration_sec - seconds) * 1e9)
    publisher.publish(message)
    # Waiting happens strictly after publish, so it cannot age the command
    # before supervisor admission.  It keeps short-lived validation nodes
    # alive until every reliable subscriber (including rosbag) acknowledges
    # the sample instead of losing evidence when the publisher is destroyed.
    wait_for_acks = getattr(publisher, 'wait_for_all_acked', None)
    if wait_for_acks is not None:
        wait_for_acks(timeout=Duration(seconds=1.0))
    return message
