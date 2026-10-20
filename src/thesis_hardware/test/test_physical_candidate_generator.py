from types import SimpleNamespace

from builtin_interfaces.msg import Time
from thesis_core.safety_supervisor_node import (
    RecentCommandIds,
    admit_point_command,
)
from thesis_hardware.physical_candidate_generator import (
    publish_fresh_candidate,
)


class Publisher:
    def __init__(self):
        self.messages = []
        self.ack_waits = 0

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)

    def wait_for_all_acked(self, timeout):
        assert timeout.nanoseconds == 1_000_000_000
        self.ack_waits += 1
        return True


def test_old_point_stamp_remains_rejected_by_supervisor_contract():
    accepted, reason = admit_point_command(
        10_000_000_000, Time(sec=9, nanosec=800_000_000),
        'old-candidate', 0.15, RecentCommandIds(),
    )
    assert not accepted
    assert reason == 'POINT_COMMAND_STAMP_STALE'


def test_fresh_candidate_is_stamped_at_publish_and_accepted():
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=10_000_000_000,
            to_msg=lambda: Time(sec=10, nanosec=0),
        )
    )
    node = SimpleNamespace(get_clock=lambda: clock)
    publisher = Publisher()
    message = publish_fresh_candidate(
        node, publisher, [0.0] * 6, 1.0, 'fresh-candidate'
    )
    accepted, reason = admit_point_command(
        10_000_000_000, message.stamp, message.command_id, 0.15,
        RecentCommandIds(),
    )
    assert accepted
    assert reason == ''
    assert publisher.messages == [message]
    assert publisher.ack_waits == 1
