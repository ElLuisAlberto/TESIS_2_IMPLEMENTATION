#!/usr/bin/env python3
"""Reliably publish one invalid candidate command for E14."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import JointCommand


class CandidateRejectionStimulus(Node):
    """Publish the same invalid command repeatedly after DDS discovery."""

    def __init__(self) -> None:
        super().__init__('e14_candidate_rejection_stimulus')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10,
        )

    def wait_for_supervisor(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() >= 1:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() >= 1

    def publish_invalid(self, command_id: str, repeats: int, rate_hz: float) -> None:
        message = JointCommand()
        message.command_id = command_id
        message.joint_names = list(JOINT_NAMES)
        message.positions = [99.0, 3.141592653589793, 3.141592653589793,
                             0.0, 0.0, 0.0]
        message.duration.sec = 3
        message.duration.nanosec = 0
        interval = 1.0 / rate_hz
        for _ in range(repeats):
            message.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)
            time.sleep(interval)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--repetition', type=int, required=True)
    parser.add_argument('--repeats', type=int, default=8)
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument('--discovery-timeout-sec', type=float, default=5.0)
    parsed, ros_args = parser.parse_known_args()
    if parsed.repeats < 1 or parsed.rate_hz <= 0.0:
        raise SystemExit('repeats debe ser >= 1 y rate-hz debe ser positivo.')
    rclpy.init(args=ros_args)
    node = CandidateRejectionStimulus()
    try:
        if not node.wait_for_supervisor(parsed.discovery_timeout_sec):
            raise RuntimeError('No se descubrió el suscriptor del supervisor.')
        command_id = f'e14-r{parsed.repetition}-{time.monotonic_ns()}'
        node.publish_invalid(command_id, parsed.repeats, parsed.rate_hz)
        print('COMMAND_ID=' + command_id)
        print('MENSAJES_CANDIDATE=' + str(parsed.repeats))
        print('FLUJO_FINALIZADO=debe_rechazar_limite_articular')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
