#!/usr/bin/env python3
"""Emit a repeated, bounded candidate command after supervisor discovery."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import JointCommand


class CandidateSafeStimulus(Node):
    def __init__(self):
        super().__init__('candidate_safe_stimulus')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10,
        )

    def wait_for_supervisor(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() >= 1:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() >= 1

    def publish(self, command_id, repeats, rate_hz):
        message = JointCommand()
        message.command_id = command_id
        message.joint_names = list(JOINT_NAMES)
        message.positions = [0.0, math.pi, math.pi, 0.0, 0.0, 0.0]
        message.duration.sec = 8
        message.duration.nanosec = 0
        for _ in range(repeats):
            message.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)
            time.sleep(1.0 / rate_hz)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--repetition', type=int, required=True)
    parser.add_argument('--repeats', type=int, default=8)
    parser.add_argument('--rate-hz', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    if args.repeats < 1 or args.rate_hz <= 0.0:
        raise SystemExit('repeats debe ser >= 1 y rate-hz debe ser positivo.')
    rclpy.init(args=ros_args)
    node = CandidateSafeStimulus()
    try:
        if not node.wait_for_supervisor(5.0):
            raise RuntimeError('No se descubrió el suscriptor del supervisor.')
        command_id = f'{args.scenario.lower()}-r{args.repetition}-{time.monotonic_ns()}'
        node.publish(command_id, args.repeats, args.rate_hz)
        print('COMMAND_ID=' + command_id)
        print('MENSAJES_CANDIDATE=' + str(args.repeats))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
