#!/usr/bin/env python3
"""Request a bounded J4 jog deliberately above the operational Vmax."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
)
from thesis_interfaces.msg import JointCommand


JOINT_INDEX = 3
DEFAULT_DELTA_RAD = 0.50
DEFAULT_DURATION_SEC = 1.0


class E13VelocityStimulus(Node):
    """Publish a correlated JOG stream from the current measured state."""

    def __init__(self):
        super().__init__('e13_velocity_saturation_stimulus')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.positions = None
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 20,
        )
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', 10,
        )

    def _joint_cb(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self.positions = [float(values[name]) for name in JOINT_NAMES]

    def wait_until_ready(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.positions is not None
                    and self.publisher.get_subscription_count() >= 1):
                return True
        return False

    def target(self, requested_direction, delta_rad):
        current = list(self.positions)
        joint_name = JOINT_NAMES[JOINT_INDEX]
        lower, upper = JOINT_POSITION_LIMITS[joint_name]
        direction = 1.0 if requested_direction >= 0 else -1.0
        candidate = current[JOINT_INDEX] + direction * delta_rad
        if not lower <= candidate <= upper:
            direction *= -1.0
            candidate = current[JOINT_INDEX] + direction * delta_rad
        if not lower <= candidate <= upper:
            raise RuntimeError('No existe dirección segura dentro de límites.')
        current[JOINT_INDEX] = candidate
        return current, direction

    def publish_stream(
        self, command_id, target, duration_sec, stream_sec, rate_hz,
    ):
        message = JointCommand()
        message.command_id = command_id
        message.joint_names = list(JOINT_NAMES)
        message.positions = list(target)
        whole = int(duration_sec)
        message.duration.sec = whole
        message.duration.nanosec = int(
            round((duration_sec - whole) * 1_000_000_000)
        )
        interval = 1.0 / rate_hz
        deadline = time.monotonic() + stream_sec
        next_publish = time.monotonic()
        count = 0
        while rclpy.ok() and time.monotonic() < deadline:
            message.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)
            count += 1
            rclpy.spin_once(self, timeout_sec=0.0)
            next_publish += interval
            remaining = next_publish - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repetition', type=int, required=True)
    parser.add_argument('--direction', type=int, choices=(-1, 1), required=True)
    parser.add_argument('--delta-rad', type=float, default=DEFAULT_DELTA_RAD)
    parser.add_argument('--duration-sec', type=float, default=DEFAULT_DURATION_SEC)
    parser.add_argument('--stream-sec', type=float, default=1.2)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    args, ros_args = parser.parse_known_args()
    if not 0.45 <= args.delta_rad <= 0.60:
        raise SystemExit('delta-rad debe estar entre 0.45 y 0.60 rad.')
    if not 0.8 <= args.duration_sec <= 1.2:
        raise SystemExit('duration-sec debe estar entre 0.8 y 1.2 s.')
    if args.stream_sec <= 0.0 or args.rate_hz <= 0.0:
        raise SystemExit('stream-sec y rate-hz deben ser positivos.')

    rclpy.init(args=ros_args)
    node = E13VelocityStimulus()
    try:
        if not node.wait_until_ready(5.0):
            raise RuntimeError('No se obtuvo estado o suscriptor JOG.')
        target, direction = node.target(args.direction, args.delta_rad)
        command_id = f'e13-r{args.repetition}-{time.monotonic_ns()}'
        count = node.publish_stream(
            command_id, target, args.duration_sec,
            args.stream_sec, args.rate_hz,
        )
        requested_velocity = args.delta_rad / args.duration_sec
        limit = JOINT_VELOCITY_LIMITS[JOINT_NAMES[JOINT_INDEX]]
        print('COMMAND_ID=' + command_id)
        print('JOINT=' + JOINT_NAMES[JOINT_INDEX])
        print(f'DIRECTION={int(direction):+d}')
        print(f'DELTA_RAD={args.delta_rad:.6f}')
        print(f'VELOCIDAD_SOLICITADA_RAD_S={requested_velocity:.6f}')
        print(f'VELOCIDAD_LIMITE_RAD_S={limit:.6f}')
        print(f'SATURACION_SOLICITADA={str(requested_velocity > limit).lower()}')
        print('MENSAJES_JOG=' + str(count))
        print('TARGET=' + ','.join(f'{value:.6f}' for value in target))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
