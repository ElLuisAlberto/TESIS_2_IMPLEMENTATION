#!/usr/bin/env python3
"""Emit a bounded JOG stream, then intentionally let it expire for E11."""

import argparse
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import JointCommand


class E11WatchdogStimulus(Node):
    """Publish a small state-relative JOG without using the GUI."""

    def __init__(self) -> None:
        super().__init__('e11_watchdog_stimulus')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.positions: Optional[dict[str, float]] = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            20,
        )
        self.publisher = self.create_publisher(
            JointCommand,
            '/thesis/jog_intent',
            10,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        values = {
            name: float(position)
            for name, position in zip(message.name, message.position)
            if name in JOINT_NAMES
        }
        if all(name in values for name in JOINT_NAMES):
            self.positions = values

    def wait_for_state(self, timeout_sec: float) -> bool:
        """Wait only for a complete measured joint configuration."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.positions is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.positions is not None

    def publish_stream(
        self,
        command_id: str,
        joint_index: int,
        delta_rad: float,
        stream_sec: float,
        rate_hz: float,
    ) -> tuple[int, tuple[float, ...]]:
        """Publish one safe state-relative target repeatedly for E11."""
        if self.positions is None:
            raise RuntimeError('No hay estado articular completo.')
        if not 0 <= joint_index < len(JOINT_NAMES):
            raise ValueError('joint_index fuera del rango [0, 5].')
        if not 0.001 <= delta_rad <= 0.05:
            raise ValueError('delta_rad debe estar en [0.001, 0.05].')
        if stream_sec <= 0.0 or rate_hz <= 0.0:
            raise ValueError('stream_sec y rate_hz deben ser positivos.')

        target = [self.positions[name] for name in JOINT_NAMES]
        direction = -1.0 if target[joint_index] > 0.0 else 1.0
        target[joint_index] += direction * delta_rad
        interval = 1.0 / rate_hz
        deadline = time.monotonic() + stream_sec
        next_publish = time.monotonic()
        count = 0

        while rclpy.ok() and time.monotonic() < deadline:
            message = JointCommand()
            message.stamp = self.get_clock().now().to_msg()
            message.command_id = command_id
            message.joint_names = list(JOINT_NAMES)
            message.positions = target
            message.duration.sec = 1
            message.duration.nanosec = 0
            self.publisher.publish(message)
            count += 1
            next_publish += interval
            remaining = next_publish - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

        return count, tuple(target)


def main() -> None:
    """Run one reproducible E11 stimulus and exit without publishing a HOLD."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--repetition', type=int, required=True)
    parser.add_argument('--joint-index', type=int, default=0)
    parser.add_argument('--delta-rad', type=float, default=0.03)
    parser.add_argument('--stream-sec', type=float, default=1.0)
    parser.add_argument('--rate-hz', type=float, default=20.0)
    parser.add_argument('--state-timeout-sec', type=float, default=5.0)
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = E11WatchdogStimulus()
    try:
        if not node.wait_for_state(parsed.state_timeout_sec):
            raise RuntimeError('No llegó /joint_states completo a tiempo.')
        command_id = f'e11-r{parsed.repetition}-{time.monotonic_ns()}'
        count, target = node.publish_stream(
            command_id,
            parsed.joint_index,
            parsed.delta_rad,
            parsed.stream_sec,
            parsed.rate_hz,
        )
        print('COMMAND_ID=' + command_id)
        print('MENSAJES_JOG=' + str(count))
        print('TARGET=' + ','.join(f'{value:.6f}' for value in target))
        print('FLUJO_FINALIZADO=watchdog_debe_publicar_hold')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
