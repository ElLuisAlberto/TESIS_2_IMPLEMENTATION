#!/usr/bin/env python3
"""Record changes published by the system readiness monitor."""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ReadinessTransitionRecorder(Node):
    """Write readiness status transitions with wall and monotonic time."""

    def __init__(self, output_path, duration_sec):
        super().__init__('readiness_transition_recorder')
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output_path.open(
            'w', newline='', encoding='utf-8'
        )
        self.writer = csv.DictWriter(
            self.stream,
            fieldnames=(
                'wall_time_unix_sec',
                'monotonic_sec',
                'ready_bool',
                'readiness_status',
            ),
        )
        self.writer.writeheader()
        self.stream.flush()
        self.duration_sec = float(duration_sec)
        self.started_at = time.monotonic()
        self.last_status = None
        self.ready_bool = None
        self.status_count = 0
        self.ready_count = 0
        self.waiting_count = 0
        self.create_subscription(
            Bool, '/thesis/system_ready', self.ready_callback, 10
        )
        self.create_subscription(
            String, '/thesis/system_readiness', self.status_callback, 10
        )
        self.create_timer(0.1, self.check_duration)
        print(
            f'READINESS_RECORDER_READY=1 output={self.output_path}',
            flush=True,
        )

    def ready_callback(self, message):
        """Store the boolean readiness sample for the next status change."""
        self.ready_bool = bool(message.data)

    def status_callback(self, message):
        """Write only status transitions, avoiding redundant samples."""
        status = str(message.data)
        if status == self.last_status:
            return
        self.last_status = status
        self.status_count += 1
        is_ready = status == 'READY'
        if is_ready:
            self.ready_count += 1
        elif status.startswith('WAITING'):
            self.waiting_count += 1
        self.writer.writerow({
            'wall_time_unix_sec': f'{time.time():.6f}',
            'monotonic_sec': f'{time.monotonic():.6f}',
            'ready_bool': (
                '' if self.ready_bool is None else str(self.ready_bool).lower()
            ),
            'readiness_status': status,
        })
        self.stream.flush()
        print(f'READINESS_TRANSITION={status}', flush=True)

    def check_duration(self):
        """Stop after the requested wall-time interval."""
        if time.monotonic() - self.started_at >= self.duration_sec:
            print(f'READINESS_CSV={self.output_path}', flush=True)
            print(f'READY_TRANSITIONS={self.ready_count}', flush=True)
            print(f'WAITING_TRANSITIONS={self.waiting_count}', flush=True)
            print(f'TOTAL_TRANSITIONS={self.status_count}', flush=True)
            rclpy.shutdown()

    def close(self):
        """Flush and close the evidence file."""
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


def main():
    """Run a wall-clock recorder independent of simulation time."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration-sec', type=float, default=150.0)
    parser.add_argument('--output', required=True)
    args, ros_args = parser.parse_known_args()
    if args.duration_sec <= 0.0:
        raise SystemExit('--duration-sec debe ser mayor que cero.')
    rclpy.init(args=ros_args)
    node = ReadinessTransitionRecorder(args.output, args.duration_sec)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
