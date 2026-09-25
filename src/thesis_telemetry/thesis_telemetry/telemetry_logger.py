"""Correlate safety telemetry and export complete CSV records in real time."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    ExecutionControl,
    JointCommand,
    PipelineTiming,
    ProximityStatus,
)

from thesis_core.joint_model import JOINT_NAMES, shortest_joint_delta
from thesis_telemetry.telemetry_math import (
    RunningStats,
    ZeroVelocityDetector,
    stamp_to_ns,
    vector_text,
)


NO_TIME = -1.0


@dataclass
class Trace:
    """Timing events for one exact command intention."""

    command_id: str
    intent_stamp_ns: int
    stages: dict[str, PipelineTiming] = field(default_factory=dict)
    latency_written: bool = False


class TelemetryLogger(Node):
    """Observe the pipeline without participating in safety decisions."""

    SAMPLE_FIELDS = (
        'timestamp_ros_sec', 'command_id', 'intent_stamp_ros_sec',
        'q_actual', 'q_objetivo', 'qdot_solicitada', 'qdot_medida',
        'd_actual', 'd_nominal', 'd_supervisada', 'TTC_nominal',
        't_min', 'state', 'speed_scale', 'limiting_segment',
        'reason_code', 'joint_state_age_sec', 'proximity_age_sec',
        'missing_messages', 'instrumentation_cost_us',
    )
    LATENCY_FIELDS = (
        'timestamp_ros_sec', 'command_id', 'intent_stamp_ros_sec',
        'tau_prediction_sec', 'tau_supervisor_sec',
        'tau_end_to_end_sec', 'tau_stop_sec',
        'instrumentation_cost_us', 'event',
    )
    SUMMARY_FIELDS = (
        'metric', 'count', 'mean_sec', 'stddev_sec',
        'minimum_sec', 'maximum_sec', 'percentile95_sec',
    )

    def __init__(self) -> None:
        super().__init__('telemetry_logger')
        self.declare_parameter('output_directory', '~/thesis_telemetry')
        self.declare_parameter('velocity_epsilon_rad_s', 0.01)
        self.declare_parameter('zero_velocity_required_samples', 3)
        self.declare_parameter('flush_every_rows', 1)

        root = Path(str(self.get_parameter(
            'output_directory'
        ).value)).expanduser()
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.run_directory = root / run_id
        self.run_directory.mkdir(parents=True, exist_ok=False)

        self.samples_handle, self.samples_writer = self._open_csv(
            'telemetry_samples.csv', self.SAMPLE_FIELDS
        )
        self.latency_handle, self.latency_writer = self._open_csv(
            'latency_metrics.csv', self.LATENCY_FIELDS
        )
        self.summary_path = self.run_directory / 'statistics.csv'

        self.traces: dict[tuple[str, int], Trace] = {}
        self.latest_key_by_command: dict[str, tuple[str, int]] = {}
        self.targets: dict[str, tuple[tuple[float, ...], float, int]] = {}
        self.current_positions: tuple[float, ...] | None = None
        self.current_velocities: tuple[float, ...] | None = None
        self.last_joint_stamp_ns: int | None = None
        self.last_proximity_stamp_ns: int | None = None
        self.last_sequence: dict[str, int] = {}
        self.missing_messages = 0
        self.rows_since_flush = 0
        self.pending_stop: dict[str, tuple[int, int]] = {}

        epsilon = float(self.get_parameter(
            'velocity_epsilon_rad_s'
        ).value)
        required = int(self.get_parameter(
            'zero_velocity_required_samples'
        ).value)
        self.zero_detectors: dict[str, ZeroVelocityDetector] = {}
        self.zero_epsilon = epsilon
        self.zero_required = required
        ZeroVelocityDetector(epsilon, required)

        self.stats = {
            'tau_prediction': RunningStats(),
            'tau_supervisor': RunningStats(),
            'tau_end_to_end': RunningStats(),
            'tau_stop': RunningStats(),
            'instrumentation_cost': RunningStats(),
        }

        self.create_subscription(
            PipelineTiming,
            '/thesis/pipeline_timing',
            self.timing_callback,
            100,
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 50
        )
        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_callback,
            20,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self.target_callback,
            20,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.target_callback,
            20,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.control_callback,
            50,
        )
        self.summary_timer = self.create_timer(1.0, self.write_summary)
        self.get_logger().info(
            f'Telemetría CSV activa en {self.run_directory}'
        )

    def _open_csv(self, name, fields):
        handle = (self.run_directory / name).open(
            'w', newline='', encoding='utf-8'
        )
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        handle.flush()
        return handle, writer

    def now_ns(self) -> int:
        """Return current ROS time without mixing it with monotonic time."""
        return self.get_clock().now().nanoseconds

    def timing_callback(self, msg: PipelineTiming) -> None:
        callback_start = time.perf_counter_ns()
        intent_ns = stamp_to_ns(msg.intent_stamp)
        key = (msg.command_id, intent_ns)
        trace = self.traces.setdefault(
            key, Trace(msg.command_id, intent_ns)
        )
        trace.stages[msg.stage] = msg
        self.latest_key_by_command[msg.command_id] = key

        previous = self.last_sequence.get(msg.source)
        if previous is not None and msg.sequence > previous + 1:
            self.missing_messages += int(msg.sequence - previous - 1)
        if previous is None or msg.sequence > previous:
            self.last_sequence[msg.source] = int(msg.sequence)

        if 'CONTROLLER_PUBLISH' in trace.stages:
            self._write_pipeline_latency(trace, callback_start)
        if msg.stage == 'STOP_DETECTED':
            self.pending_stop[msg.command_id] = (
                stamp_to_ns(msg.stamp), intent_ns
            )
            detector = self.zero_detectors.setdefault(
                msg.command_id,
                ZeroVelocityDetector(self.zero_epsilon, self.zero_required),
            )
            detector.reset()
        self._record_cost(callback_start)

    @staticmethod
    def _monotonic_delta(trace: Trace, start: str, end: str) -> float:
        first = trace.stages.get(start)
        last = trace.stages.get(end)
        if first is None or last is None or first.source != last.source:
            return NO_TIME
        delta = int(last.monotonic_ns) - int(first.monotonic_ns)
        return delta / 1e9 if delta >= 0 else NO_TIME

    def _write_pipeline_latency(self, trace, callback_start):
        if trace.latency_written:
            return
        controller = trace.stages.get('CONTROLLER_PUBLISH')
        required = {
            'SUPERVISOR_RECEIVE',
            'PREDICTION_START',
            'PREDICTION_END',
            'SUPERVISED_PUBLISH',
            'ADAPTER_RECEIVE',
            'CONTROLLER_PUBLISH',
        }
        if controller is None or not required.issubset(trace.stages):
            return
        tau_prediction = self._monotonic_delta(
            trace, 'PREDICTION_START', 'PREDICTION_END'
        )
        tau_supervisor = self._monotonic_delta(
            trace, 'SUPERVISOR_RECEIVE', 'SUPERVISED_PUBLISH'
        )
        controller_ns = stamp_to_ns(controller.stamp)
        tau_end_to_end = (
            (controller_ns - trace.intent_stamp_ns) / 1e9
            if controller_ns >= trace.intent_stamp_ns else NO_TIME
        )
        cost_us = (time.perf_counter_ns() - callback_start) / 1000.0
        self._write_latency_row(
            trace.command_id,
            trace.intent_stamp_ns,
            tau_prediction,
            tau_supervisor,
            tau_end_to_end,
            NO_TIME,
            cost_us,
            'CONTROLLER_PUBLISH',
        )
        trace.latency_written = True

    def _write_latency_row(
        self, command_id, intent_ns, prediction, supervisor,
        end_to_end, stop, cost_us, event,
    ):
        self.latency_writer.writerow({
            'timestamp_ros_sec': f'{self.now_ns() / 1e9:.9f}',
            'command_id': command_id or 'UNAVAILABLE',
            'intent_stamp_ros_sec': f'{intent_ns / 1e9:.9f}',
            'tau_prediction_sec': f'{prediction:.9f}',
            'tau_supervisor_sec': f'{supervisor:.9f}',
            'tau_end_to_end_sec': f'{end_to_end:.9f}',
            'tau_stop_sec': f'{stop:.9f}',
            'instrumentation_cost_us': f'{cost_us:.3f}',
            'event': event,
        })
        self.stats['tau_prediction'].add(prediction)
        self.stats['tau_supervisor'].add(supervisor)
        self.stats['tau_end_to_end'].add(end_to_end)
        self.stats['tau_stop'].add(stop)
        self._flush_if_required()

    def joint_callback(self, msg: JointState) -> None:
        callback_start = time.perf_counter_ns()
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if all(name in positions for name in JOINT_NAMES):
            values = tuple(float(positions[name]) for name in JOINT_NAMES)
            if all(math.isfinite(value) for value in values):
                self.current_positions = values
        if all(name in velocities for name in JOINT_NAMES):
            values = tuple(float(velocities[name]) for name in JOINT_NAMES)
            if all(math.isfinite(value) for value in values):
                self.current_velocities = values
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if self.last_joint_stamp_ns is not None:
            gap = stamp_ns - self.last_joint_stamp_ns
            if gap > 50_000_000:
                self.missing_messages += max(1, round(gap / 10_000_000) - 1)
        self.last_joint_stamp_ns = stamp_ns

        if self.current_velocities is not None:
            for command_id, (stop_ns, intent_ns) in list(
                self.pending_stop.items()
            ):
                detector = self.zero_detectors[command_id]
                if detector.update(self.current_velocities):
                    zero_ns = self.now_ns()
                    tau_stop = (
                        (zero_ns - stop_ns) / 1e9
                        if zero_ns >= stop_ns else NO_TIME
                    )
                    cost_us = (
                        time.perf_counter_ns() - callback_start
                    ) / 1000.0
                    self._write_latency_row(
                        command_id, intent_ns,
                        NO_TIME, NO_TIME, NO_TIME, tau_stop,
                        cost_us, 'VELOCITY_ZERO',
                    )
                    del self.pending_stop[command_id]
                    detector.reset()
        self._record_cost(callback_start)

    def proximity_callback(self, msg: ProximityStatus) -> None:
        self.last_proximity_stamp_ns = stamp_to_ns(msg.stamp)

    def target_callback(self, msg: JointCommand) -> None:
        if tuple(msg.joint_names) != tuple(JOINT_NAMES):
            return
        target = tuple(float(value) for value in msg.positions)
        if len(target) != len(JOINT_NAMES):
            return
        duration = float(msg.duration.sec) + float(msg.duration.nanosec) * 1e-9
        self.targets[msg.command_id] = (
            target, duration, stamp_to_ns(msg.stamp)
        )

    def control_callback(self, msg: ExecutionControl) -> None:
        callback_start = time.perf_counter_ns()
        if self.current_positions is None or self.current_velocities is None:
            self.missing_messages += 1
            return
        target_data = self.targets.get(msg.command_id)
        if target_data is None:
            target = self.current_positions
            duration = 1.0
            intent_ns = 0
        else:
            target, duration, intent_ns = target_data
        duration = max(duration, 1.0e-9)
        requested_velocity = tuple(
            shortest_joint_delta(name, goal, current) / duration
            for name, current, goal in zip(
                JOINT_NAMES, self.current_positions, target
            )
        )
        now_ns = self.now_ns()
        joint_age = (
            (now_ns - self.last_joint_stamp_ns) / 1e9
            if self.last_joint_stamp_ns is not None else NO_TIME
        )
        proximity_age = (
            (now_ns - self.last_proximity_stamp_ns) / 1e9
            if self.last_proximity_stamp_ns is not None else NO_TIME
        )
        ttc = float(msg.time_to_collision)
        cost_us = (time.perf_counter_ns() - callback_start) / 1000.0
        row = {
            'timestamp_ros_sec': f'{now_ns / 1e9:.9f}',
            'command_id': msg.command_id or 'UNAVAILABLE',
            'intent_stamp_ros_sec': f'{intent_ns / 1e9:.9f}',
            'q_actual': vector_text(self.current_positions),
            'q_objetivo': vector_text(target),
            'qdot_solicitada': vector_text(requested_velocity),
            'qdot_medida': vector_text(self.current_velocities),
            'd_actual': f'{float(msg.current_clearance):.9f}',
            'd_nominal': f'{float(msg.nominal_clearance):.9f}',
            'd_supervisada': f'{float(msg.supervised_clearance):.9f}',
            'TTC_nominal': f'{ttc:.9f}',
            't_min': f'{float(msg.minimum_time_from_now):.9f}',
            'state': msg.state or 'UNAVAILABLE',
            'speed_scale': f'{float(msg.speed_scale):.9f}',
            'limiting_segment': msg.limiting_segment or 'NONE',
            'reason_code': msg.reason_code or 'UNAVAILABLE',
            'joint_state_age_sec': f'{joint_age:.9f}',
            'proximity_age_sec': f'{proximity_age:.9f}',
            'missing_messages': str(self.missing_messages),
            'instrumentation_cost_us': f'{cost_us:.3f}',
        }
        self.samples_writer.writerow(row)
        self._record_cost(callback_start)
        self._flush_if_required()

    def _record_cost(self, callback_start: int) -> None:
        elapsed_sec = (time.perf_counter_ns() - callback_start) / 1e9
        self.stats['instrumentation_cost'].add(elapsed_sec)

    def _flush_if_required(self) -> None:
        self.rows_since_flush += 1
        threshold = int(self.get_parameter('flush_every_rows').value)
        if self.rows_since_flush >= max(1, threshold):
            self.samples_handle.flush()
            self.latency_handle.flush()
            self.rows_since_flush = 0

    def write_summary(self) -> None:
        """Atomically replace the accumulated statistics CSV."""
        temporary = self.summary_path.with_suffix('.tmp')
        with temporary.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.SUMMARY_FIELDS)
            writer.writeheader()
            for metric, accumulator in self.stats.items():
                count, mean, stddev, minimum, maximum, p95 = (
                    accumulator.summary()
                )
                writer.writerow({
                    'metric': metric,
                    'count': count,
                    'mean_sec': f'{mean:.9f}',
                    'stddev_sec': f'{stddev:.9f}',
                    'minimum_sec': f'{minimum:.9f}',
                    'maximum_sec': f'{maximum:.9f}',
                    'percentile95_sec': f'{p95:.9f}',
                })
        temporary.replace(self.summary_path)

    def destroy_node(self):
        self.write_summary()
        self.samples_handle.flush()
        self.latency_handle.flush()
        self.samples_handle.close()
        self.latency_handle.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
