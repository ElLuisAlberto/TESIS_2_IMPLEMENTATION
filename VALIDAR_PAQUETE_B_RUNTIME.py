#!/usr/bin/env python3
"""Run two reproducible package-B telemetry scenarios in Gazebo."""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import JointCommand, ProximityStatus
from visualization_msgs.msg import MarkerArray

from thesis_core.joint_model import JOINT_NAMES


class RuntimeProbe(Node):
    """Publish deterministic JOG intentions and measure horizon frequency."""

    def __init__(self) -> None:
        super().__init__('package_b_runtime_validator')
        self.positions = None
        self.proximity_seen = False
        self.horizon_arrivals: list[float] = []
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', 10
        )
        self.create_subscription(
            JointState, '/joint_states', self.state_callback, 20
        )
        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_callback,
            10,
        )
        self.create_subscription(
            MarkerArray,
            '/thesis/horizon_volume',
            self.horizon_callback,
            10,
        )

    def state_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINT_NAMES):
            candidate = tuple(float(values[name]) for name in JOINT_NAMES)
            if all(math.isfinite(value) for value in candidate):
                self.positions = candidate

    def proximity_callback(self, _msg: ProximityStatus) -> None:
        self.proximity_seen = True

    def horizon_callback(self, _msg: MarkerArray) -> None:
        self.horizon_arrivals.append(time.monotonic())

    def wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.positions is not None
                and self.proximity_seen
                and self.publisher.get_subscription_count() == 1
            ):
                return True
        return False

    def publish_step(self, command_id: str, direction: float) -> None:
        if self.positions is None:
            raise RuntimeError('joint state unavailable')
        msg = JointCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.joint_names = list(JOINT_NAMES)
        target = list(self.positions)
        target[0] += direction * 0.12
        msg.positions = target
        msg.duration.sec = 1
        self.publisher.publish(msg)

    def run_motion(self, command_id: str) -> None:
        self.horizon_arrivals.clear()
        for direction in (1.0, -1.0):
            for _ in range(30):
                self.publish_step(command_id, direction)
                end = time.monotonic() + 0.10
                while time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.005)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def horizon_rate(self) -> float:
        if len(self.horizon_arrivals) < 2:
            return 0.0
        duration = self.horizon_arrivals[-1] - self.horizon_arrivals[0]
        return (
            (len(self.horizon_arrivals) - 1) / duration
            if duration > 0.0 else 0.0
        )


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def newest_directory(root: Path) -> Path:
    directories = sorted(path for path in root.iterdir() if path.is_dir())
    if not directories:
        raise RuntimeError(f'no telemetry run found in {root}')
    return directories[-1]


def finite_nonnegative(rows, field):
    values = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, ValueError):
            continue
        if math.isfinite(value) and value >= 0.0:
            values.append(value)
    return values


def validate_run(run_directory: Path, horizon_rate: float) -> dict:
    samples = read_csv(run_directory / 'telemetry_samples.csv')
    latencies = read_csv(run_directory / 'latency_metrics.csv')
    summary = read_csv(run_directory / 'statistics.csv')
    critical = (
        'timestamp_ros_sec', 'command_id', 'q_actual', 'q_objetivo',
        'qdot_solicitada', 'qdot_medida', 'd_actual', 'd_nominal',
        'd_supervisada', 'TTC_nominal', 't_min', 'state',
        'speed_scale', 'limiting_segment', 'reason_code',
        'joint_state_age_sec', 'proximity_age_sec',
    )
    empty = sum(
        1 for row in samples for field in critical
        if field not in row or row[field].strip() == ''
    )
    prediction = finite_nonnegative(latencies, 'tau_prediction_sec')
    supervisor = finite_nonnegative(latencies, 'tau_supervisor_sec')
    end_to_end = finite_nonnegative(latencies, 'tau_end_to_end_sec')
    stop = finite_nonnegative(latencies, 'tau_stop_sec')
    metrics = {row.get('metric', '') for row in summary}
    required_metrics = {
        'tau_prediction', 'tau_supervisor', 'tau_end_to_end',
        'tau_stop', 'instrumentation_cost',
    }
    result = {
        'directory': str(run_directory),
        'sample_count': len(samples),
        'latency_count': len(latencies),
        'empty_critical': empty,
        'prediction': prediction,
        'supervisor': supervisor,
        'end_to_end': end_to_end,
        'stop': stop,
        'summary_complete': required_metrics.issubset(metrics),
        'horizon_rate': horizon_rate,
    }
    result['pass'] = (
        len(samples) >= 10
        and empty == 0
        and bool(prediction)
        and bool(supervisor)
        and bool(end_to_end)
        and bool(stop)
        and result['summary_complete']
        and horizon_rate >= 8.0
    )
    return result


def metric_mean(result, name):
    values = result[name]
    return statistics.fmean(values) if values else -1.0


def comparable(first: float, second: float) -> bool:
    if first < 0.0 or second < 0.0:
        return False
    return abs(first - second) <= max(0.020, 0.50 * max(first, second))


def start_logger(output_root: Path):
    output_root.mkdir(parents=True, exist_ok=True)
    log_path = output_root / 'telemetry_node.log'
    handle = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        [
            'ros2', 'run', 'thesis_telemetry', 'telemetry_logger',
            '--ros-args', '-p', 'use_sim_time:=true',
            '-p', f'output_directory:={output_root}',
        ],
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, handle


def stop_logger(process, handle):
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        process.wait(timeout=3.0)
    handle.close()


def main() -> int:
    print('============================================================')
    print('PAQUETE CONSOLIDADO B — VALIDACIÓN RUNTIME')
    print('============================================================')
    result = subprocess.run(
        ['ros2', 'node', 'list'], capture_output=True, text=True, check=False
    )
    nodes = result.stdout.splitlines()
    required_nodes = {
        '/safety_supervisor_node', '/simulation_command_adapter',
        '/horizon_preview', '/proximity_monitor',
    }
    missing = sorted(required_nodes.difference(nodes))
    if missing:
        print(f'NODOS_REQUERIDOS=FAIL faltantes={missing}')
        return 2
    if '/telemetry_logger' in nodes:
        print('TELEMETRY_LOGGER_PREEXISTENTE=FAIL')
        return 2
    print('NODOS_REQUERIDOS=PASS')

    root = Path.home() / 'thesis_telemetry_validation'
    rclpy.init()
    probe = RuntimeProbe()
    try:
        if not probe.wait_ready():
            print('ENTRADAS_RUNTIME=FAIL')
            return 2
        print('ENTRADAS_RUNTIME=PASS')
        results = []
        for index in (1, 2):
            output_root = root / f'run_{index}'
            process, handle = start_logger(output_root)
            time.sleep(1.5)
            probe.run_motion(f'package_b_run_{index}')
            rate = probe.horizon_rate()
            time.sleep(1.0)
            stop_logger(process, handle)
            directory = newest_directory(output_root)
            validated = validate_run(directory, rate)
            results.append(validated)
            print(
                f'CORRIDA_{index}=' + ('PASS' if validated['pass'] else 'FAIL')
            )
            print(f'CORRIDA_{index}_DIRECTORIO={directory}')
            print(f'CORRIDA_{index}_MUESTRAS={validated["sample_count"]}')
            print(f'CORRIDA_{index}_VACIOS={validated["empty_critical"]}')
            print(f'CORRIDA_{index}_HZ={rate:.6f}')
            for metric in ('prediction', 'supervisor', 'end_to_end', 'stop'):
                print(
                    f'CORRIDA_{index}_{metric.upper()}_MEDIA='
                    f'{metric_mean(validated, metric):.9f}'
                )
            time.sleep(1.1)

        comparison = all(
            comparable(metric_mean(results[0], metric),
                       metric_mean(results[1], metric))
            for metric in ('prediction', 'supervisor', 'end_to_end', 'stop')
        )
        print('REPETIBILIDAD=' + ('PASS' if comparison else 'FAIL'))
        global_pass = all(item['pass'] for item in results) and comparison
        print('RESULTADO_GLOBAL_RUNTIME_B=' + (
            'PASS' if global_pass else 'FAIL'
        ))
        return 0 if global_pass else 1
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
