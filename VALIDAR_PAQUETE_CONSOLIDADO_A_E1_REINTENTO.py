#!/usr/bin/env python3
"""Runtime validation of JOG scaling, horizon scaling, watchdog and STOP."""

from __future__ import annotations

import math
import statistics
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    ExecutionControl,
    JointCommand,
    JointTrajectoryPrediction,
)


JOINT_NAMES = (
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
)
TEST_JOINT = JOINT_NAMES[0]
NOMINAL_SPEED = math.radians(18.0)
HORIZON_SEC = 1.0
PUBLISH_PERIOD_SEC = 0.10
ZERO_SPEED_THRESHOLD = 0.02


class RuntimeValidator(Node):
    """Publish controlled supervised inputs and measure Gazebo feedback."""

    def __init__(self) -> None:
        super().__init__('package_a_runtime_validator')
        self.positions: Dict[str, float] = {}
        self.velocity = math.nan
        self.state_messages = 0
        self.active_phase: Optional[str] = None
        self.phase_start = 0.0
        self.velocities: Dict[str, List[float]] = {}
        self.prediction_displacements: Dict[str, List[float]] = {}
        self.observed_predictions: Dict[str, int] = {}
        self.jog_publisher = self.create_publisher(
            JointCommand,
            '/thesis/supervised_jog_command',
            10,
        )
        self.control_publisher = self.create_publisher(
            ExecutionControl,
            '/thesis/execution_control',
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            20,
        )
        self.create_subscription(
            JointTrajectoryPrediction,
            '/thesis/joint_trajectory_prediction',
            self.prediction_callback,
            10,
        )

    def state_callback(self, msg: JointState) -> None:
        """Store the canonical measured state and velocity samples."""
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if all(name in positions for name in JOINT_NAMES):
            self.positions = {
                name: float(positions[name]) for name in JOINT_NAMES
            }
            self.state_messages += 1
        value = velocities.get(TEST_JOINT)
        if value is not None and math.isfinite(value):
            self.velocity = float(value)
            if (
                self.active_phase is not None
                and time.monotonic() - self.phase_start >= 0.35
            ):
                self.velocities.setdefault(
                    self.active_phase,
                    [],
                ).append(abs(self.velocity))

    def prediction_callback(self, msg: JointTrajectoryPrediction) -> None:
        """Measure the predicted J1 displacement for each test command."""
        observation = f'{msg.command_id}|{msg.source}'
        self.observed_predictions[observation] = (
            self.observed_predictions.get(observation, 0) + 1
        )
        if msg.command_id not in {
            'package_a_scale_100',
            'package_a_scale_050',
        }:
            return
        if msg.sample_count < 2 or len(msg.joint_names) != 6:
            return
        expected = int(msg.sample_count) * len(msg.joint_names)
        if len(msg.positions) != expected:
            return
        try:
            joint_index = list(msg.joint_names).index(TEST_JOINT)
        except ValueError:
            return
        first = float(msg.positions[joint_index])
        last_index = (
            (int(msg.sample_count) - 1) * len(msg.joint_names)
            + joint_index
        )
        last = float(msg.positions[last_index])
        displacement = abs(math.atan2(
            math.sin(last - first),
            math.cos(last - first),
        ))
        self.prediction_displacements.setdefault(
            msg.command_id,
            [],
        ).append(displacement)

    def spin_for(self, duration: float) -> None:
        """Process ROS callbacks for a wall-clock duration."""
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until_ready(self, timeout: float = 8.0) -> bool:
        """Wait for feedback and both required subscribers."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                len(self.positions) == 6
                and self.jog_publisher.get_subscription_count() >= 2
                and self.control_publisher.get_subscription_count() >= 1
                and self.count_publishers(
                    '/thesis/joint_trajectory_prediction'
                ) >= 1
            ):
                self.spin_for(0.50)
                return True
        return False

    def make_command(
        self,
        command_id: str,
        scale: float,
        direction: float,
    ) -> JointCommand:
        """Build a valid one-second horizon from the measured state."""
        command = JointCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_id = command_id
        command.joint_names = list(JOINT_NAMES)
        target = [self.positions[name] for name in JOINT_NAMES]
        target[0] += direction * scale * NOMINAL_SPEED * HORIZON_SEC
        command.positions = target
        command.duration.sec = 1
        command.duration.nanosec = 0
        return command

    def publish_stream(
        self,
        phase: str,
        command_id: str,
        scale: float,
        direction: float,
        duration: float,
    ) -> None:
        """Publish a rolling supervised horizon at exactly 10 Hz."""
        self.active_phase = phase
        self.phase_start = time.monotonic()
        self.velocities[phase] = []
        next_publish = self.phase_start
        deadline = self.phase_start + duration
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish and len(self.positions) == 6:
                self.jog_publisher.publish(self.make_command(
                    command_id,
                    scale,
                    direction,
                ))
                next_publish += PUBLISH_PERIOD_SEC
            rclpy.spin_once(self, timeout_sec=0.01)
        self.active_phase = None

    def wait_for_zero_speed(self, timeout: float) -> Optional[float]:
        """Return latency until ten consecutive near-zero measurements."""
        start = time.monotonic()
        consecutive = 0
        while rclpy.ok() and time.monotonic() - start <= timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if math.isfinite(self.velocity) and (
                abs(self.velocity) <= ZERO_SPEED_THRESHOLD
            ):
                consecutive += 1
                if consecutive >= 10:
                    return time.monotonic() - start
            else:
                consecutive = 0
        return None

    def publish_stop(self, command_id: str) -> None:
        """Publish an explicit supervised STOP for the active JOG."""
        message = ExecutionControl()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = command_id
        message.state = 'STOP'
        message.speed_scale = 0.0
        message.reason_code = 'PACKAGE_A_TEST_STOP'
        message.reason = 'STOP controlado de validación del paquete A.'
        message.requested_state = 'STOP'
        message.requested_speed_scale = 0.0
        message.requested_reason_code = 'PACKAGE_A_TEST_STOP'
        message.transition = True
        message.transition_reason = 'TEST_STIMULUS'
        for _ in range(3):
            self.control_publisher.publish(message)
            self.spin_for(0.03)


def robust_median(values: List[float], name: str) -> float:
    """Return a finite median or raise for insufficient evidence."""
    usable = [value for value in values if math.isfinite(value)]
    if len(usable) < 10:
        raise RuntimeError(f'{name}: only {len(usable)} usable samples')
    return float(statistics.median(usable))


def main() -> int:
    """Execute the four deterministic runtime checks."""
    rclpy.init()
    node = RuntimeValidator()
    failures: List[str] = []
    try:
        if not node.wait_until_ready():
            raise RuntimeError(
                'missing joint state or adapter topic subscriptions'
            )
        print(f'JOINT_STATE_MESSAGES_INICIALES={node.state_messages}')

        node.publish_stream(
            'scale_100',
            'package_a_scale_100',
            1.0,
            1.0,
            1.60,
        )
        watchdog_latency_100 = node.wait_for_zero_speed(0.80)
        node.spin_for(0.30)

        node.publish_stream(
            'scale_050',
            'package_a_scale_050',
            0.5,
            -1.0,
            1.60,
        )
        watchdog_latency_050 = node.wait_for_zero_speed(0.80)
        node.spin_for(0.30)

        node.publish_stream(
            'stop_test',
            'package_a_stop_test',
            1.0,
            -1.0,
            0.80,
        )
        node.publish_stop('package_a_stop_test')
        stop_latency = node.wait_for_zero_speed(0.50)
        node.spin_for(0.30)

        nominal_velocity = robust_median(
            node.velocities['scale_100'],
            'scale_100',
        )
        half_velocity = robust_median(
            node.velocities['scale_050'],
            'scale_050',
        )
        velocity_ratio = half_velocity / nominal_velocity

        print(f'QDOT_NOMINAL_ESPERADA={NOMINAL_SPEED:.9f}')
        print(f'QDOT_ESCALA_1_MEDIANA={nominal_velocity:.9f}')
        print(f'QDOT_ESCALA_05_MEDIANA={half_velocity:.9f}')
        print(f'RATIO_VELOCIDAD_05_1={velocity_ratio:.6f}')
        print(f'LATENCIA_WATCHDOG_ESCALA_1={watchdog_latency_100}')
        print(f'LATENCIA_WATCHDOG_ESCALA_05={watchdog_latency_050}')
        print(f'LATENCIA_STOP={stop_latency}')
        print(f'PREDICCIONES_OBSERVADAS={node.observed_predictions}')

        prediction_100_values = node.prediction_displacements.get(
            'package_a_scale_100',
            [],
        )
        prediction_050_values = node.prediction_displacements.get(
            'package_a_scale_050',
            [],
        )
        prediction_ratio = math.nan
        if len(prediction_100_values) >= 10 and len(
            prediction_050_values
        ) >= 10:
            prediction_100 = robust_median(
                prediction_100_values,
                'prediction_100',
            )
            prediction_050 = robust_median(
                prediction_050_values,
                'prediction_050',
            )
            prediction_ratio = prediction_050 / prediction_100
            print(f'HORIZONTE_ESCALA_1={prediction_100:.9f}')
            print(f'HORIZONTE_ESCALA_05={prediction_050:.9f}')
            print(f'RATIO_HORIZONTE_05_1={prediction_ratio:.6f}')
        else:
            failures.append(
                'insufficient matching prediction samples: '
                f's1={len(prediction_100_values)}, '
                f's05={len(prediction_050_values)}'
            )

        if not 0.80 * NOMINAL_SPEED <= nominal_velocity <= (
            1.20 * NOMINAL_SPEED
        ):
            failures.append('nominal measured velocity outside ±20%')
        if not 0.35 <= velocity_ratio <= 0.65:
            failures.append('half-speed ratio outside [0.35, 0.65]')
        if math.isfinite(prediction_ratio) and not (
            0.40 <= prediction_ratio <= 0.60
        ):
            failures.append('horizon ratio outside [0.40, 0.60]')
        if watchdog_latency_100 is None or watchdog_latency_100 > 0.55:
            failures.append('scale-1 watchdog did not stop within 0.55 s')
        if watchdog_latency_050 is None or watchdog_latency_050 > 0.55:
            failures.append('scale-0.5 watchdog did not stop within 0.55 s')
        if stop_latency is None or stop_latency > 0.25:
            failures.append('explicit STOP did not stop within 0.25 s')

        if failures:
            for failure in failures:
                print(f'FAIL={failure}')
            print('RESULTADO_GLOBAL_E1=FAIL')
            return 1
        print('RESULTADO_GLOBAL_E1=PASS')
        return 0
    except Exception as exc:
        print(f'ERROR={exc}')
        print('RESULTADO_GLOBAL_E1=FAIL')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
