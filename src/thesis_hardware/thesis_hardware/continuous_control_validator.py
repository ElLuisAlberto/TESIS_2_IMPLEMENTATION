"""Measure the live continuous-control path without publishing commands."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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


class StreamStats:
    """Collect arrival times for one stream."""

    def __init__(self):
        self.times = []

    def add(self):
        self.times.append(time.monotonic())

    @property
    def count(self):
        return len(self.times)

    @property
    def rate(self):
        if len(self.times) < 2:
            return 0.0
        elapsed = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / elapsed if elapsed > 0.0 else 0.0

    @property
    def maximum_gap(self):
        if len(self.times) < 2:
            return math.inf
        return max(
            second - first
            for first, second in zip(self.times, self.times[1:])
        )


class ContinuousControlValidator(Node):
    """Observe intent, supervision, geometry and hardware diagnostics."""

    def __init__(self):
        super().__init__('continuous_control_validator')
        self.declare_parameter('duration_sec', 35.0)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        if not math.isfinite(self.duration_sec) or self.duration_sec < 10.0:
            raise ValueError('duration_sec must be finite and at least 10 s')

        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.streams = {
            'joint_states': StreamStats(),
            'jog_intent': StreamStats(),
            'supervised_jog': StreamStats(),
            'execution_control': StreamStats(),
            'jog_prediction': StreamStats(),
            'diagnostics': StreamStats(),
        }
        self.position_minimum = {}
        self.position_maximum = {}
        self.supervised_latencies_ms = []
        self.control_states = set()
        self.latest_diagnostics = {}

        self.create_subscription(
            JointState, '/joint_states', self.receive_state, 10
        )
        self.create_subscription(
            JointCommand,
            '/thesis/jog_intent',
            self.receive_jog_intent,
            jog_qos,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self.receive_supervised_jog,
            jog_qos,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.receive_control,
            10,
        )
        self.create_subscription(
            JointTrajectoryPrediction,
            '/thesis/joint_trajectory_prediction',
            self.receive_prediction,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            '/thesis/hardware/diagnostics',
            self.receive_diagnostics,
            10,
        )

    def receive_state(self, message):
        self.streams['joint_states'].add()
        positions = dict(zip(message.name, message.position))
        for name in JOINT_NAMES:
            value = positions.get(name)
            if value is None or not math.isfinite(value):
                continue
            self.position_minimum[name] = min(
                value, self.position_minimum.get(name, value)
            )
            self.position_maximum[name] = max(
                value, self.position_maximum.get(name, value)
            )

    def receive_jog_intent(self, message):
        if message.command_id == 'jog_continuous':
            self.streams['jog_intent'].add()

    def receive_supervised_jog(self, message):
        if message.command_id != 'jog_continuous':
            return
        self.streams['supervised_jog'].add()
        stamp_ns = (
            int(message.stamp.sec) * 1_000_000_000
            + int(message.stamp.nanosec)
        )
        if stamp_ns > 0:
            latency_ms = (
                self.get_clock().now().nanoseconds - stamp_ns
            ) * 1.0e-6
            if 0.0 <= latency_ms <= 10_000.0:
                self.supervised_latencies_ms.append(latency_ms)

    def receive_control(self, message):
        if message.command_id == 'jog_continuous':
            self.streams['execution_control'].add()
            self.control_states.add(message.state)

    def receive_prediction(self, message):
        if (
            message.command_id == 'jog_continuous'
            and message.source == 'jog'
        ):
            self.streams['jog_prediction'].add()

    def receive_diagnostics(self, message):
        for status in message.status:
            if status.name != 'thesis_hardware_bridge/JACO2':
                continue
            self.streams['diagnostics'].add()
            self.latest_diagnostics = {
                item.key: item.value for item in status.values
            }

    @staticmethod
    def in_range(value, lower, upper):
        return math.isfinite(value) and lower <= value <= upper

    def report(self):
        checks = []

        def add_check(name, passed, detail):
            checks.append(bool(passed))
            state = 'OK' if passed else 'FALLO'
            print(f'CHECK_{name}={state} | {detail}')

        print('=== VALIDACION DE CONTROL CONTINUO JACO2 ===')
        print(f'DURACION_SOLICITADA_SEC={self.duration_sec:.1f}')
        for name, stats in self.streams.items():
            maximum_gap_ms = (
                stats.maximum_gap * 1000.0
                if math.isfinite(stats.maximum_gap) else -1.0
            )
            print(
                f'FLUJO_{name.upper()}='
                f'mensajes:{stats.count},frecuencia_hz:{stats.rate:.2f},'
                f'hueco_max_ms:{maximum_gap_ms:.1f}'
            )

        joint_stats = self.streams['joint_states']
        jog_stats = self.streams['jog_intent']
        safe_stats = self.streams['supervised_jog']
        control_stats = self.streams['execution_control']
        prediction_stats = self.streams['jog_prediction']
        add_check(
            'JOINT_STATES_20HZ',
            joint_stats.count >= 20
            and self.in_range(joint_stats.rate, 15.0, 25.0),
            f'{joint_stats.rate:.2f} Hz',
        )
        add_check(
            'INTENCION_JOG_20HZ',
            jog_stats.count >= 10
            and self.in_range(jog_stats.rate, 15.0, 25.0)
            and jog_stats.maximum_gap <= 0.18,
            f'{jog_stats.rate:.2f} Hz; '
            f'hueco máximo {jog_stats.maximum_gap * 1000.0:.1f} ms',
        )
        add_check(
            'JOG_SUPERVISADO_20HZ',
            safe_stats.count >= 10
            and self.in_range(safe_stats.rate, 15.0, 25.0)
            and safe_stats.maximum_gap <= 0.18,
            f'{safe_stats.rate:.2f} Hz; '
            f'hueco máximo {safe_stats.maximum_gap * 1000.0:.1f} ms',
        )
        add_check(
            'CONTROL_PREVENTIVO',
            control_stats.count >= 10,
            f'{control_stats.count} decisiones; '
            f'estados={sorted(self.control_states)}',
        )
        add_check(
            'VOLUMEN_SUPERVISADO_10HZ',
            prediction_stats.count >= 5
            and self.in_range(prediction_stats.rate, 7.0, 13.0),
            f'{prediction_stats.rate:.2f} Hz',
        )

        ranges_deg = {
            name: math.degrees(
                self.position_maximum[name] - self.position_minimum[name]
            )
            for name in JOINT_NAMES
            if name in self.position_minimum and name in self.position_maximum
        }
        maximum_motion = max(ranges_deg.values(), default=0.0)
        print(
            'RECORRIDO_ARTICULAR_GRADOS='
            + ','.join(
                f'{name}:{value:.3f}'
                for name, value in ranges_deg.items()
            )
        )
        add_check(
            'MOVIMIENTO_MEDIDO',
            maximum_motion >= 0.20,
            f'recorrido máximo {maximum_motion:.3f} grados',
        )

        if self.supervised_latencies_ms:
            mean_latency = sum(self.supervised_latencies_ms) / len(
                self.supervised_latencies_ms
            )
            maximum_latency = max(self.supervised_latencies_ms)
        else:
            mean_latency = math.inf
            maximum_latency = math.inf
        add_check(
            'LATENCIA_SUPERVISION',
            maximum_latency <= 150.0,
            f'media {mean_latency:.1f} ms; máxima {maximum_latency:.1f} ms',
        )

        diagnostics = self.latest_diagnostics
        for key in (
            'connected',
            'armed',
            'mode',
            'control_rate_hz_measured',
            'feedback_rate_hz_measured',
            'output_command_rate_hz_measured',
            'control_deadline_resets',
            'stale_jog_commands',
            'last_jog_source_age_ms',
        ):
            value = diagnostics.get(key, 'AUSENTE')
            print(f'DIAGNOSTICO_{key.upper()}={value}')
        try:
            control_rate = float(diagnostics['control_rate_hz_measured'])
            feedback_rate = float(diagnostics['feedback_rate_hz_measured'])
            output_rate = float(
                diagnostics['output_command_rate_hz_measured']
            )
            deadline_resets = int(diagnostics['control_deadline_resets'])
            stale_commands = int(diagnostics['stale_jog_commands'])
        except (KeyError, TypeError, ValueError):
            control_rate = feedback_rate = output_rate = math.nan
            deadline_resets = stale_commands = -1
        add_check(
            'ADAPTADOR_100HZ',
            self.in_range(control_rate, 95.0, 105.0)
            and self.in_range(output_rate, 95.0, 105.0)
            and deadline_resets == 0,
            f'control {control_rate:.2f} Hz; salida {output_rate:.2f} Hz; '
            f'reinicios {deadline_resets}',
        )
        add_check(
            'FEEDBACK_20HZ',
            self.in_range(feedback_rate, 15.0, 25.0),
            f'{feedback_rate:.2f} Hz',
        )
        add_check(
            'SIN_ORDENES_OBSOLETAS',
            stale_commands == 0,
            f'órdenes obsoletas rechazadas: {stale_commands}',
        )

        print(f'RESULTADO={"OK" if all(checks) else "REVISAR"}')


def main(args=None):
    """Run one finite, read-only observation interval."""
    rclpy.init(args=args)
    node = ContinuousControlValidator()
    deadline = time.monotonic() + node.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
        node.report()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
