"""Observe one physical point command and quantify terminal settling."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_core.joint_model import JOINT_NAMES, shortest_joint_delta
from thesis_interfaces.msg import ExecutionTrajectory


TERMINAL_STATES = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'DRY_RUN'}
TOLERANCE_RAD = 0.005


class PointSettlingValidator(Node):
    """Collect measured positions around one accepted point command."""

    def __init__(self):
        super().__init__('point_settling_validator')
        self.declare_parameter('duration_sec', 40.0)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        if not math.isfinite(self.duration_sec) or self.duration_sec < 10.0:
            raise ValueError('duration_sec must be finite and at least 10 s')

        self.command_id = None
        self.target = None
        self.accepted_at = None
        self.terminal_at = None
        self.terminal_status = None
        self.terminal_detail = ''
        self.accepted_messages = 0
        self.samples = []
        self.create_subscription(
            JointState, '/joint_states', self.receive_state, 20
        )
        self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.receive_execution,
            20,
        )

    def receive_execution(self, message):
        now = time.monotonic()
        if message.status == 'ACCEPTED':
            if self.command_id is None:
                self.command_id = message.command_id
                self.accepted_at = now
            if message.command_id != self.command_id:
                return
            if (
                tuple(message.joint_names) == JOINT_NAMES
                and len(message.target_positions) == len(JOINT_NAMES)
            ):
                self.target = tuple(message.target_positions)
            self.accepted_messages += 1
            return
        if (
            self.command_id is not None
            and message.command_id == self.command_id
            and message.status in TERMINAL_STATES
        ):
            self.terminal_status = message.status
            self.terminal_detail = message.detail
            self.terminal_at = now

    def receive_state(self, message):
        if self.command_id is None:
            return
        positions_by_name = dict(zip(message.name, message.position))
        if not all(name in positions_by_name for name in JOINT_NAMES):
            return
        positions = tuple(
            float(positions_by_name[name]) for name in JOINT_NAMES
        )
        if not all(math.isfinite(value) for value in positions):
            return
        self.samples.append((time.monotonic(), positions))

    def observation_complete(self):
        return (
            self.terminal_at is not None
            and time.monotonic() - self.terminal_at >= 3.0
        )

    def errors(self, positions):
        if self.target is None:
            return None
        return tuple(
            shortest_joint_delta(name, target, current)
            for name, target, current in zip(
                JOINT_NAMES, self.target, positions
            )
        )

    def direction_reversals(self):
        previous_signs = [0] * len(JOINT_NAMES)
        reversals = [0] * len(JOINT_NAMES)
        for sample_time, positions in self.samples:
            if self.terminal_at is not None and sample_time > self.terminal_at:
                break
            errors = self.errors(positions)
            if errors is None:
                continue
            for index, error in enumerate(errors):
                if abs(error) <= TOLERANCE_RAD:
                    continue
                sign = 1 if error > 0.0 else -1
                if previous_signs[index] and sign != previous_signs[index]:
                    reversals[index] += 1
                previous_signs[index] = sign
        return reversals

    def report(self):
        checks = []

        def add_check(name, passed, detail):
            checks.append(bool(passed))
            state = 'OK' if passed else 'FALLO'
            print(f'CHECK_{name}={state} | {detail}')

        print('=== VALIDACION DE ASENTAMIENTO EN POSICION FINAL ===')
        print(f'COMANDO={self.command_id or "AUSENTE"}')
        print(f'ESTADO_TERMINAL={self.terminal_status or "AUSENTE"}')
        print(f'DETALLE_TERMINAL={self.terminal_detail or "AUSENTE"}')
        print(f'MENSAJES_ACCEPTED={self.accepted_messages}')
        print(f'MUESTRAS_ARTICULARES={len(self.samples)}')

        add_check(
            'COMANDO_ACEPTADO',
            self.command_id is not None and self.target is not None,
            'referencia y objetivo observados',
        )
        add_check(
            'FINALIZACION',
            self.terminal_status == 'SUCCEEDED',
            f'estado={self.terminal_status}',
        )

        final_errors_deg = []
        if self.samples and self.target is not None:
            final_errors = self.errors(self.samples[-1][1])
            final_errors_deg = [
                abs(math.degrees(value)) for value in final_errors
            ]
        maximum_final_error = max(final_errors_deg, default=math.inf)
        print(
            'ERROR_FINAL_GRADOS='
            + ','.join(f'{value:.3f}' for value in final_errors_deg)
        )
        add_check(
            'ERROR_FINAL',
            maximum_final_error <= 0.75,
            f'máximo={maximum_final_error:.3f} grados',
        )

        reversals = self.direction_reversals()
        total_reversals = sum(reversals)
        print(
            'INVERSIONES_ANTES_DEL_FINAL='
            + ','.join(str(value) for value in reversals)
        )
        add_check(
            'SIN_OSCILACION',
            total_reversals <= 1,
            f'inversiones de error={total_reversals}',
        )

        post_terminal = []
        if self.terminal_at is not None:
            post_terminal = [
                positions for sample_time, positions in self.samples
                if sample_time >= self.terminal_at
            ]
        drift_deg = []
        if post_terminal:
            for index in range(len(JOINT_NAMES)):
                values = [sample[index] for sample in post_terminal]
                drift_deg.append(math.degrees(max(values) - min(values)))
        maximum_drift = max(drift_deg, default=math.inf)
        print(
            'DERIVA_POSTERIOR_GRADOS='
            + ','.join(f'{value:.3f}' for value in drift_deg)
        )
        add_check(
            'RETENCION_POSTERIOR',
            len(post_terminal) >= 20 and maximum_drift <= 0.75,
            f'muestras={len(post_terminal)}; '
            f'deriva máxima={maximum_drift:.3f} grados',
        )

        if self.accepted_at is not None and self.terminal_at is not None:
            settling_sec = self.terminal_at - self.accepted_at
        else:
            settling_sec = math.inf
        print(f'TIEMPO_HASTA_FINAL_SEC={settling_sec:.3f}')
        print(f'RESULTADO={"OK" if all(checks) else "REVISAR"}')


def main(args=None):
    """Run one finite read-only observation."""
    rclpy.init(args=args)
    node = PointSettlingValidator()
    deadline = time.monotonic() + node.duration_sec
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not node.observation_complete()
        ):
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
