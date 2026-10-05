"""Observe one GUI Cartesian command and verify its Gazebo result."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from thesis_core.cartesian_kinematics import (
    pose_error,
    transform_from_translation_quaternion,
)
from thesis_core.jaco_kinematics import transform_matrix
from thesis_interfaces.msg import (
    CartesianCommand,
    CommandDecision,
    ExecutionTrajectory,
    JointCommand,
)
import tf2_ros


TERMINAL_STATES = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}


def transform_from_tf(message):
    """Convert one geometry transform into a homogeneous matrix."""
    translation = message.transform.translation
    quaternion = message.transform.rotation
    return transform_from_translation_quaternion(
        (translation.x, translation.y, translation.z),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    )


class CartesianSimulationValidator(Node):
    """Collect the conversion, safety and execution result of one pose."""

    def __init__(self):
        super().__init__('cartesian_sim_validator')
        self.declare_parameter('duration_sec', 60.0)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        if not math.isfinite(self.duration_sec) or self.duration_sec < 10.0:
            raise ValueError('duration_sec must be finite and at least 10 s')
        self.command_id = None
        self.target_transform = None
        self.command_start_transform = None
        self.converted = False
        self.decision = None
        self.terminal = None
        self.terminal_at = None
        self.measured_transform = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )
        self.create_subscription(
            CartesianCommand,
            '/thesis/cartesian_candidate',
            self.receive_cartesian,
            10,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/candidate_command',
            self.receive_joint_candidate,
            10,
        )
        self.create_subscription(
            CommandDecision,
            '/thesis/command_decision',
            self.receive_decision,
            10,
        )
        self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.receive_execution,
            20,
        )

    def receive_cartesian(self, message):
        if self.command_id is not None:
            return
        self.command_id = message.command_id
        self.target_transform = transform_matrix(
            (message.x, message.y, message.z),
            (message.roll, message.pitch, message.yaw),
        )
        if self.measured_transform is not None:
            self.command_start_transform = [
                row[:] for row in self.measured_transform
            ]

    def receive_joint_candidate(self, message):
        if self.command_id is not None and message.command_id == self.command_id:
            self.converted = True

    def receive_decision(self, message):
        if self.command_id is not None and message.command_id == self.command_id:
            self.decision = message
            if not message.accepted:
                self.terminal_at = time.monotonic()

    def receive_execution(self, message):
        if self.command_id is None or message.command_id != self.command_id:
            return
        if message.status in TERMINAL_STATES:
            self.terminal = message
            self.terminal_at = time.monotonic()

    def refresh_transform(self):
        try:
            message = self.tf_buffer.lookup_transform(
                'world', 'j2n6s300_end_effector', Time()
            )
        except tf2_ros.TransformException:
            return
        try:
            self.measured_transform = transform_from_tf(message)
        except ValueError:
            return

    def observation_complete(self):
        return (
            self.terminal_at is not None
            and time.monotonic() - self.terminal_at >= 2.0
        )

    def report(self):
        checks = []

        def add_check(name, passed, detail):
            checks.append(bool(passed))
            state = 'OK' if passed else 'FALLO'
            print(f'CHECK_{name}={state} | {detail}')

        print('=== VALIDACION CARTESIANA EN GAZEBO ===')
        print(f'COMANDO={self.command_id or "AUSENTE"}')
        if self.target_transform is not None:
            target_position = (
                self.target_transform[0][3],
                self.target_transform[1][3],
                self.target_transform[2][3],
            )
            print(
                'OBJETIVO_XYZ_M='
                f'{target_position[0]:.4f},'
                f'{target_position[1]:.4f},'
                f'{target_position[2]:.4f}'
            )
        if (
            self.target_transform is not None
            and self.command_start_transform is not None
        ):
            initial_position, initial_orientation = pose_error(
                self.target_transform,
                self.command_start_transform,
            )
            initial_position_norm = math.sqrt(sum(
                value * value for value in initial_position
            ))
            initial_orientation_norm = math.sqrt(sum(
                value * value for value in initial_orientation
            ))
            print(f'PASO_SOLICITADO_M={initial_position_norm:.6f}')
            print(
                'GIRO_SOLICITADO_GRADOS='
                f'{math.degrees(initial_orientation_norm):.3f}'
            )
        else:
            print('PASO_SOLICITADO_M=NO_DISPONIBLE')
            print('GIRO_SOLICITADO_GRADOS=NO_DISPONIBLE')
        add_check(
            'INTENCION_CARTESIANA',
            self.command_id is not None and self.target_transform is not None,
            'objetivo X/Y/Z/Roll/Pitch/Yaw observado',
        )
        add_check(
            'CONVERSION_IK',
            self.converted,
            'JointCommand generado por el adaptador',
        )
        accepted = self.decision is not None and self.decision.accepted
        decision_detail = (
            'ausente' if self.decision is None
            else f'{self.decision.state}: {self.decision.reason}'
        )
        add_check('SUPERVISION', accepted, decision_detail)
        terminal_state = (
            'AUSENTE' if self.terminal is None else self.terminal.status
        )
        add_check(
            'EJECUCION',
            terminal_state == 'SUCCEEDED',
            f'estado={terminal_state}',
        )

        position_norm = math.inf
        orientation_norm = math.inf
        if self.target_transform is not None and self.measured_transform is not None:
            position_error, orientation_error = pose_error(
                self.target_transform, self.measured_transform
            )
            position_norm = math.sqrt(sum(
                value * value for value in position_error
            ))
            orientation_norm = math.sqrt(sum(
                value * value for value in orientation_error
            ))
        print(f'ERROR_POSICION_M={position_norm:.6f}')
        print(
            'ERROR_ORIENTACION_GRADOS='
            f'{math.degrees(orientation_norm):.3f}'
        )
        add_check(
            'POSICION_FINAL',
            position_norm <= 0.015,
            f'error={position_norm:.4f} m; límite=0.0150 m',
        )
        add_check(
            'ORIENTACION_FINAL',
            orientation_norm <= math.radians(3.0),
            'error='
            f'{math.degrees(orientation_norm):.2f} grados; límite=3.00',
        )
        print(f'RESULTADO={"OK" if all(checks) else "REVISAR"}')


def main(args=None):
    """Run a finite read-only Cartesian simulation observation."""
    rclpy.init(args=args)
    node = CartesianSimulationValidator()
    deadline = time.monotonic() + node.duration_sec
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not node.observation_complete()
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
            node.refresh_transform()
        node.refresh_transform()
        node.report()
    except KeyboardInterrupt:
        node.refresh_transform()
        node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
