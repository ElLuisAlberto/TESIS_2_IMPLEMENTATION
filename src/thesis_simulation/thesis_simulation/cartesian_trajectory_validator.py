"""Run one deterministic long Cartesian trajectory in Gazebo."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from thesis_core.cartesian_kinematics import (
    pose_error,
    transform_from_translation_quaternion,
)
from thesis_core.jaco_kinematics import transform_matrix
from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import (
    CartesianTrajectoryCommand,
    CartesianTrajectoryStatus,
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
)
import tf2_ros


READY_POSITIONS = tuple(
    math.radians(value) for value in (-45.0, 160.0, 200.0, 0.0, 15.0, 0.0)
)
TERMINAL_STATES = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED'}


def transform_from_tf(message):
    """Convert one TF message into a homogeneous transform."""
    translation = message.transform.translation
    quaternion = message.transform.rotation
    return transform_from_translation_quaternion(
        (translation.x, translation.y, translation.z),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    )


class CartesianTrajectoryValidator(Node):
    """Prepare a reachable pose and validate one supervised long path."""

    def __init__(self):
        super().__init__('cartesian_trajectory_validator')
        self.declare_parameter('timeout_sec', 45.0)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        if not math.isfinite(self.timeout_sec) or self.timeout_sec < 20.0:
            raise ValueError('timeout_sec must be finite and at least 20 s')

        self.started_at = time.monotonic()
        self.phase = 'WAITING'
        self.phase_started_at = self.started_at
        self.system_ready = False
        self.ready_command_id = None
        self.ready_terminal = None
        self.trajectory_command_id = None
        self.trajectory_status = None
        self.trajectory_started = False
        self.terminal_at = None
        self.control_states = []
        self.measured_transform = None
        self.target_transform = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ready_publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10
        )
        self.trajectory_publisher = self.create_publisher(
            CartesianTrajectoryCommand,
            '/thesis/cartesian_trajectory_candidate',
            10,
        )
        self.create_subscription(
            String, '/thesis/system_readiness', self.readiness_callback, 10
        )
        self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.execution_callback,
            20,
        )
        self.create_subscription(
            CartesianTrajectoryStatus,
            '/thesis/cartesian_trajectory_status',
            self.trajectory_status_callback,
            20,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.control_callback,
            20,
        )

    def readiness_callback(self, message):
        self.system_ready = str(message.data) == 'READY'

    def execution_callback(self, message):
        if message.command_id != self.ready_command_id:
            return
        if message.status in TERMINAL_STATES:
            self.ready_terminal = message
            self.phase = 'READY_TERMINAL'
            self.phase_started_at = time.monotonic()

    def trajectory_status_callback(self, message):
        if message.command_id != self.trajectory_command_id:
            return
        self.trajectory_status = message
        if message.status in ('RUNNING', 'PAUSED'):
            self.trajectory_started = True
        if message.status in TERMINAL_STATES and self.terminal_at is None:
            self.terminal_at = time.monotonic()

    def control_callback(self, message):
        if message.command_id != self.trajectory_command_id:
            return
        self.control_states.append(str(message.state))

    def refresh_transform(self):
        try:
            message = self.tf_buffer.lookup_transform(
                'world', 'j2n6s300_end_effector', Time()
            )
            self.measured_transform = transform_from_tf(message)
        except (tf2_ros.TransformException, ValueError):
            return

    @staticmethod
    def set_duration(message, seconds):
        whole = int(seconds)
        message.duration.sec = whole
        message.duration.nanosec = int((seconds - whole) * 1.0e9)

    def send_ready_pose(self):
        now = self.get_clock().now()
        message = JointCommand()
        message.stamp = now.to_msg()
        message.command_id = f'cartesian_path_ready_{now.nanoseconds}'
        message.joint_names = list(JOINT_NAMES)
        message.positions = list(READY_POSITIONS)
        self.set_duration(message, 5.0)
        self.ready_command_id = message.command_id
        self.ready_publisher.publish(message)
        self.phase = 'MOVING_READY'
        self.phase_started_at = time.monotonic()

    def send_trajectory(self):
        now = self.get_clock().now()
        message = CartesianTrajectoryCommand()
        message.stamp = now.to_msg()
        message.command_id = f'cartesian_path_validation_{now.nanoseconds}'
        message.frame_id = 'world'
        message.mode = 'RELATIVE'
        message.x = -0.12
        message.y = 0.0
        message.z = 0.0
        message.roll = 0.0
        message.pitch = 0.0
        message.yaw = math.radians(20.0)
        message.linear_speed_mps = 0.04
        message.angular_speed_radps = math.radians(10.0)
        self.trajectory_command_id = message.command_id
        self.trajectory_publisher.publish(message)
        self.phase = 'MOVING_CARTESIAN'
        self.phase_started_at = time.monotonic()

    def tick(self):
        self.refresh_transform()
        now = time.monotonic()
        if self.phase == 'WAITING':
            connected = (
                self.ready_publisher.get_subscription_count() > 0
                and self.trajectory_publisher.get_subscription_count() > 0
            )
            if self.system_ready and connected and self.measured_transform is not None:
                self.send_ready_pose()
            return
        if self.phase == 'READY_TERMINAL':
            if (
                self.ready_terminal is not None
                and self.ready_terminal.status == 'SUCCEEDED'
                and now - self.phase_started_at >= 1.0
            ):
                self.send_trajectory()

    def complete(self):
        if self.terminal_at is not None:
            return time.monotonic() - self.terminal_at >= 2.0
        if (
            self.ready_terminal is not None
            and self.ready_terminal.status != 'SUCCEEDED'
        ):
            return True
        return time.monotonic() - self.started_at >= self.timeout_sec

    def final_target(self):
        status = self.trajectory_status
        if status is None or not status.frame_id:
            return None
        return transform_matrix(
            (status.target_x, status.target_y, status.target_z),
            (status.target_roll, status.target_pitch, status.target_yaw),
        )

    def report(self):
        self.target_transform = self.final_target()
        checks = []

        def add_check(name, passed, detail):
            checks.append(bool(passed))
            result = 'OK' if passed else 'FALLO'
            print(f'CHECK_{name}={result} | {detail}')

        print('=== VALIDACION TRAYECTORIA CARTESIANA SIMULADA V5 ===')
        print(f'COMANDO={self.trajectory_command_id or "AUSENTE"}')
        print('MODO=RELATIVE')
        print('DESPLAZAMIENTO_SOLICITADO_M=-0.120,0.000,0.000')
        print('GIRO_SOLICITADO_GRADOS=0.0,0.0,20.0')
        print('VELOCIDAD_LINEAL_MPS=0.040')
        print('VELOCIDAD_ANGULAR_GRADOS_S=10.0')

        add_check(
            'SISTEMA_READY', self.system_ready, 'readiness global observado'
        )
        ready_state = (
            'AUSENTE'
            if self.ready_terminal is None else self.ready_terminal.status
        )
        add_check(
            'POSTURA_RECOMENDADA',
            ready_state == 'SUCCEEDED',
            f'estado={ready_state}',
        )
        add_check(
            'PLAN_CARTESIANO',
            self.trajectory_started,
            'trayectoria IK segmentada observada',
        )
        unique_states = list(dict.fromkeys(self.control_states))
        add_check(
            'SUPERVISION_CONTINUA',
            bool(unique_states),
            'estados=' + (','.join(unique_states) or 'AUSENTES'),
        )
        terminal_state = (
            'AUSENTE'
            if self.trajectory_status is None
            else self.trajectory_status.status
        )
        terminal_detail = (
            ''
            if self.trajectory_status is None
            else self.trajectory_status.detail
        )
        add_check(
            'EJECUCION_CARTESIANA',
            terminal_state == 'SUCCEEDED',
            f'estado={terminal_state}; {terminal_detail}',
        )

        planned_duration = (
            math.nan
            if self.trajectory_status is None
            else self.trajectory_status.planned_duration_sec
        )
        print(f'DURACION_NOMINAL_S={planned_duration:.3f}')
        add_check(
            'VELOCIDAD_CONFIGURABLE',
            math.isfinite(planned_duration)
            and math.isclose(planned_duration, 3.0, abs_tol=0.05),
            f'duración esperada=3.000 s; observada={planned_duration:.3f} s',
        )

        position_norm = math.inf
        orientation_norm = math.inf
        if self.target_transform is not None and self.measured_transform is not None:
            position_error, orientation_error = pose_error(
                self.target_transform, self.measured_transform
            )
            position_norm = math.sqrt(
                sum(value * value for value in position_error)
            )
            orientation_norm = math.sqrt(
                sum(value * value for value in orientation_error)
            )
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
        passed = all(checks)
        print(f'RESULTADO={"OK" if passed else "REVISAR"}')
        return passed


def main(args=None):
    """Run the finite automatic Gazebo validation."""
    rclpy.init(args=args)
    node = CartesianTrajectoryValidator()
    passed = False
    try:
        while rclpy.ok() and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
        node.refresh_transform()
        passed = node.report()
    except KeyboardInterrupt:
        node.refresh_transform()
        passed = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
