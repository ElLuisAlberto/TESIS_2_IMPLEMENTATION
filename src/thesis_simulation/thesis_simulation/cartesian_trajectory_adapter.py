"""Plan long Cartesian paths and stream them through the JOG supervisor."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from thesis_core.cartesian_kinematics import (
    align_relative_target,
    pose_error,
    solve_inverse_kinematics_transform,
    transform_from_translation_quaternion,
)
from thesis_core.cartesian_path import (
    cartesian_distance,
    interpolate_joint_path,
    interpolate_transform,
    path_segment_count,
    synchronized_duration,
)
from thesis_core.jaco_kinematics import (
    end_effector_transform,
    rotation_matrix_to_rpy,
    transform_matrix,
    translation,
)
from thesis_core.joint_model import JOINT_NAMES
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import (
    CartesianTrajectoryCommand,
    CartesianTrajectoryStatus,
    ExecutionControl,
    JointCommand,
)
from thesis_simulation.cartesian_trajectory_model import (
    advance_progress,
    minimum_joint_path_duration,
    rolling_target_fraction,
)
import tf2_ros


class CartesianTrajectoryAdapter(Node):
    """Convert one Cartesian intention into supervised rolling joint intents."""

    def __init__(self):
        super().__init__('cartesian_trajectory_adapter')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('horizon_sec', 1.0)
        self.declare_parameter('maximum_state_age_sec', 0.50)
        self.declare_parameter('maximum_displacement_m', 0.60)
        self.declare_parameter('maximum_rotation_rad', math.pi)
        self.declare_parameter('minimum_linear_speed_mps', 0.005)
        self.declare_parameter('maximum_linear_speed_mps', 0.15)
        self.declare_parameter(
            'minimum_angular_speed_radps', math.radians(0.5)
        )
        self.declare_parameter(
            'maximum_angular_speed_radps', math.radians(30.0)
        )
        self.declare_parameter('maximum_nominal_duration_sec', 120.0)
        self.declare_parameter('position_tolerance_m', 0.003)
        self.declare_parameter(
            'orientation_tolerance_rad', math.radians(1.0)
        )
        self.declare_parameter('completion_position_tolerance_m', 0.015)
        self.declare_parameter(
            'completion_orientation_tolerance_rad', math.radians(3.0)
        )
        self.declare_parameter('completion_timeout_sec', 5.0)
        self.declare_parameter('maximum_control_age_sec', 0.50)
        self.declare_parameter('joint_timing_margin', 1.20)

        self.rate_hz = self.positive_parameter('rate_hz')
        self.horizon_sec = self.positive_parameter('horizon_sec')
        if not 0.1 <= self.horizon_sec <= 2.0:
            raise ValueError('horizon_sec must belong to [0.1, 2.0]')
        self.maximum_state_age_sec = self.positive_parameter(
            'maximum_state_age_sec'
        )
        self.maximum_displacement_m = self.positive_parameter(
            'maximum_displacement_m'
        )
        self.maximum_rotation_rad = self.positive_parameter(
            'maximum_rotation_rad'
        )
        self.minimum_linear_speed_mps = self.positive_parameter(
            'minimum_linear_speed_mps'
        )
        self.maximum_linear_speed_mps = self.positive_parameter(
            'maximum_linear_speed_mps'
        )
        self.minimum_angular_speed_radps = self.positive_parameter(
            'minimum_angular_speed_radps'
        )
        self.maximum_angular_speed_radps = self.positive_parameter(
            'maximum_angular_speed_radps'
        )
        self.maximum_nominal_duration_sec = self.positive_parameter(
            'maximum_nominal_duration_sec'
        )
        self.position_tolerance_m = self.positive_parameter(
            'position_tolerance_m'
        )
        self.orientation_tolerance_rad = self.positive_parameter(
            'orientation_tolerance_rad'
        )
        self.completion_position_tolerance_m = self.positive_parameter(
            'completion_position_tolerance_m'
        )
        self.completion_orientation_tolerance_rad = self.positive_parameter(
            'completion_orientation_tolerance_rad'
        )
        self.completion_timeout_sec = self.positive_parameter(
            'completion_timeout_sec'
        )
        self.maximum_control_age_sec = self.positive_parameter(
            'maximum_control_age_sec'
        )
        self.joint_timing_margin = self.positive_parameter(
            'joint_timing_margin'
        )

        if self.minimum_linear_speed_mps > self.maximum_linear_speed_mps:
            raise ValueError('linear speed range is inverted')
        if self.minimum_angular_speed_radps > self.maximum_angular_speed_radps:
            raise ValueError('angular speed range is inverted')
        if self.joint_timing_margin < 1.0:
            raise ValueError('joint_timing_margin must be at least 1.0')

        self.positions = {}
        self.state_received_at = None
        self.active = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rolling_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.intent_publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', rolling_qos
        )
        self.status_publisher = self.create_publisher(
            CartesianTrajectoryStatus,
            '/thesis/cartesian_trajectory_status',
            10,
        )
        self.create_subscription(
            CartesianTrajectoryCommand,
            '/thesis/cartesian_trajectory_candidate',
            self.command_callback,
            10,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.control_callback,
            20,
        )
        self.create_subscription(
            JointState, '/joint_states', self.state_callback, 20
        )
        self.create_subscription(
            String,
            '/thesis/cartesian_trajectory_cancel',
            self.cancel_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
        self.get_logger().info(
            'Cartesian trajectory adapter ready: simulation only, '
            f'maximum displacement={self.maximum_displacement_m:.2f} m, '
            f'horizon={self.horizon_sec:.2f} s'
        )

    def positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value

    def state_callback(self, message):
        positions = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if name in JOINT_NAMES and math.isfinite(value)
        }
        if all(name in positions for name in JOINT_NAMES):
            self.positions = positions
            self.state_received_at = time.monotonic()

    def current_joint_vector(self):
        if self.state_received_at is None:
            return None
        if time.monotonic() - self.state_received_at > self.maximum_state_age_sec:
            return None
        return tuple(self.positions[name] for name in JOINT_NAMES)

    def current_world_transform(self):
        message = self.tf_buffer.lookup_transform(
            'world', 'j2n6s300_end_effector', Time()
        )
        transform = message.transform
        return transform_from_translation_quaternion(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    @staticmethod
    def command_values(message):
        return (
            float(message.x),
            float(message.y),
            float(message.z),
            float(message.roll),
            float(message.pitch),
            float(message.yaw),
            float(message.linear_speed_mps),
            float(message.angular_speed_radps),
        )

    def target_transform(self, message, current_world):
        values = self.command_values(message)
        pose = values[:6]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('la orden contiene un valor no finito')
        if message.mode == 'ABSOLUTE':
            return transform_matrix(pose[:3], pose[3:])
        if message.mode != 'RELATIVE':
            raise ValueError('mode debe ser RELATIVE o ABSOLUTE')
        current_position = translation(current_world)
        current_rpy = rotation_matrix_to_rpy(current_world)
        target_position = tuple(
            current + delta
            for current, delta in zip(current_position, pose[:3])
        )
        target_rpy = tuple(
            current + delta
            for current, delta in zip(current_rpy, pose[3:])
        )
        return transform_matrix(target_position, target_rpy)

    def validate_speed(self, message):
        linear = float(message.linear_speed_mps)
        angular = float(message.angular_speed_radps)
        if not self.minimum_linear_speed_mps <= linear <= self.maximum_linear_speed_mps:
            raise ValueError(
                'velocidad lineal fuera de '
                f'[{self.minimum_linear_speed_mps:.3f}, '
                f'{self.maximum_linear_speed_mps:.3f}] m/s'
            )
        if not self.minimum_angular_speed_radps <= angular <= self.maximum_angular_speed_radps:
            raise ValueError(
                'velocidad angular fuera de '
                f'[{math.degrees(self.minimum_angular_speed_radps):.1f}, '
                f'{math.degrees(self.maximum_angular_speed_radps):.1f}] grados/s'
            )

    def plan_joint_path(
        self, start_joints, current_world, target_world, distance, angle
    ):
        segment_count = path_segment_count(distance, angle)
        current_model = end_effector_transform(start_joints)
        seed = tuple(start_joints)
        path = [seed]
        for index in range(1, segment_count + 1):
            fraction = index / float(segment_count)
            world_waypoint = interpolate_transform(
                current_world, target_world, fraction
            )
            model_waypoint = align_relative_target(
                current_world, world_waypoint, current_model
            )
            result = solve_inverse_kinematics_transform(
                seed,
                model_waypoint,
                position_tolerance_m=self.position_tolerance_m,
                orientation_tolerance_rad=self.orientation_tolerance_rad,
            )
            if not result.success:
                raise ValueError(
                    f'IK falló en {index}/{segment_count}: '
                    f'error lineal={result.position_error_m:.4f} m, '
                    'error angular='
                    f'{math.degrees(result.orientation_error_rad):.2f} grados'
                )
            seed = tuple(result.positions)
            path.append(seed)
        return tuple(path)

    def command_callback(self, message):
        if not message.command_id:
            self.reject(message.command_id, 'ID vacío.')
            return
        if self.active is not None:
            self.reject(
                message.command_id,
                f'Ya existe una trayectoria activa: {self.active["command_id"]}.',
            )
            return
        if message.frame_id != 'world':
            self.reject(message.command_id, 'frame_id debe ser world.')
            return

        start_joints = self.current_joint_vector()
        if start_joints is None:
            self.reject(
                message.command_id,
                'No existe un estado articular completo y reciente.',
            )
            return
        try:
            self.validate_speed(message)
            current_world = self.current_world_transform()
            target_world = self.target_transform(message, current_world)
            distance, angle = cartesian_distance(current_world, target_world)
            if distance <= 1.0e-6 and angle <= 1.0e-6:
                raise ValueError('el desplazamiento cartesiano es nulo')
            if distance > self.maximum_displacement_m + 1.0e-9:
                raise ValueError(
                    f'desplazamiento {distance:.3f} m; máximo '
                    f'{self.maximum_displacement_m:.3f} m'
                )
            if angle > self.maximum_rotation_rad + 1.0e-9:
                raise ValueError(
                    f'rotación {math.degrees(angle):.1f} grados; máximo '
                    f'{math.degrees(self.maximum_rotation_rad):.1f}'
                )
            duration = synchronized_duration(
                distance,
                angle,
                message.linear_speed_mps,
                message.angular_speed_radps,
            )
            if duration > self.maximum_nominal_duration_sec:
                raise ValueError(
                    f'duración nominal {duration:.1f} s; máximo '
                    f'{self.maximum_nominal_duration_sec:.1f} s'
                )
            path = self.plan_joint_path(
                start_joints,
                current_world,
                target_world,
                distance,
                angle,
            )
            minimum_duration = minimum_joint_path_duration(path)
            guarded_minimum_duration = (
                minimum_duration * self.joint_timing_margin
            )
            if duration + 1.0e-9 < guarded_minimum_duration:
                raise ValueError(
                    'la velocidad solicitada excede límites articulares; '
                    f'duración cartesiana={duration:.2f} s, mínimo con '
                    'margen de seguimiento='
                    f'{guarded_minimum_duration:.2f} s. Reduzca la velocidad.'
                )
        except (tf2_ros.TransformException, ValueError) as exception:
            self.reject(message.command_id, str(exception))
            return

        target_position = translation(target_world)
        target_rpy = rotation_matrix_to_rpy(target_world)
        now = time.monotonic()
        self.active = {
            'command_id': message.command_id,
            'path': path,
            'target_world': target_world,
            'target_position': target_position,
            'target_rpy': target_rpy,
            'distance': distance,
            'angle': angle,
            'duration': duration,
            'progress': 0.0,
            'speed_scale': 0.0,
            'control_state': 'WAITING',
            'control_detail': 'Esperando la primera decisión preventiva.',
            'control_seen': False,
            'control_received_at': None,
            'last_tick': now,
            'completion_started_at': None,
        }
        self.publish_active_status('RUNNING')
        self.get_logger().info(
            f'CARTESIAN TRAJECTORY PLANNED {message.command_id}: '
            f'distance={distance:.3f} m, '
            f'rotation={math.degrees(angle):.1f} deg, '
            f'duration={duration:.2f} s, segments={len(path) - 1}'
        )

    def control_callback(self, message):
        active = self.active
        if active is None or message.command_id != active['command_id']:
            return
        scale = float(message.speed_scale)
        if not math.isfinite(scale):
            scale = 0.0
        active['speed_scale'] = min(max(scale, 0.0), 1.0)
        active['control_state'] = str(message.state)
        active['control_detail'] = str(message.reason)
        active['control_seen'] = True
        active['control_received_at'] = time.monotonic()

    def cancel_callback(self, message):
        active = self.active
        if active is None:
            return
        requested = str(message.data).strip()
        if requested not in ('*', active['command_id']):
            return
        self.finish('CANCELED', 'Cancelada por el operador; watchdog en HOLD.')

    @staticmethod
    def set_duration(message, seconds):
        whole = int(seconds)
        nanoseconds = int(round((seconds - whole) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            whole += 1
            nanoseconds -= 1_000_000_000
        message.duration.sec = whole
        message.duration.nanosec = nanoseconds

    def publish_intent(self, active):
        target_fraction = rolling_target_fraction(
            active['progress'], active['duration'], self.horizon_sec
        )
        target = interpolate_joint_path(active['path'], target_fraction)
        message = JointCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = active['command_id']
        message.joint_names = list(JOINT_NAMES)
        message.positions = list(target)
        self.set_duration(message, self.horizon_sec)
        self.intent_publisher.publish(message)

    def measured_errors(self, active):
        try:
            current_world = self.current_world_transform()
        except (tf2_ros.TransformException, ValueError):
            return -1.0, -1.0
        position_error, orientation_error = pose_error(
            active['target_world'], current_world
        )
        return (
            math.sqrt(sum(value * value for value in position_error)),
            math.sqrt(sum(value * value for value in orientation_error)),
        )

    def timer_callback(self):
        active = self.active
        if active is None:
            return
        now = time.monotonic()
        elapsed = min(max(0.0, now - active['last_tick']), 0.25)
        active['last_tick'] = now

        control_time = active['control_received_at']
        if (
            active['control_seen']
            and control_time is not None
            and now - control_time > self.maximum_control_age_sec
        ):
            self.finish(
                'FAILED',
                'Se perdió la decisión preventiva; watchdog en HOLD.',
            )
            return

        if active['control_seen']:
            active['progress'] = advance_progress(
                active['progress'],
                active['speed_scale'],
                elapsed,
                active['duration'],
            )
        self.publish_intent(active)

        position_error, orientation_error = self.measured_errors(active)
        if active['progress'] >= 1.0 - 1.0e-9:
            if active['completion_started_at'] is None:
                active['completion_started_at'] = now
            if (
                0.0 <= position_error <= self.completion_position_tolerance_m
                and 0.0 <= orientation_error
                <= self.completion_orientation_tolerance_rad
            ):
                self.finish(
                    'SUCCEEDED',
                    'Objetivo cartesiano alcanzado dentro de tolerancia.',
                    position_error,
                    orientation_error,
                )
                return
            if now - active['completion_started_at'] > self.completion_timeout_sec:
                self.finish(
                    'FAILED',
                    'El controlador no alcanzó la tolerancia final a tiempo.',
                    position_error,
                    orientation_error,
                )
                return
        status = (
            'PAUSED'
            if active['control_seen'] and active['speed_scale'] <= 0.0
            else 'RUNNING'
        )
        self.publish_active_status(status, position_error, orientation_error)

    def fill_target(self, message, active):
        message.frame_id = 'world'
        (
            message.target_x,
            message.target_y,
            message.target_z,
        ) = active['target_position']
        (
            message.target_roll,
            message.target_pitch,
            message.target_yaw,
        ) = active['target_rpy']

    def publish_active_status(
        self, status, position_error=-1.0, orientation_error=-1.0
    ):
        active = self.active
        if active is None:
            return
        message = CartesianTrajectoryStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = active['command_id']
        message.status = status
        control_text = active['control_state']
        message.detail = (
            f'{control_text}: {active["control_detail"]}'
            if active['control_seen'] else active['control_detail']
        )
        message.progress = float(active['progress'])
        message.speed_scale = float(active['speed_scale'])
        message.planned_duration_sec = float(active['duration'])
        message.remaining_distance_m = max(
            0.0, active['distance'] * (1.0 - active['progress'])
        )
        message.position_error_m = float(position_error)
        message.orientation_error_rad = float(orientation_error)
        self.fill_target(message, active)
        self.status_publisher.publish(message)

    def reject(self, command_id, detail):
        message = CartesianTrajectoryStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = command_id
        message.status = 'REJECTED'
        message.detail = detail
        message.progress = 0.0
        message.speed_scale = 0.0
        message.planned_duration_sec = 0.0
        message.remaining_distance_m = -1.0
        message.position_error_m = -1.0
        message.orientation_error_rad = -1.0
        message.frame_id = 'world'
        self.status_publisher.publish(message)
        self.get_logger().warning(
            f'CARTESIAN TRAJECTORY REJECTED {command_id}: {detail}'
        )

    def finish(
        self,
        status,
        detail,
        position_error=-1.0,
        orientation_error=-1.0,
    ):
        active = self.active
        if active is None:
            return
        message = CartesianTrajectoryStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = active['command_id']
        message.status = status
        message.detail = detail
        message.progress = float(active['progress'])
        message.speed_scale = float(active['speed_scale'])
        message.planned_duration_sec = float(active['duration'])
        message.remaining_distance_m = max(
            0.0, active['distance'] * (1.0 - active['progress'])
        )
        message.position_error_m = float(position_error)
        message.orientation_error_rad = float(orientation_error)
        self.fill_target(message, active)
        self.status_publisher.publish(message)
        self.get_logger().info(
            f'CARTESIAN TRAJECTORY {status} {active["command_id"]}: {detail}'
        )
        self.active = None


def main(args=None):
    """Run the simulation-only long Cartesian trajectory adapter."""
    rclpy.init(args=args)
    spin_node(CartesianTrajectoryAdapter())


if __name__ == '__main__':
    main()
