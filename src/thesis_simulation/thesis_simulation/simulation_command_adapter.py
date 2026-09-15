"""Forward supervised commands and apply runtime safety controls."""

import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from thesis_core.execution_reference import normalize_target


JOINTS = tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))
MIN_REPLAN_DURATION_SEC = 0.15
MAX_REPLAN_DURATION_SEC = 30.0


class SimulationCommandAdapter(Node):
    """Send accepted commands to Gazebo and control the active goal."""

    def __init__(self):
        super().__init__('simulation_command_adapter')

        self.declare_parameter('simulation_output_enabled', False)
        self.declare_parameter(
            'action_name',
            '/arm_controller/follow_joint_trajectory',
        )
        self.declare_parameter('max_state_age_sec', 0.5)
        self.declare_parameter('control_replan_cooldown_sec', 0.75)

        action_name = self.get_parameter('action_name').value
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name,
        )

        self.execution_publisher = self.create_publisher(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            10,
        )
        self.state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.command_callback,
            10,
        )
        self.control_subscription = self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.control_callback,
            10,
        )

        self.current_positions = {}
        self.last_state_monotonic = None
        self.goal_active = False
        self.goal_handle = None
        self.active_metadata = None
        self.goal_generation = 0
        self.control_in_progress = False
        self.pending_control = None
        self.applied_speed_scale = 1.0
        self.last_control_change_monotonic = None

        output_enabled = bool(
            self.get_parameter('simulation_output_enabled').value
        )
        output_state = 'habilitada' if output_enabled else 'desactivada'
        self.get_logger().info(
            f'Adaptador Gazebo preparado; salida simulada {output_state}'
        )

    def state_callback(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS and math.isfinite(position):
                positions[name] = float(position)

        if all(name in positions for name in JOINTS):
            self.current_positions = positions
            self.last_state_monotonic = time.monotonic()

    def duration_seconds(self, msg):
        return float(msg.duration.sec) + float(msg.duration.nanosec) * 1e-9

    def current_state(self):
        if self.last_state_monotonic is None:
            return None

        max_age = float(self.get_parameter('max_state_age_sec').value)
        if time.monotonic() - self.last_state_monotonic > max_age:
            return None

        return tuple(self.current_positions[name] for name in JOINTS)

    def publish_status(
        self,
        command_id,
        status,
        detail,
        start_time=None,
        start_positions=(),
        target_positions=(),
        duration=0.0,
    ):
        """Publish one execution lifecycle update."""
        msg = ExecutionTrajectory()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.status = status
        msg.detail = detail
        if start_time is not None:
            msg.start_time = start_time
        msg.joint_names = list(JOINTS)
        msg.start_positions = list(start_positions)
        msg.target_positions = list(target_positions)
        msg.duration_sec = float(duration)
        self.execution_publisher.publish(msg)

    def reject(self, msg, status, detail):
        self.publish_status(
            msg.command_id,
            status,
            detail,
            target_positions=msg.positions,
            duration=self.duration_seconds(msg),
        )
        self.get_logger().warning(f'{status} {msg.command_id}: {detail}')

    @staticmethod
    def point_time_from_seconds(point, duration):
        whole_seconds = int(duration)
        nanoseconds = int(round(
            (duration - whole_seconds) * 1_000_000_000
        ))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

    def build_goal(self, start, target, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)

        initial_point = JointTrajectoryPoint()
        initial_point.positions = list(start)
        initial_point.time_from_start.nanosec = 1

        target_point = JointTrajectoryPoint()
        target_point.positions = list(target)
        self.point_time_from_seconds(target_point, duration)

        goal.trajectory.points = [initial_point, target_point]
        return goal

    def start_goal(self, metadata, detail):
        """Send a goal and associate every callback with its generation."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.publish_status(
                metadata['command_id'],
                'FAILED',
                'no se encontró el action server de Gazebo',
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.clear_active_goal()
            return

        self.goal_generation += 1
        generation = self.goal_generation
        metadata['generation'] = generation
        metadata['accepted'] = False
        metadata['start_time'] = None
        self.active_metadata = metadata
        self.goal_handle = None
        self.goal_active = True

        self.publish_status(
            metadata['command_id'],
            'PENDING',
            detail,
            start_positions=metadata['start'],
            target_positions=metadata['target'],
            duration=metadata['duration'],
        )

        goal = self.build_goal(
            metadata['start'],
            metadata['target'],
            metadata['duration'],
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result_future, token=generation:
            self.goal_response_callback(result_future, token)
        )
        self.get_logger().info(
            f'GOAL PENDING id={metadata["command_id"]}, '
            f'duration={metadata["duration"]:.2f} s, '
            f'scale={metadata["speed_scale"]:.2f}'
        )

    def initial_metadata(self, msg, start, target, duration):
        return {
            'command_id': msg.command_id,
            'origin_start': tuple(start),
            'origin_target': tuple(target),
            'start': tuple(start),
            'target': tuple(target),
            'nominal_duration': float(duration),
            'duration': float(duration),
            'speed_scale': 1.0,
        }

    def command_callback(self, msg):
        duration = self.duration_seconds(msg)
        output_enabled = bool(
            self.get_parameter('simulation_output_enabled').value
        )

        if not output_enabled:
            self.publish_status(
                msg.command_id,
                'DRY_RUN',
                'Salida a Gazebo desactivada; no existe referencia aceptada.',
                target_positions=msg.positions,
                duration=duration,
            )
            self.get_logger().info(
                f'SIMULATION-DRY-RUN id={msg.command_id}: '
                'salida a Gazebo desactivada'
            )
            return

        if self.goal_active:
            self.reject(
                msg,
                'REJECTED',
                'ya existe una trayectoria activa en Gazebo',
            )
            return

        if list(msg.joint_names) != list(JOINTS):
            self.reject(
                msg,
                'REJECTED',
                'se esperaban las seis articulaciones en orden canónico',
            )
            return

        if not math.isfinite(duration) or duration <= 0:
            self.reject(msg, 'REJECTED', 'duración inválida')
            return

        if not all(math.isfinite(value) for value in msg.positions):
            self.reject(msg, 'REJECTED', 'posición articular no finita')
            return

        start = self.current_state()
        if start is None:
            self.reject(
                msg,
                'FAILED',
                'no existe un /joint_states reciente para iniciar',
            )
            return

        try:
            target = normalize_target(start, tuple(msg.positions))
        except ValueError as exc:
            self.reject(msg, 'REJECTED', str(exc))
            return

        metadata = self.initial_metadata(msg, start, target, duration)
        self.start_goal(
            metadata,
            'Esperando aceptación del action server de Gazebo.',
        )

    def goal_response_callback(self, future, generation):
        metadata = self.active_metadata
        if metadata is None or metadata['generation'] != generation:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.publish_status(
                metadata['command_id'],
                'FAILED',
                f'Error enviando goal a Gazebo: {exc}',
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.clear_active_goal()
            return

        if not goal_handle.accepted:
            self.publish_status(
                metadata['command_id'],
                'REJECTED',
                'Gazebo rechazó la trayectoria.',
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.clear_active_goal()
            self.get_logger().warning(
                f'Gazebo rechazó la trayectoria {metadata["command_id"]}'
            )
            return

        self.goal_handle = goal_handle
        metadata['accepted'] = True
        metadata['start_time'] = self.get_clock().now().to_msg()
        self.applied_speed_scale = metadata['speed_scale']
        self.publish_status(
            metadata['command_id'],
            'ACCEPTED',
            'Referencia aceptada por el action server de Gazebo.',
            start_time=metadata['start_time'],
            start_positions=metadata['start'],
            target_positions=metadata['target'],
            duration=metadata['duration'],
        )
        self.get_logger().info(
            f'GAZEBO ACCEPTED id={metadata["command_id"]}, '
            f'scale={metadata["speed_scale"]:.2f}'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, token=generation:
            self.result_callback(result, token)
        )

        if self.pending_control is not None:
            pending = self.pending_control
            self.pending_control = None
            self.control_callback(pending)

    def result_callback(self, future, generation):
        metadata = self.active_metadata
        if metadata is None or metadata['generation'] != generation:
            return
        if self.control_in_progress:
            return

        try:
            result = future.result().result
            error_code = result.error_code
            detail = result.error_string or f'error_code={error_code}'
            if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                status = 'SUCCEEDED'
            else:
                status = 'FAILED'
        except Exception as exc:
            status = 'FAILED'
            detail = f'Error obteniendo resultado de Gazebo: {exc}'

        self.publish_status(
            metadata['command_id'],
            status,
            detail,
            start_time=metadata['start_time'],
            start_positions=metadata['start'],
            target_positions=metadata['target'],
            duration=metadata['duration'],
        )
        self.get_logger().info(
            f'GAZEBO {status} id={metadata["command_id"]}: {detail}'
        )
        self.clear_active_goal()

    def control_callback(self, msg):
        """Apply the latest runtime safety decision to the active goal."""
        metadata = self.active_metadata
        if (
            not self.goal_active
            or metadata is None
            or msg.command_id != metadata['command_id']
        ):
            return

        if not metadata['accepted'] or self.goal_handle is None:
            self.pending_control = msg
            return

        if self.control_in_progress:
            self.pending_control = msg
            return

        if msg.state == 'STOP' or msg.speed_scale <= 0.0:
            desired_scale = 0.0
        elif msg.state == 'REDUCTION':
            desired_scale = min(max(float(msg.speed_scale), 0.05), 1.0)
        else:
            desired_scale = 1.0

        if abs(desired_scale - self.applied_speed_scale) <= 0.05:
            return

        # Do not immediately restore nominal speed after a reduction.  The
        # supervisor has hysteresis as well, but this second guard prevents a
        # cancel/replan storm if consecutive runtime samples disagree.
        cooldown = float(
            self.get_parameter('control_replan_cooldown_sec').value
        )
        now_monotonic = time.monotonic()
        restoring_speed = (
            desired_scale > self.applied_speed_scale + 0.05
        )
        if (
            restoring_speed
            and self.last_control_change_monotonic is not None
            and now_monotonic - self.last_control_change_monotonic < cooldown
        ):
            self.pending_control = msg
            return

        self.pending_control = msg
        self.control_in_progress = True
        self.last_control_change_monotonic = now_monotonic
        generation = metadata['generation']
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda result, token=generation:
            self.cancel_callback(result, token)
        )
        self.get_logger().warning(
            f'CONTROL {metadata["command_id"]}: {msg.state}, '
            f'scale={desired_scale:.2f}; cancelando goal activo'
        )

    def trajectory_progress(self, current, metadata):
        """Estimate progress using the slowest remaining joint."""
        start = metadata['origin_start']
        target = metadata['origin_target']
        remaining_fractions = []
        for index, (start_value, target_value, current_value) in enumerate(
                zip(start, target, current)):
            total_delta = target_value - start_value
            current_target = target_value
            if index in (0, 3, 4, 5):
                current_target = current_value + math.atan2(
                    math.sin(target_value - current_value),
                    math.cos(target_value - current_value),
                )
                total_delta = math.atan2(
                    math.sin(total_delta),
                    math.cos(total_delta),
                )
                current_delta = current_target - current_value
            else:
                current_delta = target_value - current_value

            if abs(total_delta) <= 1.0e-6:
                continue
            remaining_fractions.append(
                min(1.0, max(0.0, abs(current_delta / total_delta)))
            )

        if not remaining_fractions:
            return 1.0
        return 1.0 - max(remaining_fractions)

    def cancel_callback(self, future, generation):
        metadata = self.active_metadata
        if metadata is None or metadata['generation'] != generation:
            return

        control = self.pending_control
        self.pending_control = None
        self.goal_handle = None

        if (
            control is None
            or control.state == 'STOP'
            or control.speed_scale <= 0.0
        ):
            detail = (
                control.reason
                if control is not None
                else 'Parada solicitada por el supervisor.'
            )
            self.publish_status(
                metadata['command_id'],
                'CANCELED',
                f'Trayectoria cancelada por seguridad: {detail}',
                start_time=metadata['start_time'],
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.get_logger().warning(
                f'STOPPED id={metadata["command_id"]}: {detail}'
            )
            self.clear_active_goal()
            return

        current = self.current_state()
        if current is None:
            self.publish_status(
                metadata['command_id'],
                'CANCELED',
                'Trayectoria cancelada: estado articular no disponible.',
                start_time=metadata['start_time'],
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.clear_active_goal()
            return

        progress = self.trajectory_progress(current, metadata)
        remaining_nominal = metadata['nominal_duration'] * (
            1.0 - progress
        )
        scale = min(max(float(control.speed_scale), 0.05), 1.0)
        duration = max(
            MIN_REPLAN_DURATION_SEC,
            remaining_nominal / scale,
        )
        duration = min(duration, MAX_REPLAN_DURATION_SEC)
        try:
            target = normalize_target(current, metadata['origin_target'])
        except ValueError as exc:
            self.publish_status(
                metadata['command_id'],
                'CANCELED',
                f'Trayectoria cancelada: {exc}',
                start_time=metadata['start_time'],
                start_positions=metadata['start'],
                target_positions=metadata['target'],
                duration=metadata['duration'],
            )
            self.clear_active_goal()
            return

        replacement = {
            'command_id': metadata['command_id'],
            'origin_start': metadata['origin_start'],
            'origin_target': metadata['origin_target'],
            'start': tuple(current),
            'target': tuple(target),
            'nominal_duration': metadata['nominal_duration'],
            'duration': duration,
            'speed_scale': scale,
        }
        self.control_in_progress = False
        self.start_goal(
            replacement,
            (
                f'Replanificación preventiva; escala de velocidad '
                f'{scale:.2f}, duración restante {duration:.2f} s.'
            ),
        )

    def clear_active_goal(self):
        self.goal_active = False
        self.goal_handle = None
        self.active_metadata = None
        self.control_in_progress = False
        self.pending_control = None
        self.applied_speed_scale = 1.0
        self.last_control_change_monotonic = None


def main(args=None):
    rclpy.init(args=args)
    node = SimulationCommandAdapter()
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
