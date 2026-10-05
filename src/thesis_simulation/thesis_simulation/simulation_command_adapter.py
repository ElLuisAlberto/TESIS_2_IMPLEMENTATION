"""Forward supervised commands and apply runtime safety controls."""

import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    PipelineTiming,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from thesis_core.joint_model import (
    JOINT_NAMES,
    normalize_target,
    saturate_target_by_velocity,
)
from thesis_simulation.control_arbitration import (
    effective_speed_scale,
    is_more_restrictive,
)
from thesis_simulation.jog_reference import advance_jog_reference
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
        self.declare_parameter('jog_control_period_sec', 0.10)
        self.declare_parameter('jog_command_timeout_sec', 0.18)
        self.declare_parameter('max_jog_message_age_sec', 0.15)
        self.declare_parameter('jog_reference_max_lead_sec', 0.25)
        self.declare_parameter(
            'controller_reference_max_age_sec', 0.10
        )
        self.declare_parameter('jog_stop_duration_sec', 0.02)

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
        self.controller_state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.controller_state_callback,
            20,
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
        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.jog_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            jog_qos,
        )
        self.timing_publisher = self.create_publisher(
            PipelineTiming, '/thesis/pipeline_timing', 100
        )
        self.timing_sequence = 0
        jog_command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.jog_subscription = self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self.jog_command_callback,
            jog_command_qos,
        )

        maximum_jog_age = float(
            self.get_parameter('max_jog_message_age_sec').value
        )
        jog_timeout = float(
            self.get_parameter('jog_command_timeout_sec').value
        )
        if (
            not math.isfinite(maximum_jog_age)
            or not 0.0 < maximum_jog_age < jog_timeout
        ):
            raise ValueError(
                'max_jog_message_age_sec must be positive and smaller '
                'than jog_command_timeout_sec'
            )

        self.current_positions = {}
        self.last_state_monotonic = None
        self.controller_reference_positions = {}
        self.last_controller_reference_monotonic = None
        self.goal_active = False
        self.goal_handle = None
        self.active_metadata = None
        self.goal_generation = 0
        self.control_in_progress = False
        self.pending_control = None
        self.applied_speed_scale = 1.0
        self.last_control_change_monotonic = None
        self.jog_active = False
        self.last_jog_receive_monotonic = None
        self.jog_hold_sent = False
        self.jog_reference_target = None
        self.jog_reference_velocity = None
        self.last_jog_intent_stamp = None
        self.last_jog_command_id = None
        self.jog_watchdog = self.create_timer(
            0.05,
            self.jog_watchdog_callback,
        )

        output_enabled = bool(
            self.get_parameter('simulation_output_enabled').value
        )
        output_state = 'habilitada' if output_enabled else 'desactivada'
        self.get_logger().info(
            f'Adaptador Gazebo preparado; salida simulada {output_state}'
        )

    def publish_timing(
        self,
        command_id,
        intent_stamp,
        stage,
        detail='',
        internal_duration_sec=-1.0,
        monotonic_ns=None,
    ):
        """Publish one adapter trace event on the shared ROS clock."""
        event = PipelineTiming()
        event.stamp = self.get_clock().now().to_msg()
        event.intent_stamp = intent_stamp
        event.command_id = command_id
        event.source = 'adapter'
        event.stage = stage
        self.timing_sequence += 1
        event.sequence = self.timing_sequence
        event.monotonic_ns = int(
            time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        )
        event.internal_duration_sec = float(internal_duration_sec)
        event.detail = str(detail)
        self.timing_publisher.publish(event)

    def state_callback(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_NAMES and math.isfinite(position):
                positions[name] = float(position)

        if all(name in positions for name in JOINT_NAMES):
            self.current_positions = positions
            self.last_state_monotonic = time.monotonic()

    def controller_state_callback(self, msg):
        """Store the controller's current desired position as JOG anchor."""
        if tuple(msg.joint_names) != JOINT_NAMES:
            return
        positions = tuple(float(value) for value in msg.desired.positions)
        if (
            len(positions) != len(JOINT_NAMES)
            or not all(math.isfinite(value) for value in positions)
        ):
            return
        self.controller_reference_positions = dict(zip(JOINT_NAMES, positions))
        self.last_controller_reference_monotonic = time.monotonic()

    def controller_reference_state(self):
        """Return a fresh controller reference, or None for safe fallback."""
        if self.last_controller_reference_monotonic is None:
            return None
        maximum_age = float(
            self.get_parameter('controller_reference_max_age_sec').value
        )
        if (
            not math.isfinite(maximum_age)
            or maximum_age <= 0.0
            or time.monotonic() - self.last_controller_reference_monotonic
            > maximum_age
        ):
            return None
        if not all(
            name in self.controller_reference_positions
            for name in JOINT_NAMES
        ):
            return None
        return tuple(
            self.controller_reference_positions[name]
            for name in JOINT_NAMES
        )

    def duration_seconds(self, msg):
        return float(msg.duration.sec) + float(msg.duration.nanosec) * 1e-9

    def current_state(self):
        if self.last_state_monotonic is None:
            return None

        max_age = float(self.get_parameter('max_state_age_sec').value)
        if time.monotonic() - self.last_state_monotonic > max_age:
            return None

        return tuple(self.current_positions[name] for name in JOINT_NAMES)

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
        msg.joint_names = list(JOINT_NAMES)
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
        goal.trajectory.joint_names = list(JOINT_NAMES)

        initial_point = JointTrajectoryPoint()
        initial_point.positions = list(start)
        initial_point.time_from_start.nanosec = 1

        target_point = JointTrajectoryPoint()
        target_point.positions = list(target)
        self.point_time_from_seconds(target_point, duration)

        goal.trajectory.points = [initial_point, target_point]
        return goal

    def publish_jog_target(self, target, velocities, duration):
        """Publish one short rolling position-and-slope JOG reference."""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = list(velocities)
        self.point_time_from_seconds(point, duration)
        trajectory.points = [point]
        self.jog_publisher.publish(trajectory)

    def jog_command_callback(self, msg):
        """Convert a safe one-second jog horizon into a short control step."""
        self.publish_timing(
            msg.command_id, msg.stamp, 'ADAPTER_RECEIVE'
        )
        if not bool(self.get_parameter('simulation_output_enabled').value):
            return
        if self.goal_active:
            return
        source_time = Time.from_msg(msg.stamp)
        source_age = (
            self.get_clock().now() - source_time
        ).nanoseconds * 1.0e-9
        maximum_age = float(
            self.get_parameter('max_jog_message_age_sec').value
        )
        if source_age < -0.05 or source_age > maximum_age:
            if self.jog_active:
                self.stop_active_jog(
                    f'comando JOG obsoleto ({source_age:.3f} s)'
                )
            return
        if tuple(msg.joint_names) != JOINT_NAMES:
            return

        horizon = self.duration_seconds(msg)
        control_period = float(
            self.get_parameter('jog_control_period_sec').value
        )
        maximum_lead = float(
            self.get_parameter('jog_reference_max_lead_sec').value
        )
        if (
            not math.isfinite(horizon)
            or horizon <= 0.0
            or not 0.02 <= control_period <= 0.25
            or not control_period <= maximum_lead <= 0.50
        ):
            return

        current = self.current_state()
        if current is None:
            return
        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                horizon,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f'JOG ADAPTER REJECTED: {exc}'
            )
            return
        if saturation.was_limited:
            self.get_logger().error(
                'JOG ADAPTER REJECTED: supervisor velocity invariant '
                f'violated by {saturation.limiting_joint}; '
                f'requested={saturation.requested_velocity:.4f}rad/s, '
                f'limit={saturation.limited_velocity:.4f}rad/s'
            )
            return
        horizon_target = saturation.positions

        previous_target = (
            self.controller_reference_state() if self.jog_active else None
        )
        previous_velocity = (
            self.jog_reference_velocity
            if previous_target is not None
            else None
        )
        try:
            control_target, control_velocity = advance_jog_reference(
                current,
                horizon_target,
                horizon,
                control_period,
                previous_target,
                previous_velocity,
                maximum_lead,
            )
        except ValueError as exc:
            self.get_logger().warning(f'JOG ADAPTER REJECTED: {exc}')
            return
        self.publish_jog_target(
            control_target,
            control_velocity,
            control_period,
        )
        self.publish_timing(
            msg.command_id, msg.stamp, 'CONTROLLER_PUBLISH'
        )
        self.jog_reference_target = control_target
        self.jog_reference_velocity = control_velocity
        self.last_jog_intent_stamp = msg.stamp
        self.last_jog_command_id = msg.command_id
        self.jog_active = True
        self.last_jog_receive_monotonic = time.monotonic()
        self.jog_hold_sent = False

    def jog_watchdog_callback(self):
        """Hold the measured pose if the continuous command stream stops."""
        if (
            not self.jog_active
            or self.last_jog_receive_monotonic is None
            or self.jog_hold_sent
        ):
            return
        timeout = float(
            self.get_parameter('jog_command_timeout_sec').value
        )
        if time.monotonic() - self.last_jog_receive_monotonic <= timeout:
            return
        if (
            self.last_jog_intent_stamp is not None
            and self.last_jog_command_id is not None
        ):
            self.publish_timing(
                self.last_jog_command_id,
                self.last_jog_intent_stamp,
                'STOP_DETECTED',
                'JOG_WATCHDOG_EXPIRED',
            )
        current = self.current_state()
        if current is not None:
            duration = float(
                self.get_parameter('jog_stop_duration_sec').value
            )
            if not math.isfinite(duration) or not 0.01 <= duration <= 0.10:
                self.get_logger().error(
                    'JOG HOLD REJECTED: invalid jog_stop_duration_sec'
                )
                return
            self.publish_jog_target(
                current,
                (0.0,) * len(JOINT_NAMES),
                duration,
            )
            hold_publish_monotonic = time.monotonic()
            hold_age_sec = max(
                0.0,
                hold_publish_monotonic
                - self.last_jog_receive_monotonic,
            )
            if (
                self.last_jog_intent_stamp is not None
                and self.last_jog_command_id is not None
            ):
                self.publish_timing(
                    self.last_jog_command_id,
                    self.last_jog_intent_stamp,
                    'HOLD_PUBLISH',
                    'JOG_WATCHDOG_EXPIRED',
                    internal_duration_sec=hold_age_sec,
                )
        self.jog_hold_sent = True
        self.jog_active = False
        self.jog_reference_target = None
        self.jog_reference_velocity = None

    def stop_active_jog(self, reason):
        """Hold the measured pose immediately after a supervised STOP."""
        current = self.current_state()
        if current is not None:
            duration = float(
                self.get_parameter('jog_stop_duration_sec').value
            )
            if not math.isfinite(duration) or not 0.01 <= duration <= 0.10:
                self.get_logger().error(
                    'JOG HOLD REJECTED: invalid jog_stop_duration_sec'
                )
                return
            self.publish_jog_target(
                current,
                (0.0,) * len(JOINT_NAMES),
                duration,
            )
        self.jog_hold_sent = True
        self.jog_active = False
        self.jog_reference_target = None
        self.jog_reference_velocity = None
        self.last_jog_receive_monotonic = None
        self.get_logger().warning(f'JOG STOP: {reason}')

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
        self.publish_timing(
            metadata['command_id'], metadata['intent_stamp'],
            'CONTROLLER_PUBLISH',
        )
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
            'intent_stamp': msg.stamp,
            'origin_start': tuple(start),
            'origin_target': tuple(target),
            'start': tuple(start),
            'target': tuple(target),
            'nominal_duration': float(duration),
            'duration': float(duration),
            'speed_scale': 1.0,
        }

    def command_callback(self, msg):
        self.publish_timing(
            msg.command_id, msg.stamp, 'ADAPTER_RECEIVE'
        )
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

        if self.jog_active:
            self.reject(
                msg,
                'REJECTED',
                'el control manual continuo está activo',
            )
            return

        if self.goal_active:
            self.reject(
                msg,
                'REJECTED',
                'ya existe una trayectoria activa en Gazebo',
            )
            return

        if list(msg.joint_names) != list(JOINT_NAMES):
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
            saturation = saturate_target_by_velocity(
                start,
                msg.positions,
                duration,
            )
        except ValueError as exc:
            self.reject(msg, 'REJECTED', str(exc))
            return
        if saturation.was_limited:
            self.reject(
                msg,
                'REJECTED',
                'invariante de velocidad del supervisor incumplida: '
                f'{saturation.limiting_joint}, '
                f'solicitada={saturation.requested_velocity:.4f}rad/s, '
                f'límite={saturation.limited_velocity:.4f}rad/s',
            )
            return
        target = saturation.positions

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

    def store_pending_control(self, msg):
        """Keep only the most restrictive request awaiting cancellation."""
        if (
            self.pending_control is None
            or is_more_restrictive(
                msg.state,
                msg.speed_scale,
                self.pending_control.state,
                self.pending_control.speed_scale,
            )
        ):
            self.pending_control = msg

    def control_callback(self, msg):
        """Apply the latest runtime safety decision to the active goal."""
        if (
            self.jog_active
            and not self.goal_active
            and (msg.state == 'STOP' or msg.speed_scale <= 0.0)
        ):
            if self.last_jog_intent_stamp is not None:
                self.publish_timing(
                    msg.command_id, self.last_jog_intent_stamp,
                    'STOP_DETECTED', msg.reason_code,
                )
            self.stop_active_jog(msg.reason)
            return

        metadata = self.active_metadata
        if (
            not self.goal_active
            or metadata is None
            or msg.command_id != metadata['command_id']
        ):
            return

        if not metadata['accepted'] or self.goal_handle is None:
            self.store_pending_control(msg)
            return

        if self.control_in_progress:
            self.store_pending_control(msg)
            return

        try:
            desired_scale = effective_speed_scale(
                msg.state,
                msg.speed_scale,
            )
        except ValueError as exc:
            self.get_logger().error(f'CONTROL REJECTED: {exc}')
            return

        if msg.state == 'STOP' or desired_scale <= 0.0:
            self.publish_timing(
                metadata['command_id'], metadata['intent_stamp'],
                'STOP_DETECTED', msg.reason_code,
            )

        if math.isclose(
            desired_scale,
            self.applied_speed_scale,
            abs_tol=1.0e-6,
        ):
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
            'intent_stamp': metadata['intent_stamp'],
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
