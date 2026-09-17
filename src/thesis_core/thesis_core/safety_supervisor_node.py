import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    CommandDecision,
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    ProximityStatus,
    TrajectoryPrediction,
)

from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    minimum_sphere_clearance,
)

EXPECTED_ARM_JOINTS = [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
]

JOINT_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[1]: (
        47.0 * math.pi / 180.0,
        313.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[2]: (
        19.0 * math.pi / 180.0,
        341.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[3]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[4]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[5]: (-2.0 * math.pi, 2.0 * math.pi),
}

JOINT_VELOCITY_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[1]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[2]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[3]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[4]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[5]: 48.0 * math.pi / 180.0,
}

CONTINUOUS_JOINTS = {
    EXPECTED_ARM_JOINTS[0],
    EXPECTED_ARM_JOINTS[3],
    EXPECTED_ARM_JOINTS[4],
    EXPECTED_ARM_JOINTS[5],
}

MIN_DURATION_SEC = 0.1
MAX_DURATION_SEC = 30.0


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.declare_parameter(
            'state_topic',
            '/joint_states',
        )

        self.declare_parameter(
            'velocity_scale',
            0.5,
        )

        self.declare_parameter(
            'max_state_age_sec',
            0.5,
        )

        self.declare_parameter(
            'require_current_state',
            False,
        )

        self.declare_parameter(
            'require_proximity_status',
            True,
        )

        self.declare_parameter(
            'max_proximity_age_sec',
            0.5,
        )

        self.declare_parameter(
            'reduction_duration_scale',
            2.0,
        )

        self.declare_parameter(
            'prediction_enabled',
            True,
        )

        self.declare_parameter(
            'prediction_samples',
            25,
        )
        self.declare_parameter('prediction_horizon_sec', 1.0)

        self.declare_parameter('warning_distance', 0.30)
        self.declare_parameter('reduction_distance', 0.15)
        self.declare_parameter('stop_distance', 0.05)
        self.declare_parameter('runtime_monitor_enabled', True)
        self.declare_parameter('runtime_rate_hz', 10.0)
        self.declare_parameter('runtime_horizon_sec', 1.0)
        self.declare_parameter('runtime_samples', 21)
        self.declare_parameter('predictive_margin_m', 0.02)
        self.declare_parameter('hard_stop_clearance_m', 0.0)
        self.declare_parameter('runtime_minimum_speed_scale', 0.05)
        self.declare_parameter('runtime_scale_search_iterations', 10)

        state_topic = self.get_parameter(
            'state_topic'
        ).value

        self.current_positions = {}
        self.last_state_receive_ns = None
        self.proximity_status = None
        self.last_proximity_receive_ns = None
        self.last_proximity_decision = 'UNAVAILABLE'
        self.active_execution = None
        self.last_execution_receive_ns = None
        self.last_runtime_command_id = None
        self.last_runtime_state = None
        self.last_runtime_scale = None
        self.runtime_recovery_count = 0
        self.last_prediction_speed_scale = 1.0
        self.last_jog_state = None
        self.last_jog_scale = None

        self.state_subscription = self.create_subscription(
            JointState,
            state_topic,
            self.joint_state_callback,
            10,
        )

        self.candidate_subscription = self.create_subscription(
            JointCommand,
            '/thesis/candidate_command',
            self.candidate_command_callback,
            10,
        )
        self.jog_subscription = self.create_subscription(
            JointCommand,
            '/thesis/jog_intent',
            self.jog_intent_callback,
            10,
        )

        self.proximity_subscription = self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_status_callback,
            10,
        )

        self.execution_subscription = self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.execution_callback,
            10,
        )

        self.supervised_publisher = self.create_publisher(
            JointCommand,
            '/thesis/supervised_command',
            10,
        )
        self.supervised_jog_publisher = self.create_publisher(
            JointCommand,
            '/thesis/supervised_jog_command',
            10,
        )

        self.prediction_publisher = self.create_publisher(
            TrajectoryPrediction,
            '/thesis/trajectory_prediction',
            10,
        )

        self.decision_publisher = self.create_publisher(
            CommandDecision,
            '/thesis/command_decision',
            10,
        )

        self.execution_control_publisher = self.create_publisher(
            ExecutionControl,
            '/thesis/execution_control',
            10,
        )

        runtime_rate = float(
            self.get_parameter('runtime_rate_hz').value
        )
        if runtime_rate <= 0.0:
            raise ValueError('runtime_rate_hz must be greater than zero')
        self.runtime_timer = None
        if bool(self.get_parameter('runtime_monitor_enabled').value):
            self.runtime_timer = self.create_timer(
                1.0 / runtime_rate,
                self.runtime_monitor_callback,
            )

        self.get_logger().info(
            'Safety supervisor started in validation/pass-through mode'
        )

        self.get_logger().info(
            f'State topic configured: {state_topic}'
        )

        self.get_logger().info(
            'Proximity policy enabled on /thesis/proximity_status'
        )

        self.get_logger().info(
            'Preventive trajectory prediction enabled'
        )

        self.get_logger().info(
            f'Runtime execution monitor enabled at {runtime_rate:.1f} Hz'
        )

    def joint_state_callback(self, msg):
        positions = {}

        for joint_name, position in zip(
            msg.name,
            msg.position,
        ):
            if math.isfinite(position):
                positions[joint_name] = float(position)

        self.current_positions = positions
        self.last_state_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def proximity_status_callback(self, msg):
        self.proximity_status = msg
        self.last_proximity_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def execution_callback(self, msg):
        """Track the trajectory currently accepted by the adapter."""
        now_ns = self.get_clock().now().nanoseconds
        if msg.status == 'ACCEPTED':
            if self.last_runtime_command_id != msg.command_id:
                self.last_runtime_state = None
                self.last_runtime_scale = None
                self.runtime_recovery_count = 0
            self.active_execution = msg
            self.last_execution_receive_ns = now_ns
            self.last_runtime_command_id = msg.command_id
            return

        terminal_statuses = {
            'SUCCEEDED',
            'FAILED',
            'CANCELED',
            'REJECTED',
            'DRY_RUN',
        }
        if (
            msg.status in terminal_statuses
            and self.active_execution is not None
            and msg.command_id == self.active_execution.command_id
        ):
            self.active_execution = None
            self.last_execution_receive_ns = None
            self.last_runtime_command_id = None
            self.last_runtime_state = None
            self.last_runtime_scale = None
            self.runtime_recovery_count = 0

    @staticmethod
    def message_time_seconds(value):
        return float(value.sec) + float(value.nanosec) * 1e-9

    def runtime_obstacle(self):
        """Return the latest obstacle state when it is fresh."""
        if (
            self.proximity_status is None
            or self.last_proximity_receive_ns is None
        ):
            return None

        age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_proximity_receive_ns
        ) / 1e9
        max_age_sec = float(
            self.get_parameter('max_proximity_age_sec').value
        )
        if age_sec > max_age_sec:
            return None

        center = self.proximity_status.obstacle_center
        velocity = self.proximity_status.obstacle_velocity
        return (
            (center.x, center.y, center.z),
            (velocity.x, velocity.y, velocity.z),
            float(self.proximity_status.obstacle_radius),
        )

    def publish_execution_control(
        self,
        command_id,
        state,
        speed_scale,
        clearance,
        segment,
        time_to_collision,
        reason_code,
        reason,
        minimum_time_from_now=-1.0,
        minimum_sample_index=0,
        minimum_sample_count=0,
        minimum_horizon_fraction=-1.0,
    ):
        control = ExecutionControl()
        control.stamp = self.get_clock().now().to_msg()
        control.command_id = command_id
        control.state = state
        control.speed_scale = float(speed_scale)
        control.minimum_clearance = float(clearance)
        control.limiting_segment = segment
        control.time_to_collision = float(time_to_collision)
        control.minimum_time_from_now = float(minimum_time_from_now)
        control.minimum_sample_index = int(minimum_sample_index)
        control.minimum_sample_count = int(minimum_sample_count)
        control.minimum_horizon_fraction = float(
            minimum_horizon_fraction
        )
        control.reason_code = reason_code
        control.reason = reason
        self.execution_control_publisher.publish(control)

    def evaluate_runtime_horizon(
        self,
        current,
        target,
        nominal_remaining,
        speed_scale,
        horizon,
        sample_count,
        obstacle,
    ):
        """Evaluate one candidate absolute speed scale over the horizon."""
        if speed_scale <= 0.0:
            samples = [tuple(current) for _ in range(sample_count)]
        else:
            effective_duration = nominal_remaining / speed_scale
            samples = sample_reference(
                current,
                target,
                effective_duration,
                0.0,
                horizon,
                sample_count,
            )

        obstacle_center, obstacle_velocity, obstacle_radius = obstacle
        best = None
        minimum_sample_index = 0
        minimum_time_from_now = 0.0
        collision_time = -1.0
        for sample_index, sample in enumerate(samples):
            sample_time = horizon * sample_index / float(sample_count - 1)
            predicted_obstacle = tuple(
                obstacle_center[axis]
                + obstacle_velocity[axis] * sample_time
                for axis in range(3)
            )
            result = minimum_sphere_clearance(
                sample,
                predicted_obstacle,
                obstacle_radius,
            )
            if result[0] <= 0.0 and collision_time < 0.0:
                collision_time = sample_time
            if best is None or result[0] < best[0]:
                best = result
                minimum_sample_index = sample_index
                minimum_time_from_now = sample_time

        return (
            best,
            minimum_sample_index,
            minimum_time_from_now,
            collision_time,
        )

    def publish_supervised_jog(self, source, positions, horizon):
        output = JointCommand()
        output.stamp = self.get_clock().now().to_msg()
        output.command_id = source.command_id
        output.joint_names = list(EXPECTED_ARM_JOINTS)
        output.positions = list(positions)
        whole_seconds = int(horizon)
        nanoseconds = int(round(
            (horizon - whole_seconds) * 1_000_000_000
        ))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        output.duration.sec = whole_seconds
        output.duration.nanosec = nanoseconds
        self.supervised_jog_publisher.publish(output)

    def jog_intent_callback(self, msg):
        """Supervise a rolling joint-space velocity intention."""
        if self.active_execution is not None:
            return
        if tuple(msg.joint_names) != tuple(EXPECTED_ARM_JOINTS):
            return
        if len(msg.positions) != len(EXPECTED_ARM_JOINTS):
            return
        if not all(math.isfinite(value) for value in msg.positions):
            return
        if not all(
            name in self.current_positions
            for name in EXPECTED_ARM_JOINTS
        ):
            return
        if self.last_state_receive_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        state_age = (now_ns - self.last_state_receive_ns) / 1e9
        if state_age > float(
                self.get_parameter('max_state_age_sec').value):
            return

        current = tuple(
            self.current_positions[name]
            for name in EXPECTED_ARM_JOINTS
        )
        horizon = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )
        if not math.isfinite(horizon) or not 0.1 <= horizon <= 2.0:
            self.publish_supervised_jog(msg, current, 1.0)
            return

        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_supervised_jog(msg, current, horizon)
            return

        bounded_target = []
        for name, current_value, requested_value in zip(
            EXPECTED_ARM_JOINTS,
            current,
            msg.positions,
        ):
            delta = self.shortest_joint_delta(
                name,
                float(requested_value),
                current_value,
            )
            maximum_delta = JOINT_VELOCITY_LIMITS[name] * horizon
            delta = min(max(delta, -maximum_delta), maximum_delta)
            bounded_target.append(current_value + delta)
        bounded_target = tuple(bounded_target)

        sample_count = int(
            self.get_parameter('runtime_samples').value
        )
        if sample_count < 2:
            self.publish_supervised_jog(msg, current, horizon)
            return

        obstacle_center, _, obstacle_radius = obstacle
        current_result = minimum_sphere_clearance(
            current,
            obstacle_center,
            obstacle_radius,
        )
        full_evaluation = self.evaluate_runtime_horizon(
            current,
            bounded_target,
            horizon,
            1.0,
            horizon,
            sample_count,
            obstacle,
        )
        best, sample_index, minimum_time, collision_time = full_evaluation
        full_clearance = best[0]
        current_clearance = current_result[0]
        margin = float(
            self.get_parameter('predictive_margin_m').value
        )
        hard_stop = float(
            self.get_parameter('hard_stop_clearance_m').value
        )
        minimum_scale = float(
            self.get_parameter('runtime_minimum_speed_scale').value
        )
        iterations = max(
            4,
            int(self.get_parameter(
                'runtime_scale_search_iterations'
            ).value),
        )

        speed_scale = 1.0
        if current_clearance <= hard_stop:
            state = 'STOP'
            speed_scale = 0.0
        elif full_clearance >= margin:
            state = (
                'WARNING'
                if full_clearance <= float(
                    self.get_parameter('warning_distance').value
                )
                else 'ALLOW'
            )
        else:
            low = 0.0
            high = 1.0
            safe_evaluation = self.evaluate_runtime_horizon(
                current,
                bounded_target,
                horizon,
                0.0,
                horizon,
                sample_count,
                obstacle,
            )
            for _ in range(iterations):
                candidate = 0.5 * (low + high)
                evaluation = self.evaluate_runtime_horizon(
                    current,
                    bounded_target,
                    horizon,
                    candidate,
                    horizon,
                    sample_count,
                    obstacle,
                )
                if evaluation[0][0] >= margin:
                    low = candidate
                    safe_evaluation = evaluation
                else:
                    high = candidate
            if low < minimum_scale:
                state = 'STOP'
                speed_scale = 0.0
            else:
                state = 'REDUCTION'
                speed_scale = low
                best, sample_index, minimum_time, collision_time = (
                    safe_evaluation
                )

        safe_target = tuple(
            current_value + speed_scale * (target_value - current_value)
            for current_value, target_value in zip(
                current,
                bounded_target,
            )
        )
        self.publish_supervised_jog(msg, safe_target, horizon)

        clearance, segment, _, _, _ = best
        reason = (
            f'JOG H={horizon:.2f}s; d_actual={current_clearance:.3f}m; '
            f'd_nominal={full_clearance:.3f}m; '
            f'd_seguro={clearance:.3f}m; margen={margin:.3f}m; '
            f'escala={speed_scale:.3f}'
        )
        self.publish_execution_control(
            msg.command_id,
            state,
            speed_scale,
            clearance,
            segment,
            collision_time,
            f'JOG_{state}',
            reason,
            minimum_time_from_now=minimum_time,
            minimum_sample_index=sample_index,
            minimum_sample_count=sample_count,
            minimum_horizon_fraction=(
                sample_index / float(sample_count - 1)
            ),
        )
        if (
            state != self.last_jog_state
            or self.last_jog_scale is None
            or abs(speed_scale - self.last_jog_scale) >= 0.05
        ):
            self.get_logger().info(
                f'JOG {state}: scale={speed_scale:.3f}, '
                f'd_nominal={full_clearance:.3f}m, '
                f'd_safe={clearance:.3f}m'
            )
            self.last_jog_state = state
            self.last_jog_scale = speed_scale

    def runtime_monitor_callback(self):
        """Evaluate the short horizon for the trajectory in execution."""
        execution = self.active_execution
        if execution is None:
            return

        command_id = execution.command_id
        if not all(
            joint_name in self.current_positions
            for joint_name in EXPECTED_ARM_JOINTS
        ):
            self.publish_execution_control(
                command_id,
                'STOP',
                0.0,
                -1.0,
                '',
                0.0,
                'RUNTIME_STATE_UNAVAILABLE',
                'No se recibió una configuración articular completa.',
            )
            return

        state_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_state_receive_ns
        ) / 1e9
        if state_age_sec > float(
                self.get_parameter('max_state_age_sec').value):
            self.publish_execution_control(
                command_id,
                'STOP',
                0.0,
                -1.0,
                '',
                0.0,
                'RUNTIME_STATE_STALE',
                f'Estado articular obsoleto ({state_age_sec:.3f} s).',
            )
            return

        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_execution_control(
                command_id,
                'STOP',
                0.0,
                -1.0,
                '',
                0.0,
                'RUNTIME_PROXIMITY_STALE',
                'No existe una medición de proximidad reciente.',
            )
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        start_sec = self.message_time_seconds(execution.start_time)
        duration = float(execution.duration_sec)
        elapsed = max(0.0, now_sec - start_sec)
        remaining = duration - elapsed
        if not math.isfinite(remaining) or remaining <= 0.05:
            return

        current = tuple(
            self.current_positions[name]
            for name in EXPECTED_ARM_JOINTS
        )
        target = tuple(float(value) for value in execution.target_positions)
        horizon = min(
            float(self.get_parameter('runtime_horizon_sec').value),
            remaining,
        )
        sample_count = int(
            self.get_parameter('runtime_samples').value
        )
        if horizon <= 0.0 or sample_count < 2:
            return

        previous_scale = self.last_runtime_scale
        if previous_scale is None or not 0.0 < previous_scale <= 1.0:
            previous_scale = 1.0
        nominal_remaining = remaining * previous_scale
        obstacle_center, obstacle_velocity, obstacle_radius = obstacle

        try:
            current_result = minimum_sphere_clearance(
                current,
                obstacle_center,
                obstacle_radius,
            )
            full_speed_evaluation = self.evaluate_runtime_horizon(
                current,
                target,
                nominal_remaining,
                1.0,
                horizon,
                sample_count,
                obstacle,
            )
        except (TypeError, ValueError, ZeroDivisionError):
            self.publish_execution_control(
                command_id,
                'STOP',
                0.0,
                -1.0,
                '',
                0.0,
                'RUNTIME_PREDICTION_ERROR',
                'No se pudo muestrear la trayectoria activa.',
            )
            return

        predictive_margin = float(
            self.get_parameter('predictive_margin_m').value
        )
        hard_stop_clearance = float(
            self.get_parameter('hard_stop_clearance_m').value
        )
        minimum_scale = float(
            self.get_parameter('runtime_minimum_speed_scale').value
        )
        search_iterations = max(
            4,
            int(self.get_parameter(
                'runtime_scale_search_iterations'
            ).value),
        )
        if not 0.0 <= hard_stop_clearance < predictive_margin:
            predictive_margin = 0.02
            hard_stop_clearance = 0.0
        if not 0.0 < minimum_scale < 1.0:
            minimum_scale = 0.05

        current_clearance = current_result[0]
        best, minimum_sample_index, minimum_time_from_now, collision_time = (
            full_speed_evaluation
        )
        full_speed_clearance = best[0]
        speed_scale = 1.0

        if current_clearance <= hard_stop_clearance:
            state = 'STOP'
            speed_scale = 0.0
        elif full_speed_clearance >= predictive_margin:
            state = (
                'WARNING'
                if full_speed_clearance <= float(
                    self.get_parameter('warning_distance').value
                )
                else 'ALLOW'
            )
        else:
            low = 0.0
            high = 1.0
            safe_evaluation = self.evaluate_runtime_horizon(
                current,
                target,
                nominal_remaining,
                0.0,
                horizon,
                sample_count,
                obstacle,
            )
            for _ in range(search_iterations):
                candidate_scale = 0.5 * (low + high)
                candidate_evaluation = self.evaluate_runtime_horizon(
                    current,
                    target,
                    nominal_remaining,
                    candidate_scale,
                    horizon,
                    sample_count,
                    obstacle,
                )
                if candidate_evaluation[0][0] >= predictive_margin:
                    low = candidate_scale
                    safe_evaluation = candidate_evaluation
                else:
                    high = candidate_scale

            if low < minimum_scale:
                state = 'STOP'
                speed_scale = 0.0
            else:
                state = 'REDUCTION'
                speed_scale = low
                (
                    best,
                    minimum_sample_index,
                    minimum_time_from_now,
                    collision_time,
                ) = safe_evaluation

        clearance, segment, _, _, _ = best
        reason_code = {
            'ALLOW': 'RUNTIME_CLEAR',
            'WARNING': 'RUNTIME_WARNING_OUTSIDE_VOLUME',
            'REDUCTION': 'RUNTIME_SCALE_TO_PREDICTIVE_MARGIN',
            'STOP': 'RUNTIME_HARD_STOP_OR_NO_SAFE_SCALE',
        }.get(state, 'RUNTIME_UNKNOWN_STATE')
        reason = (
            f'H={horizon:.2f}s; d_min={clearance:.3f} m; '
            f'd_actual={current_clearance:.3f} m; '
            f'margen={predictive_margin:.3f} m; '
            f'd_nominal={full_speed_clearance:.3f} m; '
            f't_min={minimum_time_from_now:.2f}s; '
            f'muestra={minimum_sample_index + 1}/{sample_count}; '
            f'segment={segment}; '
            f'escala={speed_scale:.2f}'
        )
        self.publish_execution_control(
            command_id,
            state,
            speed_scale,
            clearance,
            segment,
            collision_time,
            reason_code,
            reason,
            minimum_time_from_now=minimum_time_from_now,
            minimum_sample_index=minimum_sample_index,
            minimum_sample_count=sample_count,
            minimum_horizon_fraction=(
                minimum_sample_index / float(sample_count - 1)
            ),
        )

        if (
            state != self.last_runtime_state
            or self.last_runtime_scale is None
            or abs(speed_scale - self.last_runtime_scale) > 1.0e-3
        ):
            self.get_logger().warning(
                f'RUNTIME {command_id}: {state}, '
                f'd_min={clearance:.3f} m, scale={speed_scale:.2f}, '
                f't_min={minimum_time_from_now:.3f} s, '
                f'TTC={collision_time:.3f} s'
            )
            self.last_runtime_state = state
            self.last_runtime_scale = speed_scale

    def copy_command_with_duration(self, msg, duration_sec):
        output = JointCommand()
        output.stamp = msg.stamp
        output.command_id = msg.command_id
        output.joint_names = list(msg.joint_names)
        output.positions = list(msg.positions)

        whole_seconds = int(duration_sec)
        nanoseconds = int(round(
            (duration_sec - whole_seconds) * 1_000_000_000
        ))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        output.duration.sec = whole_seconds
        output.duration.nanosec = nanoseconds
        return output

    def state_for_clearance(self, clearance):
        if clearance <= float(self.get_parameter('stop_distance').value):
            return 'STOP'
        if clearance <= float(
                self.get_parameter('reduction_distance').value):
            return 'REDUCTION'
        if clearance <= float(
                self.get_parameter('warning_distance').value):
            return 'WARNING'
        return 'ALLOW'

    def predictive_state(self, current_clearance, minimum_clearance):
        """Classify motion using the displayed predictive envelope."""
        hard_stop = float(
            self.get_parameter('hard_stop_clearance_m').value
        )
        margin = float(
            self.get_parameter('predictive_margin_m').value
        )
        warning = float(self.get_parameter('warning_distance').value)
        if current_clearance <= hard_stop:
            return 'STOP'
        if minimum_clearance < margin:
            return 'REDUCTION'
        if minimum_clearance <= warning:
            return 'WARNING'
        return 'ALLOW'

    def publish_decision(
            self, command, accepted, state, reason_code, reason):
        """Publish the supervisor decision for a candidate command."""
        decision = CommandDecision()
        decision.stamp = self.get_clock().now().to_msg()
        decision.command_id = command.command_id
        decision.accepted = accepted
        decision.state = state
        decision.reason_code = reason_code
        decision.reason = reason
        self.decision_publisher.publish(decision)

    def joint_limit_violation(self, joint_positions):
        """Return the first joint-limit violation in a configuration."""
        for joint_name, position in zip(
                EXPECTED_ARM_JOINTS,
                joint_positions):
            lower, upper = JOINT_LIMITS[joint_name]
            if not math.isfinite(position) or not lower <= position <= upper:
                return joint_name, position, lower, upper
        return None

    def predict_candidate(self, msg):
        self.last_prediction_speed_scale = 1.0
        if not bool(self.get_parameter('prediction_enabled').value):
            return None

        if not all(
            joint_name in self.current_positions
            for joint_name in EXPECTED_ARM_JOINTS
        ):
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'current state unavailable for prediction'
            )
            return False

        max_state_age_sec = float(
            self.get_parameter('max_state_age_sec').value
        )
        state_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_state_receive_ns
        ) / 1e9
        if state_age_sec > max_state_age_sec:
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: state stale for prediction '
                f'({state_age_sec:.3f} s)'
            )
            return False

        sample_count = int(
            self.get_parameter('prediction_samples').value
        )
        if sample_count < 2:
            self.get_logger().error(
                'prediction_samples must be at least 2'
            )
            return False

        current = tuple(
            self.current_positions[name]
            for name in EXPECTED_ARM_JOINTS
        )
        target = tuple(float(value) for value in msg.positions)
        trajectory_duration = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )
        prediction_horizon = min(
            trajectory_duration,
            float(self.get_parameter('prediction_horizon_sec').value),
        )
        if prediction_horizon <= 0.0:
            self.get_logger().error(
                'prediction_horizon_sec must be greater than zero'
            )
            return False
        deltas = tuple(
            self.shortest_joint_delta(name, target_value, current_value)
            for name, target_value, current_value in zip(
                EXPECTED_ARM_JOINTS,
                target,
                current,
            )
        )
        obstacle = (
            self.proximity_status.obstacle_center.x,
            self.proximity_status.obstacle_center.y,
            self.proximity_status.obstacle_center.z,
        )
        obstacle_velocity = (
            self.proximity_status.obstacle_velocity.x,
            self.proximity_status.obstacle_velocity.y,
            self.proximity_status.obstacle_velocity.z,
        )
        obstacle_radius = self.proximity_status.obstacle_radius

        best = None
        best_index = 0
        current_clearance = math.inf
        for sample_index in range(sample_count):
            fraction = sample_index / float(sample_count - 1)
            sample_time = fraction * prediction_horizon
            trajectory_fraction = sample_time / trajectory_duration
            sample = tuple(
                current_value + trajectory_fraction * delta
                for current_value, delta in zip(current, deltas)
            )
            violation = self.joint_limit_violation(sample)
            if violation is not None:
                joint_name, position, lower, upper = violation
                reason = (
                    f'{joint_name}={position:.4f} rad fuera de '
                    f'[{lower:.4f}, {upper:.4f}] rad '
                    f'en la muestra {sample_index + 1}/{sample_count}.'
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: {reason}'
                )
                self.publish_decision(
                    msg,
                    False,
                    'STOP',
                    'PREDICTED_JOINT_LIMIT',
                    reason,
                )
                return False
            result = minimum_sphere_clearance(
                sample,
                tuple(
                    obstacle[axis]
                    + obstacle_velocity[axis] * sample_time
                    for axis in range(3)
                ),
                obstacle_radius,
            )
            if sample_index == 0:
                current_clearance = result[0]
            if best is None or result[0] < best[0]:
                best = result
                best_index = sample_index

        clearance, segment, segment_index, start, end = best
        state = self.predictive_state(current_clearance, clearance)
        if state == 'REDUCTION':
            margin = float(
                self.get_parameter('predictive_margin_m').value
            )
            minimum_scale = float(
                self.get_parameter('runtime_minimum_speed_scale').value
            )
            iterations = max(
                4,
                int(self.get_parameter(
                    'runtime_scale_search_iterations'
                ).value),
            )
            obstacle_data = (
                obstacle,
                obstacle_velocity,
                obstacle_radius,
            )
            low = 0.0
            high = 1.0
            for _ in range(iterations):
                candidate_scale = 0.5 * (low + high)
                evaluation = self.evaluate_runtime_horizon(
                    current,
                    target,
                    trajectory_duration,
                    candidate_scale,
                    prediction_horizon,
                    sample_count,
                    obstacle_data,
                )
                if evaluation[0][0] >= margin:
                    low = candidate_scale
                else:
                    high = candidate_scale
            if low < minimum_scale:
                state = 'STOP'
            else:
                self.last_prediction_speed_scale = low

        prediction = TrajectoryPrediction()
        prediction.stamp = self.get_clock().now().to_msg()
        prediction.command_id = msg.command_id
        prediction.reference_frame = (
            self.proximity_status.reference_frame
        )
        prediction.state = state
        prediction.minimum_clearance = clearance
        prediction.limiting_segment = segment
        prediction.sample_index = best_index
        prediction.sample_count = sample_count
        prediction.trajectory_fraction = (
            (best_index / float(sample_count - 1))
            * prediction_horizon / trajectory_duration
        )
        prediction.capsule_start.x = start[0]
        prediction.capsule_start.y = start[1]
        prediction.capsule_start.z = start[2]
        prediction.capsule_end.x = end[0]
        prediction.capsule_end.y = end[1]
        prediction.capsule_end.z = end[2]
        prediction.capsule_radius = CAPSULE_RADII[segment_index]
        self.prediction_publisher.publish(prediction)

        self.get_logger().info(
            f'PREDICTION {msg.command_id}: {state}, '
            f'min_clearance={clearance:.3f} m, '
            f'path={prediction.trajectory_fraction:.2f}, '
            f'segment={segment}'
        )
        return prediction

    def apply_proximity_policy(self, msg, duration_sec):
        require_status = bool(
            self.get_parameter('require_proximity_status').value
        )

        if (
            self.proximity_status is None
            or self.last_proximity_receive_ns is None
        ):
            if require_status:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: proximity status unavailable',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    'proximity status unavailable'
                )
                return None

            self.get_logger().warning(
                f'PROXIMITY CHECK SKIPPED {msg.command_id}: '
                'status unavailable'
            )
            self.last_proximity_decision = 'SKIPPED'
            return self.copy_command_with_duration(msg, duration_sec)

        max_age_sec = float(
            self.get_parameter('max_proximity_age_sec').value
        )
        status_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_proximity_receive_ns
        ) / 1e9

        if status_age_sec > max_age_sec:
            if require_status:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: proximity status stale ({status_age_sec:.3f} s)',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: proximity status stale '
                    f'({status_age_sec:.3f} s)'
                )
                return None

            self.get_logger().warning(
                f'PROXIMITY CHECK SKIPPED {msg.command_id}: '
                f'status stale ({status_age_sec:.3f} s)'
            )
            self.last_proximity_decision = 'SKIPPED'
            return self.copy_command_with_duration(msg, duration_sec)

        state = self.proximity_status.state.upper()
        clearance = self.proximity_status.minimum_clearance
        segment = self.proximity_status.limiting_segment

        if not math.isfinite(clearance):
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: non-finite proximity clearance',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'non-finite proximity clearance'
            )
            return None

        prediction = self.predict_candidate(msg)
        if prediction is False:
            return None
        if prediction is not None:
            state = prediction.state
            clearance = prediction.minimum_clearance
            segment = prediction.limiting_segment

        self.last_proximity_decision = state

        if state == 'STOP':
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: STOP proximity, clearance={clearance:.3f} m, segment={segment}',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: STOP proximity, '
                f'clearance={clearance:.3f} m, segment={segment}'
            )
            return None

        if state == 'REDUCTION':
            speed_scale = min(max(
                float(self.last_prediction_speed_scale),
                float(self.get_parameter(
                    'runtime_minimum_speed_scale'
                ).value),
            ), 1.0)
            reduced_speed_duration = min(
                duration_sec / speed_scale,
                MAX_DURATION_SEC,
            )
            if reduced_speed_duration <= duration_sec + 1.0e-9:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: REDUCTION requested but duration cannot be increased safely',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: REDUCTION requested '
                    'but duration cannot be increased safely'
                )
                return None

            self.get_logger().warning(
                f'REDUCTION {msg.command_id}: '
                f'clearance={clearance:.3f} m, segment={segment}, '
                f'duration={duration_sec:.2f}s'
                f'->{reduced_speed_duration:.2f}s'
            )
            return self.copy_command_with_duration(
                msg,
                reduced_speed_duration,
            )

        if state == 'WARNING':
            self.get_logger().warning(
                f'WARNING {msg.command_id}: '
                f'clearance={clearance:.3f} m, segment={segment}'
            )
            return self.copy_command_with_duration(msg, duration_sec)

        if state == 'ALLOW':
            return self.copy_command_with_duration(msg, duration_sec)

        self.publish_decision(
            msg, False, 'REJECTED', 'VALIDATION_REJECTED',
            f'REJECTED {msg.command_id}: unknown proximity state={state}',
        )
        self.get_logger().warning(
            f'REJECTED {msg.command_id}: unknown proximity state={state}'
        )
        return None

    def shortest_joint_delta(
        self,
        joint_name,
        target_position,
        current_position,
    ):
        delta = target_position - current_position

        if joint_name in CONTINUOUS_JOINTS:
            return math.atan2(
                math.sin(delta),
                math.cos(delta),
            )

        return delta

    def validate_velocity(
        self,
        msg,
        duration_sec,
    ):
        require_state = bool(
            self.get_parameter(
                'require_current_state'
            ).value
        )

        velocity_scale = float(
            self.get_parameter(
                'velocity_scale'
            ).value
        )

        max_state_age_sec = float(
            self.get_parameter(
                'max_state_age_sec'
            ).value
        )

        if (
            not self.current_positions
            or self.last_state_receive_ns is None
        ):
            if require_state:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: current joint state unavailable',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    'current joint state unavailable'
                )
                return False

            self.get_logger().warning(
                f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                'current joint state unavailable'
            )
            return True

        state_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_state_receive_ns
        ) / 1e9

        if state_age_sec > max_state_age_sec:
            if require_state:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: joint state is stale ({state_age_sec:.3f} s)',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    f'joint state is stale '
                    f'({state_age_sec:.3f} s)'
                )
                return False

            self.get_logger().warning(
                f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                f'joint state is stale '
                f'({state_age_sec:.3f} s)'
            )
            return True

        for joint_name, target_position in zip(
            msg.joint_names,
            msg.positions,
        ):
            if joint_name not in self.current_positions:
                if require_state:
                    self.publish_decision(
                        msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                        f'REJECTED {msg.command_id}: no current state for {joint_name}',
                    )
                    self.get_logger().warning(
                        f'REJECTED {msg.command_id}: '
                        f'no current state for {joint_name}'
                    )
                    return False

                self.get_logger().warning(
                    f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                    f'no current state for {joint_name}'
                )
                return True

            current_position = self.current_positions[joint_name]

            delta = self.shortest_joint_delta(
                joint_name,
                target_position,
                current_position,
            )

            requested_velocity = abs(delta) / duration_sec

            allowed_velocity = (
                JOINT_VELOCITY_LIMITS[joint_name]
                * velocity_scale
            )

            if requested_velocity > allowed_velocity:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: {joint_name} requested velocity {requested_velocity:.4f} rad/s exceeds {allowed_velocity:.4f} rad/s',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    f'{joint_name} requested velocity '
                    f'{requested_velocity:.4f} rad/s exceeds '
                    f'{allowed_velocity:.4f} rad/s'
                )
                return False

        return True

    def candidate_command_callback(self, msg):
        receive_time = self.get_clock().now()

        if len(msg.joint_names) == 0:
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: no joints provided',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: no joints provided'
            )
            return

        if len(msg.joint_names) != len(msg.positions):
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: joint_names and positions have different sizes',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'joint_names and positions have different sizes'
            )
            return

        if not all(math.isfinite(value) for value in msg.positions):
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: non-finite joint position detected',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'non-finite joint position detected'
            )
            return

        if list(msg.joint_names) != EXPECTED_ARM_JOINTS:
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: expected six arm joints in canonical order',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'expected six arm joints in canonical order'
            )
            return

        for joint_name, position in zip(
            msg.joint_names,
            msg.positions,
        ):
            lower, upper = JOINT_LIMITS[joint_name]

            if not lower <= position <= upper:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: {joint_name}={position:.4f} rad outside [{lower:.4f}, {upper:.4f}]',
                )
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    f'{joint_name}={position:.4f} rad outside '
                    f'[{lower:.4f}, {upper:.4f}]'
                )
                return

        duration_sec = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )

        if not MIN_DURATION_SEC <= duration_sec <= MAX_DURATION_SEC:
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: duration must be between {MIN_DURATION_SEC:.1f} and {MAX_DURATION_SEC:.1f} seconds',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                f'duration must be between '
                f'{MIN_DURATION_SEC:.1f} and '
                f'{MAX_DURATION_SEC:.1f} seconds'
            )
            return

        supervised_msg = self.apply_proximity_policy(
            msg,
            duration_sec,
        )
        if supervised_msg is None:
            return

        supervised_duration_sec = (
            float(supervised_msg.duration.sec)
            + float(supervised_msg.duration.nanosec) * 1e-9
        )

        if not self.validate_velocity(
            supervised_msg,
            supervised_duration_sec,
        ):
            return

        candidate_stamp_ns = (
            int(msg.stamp.sec) * 1_000_000_000
            + int(msg.stamp.nanosec)
        )

        receive_ns = receive_time.nanoseconds

        input_latency_ms = (
            receive_ns - candidate_stamp_ns
        ) / 1e6

        self.supervised_publisher.publish(supervised_msg)

        self.get_logger().info(
            f'{self.last_proximity_decision} FORWARDED '
            f'id={msg.command_id} | '
            f'joints={len(msg.joint_names)} | '
            f'proximity={self.last_proximity_decision} | '
            f'duration={supervised_duration_sec:.2f} s | '
            f'input_latency={input_latency_ms:.3f} ms'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SafetySupervisorNode()

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
