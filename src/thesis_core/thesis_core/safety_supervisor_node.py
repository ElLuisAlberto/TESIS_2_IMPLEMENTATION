import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    CommandDecision,
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    PipelineTiming,
    ProximityStatus,
    TrajectoryPrediction,
)

from thesis_core.joint_prediction import predict_joint_samples
from thesis_core.control_stability import ControlStabilityFilter
from thesis_core.horizon_clearance import (
    KINEMATIC_TF_TOLERANCE_M,
    LATENCY_UNCERTAINTY_M,
    MODEL_UNCERTAINTY_M,
    NO_EVENT_TIME,
    SAMPLING_UNCERTAINTY_M,
    evaluate_joint_horizon,
    event_time_or_invalid,
    total_protective_margin,
)
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    minimum_sphere_clearance,
)
from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    saturate_target_by_velocity,
    shortest_joint_delta,
)
from thesis_core.safety_policy import (
    SafetyDecision,
    SafetyPolicyInput,
    decide_preventive_state,
    maximum_safe_scale,
    withdrawal_path_is_non_approaching,
    withdrawal_path_is_safe,
)
from thesis_core.ros_runtime import spin_node

MIN_DURATION_SEC = 0.1
MAX_DURATION_SEC = 30.0
MIN_RUNTIME_REMAINING_SEC = 0.10


def message_age_seconds(now_nanoseconds, stamp):
    """Return message age using the shared ROS clock."""
    stamp_nanoseconds = (
        int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    )
    if stamp_nanoseconds <= 0:
        return math.inf
    return (int(now_nanoseconds) - stamp_nanoseconds) * 1.0e-9


def runtime_remaining_time(duration, elapsed):
    """Keep monitoring an accepted goal until its terminal adapter status."""
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError('execution duration must be positive and finite')
    if not math.isfinite(elapsed) or elapsed < 0.0:
        raise ValueError(
            'execution elapsed time must be finite and nonnegative'
        )
    return max(duration - elapsed, MIN_RUNTIME_REMAINING_SEC)


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.declare_parameter(
            'state_topic',
            '/joint_states',
        )

        self.declare_parameter(
            'max_state_age_sec',
            0.5,
        )

        self.declare_parameter(
            'require_current_state',
            True,
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
        self.declare_parameter(
            'model_uncertainty_m', MODEL_UNCERTAINTY_M
        )
        self.declare_parameter(
            'sampling_uncertainty_m', SAMPLING_UNCERTAINTY_M
        )
        self.declare_parameter(
            'latency_uncertainty_m', LATENCY_UNCERTAINTY_M
        )
        self.declare_parameter('runtime_minimum_speed_scale', 0.05)
        self.declare_parameter('runtime_scale_search_iterations', 10)
        self.declare_parameter('withdrawal_speed_scale', 0.10)
        self.declare_parameter('withdrawal_minimum_progress_m', 0.002)
        self.declare_parameter('withdrawal_monotonic_tolerance_m', 0.001)
        self.declare_parameter('runtime_recovery_required_samples', 5)
        self.declare_parameter('runtime_max_scale_increment', 0.10)
        self.declare_parameter('runtime_recovery_sample_period_sec', 0.10)
        self.declare_parameter('jog_command_timeout_sec', 0.25)
        self.declare_parameter('max_jog_message_age_sec', 0.15)
        self.declare_parameter(
            'kinematic_tf_tolerance_m',
            KINEMATIC_TF_TOLERANCE_M,
        )

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
        self.last_prediction_speed_scale = 1.0
        self.last_prediction_reason_code = 'CLEAR'
        self.last_jog_state = None
        self.last_jog_scale = None
        self.last_horizon_clearance = None
        self.runtime_recovery_count = 0
        self.jog_recovery_count = 0
        self.last_jog_intent_monotonic = None

        recovery_required = int(self.get_parameter(
            'runtime_recovery_required_samples'
        ).value)
        maximum_increment = float(self.get_parameter(
            'runtime_max_scale_increment'
        ).value)
        recovery_period = float(self.get_parameter(
            'runtime_recovery_sample_period_sec'
        ).value)
        self.jog_stability = ControlStabilityFilter(
            recovery_required,
            maximum_increment,
            recovery_period,
        )
        self.runtime_stability = ControlStabilityFilter(
            recovery_required,
            maximum_increment,
            recovery_period,
        )

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
        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.jog_subscription = self.create_subscription(
            JointCommand,
            '/thesis/jog_intent',
            self.jog_intent_callback,
            jog_qos,
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
            jog_qos,
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
        self.timing_publisher = self.create_publisher(
            PipelineTiming, '/thesis/pipeline_timing', 100
        )
        self.timing_sequence = 0

        runtime_rate = float(
            self.get_parameter('runtime_rate_hz').value
        )
        if runtime_rate <= 0.0:
            raise ValueError('runtime_rate_hz must be greater than zero')
        jog_timeout = float(
            self.get_parameter('jog_command_timeout_sec').value
        )
        maximum_jog_age = float(
            self.get_parameter('max_jog_message_age_sec').value
        )
        if (
            not math.isfinite(maximum_jog_age)
            or not 0.0 < maximum_jog_age < jog_timeout
        ):
            raise ValueError(
                'max_jog_message_age_sec must be positive and smaller '
                'than jog_command_timeout_sec'
            )
        tf_tolerance = float(self.get_parameter(
            'kinematic_tf_tolerance_m'
        ).value)
        if (not math.isfinite(tf_tolerance) or
                not 0.0 < tf_tolerance <= 0.05):
            raise ValueError(
                'kinematic_tf_tolerance_m must be in (0, 0.05]'
            )
        self.protective_margin()
        self.runtime_timer = None
        if bool(self.get_parameter('runtime_monitor_enabled').value):
            self.runtime_timer = self.create_timer(
                1.0 / runtime_rate,
                self.runtime_monitor_callback,
            )
        self.jog_stability_timer = self.create_timer(
            0.05,
            self.jog_stability_watchdog_callback,
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

    def publish_timing(
        self,
        command,
        stage,
        monotonic_ns=None,
        internal_duration_sec=-1.0,
        detail='',
    ):
        """Publish one trace event without changing the control decision."""
        event = PipelineTiming()
        event.stamp = self.get_clock().now().to_msg()
        event.intent_stamp = command.stamp
        event.command_id = command.command_id
        event.source = 'supervisor'
        event.stage = stage
        self.timing_sequence += 1
        event.sequence = self.timing_sequence
        event.monotonic_ns = int(
            time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        )
        event.internal_duration_sec = float(internal_duration_sec)
        event.detail = str(detail)
        self.timing_publisher.publish(event)

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
        """Track execution and fail closed on controller failure."""
        now_ns = self.get_clock().now().nanoseconds
        if msg.status == 'ACCEPTED':
            if self.last_runtime_command_id != msg.command_id:
                self.last_runtime_state = None
                self.last_runtime_scale = None
                self.runtime_recovery_count = 0
                self.runtime_stability.reset('COMMAND_CHANGED')
            self.active_execution = msg
            self.last_execution_receive_ns = now_ns
            self.last_runtime_command_id = msg.command_id
            return

        active_matches = (
            self.active_execution is not None
            and msg.command_id == self.active_execution.command_id
        )

        # A repeated candidate can be rejected while its first instance is
        # already executing.  That duplicate must not stop or invalidate the
        # accepted execution.
        if msg.status == 'REJECTED' and active_matches:
            return

        if msg.status in {'FAILED', 'REJECTED'}:
            reason_code = f'CONTROLLER_{msg.status}'
            detail = str(msg.detail).strip()
            reason = (
                f'El controlador reportó {msg.status} para '
                f'{msg.command_id}.'
            )
            if detail:
                reason += f' Detalle: {detail}'
            self.publish_runtime_stop(
                msg.command_id,
                reason_code,
                reason,
            )

        terminal_statuses = {
            'SUCCEEDED',
            'FAILED',
            'CANCELED',
            'REJECTED',
            'DRY_RUN',
        }
        if msg.status in terminal_statuses and active_matches:
            self.active_execution = None
            self.last_execution_receive_ns = None
            self.last_runtime_command_id = None
            self.last_runtime_state = None
            self.last_runtime_scale = None
            self.runtime_recovery_count = 0
            self.runtime_stability.reset(
                f'EXECUTION_{msg.status}'
            )

    def stabilize_decision(self, channel, command_id, decision):
        """Apply the common immediate-restriction/recovery policy."""
        stability_filter = (
            self.jog_stability
            if channel == 'jog'
            else self.runtime_stability
        )
        if (
            channel == 'jog'
            and decision.reason_code == 'PROTECTIVE_WITHDRAWAL'
        ):
            # The geometric policy has verified the full-request withdrawal
            # and the scaled path. Start that escape at its low safe scale;
            # normal temporal recovery applies once outside the stop zone.
            stability_filter.reset('VERIFIED_PROTECTIVE_WITHDRAWAL')
        result = stability_filter.update(
            command_id,
            decision.state,
            decision.speed_scale,
            time.monotonic(),
        )
        if channel == 'jog':
            self.jog_recovery_count = result.recovery_count
        else:
            self.runtime_recovery_count = result.recovery_count
        unchanged = (
            result.state == decision.state
            and math.isclose(
                result.speed_scale,
                decision.speed_scale,
                abs_tol=1.0e-12,
            )
        )
        filtered_reason = (
            decision.reason_code if unchanged else 'TEMPORAL_RECOVERY'
        )
        return (
            SafetyDecision(
                result.state,
                result.speed_scale,
                filtered_reason,
            ),
            result,
        )

    def jog_stability_watchdog_callback(self):
        """Reset JOG recovery history when the intent stream expires."""
        if self.last_jog_intent_monotonic is None:
            return
        timeout = float(
            self.get_parameter('jog_command_timeout_sec').value
        )
        if time.monotonic() - self.last_jog_intent_monotonic <= timeout:
            return
        self.jog_stability.reset('JOG_INTENT_EXPIRED')
        self.jog_recovery_count = 0
        self.last_jog_state = None
        self.last_jog_scale = None
        self.last_jog_intent_monotonic = None

    def publish_runtime_stop(self, command_id, reason_code, reason):
        """Publish and latch one fail-safe runtime STOP decision."""
        raw_decision = SafetyDecision('STOP', 0.0, reason_code)
        decision, stability = self.stabilize_decision(
            'runtime',
            command_id,
            raw_decision,
        )
        self.publish_execution_control(
            command_id,
            decision.state,
            decision.speed_scale,
            -1.0,
            'UNAVAILABLE',
            NO_EVENT_TIME,
            reason_code,
            reason,
            current_clearance=-1.0,
            nominal_clearance=-1.0,
            stability=stability,
            requested_reason_code=reason_code,
        )
        self.last_runtime_state = decision.state
        self.last_runtime_scale = decision.speed_scale

    @staticmethod
    def message_time_seconds(value):
        return float(value.sec) + float(value.nanosec) * 1e-9

    def protective_margin(self):
        """Return the documented sum of modeling, sampling and latency."""
        total = total_protective_margin(
            self.get_parameter('model_uncertainty_m').value,
            self.get_parameter('sampling_uncertainty_m').value,
            self.get_parameter('latency_uncertainty_m').value,
        )
        configured = float(
            self.get_parameter('predictive_margin_m').value
        )
        if (not math.isfinite(configured) or
                not math.isclose(total, configured, abs_tol=1.0e-12)):
            raise ValueError(
                'predictive_margin_m must equal the uncertainty sum'
            )
        return total

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
        geometry=None,
        nominal_geometry=None,
        current_clearance=math.nan,
        nominal_clearance=math.nan,
        stability=None,
        requested_reason_code='',
    ):
        control = ExecutionControl()
        control.stamp = self.get_clock().now().to_msg()
        control.command_id = command_id
        control.state = state
        control.speed_scale = float(speed_scale)
        control.minimum_clearance = float(clearance)
        control.limiting_segment = segment
        control.protective_margin = self.protective_margin()
        control.time_to_protective_volume = NO_EVENT_TIME
        control.time_to_collision = NO_EVENT_TIME
        control.supervised_time_to_protective_volume = NO_EVENT_TIME
        control.supervised_time_to_collision = NO_EVENT_TIME
        if nominal_geometry is not None:
            control.time_to_protective_volume = event_time_or_invalid(
                nominal_geometry.first_protective_entry_time
            )
            control.time_to_collision = event_time_or_invalid(
                nominal_geometry.first_collision_time
            )
        if geometry is not None:
            control.supervised_time_to_protective_volume = (
                event_time_or_invalid(
                    geometry.first_protective_entry_time
                )
            )
            control.supervised_time_to_collision = event_time_or_invalid(
                geometry.first_collision_time
            )
        control.minimum_time_from_now = float(minimum_time_from_now)
        control.minimum_sample_index = int(minimum_sample_index)
        control.minimum_sample_count = int(minimum_sample_count)
        control.minimum_horizon_fraction = float(
            minimum_horizon_fraction
        )
        control.reason_code = reason_code
        control.reason = reason
        control.requested_state = state
        control.requested_speed_scale = float(speed_scale)
        control.requested_reason_code = (
            requested_reason_code or reason_code
        )
        control.recovery_count = 0
        control.recovery_required_count = 0
        control.transition = False
        control.transition_reason = 'UNFILTERED'
        if stability is not None:
            control.requested_state = stability.requested_state
            control.requested_speed_scale = float(
                stability.requested_speed_scale
            )
            control.recovery_count = int(stability.recovery_count)
            control.recovery_required_count = int(
                stability.recovery_required_count
            )
            control.transition = bool(stability.transition)
            control.transition_reason = stability.transition_reason
        control.current_clearance = float(current_clearance)
        control.nominal_clearance = float(nominal_clearance)
        control.supervised_clearance = float(clearance)
        if geometry is not None:
            witness = geometry.minimum
            control.closest_robot_point.x = witness.closest_robot_point[0]
            control.closest_robot_point.y = witness.closest_robot_point[1]
            control.closest_robot_point.z = witness.closest_robot_point[2]
            control.obstacle_center_at_minimum.x = witness.obstacle_center[0]
            control.obstacle_center_at_minimum.y = witness.obstacle_center[1]
            control.obstacle_center_at_minimum.z = witness.obstacle_center[2]
            control.capsule_start.x = witness.capsule_start[0]
            control.capsule_start.y = witness.capsule_start[1]
            control.capsule_start.z = witness.capsule_start[2]
            control.capsule_end.x = witness.capsule_end[0]
            control.capsule_end.y = witness.capsule_end[1]
            control.capsule_end.z = witness.capsule_end[2]
            control.capsule_radius = witness.capsule_radius
            control.evaluated_combinations = geometry.evaluated_combinations
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
        """Evaluate one absolute speed scale using the shared geometry."""
        if speed_scale <= 0.0:
            samples = tuple(tuple(current) for _ in range(sample_count))
        else:
            effective_duration = nominal_remaining / speed_scale
            samples = predict_joint_samples(
                current,
                target,
                effective_duration,
                horizon,
                sample_count,
            ).positions
        sample_times = tuple(
            horizon * index / float(sample_count - 1)
            for index in range(sample_count)
        )
        obstacle_center, obstacle_velocity, obstacle_radius = obstacle
        result = evaluate_joint_horizon(
            samples,
            sample_times,
            obstacle_center,
            obstacle_velocity,
            obstacle_radius,
            self.protective_margin(),
        )
        self.last_horizon_clearance = result
        collision_time = (
            -1.0
            if result.first_collision_time is None
            else result.first_collision_time
        )
        return (
            result.minimum.legacy_tuple(),
            result.minimum.sample_index,
            result.minimum.sample_time,
            collision_time,
        )

    def decide_horizon(self, current_clearance, nominal_geometry,
                       evaluator, previous_scale=1.0):
        """Apply one shared policy to a previously evaluated horizon."""
        margin = self.protective_margin()
        stop_distance = float(
            self.get_parameter('stop_distance').value
        )
        warning_distance = float(
            self.get_parameter('warning_distance').value
        )
        minimum_scale = float(
            self.get_parameter('runtime_minimum_speed_scale').value
        )
        iterations = int(
            self.get_parameter('runtime_scale_search_iterations').value
        )
        nominal_clearance = nominal_geometry.minimum.clearance

        candidate_scale = 1.0
        withdrawal_safe = False
        inside_stop_zone = current_clearance <= stop_distance
        if inside_stop_zone:
            withdrawal_scale = float(
                self.get_parameter('withdrawal_speed_scale').value
            )
            minimum_progress = float(self.get_parameter(
                'withdrawal_minimum_progress_m'
            ).value)
            monotonic_tolerance = float(self.get_parameter(
                'withdrawal_monotonic_tolerance_m'
            ).value)
            if not minimum_scale <= withdrawal_scale <= 1.0:
                raise ValueError(
                    'withdrawal_speed_scale must be between '
                    'runtime_minimum_speed_scale and 1.0'
                )
            candidate_scale = withdrawal_scale
        elif nominal_clearance < margin:
            search = maximum_safe_scale(
                lambda scale: evaluator(scale)[0][0],
                margin,
                minimum_scale,
                iterations,
            )
            candidate_scale = search.scale if search.feasible else 0.0

        selected_evaluation = evaluator(candidate_scale)
        supervised_geometry = self.last_horizon_clearance
        if inside_stop_zone:
            withdrawal_safe = withdrawal_path_is_safe(
                nominal_geometry.sample_clearances,
                minimum_progress,
                monotonic_tolerance,
            ) and withdrawal_path_is_non_approaching(
                supervised_geometry.sample_clearances,
                monotonic_tolerance,
            )
            if not withdrawal_safe:
                candidate_scale = 0.0
                selected_evaluation = evaluator(candidate_scale)
                supervised_geometry = self.last_horizon_clearance
        decision = decide_preventive_state(SafetyPolicyInput(
            current_clearance=current_clearance,
            nominal_clearance=nominal_clearance,
            supervised_clearance=(
                supervised_geometry.minimum.clearance
            ),
            protective_margin=margin,
            stop_distance=stop_distance,
            warning_distance=warning_distance,
            minimum_scale=minimum_scale,
            candidate_scale=candidate_scale,
            nominal_ttc=event_time_or_invalid(
                nominal_geometry.first_collision_time
            ),
            previous_scale=previous_scale,
            withdrawal_safe=withdrawal_safe,
        ))
        if decision.speed_scale != candidate_scale:
            selected_evaluation = evaluator(decision.speed_scale)
            supervised_geometry = self.last_horizon_clearance
        return decision, selected_evaluation, supervised_geometry

    def publish_jog_stop(self, msg, reason_code, reason, current=None):
        """Publish an explicit fail-safe JOG decision and optional hold."""
        raw_decision = SafetyDecision('STOP', 0.0, reason_code)
        decision, stability = self.stabilize_decision(
            'jog',
            msg.command_id,
            raw_decision,
        )
        if current is not None:
            self.publish_supervised_jog(msg, current, 1.0)
        self.publish_execution_control(
            msg.command_id,
            decision.state,
            decision.speed_scale,
            -1.0,
            '',
            NO_EVENT_TIME,
            reason_code,
            reason,
            stability=stability,
            requested_reason_code=reason_code,
        )
        self.last_jog_state = decision.state
        self.last_jog_scale = decision.speed_scale

    def publish_supervised_jog(self, source, positions, horizon):
        output = JointCommand()
        # Preserve the origin stamp for end-to-end correlation.
        output.stamp = source.stamp
        output.command_id = source.command_id
        output.joint_names = list(JOINT_NAMES)
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
        self.publish_timing(source, 'SUPERVISED_PUBLISH')

    def jog_intent_callback(self, msg):
        """Supervise a rolling joint-space velocity intention."""
        receive_monotonic_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'SUPERVISOR_RECEIVE', receive_monotonic_ns
        )
        now_ns = self.get_clock().now().nanoseconds
        message_age = message_age_seconds(now_ns, msg.stamp)
        maximum_age = float(
            self.get_parameter('max_jog_message_age_sec').value
        )
        if message_age < -0.05 or message_age > maximum_age:
            self.publish_jog_stop(
                msg,
                'JOG_MESSAGE_STALE',
                f'La intención JOG llegó obsoleta '
                f'({message_age:.3f} s).',
            )
            return
        self.last_jog_intent_monotonic = time.monotonic()
        if self.active_execution is not None:
            self.publish_jog_stop(
                msg,
                'EXECUTION_ACTIVE',
                'Existe una trayectoria completa activa.',
            )
            return
        if tuple(msg.joint_names) != tuple(JOINT_NAMES):
            self.publish_jog_stop(
                msg,
                'INVALID_JOINT_ORDER',
                'El JOG no usa el orden articular canónico.',
            )
            return
        if len(msg.positions) != len(JOINT_NAMES):
            self.publish_jog_stop(
                msg,
                'INVALID_JOINT_COUNT',
                'El JOG no contiene seis posiciones.',
            )
            return
        if not all(math.isfinite(value) for value in msg.positions):
            self.publish_jog_stop(
                msg,
                'NUMERIC_ERROR',
                'El JOG contiene valores no finitos.',
            )
            return
        if not all(
            name in self.current_positions
            for name in JOINT_NAMES
        ):
            self.publish_jog_stop(
                msg,
                'STATE_UNAVAILABLE_OR_STALE',
                'No existe una configuración articular completa.',
            )
            return
        if self.last_state_receive_ns is None:
            self.publish_jog_stop(
                msg,
                'STATE_UNAVAILABLE_OR_STALE',
                'No existe timestamp del estado articular.',
            )
            return

        current = tuple(
            self.current_positions[name]
            for name in JOINT_NAMES
        )

        state_age = (now_ns - self.last_state_receive_ns) / 1e9
        if state_age > float(
                self.get_parameter('max_state_age_sec').value):
            self.publish_jog_stop(
                msg,
                'STATE_UNAVAILABLE_OR_STALE',
                f'Estado articular obsoleto ({state_age:.3f} s).',
                current,
            )
            return
        horizon = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )
        if not math.isfinite(horizon) or not 0.1 <= horizon <= 2.0:
            self.publish_jog_stop(
                msg,
                'INVALID_DURATION',
                'La duración JOG no pertenece a [0.1, 2.0] s.',
                current,
            )
            return

        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                horizon,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f'JOG REJECTED: {exc}'
            )
            self.publish_jog_stop(
                msg,
                'NUMERIC_ERROR',
                f'No se pudo limitar el JOG: {exc}',
                current,
            )
            return
        bounded_target = saturation.positions

        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_jog_stop(
                msg,
                'PROXIMITY_UNAVAILABLE_OR_STALE',
                'No existe información reciente del obstáculo.',
                current,
            )
            return

        sample_count = int(
            self.get_parameter('runtime_samples').value
        )
        if sample_count < 2:
            self.publish_jog_stop(
                msg,
                'PREDICTION_ERROR',
                'runtime_samples debe ser al menos 2.',
                current,
            )
            return

        obstacle_center, _, obstacle_radius = obstacle

        def evaluator(scale):
            return self.evaluate_runtime_horizon(
                current,
                bounded_target,
                horizon,
                scale,
                horizon,
                sample_count,
                obstacle,
            )

        prediction_start_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'PREDICTION_START', prediction_start_ns
        )
        try:
            current_result = minimum_sphere_clearance(
                current,
                obstacle_center,
                obstacle_radius,
            )
            evaluator(1.0)
            nominal_geometry = self.last_horizon_clearance
            current_clearance = current_result[0]
            decision, selected_evaluation, supervised_geometry = (
                self.decide_horizon(
                    current_clearance,
                    nominal_geometry,
                    evaluator,
                    self.last_jog_scale or 1.0,
                )
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self.publish_jog_stop(
                msg,
                'PREDICTION_ERROR',
                f'Error de predicción JOG: {exc}',
                current,
            )
            return

        prediction_end_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'PREDICTION_END', prediction_end_ns,
            (prediction_end_ns - prediction_start_ns) / 1e9,
        )
        raw_decision = decision
        decision, stability = self.stabilize_decision(
            'jog',
            msg.command_id,
            raw_decision,
        )
        if not math.isclose(
            decision.speed_scale,
            raw_decision.speed_scale,
            abs_tol=1.0e-12,
        ):
            selected_evaluation = evaluator(decision.speed_scale)
            supervised_geometry = self.last_horizon_clearance
        state = decision.state
        speed_scale = decision.speed_scale
        margin = self.protective_margin()
        full_clearance = nominal_geometry.minimum.clearance
        best, sample_index, minimum_time, collision_time = selected_evaluation
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
            f'escala={speed_scale:.3f}; '
            f'raw={raw_decision.state}/{raw_decision.speed_scale:.3f}; '
            f'code={decision.reason_code}; '
            f'recovery={stability.recovery_count}/'
            f'{stability.recovery_required_count}; '
            f'joint_lim={saturation.limiting_joint}; '
            f'v_req={saturation.requested_velocity:.4f}rad/s; '
            f'v_lim={saturation.limited_velocity:.4f}rad/s'
        )
        self.publish_execution_control(
            msg.command_id,
            state,
            speed_scale,
            clearance,
            segment,
            collision_time,
            decision.reason_code,
            reason,
            minimum_time_from_now=minimum_time,
            minimum_sample_index=sample_index,
            minimum_sample_count=sample_count,
            minimum_horizon_fraction=(
                sample_index / float(sample_count - 1)
            ),
            geometry=supervised_geometry,
            nominal_geometry=nominal_geometry,
            current_clearance=current_clearance,
            nominal_clearance=nominal_geometry.minimum.clearance,
            stability=stability,
            requested_reason_code=raw_decision.reason_code,
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
            for joint_name in JOINT_NAMES
        ) or self.last_state_receive_ns is None:
            self.publish_runtime_stop(
                command_id,
                'STATE_UNAVAILABLE_OR_STALE',
                'No se recibió una configuración articular completa.',
            )
            return

        state_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_state_receive_ns
        ) / 1e9
        if state_age_sec > float(
                self.get_parameter('max_state_age_sec').value):
            self.publish_runtime_stop(
                command_id,
                'STATE_UNAVAILABLE_OR_STALE',
                f'Estado articular obsoleto ({state_age_sec:.3f} s).',
            )
            return

        obstacle = self.runtime_obstacle()
        if obstacle is None:
            self.publish_runtime_stop(
                command_id,
                'PROXIMITY_UNAVAILABLE_OR_STALE',
                'No existe una medición de proximidad reciente.',
            )
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        start_sec = self.message_time_seconds(execution.start_time)
        duration = float(execution.duration_sec)
        elapsed = max(0.0, now_sec - start_sec)
        try:
            remaining = runtime_remaining_time(duration, elapsed)
        except ValueError:
            self.publish_runtime_stop(
                command_id,
                'PREDICTION_ERROR',
                'La referencia activa tiene una temporización inválida.',
            )
            return

        current = tuple(
            self.current_positions[name]
            for name in JOINT_NAMES
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
            self.publish_runtime_stop(
                command_id,
                'PREDICTION_ERROR',
                'Horizonte o número de muestras runtime inválido.',
            )
            return

        previous_scale = self.last_runtime_scale
        if previous_scale is None or not 0.0 < previous_scale <= 1.0:
            previous_scale = 1.0
        nominal_remaining = remaining * previous_scale
        obstacle_center, obstacle_velocity, obstacle_radius = obstacle

        def evaluator(scale):
            return self.evaluate_runtime_horizon(
                current,
                target,
                nominal_remaining,
                scale,
                horizon,
                sample_count,
                obstacle,
            )

        try:
            current_result = minimum_sphere_clearance(
                current,
                obstacle_center,
                obstacle_radius,
            )
            evaluator(1.0)
            nominal_geometry = self.last_horizon_clearance
            current_clearance = current_result[0]
            decision, selected_evaluation, supervised_geometry = (
                self.decide_horizon(
                    current_clearance,
                    nominal_geometry,
                    evaluator,
                    previous_scale,
                )
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self.publish_runtime_stop(
                command_id,
                'PREDICTION_ERROR',
                f'No se pudo muestrear la trayectoria activa: {exc}',
            )
            return

        predictive_margin = self.protective_margin()
        raw_decision = decision
        decision, stability = self.stabilize_decision(
            'runtime',
            command_id,
            raw_decision,
        )
        if not math.isclose(
            decision.speed_scale,
            raw_decision.speed_scale,
            abs_tol=1.0e-12,
        ):
            selected_evaluation = evaluator(decision.speed_scale)
            supervised_geometry = self.last_horizon_clearance
        state = decision.state
        speed_scale = decision.speed_scale
        full_speed_clearance = nominal_geometry.minimum.clearance
        best, minimum_sample_index, minimum_time_from_now, collision_time = (
            selected_evaluation
        )
        clearance, segment, _, _, _ = best
        reason_code = decision.reason_code
        reason = (
            f'H={horizon:.2f}s; d_min={clearance:.3f} m; '
            f'd_actual={current_clearance:.3f} m; '
            f'margen={predictive_margin:.3f} m; '
            f'd_nominal={full_speed_clearance:.3f} m; '
            f't_min={minimum_time_from_now:.2f}s; '
            f'muestra={minimum_sample_index + 1}/{sample_count}; '
            f'segment={segment}; '
            f'escala={speed_scale:.2f}; '
            f'raw={raw_decision.state}/{raw_decision.speed_scale:.2f}; '
            f'recovery={stability.recovery_count}/'
            f'{stability.recovery_required_count}'
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
            geometry=supervised_geometry,
            nominal_geometry=nominal_geometry,
            current_clearance=current_clearance,
            nominal_clearance=nominal_geometry.minimum.clearance,
            stability=stability,
            requested_reason_code=raw_decision.reason_code,
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
                f'TTC_nom={event_time_or_invalid(nominal_geometry.first_collision_time):.3f} s'
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

    def bound_candidate_velocity(
        self,
        msg,
        duration_sec,
    ):
        """Return a velocity-bounded copy before predictive evaluation."""
        if not all(name in self.current_positions for name in JOINT_NAMES):
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: current joint state unavailable',
            )
            return None
        current = tuple(
            self.current_positions[name] for name in JOINT_NAMES
        )
        try:
            saturation = saturate_target_by_velocity(
                current,
                msg.positions,
                duration_sec,
            )
        except ValueError as exc:
            self.publish_decision(
                msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                f'REJECTED {msg.command_id}: {exc}',
            )
            return None

        bounded = self.copy_command_with_duration(msg, duration_sec)
        bounded.positions = list(saturation.positions)
        if saturation.was_limited:
            self.get_logger().warning(
                f'VELOCITY SATURATED {msg.command_id}: '
                f'joint={saturation.limiting_joint}, '
                f'requested={saturation.requested_velocity:.4f}rad/s, '
                f'limited={saturation.limited_velocity:.4f}rad/s'
            )
        return bounded, saturation

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
                JOINT_NAMES,
                joint_positions):
            lower, upper = JOINT_POSITION_LIMITS[joint_name]
            if not math.isfinite(position) or not lower <= position <= upper:
                return joint_name, position, lower, upper
        return None

    def predict_candidate(self, msg):
        prediction_start_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'PREDICTION_START', prediction_start_ns
        )
        self.last_prediction_speed_scale = 1.0
        self.last_prediction_reason_code = 'PREDICTION_ERROR'
        if not bool(self.get_parameter('prediction_enabled').value):
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                'La predicción preventiva está deshabilitada.',
            )
            return False

        if not all(
            joint_name in self.current_positions
            for joint_name in JOINT_NAMES
        ) or self.last_state_receive_ns is None:
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'current state unavailable for prediction'
            )
            self.publish_decision(
                msg,
                False,
                'STOP',
                'STATE_UNAVAILABLE_OR_STALE',
                'No existe una configuración articular completa.',
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
            self.publish_decision(
                msg,
                False,
                'STOP',
                'STATE_UNAVAILABLE_OR_STALE',
                f'Estado articular obsoleto ({state_age_sec:.3f} s).',
            )
            return False

        sample_count = int(
            self.get_parameter('prediction_samples').value
        )
        if sample_count < 2:
            self.get_logger().error(
                'prediction_samples must be at least 2'
            )
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                'prediction_samples debe ser al menos 2.',
            )
            return False

        current = tuple(
            self.current_positions[name]
            for name in JOINT_NAMES
        )
        target = tuple(float(value) for value in msg.positions)
        trajectory_duration = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )
        prediction_horizon = float(
            self.get_parameter('prediction_horizon_sec').value
        )
        if prediction_horizon <= 0.0:
            self.get_logger().error(
                'prediction_horizon_sec must be greater than zero'
            )
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                'prediction_horizon_sec debe ser positivo.',
            )
            return False
        try:
            joint_prediction = predict_joint_samples(
                current,
                target,
                trajectory_duration,
                prediction_horizon,
                sample_count,
            )
        except ValueError as exc:
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: prediction invalid: {exc}'
            )
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                f'Predicción articular inválida: {exc}',
            )
            return False
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

        for sample_index, sample in enumerate(
            joint_prediction.positions
        ):
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
        try:
            nominal_geometry = evaluate_joint_horizon(
                joint_prediction.positions,
                joint_prediction.sample_times,
                obstacle,
                obstacle_velocity,
                obstacle_radius,
                self.protective_margin(),
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                f'Error geométrico o cinemático: {exc}',
            )
            return False
        witness = nominal_geometry.minimum
        clearance = witness.clearance
        segment = witness.segment_name
        segment_index = witness.segment_index
        start = witness.capsule_start
        end = witness.capsule_end
        best_index = witness.sample_index
        current_clearance = nominal_geometry.current.clearance
        obstacle_data = (
            obstacle,
            obstacle_velocity,
            obstacle_radius,
        )

        def evaluator(scale):
            return self.evaluate_runtime_horizon(
                current,
                target,
                trajectory_duration,
                scale,
                prediction_horizon,
                sample_count,
                obstacle_data,
            )

        try:
            decision, _, supervised_geometry = self.decide_horizon(
                current_clearance,
                nominal_geometry,
                evaluator,
                self.last_prediction_speed_scale,
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PREDICTION_ERROR',
                f'Error evaluando la política preventiva: {exc}',
            )
            return False

        state = decision.state
        self.last_prediction_speed_scale = decision.speed_scale
        self.last_prediction_reason_code = decision.reason_code

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
        prediction.trajectory_fraction = min(
            joint_prediction.sample_times[best_index]
            / joint_prediction.effective_duration,
            1.0,
        )
        prediction.capsule_start.x = start[0]
        prediction.capsule_start.y = start[1]
        prediction.capsule_start.z = start[2]
        prediction.capsule_end.x = end[0]
        prediction.capsule_end.y = end[1]
        prediction.capsule_end.z = end[2]
        prediction.capsule_radius = CAPSULE_RADII[segment_index]
        prediction.minimum_time_from_now = witness.sample_time
        prediction.current_clearance = current_clearance
        prediction.nominal_clearance = nominal_geometry.minimum.clearance
        prediction.supervised_clearance = (
            supervised_geometry.minimum.clearance
        )
        prediction.protective_margin = nominal_geometry.protective_margin
        prediction.time_to_protective_volume = event_time_or_invalid(
            nominal_geometry.first_protective_entry_time
        )
        prediction.time_to_collision = event_time_or_invalid(
            nominal_geometry.first_collision_time
        )
        prediction.supervised_time_to_protective_volume = (
            event_time_or_invalid(
                supervised_geometry.first_protective_entry_time
            )
        )
        prediction.supervised_time_to_collision = event_time_or_invalid(
            supervised_geometry.first_collision_time
        )
        prediction.closest_robot_point.x = witness.closest_robot_point[0]
        prediction.closest_robot_point.y = witness.closest_robot_point[1]
        prediction.closest_robot_point.z = witness.closest_robot_point[2]
        prediction.obstacle_center_at_minimum.x = witness.obstacle_center[0]
        prediction.obstacle_center_at_minimum.y = witness.obstacle_center[1]
        prediction.obstacle_center_at_minimum.z = witness.obstacle_center[2]
        prediction.evaluated_combinations = (
            nominal_geometry.evaluated_combinations
        )
        self.prediction_publisher.publish(prediction)
        prediction_end_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'PREDICTION_END', prediction_end_ns,
            (prediction_end_ns - prediction_start_ns) / 1e9,
        )

        self.get_logger().info(
            f'PREDICTION {msg.command_id}: {state}, '
            f'min_clearance={clearance:.3f} m, '
            f'path={prediction.trajectory_fraction:.2f}, '
            f'segment={segment}'
        )
        return prediction

    def apply_proximity_policy(self, msg, duration_sec):
        if (
            self.proximity_status is None
            or self.last_proximity_receive_ns is None
        ):
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PROXIMITY_UNAVAILABLE_OR_STALE',
                f'STOP {msg.command_id}: proximity status unavailable',
            )
            return None

        max_age_sec = float(
            self.get_parameter('max_proximity_age_sec').value
        )
        status_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_proximity_receive_ns
        ) / 1e9

        if status_age_sec > max_age_sec:
            self.publish_decision(
                msg,
                False,
                'STOP',
                'PROXIMITY_UNAVAILABLE_OR_STALE',
                f'STOP {msg.command_id}: proximity status stale '
                f'({status_age_sec:.3f} s)',
            )
            return None

        state = self.proximity_status.state.upper()
        clearance = self.proximity_status.minimum_clearance
        segment = self.proximity_status.limiting_segment

        if not math.isfinite(clearance):
            self.publish_decision(
                msg,
                False,
                'STOP',
                'NUMERIC_ERROR',
                f'STOP {msg.command_id}: non-finite proximity clearance',
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
                msg, False, 'STOP', self.last_prediction_reason_code,
                f'STOP {msg.command_id}: proximity, '
                f'clearance={clearance:.3f} m, '
                f'segment={segment}',
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
                    msg, False, 'STOP', 'NO_SAFE_SCALE',
                    f'REJECTED {msg.command_id}: REDUCTION '
                    'requested but duration cannot be '
                    'increased safely',
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
        """Compatibility wrapper around the canonical joint model."""
        return shortest_joint_delta(
            joint_name,
            target_position,
            current_position,
        )

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

            allowed_velocity = JOINT_VELOCITY_LIMITS[joint_name]

            if requested_velocity > allowed_velocity + 1.0e-9:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: {joint_name} '
                    f'requested velocity '
                    f'{requested_velocity:.4f} rad/s exceeds '
                    f'{allowed_velocity:.4f} rad/s',
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
        receive_monotonic_ns = time.monotonic_ns()
        self.publish_timing(
            msg, 'SUPERVISOR_RECEIVE', receive_monotonic_ns
        )
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

        if tuple(msg.joint_names) != JOINT_NAMES:
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
            lower, upper = JOINT_POSITION_LIMITS[joint_name]

            if not lower <= position <= upper:
                self.publish_decision(
                    msg, False, 'REJECTED', 'VALIDATION_REJECTED',
                    f'REJECTED {msg.command_id}: {joint_name}='
                    f'{position:.4f} rad outside '
                    f'[{lower:.4f}, {upper:.4f}]',
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
                f'REJECTED {msg.command_id}: duration must be '
                f'between {MIN_DURATION_SEC:.1f} and '
                f'{MAX_DURATION_SEC:.1f} seconds',
            )
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                f'duration must be between '
                f'{MIN_DURATION_SEC:.1f} and '
                f'{MAX_DURATION_SEC:.1f} seconds'
            )
            return

        # Point commands preserve their requested target. An infeasible
        # duration is rejected instead of silently shortening the movement.
        if not self.validate_velocity(msg, duration_sec):
            return

        bounded_result = self.bound_candidate_velocity(
            msg,
            duration_sec,
        )
        if bounded_result is None:
            return
        bounded_msg, saturation = bounded_result

        supervised_msg = self.apply_proximity_policy(
            bounded_msg,
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
        self.publish_timing(msg, 'SUPERVISED_PUBLISH')

        self.publish_decision(
            msg,
            True,
            self.last_proximity_decision,
            self.last_prediction_reason_code,
            (
                f'{self.last_proximity_decision} {msg.command_id}; '
                f'duration={supervised_duration_sec:.3f}s; '
                f'scale={self.last_prediction_speed_scale:.6f}'
            ),
        )

        self.get_logger().info(
            f'{self.last_proximity_decision} FORWARDED '
            f'id={msg.command_id} | '
            f'joints={len(msg.joint_names)} | '
            f'proximity={self.last_proximity_decision} | '
            f'duration={supervised_duration_sec:.2f} s | '
            f'joint_lim={saturation.limiting_joint} | '
            f'v_req={saturation.requested_velocity:.4f} rad/s | '
            f'v_lim={saturation.limited_velocity:.4f} rad/s | '
            f'input_latency={input_latency_ms:.3f} ms'
        )


def main(args=None):
    rclpy.init(args=args)
    spin_node(SafetySupervisorNode())


if __name__ == '__main__':
    main()
