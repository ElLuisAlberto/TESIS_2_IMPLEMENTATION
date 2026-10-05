"""Render the nominal or accepted short-horizon arm occupancy volume."""

import math
import time
from typing import Optional, Sequence, Tuple

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.jaco_kinematics import CAPSULE_RADII
from thesis_core.swept_volume import (
    build_swept_volume,
    maximum_endpoint_travel,
)
from thesis_core.joint_model import JOINT_NAMES
from thesis_core.joint_prediction import (
    is_state_fresh,
    predict_joint_samples,
)
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
    JointTrajectoryPrediction,
    ProximityStatus,
)


FRAME_ID = 'j2n6s300_link_base'
VALID_SOURCES = ('auto', 'intent', 'execution')
STATE_COLORS = {
    'ALLOW': (0.10, 0.65, 1.00, 0.14),
    'WARNING': (1.00, 0.82, 0.05, 0.20),
    'REDUCTION': (1.00, 0.35, 0.02, 0.24),
    'STOP': (1.00, 0.03, 0.03, 0.32),
}


def movement_directions(
    measured: Sequence[float], target: Sequence[float],
) -> Tuple[int, ...]:
    """Determine target motion signs while ignoring encoder scale noise."""
    return tuple(
        1 if goal - current > 1e-3 else
        -1 if current - goal > 1e-3 else 0
        for current, goal in zip(measured, target)
    )


def direction_reversed(
    previous: Optional[Tuple[int, ...]], current: Tuple[int, ...],
) -> bool:
    """Detect a genuine change of direction on a commanded joint."""
    return previous is not None and any(
        before * after < 0 for before, after in zip(previous, current)
    )


def predict_samples(current, target, duration, horizon, count):
    """Compatibility wrapper around the deterministic predictor."""
    return list(predict_joint_samples(
        current,
        target,
        duration,
        horizon,
        count,
    ).positions)


def message_time_seconds(value):
    """Convert a ROS time message to seconds."""
    return float(value.sec) + float(value.nanosec) * 1e-9


class HorizonPreview(Node):
    """Render discrete future capsules in the model base frame."""

    def __init__(self):
        super().__init__('horizon_preview')
        for name, default in (
            ('horizon_sec', 1.0),
            ('rate_hz', 10.0),
            ('samples', 21),
            ('margin_m', 0.02),
            ('spatial_spacing_m', 0.04),
            ('max_state_age_sec', 0.5),
            ('source', 'auto'),
            ('model_frame', FRAME_ID),
        ):
            self.declare_parameter(name, default)

        self.horizon = float(self.get_parameter('horizon_sec').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.count = int(self.get_parameter('samples').value)
        self.margin = float(self.get_parameter('margin_m').value)
        self.spacing = float(
            self.get_parameter('spatial_spacing_m').value
        )
        self.max_state_age = float(
            self.get_parameter('max_state_age_sec').value
        )
        self.source = str(self.get_parameter('source').value).lower()
        self.frame_id = str(self.get_parameter('model_frame').value)
        if (not all(math.isfinite(value) for value in (
                self.horizon, self.rate, self.margin, self.spacing,
                self.max_state_age)) or
                not 0 < self.horizon <= 5 or
                not 0 < self.rate <= 30 or
                not 2 <= self.count <= 101 or
                not 0 <= self.margin <= 0.2 or
                not 0 < self.spacing <= 0.10 or
                not 0 < self.max_state_age <= 5.0 or
                self.source not in VALID_SOURCES or not self.frame_id):
            raise ValueError('Invalid preview parameters')

        self.state = None
        self.intent = None
        self.supervised_intent = None
        self.jog = None
        self._previous_jog_direction = None
        self.execution = None
        self.control = None
        self.proximity_status = None
        self.controller_feedback = None
        self.state_time = -math.inf
        self.state_stamp = None
        self.intent_time = -math.inf
        self.supervised_intent_time = -math.inf
        self.jog_time = -math.inf
        self.execution_time = -math.inf
        self.control_time = -math.inf
        self.proximity_time = -math.inf
        self.controller_time = -math.inf
        self.publisher = self.create_publisher(
            MarkerArray,
            '/thesis/horizon_volume',
            10,
        )
        self.prediction_publisher = self.create_publisher(
            JointTrajectoryPrediction,
            '/thesis/joint_trajectory_prediction',
            10,
        )
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.receive_state,
            10,
        )
        self.intent_sub = self.create_subscription(
            JointCommand,
            '/thesis/preview_intent',
            self.receive_intent,
            10,
        )
        self.supervised_intent_sub = self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.receive_supervised_intent,
            10,
        )
        jog_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.jog_sub = self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self.receive_jog,
            jog_qos,
        )
        self.execution_sub = self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.receive_execution,
            10,
        )
        self.control_sub = self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.receive_control,
            10,
        )
        self.proximity_sub = self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.receive_proximity,
            10,
        )
        self.controller_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.receive_controller_state,
            10,
        )
        self.timer = self.create_timer(1 / self.rate, self.tick)
        self.previous_tick = None
        self.get_logger().info(
            f'Horizon preview started: source={self.source}, '
            f'horizon={self.horizon:.2f}s, rate={self.rate:.1f}Hz'
        )

    def receive_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if all(joint in values and math.isfinite(values[joint])
               for joint in JOINT_NAMES):
            self.state = tuple(values[joint] for joint in JOINT_NAMES)
            self.state_stamp = msg.header.stamp
            self.state_time = time.monotonic()

    def receive_intent(self, msg):
        self.intent = msg if tuple(msg.joint_names) == JOINT_NAMES else None
        self.intent_time = time.monotonic()

    def receive_supervised_intent(self, msg):
        self.supervised_intent = (
            msg if tuple(msg.joint_names) == JOINT_NAMES else None
        )
        self.supervised_intent_time = time.monotonic()

    def receive_jog(self, msg):
        self.jog = (
            msg if tuple(msg.joint_names) == JOINT_NAMES
            and len(msg.positions) == len(JOINT_NAMES) else None
        )
        now = time.monotonic()
        self.jog_time = now
        if self.jog is None or self.state is None:
            self._previous_jog_direction = None
            return
        current = movement_directions(self.state, msg.positions)
        reversed_motion = direction_reversed(
            self._previous_jog_direction, current,
        )
        self._previous_jog_direction = current
        if not reversed_motion or not self.state_is_fresh(now):
            return
        now_ros = self.get_clock().now().nanoseconds / 1e9
        if self.select_source(now, now_ros) != 'jog':
            return
        try:
            _, metadata, prediction = self.make_poses('jog', now_ros)
        except (TypeError, ValueError, ZeroDivisionError):
            return
        self.publish_joint_prediction(
            prediction, metadata, self.get_clock().now().to_msg(),
        )

    def receive_execution(self, msg):
        if msg.status == 'ACCEPTED':
            self.execution = msg
            self.execution_time = time.monotonic()

    def receive_control(self, msg):
        self.control = msg
        self.control_time = time.monotonic()

    def receive_proximity(self, msg):
        self.proximity_status = msg
        self.proximity_time = time.monotonic()

    def receive_controller_state(self, msg):
        positions = dict(zip(
            msg.joint_names,
            msg.feedback.positions,
        ))
        if all(joint in positions and math.isfinite(positions[joint])
               for joint in JOINT_NAMES):
            self.controller_feedback = tuple(
                positions[joint] for joint in JOINT_NAMES
            )
            self.controller_time = time.monotonic()

    def state_is_fresh(self, now):
        return (
            self.state is not None
            and self.state_stamp is not None
            and is_state_fresh(
                self.state_time, now, self.max_state_age
            )
        )

    def intent_is_fresh(self, now):
        return self.intent is not None and now - self.intent_time <= 0.5

    def supervised_intent_is_fresh(self, now):
        return (
            self.supervised_intent is not None
            and now - self.supervised_intent_time <= 0.5
        )

    def jog_is_fresh(self, now):
        return self.jog is not None and now - self.jog_time <= 0.25

    def execution_is_active(self, now_ros):
        if self.execution is None:
            return False
        if len(self.execution.start_positions) != 6:
            return False
        if len(self.execution.target_positions) != 6:
            return False
        if not math.isfinite(self.execution.duration_sec):
            return False
        start = message_time_seconds(self.execution.start_time)
        if now_ros <= start + self.execution.duration_sec + 0.5:
            return True
        return (
            self.control is not None
            and self.control.command_id == self.execution.command_id
            and self.control.state == 'STOP'
            and time.monotonic() - self.control_time <= 1.0
        )

    def runtime_state(self):
        if (
            self.control is None
            or time.monotonic() - self.control_time > 1.0
            or self.execution is None
            or self.control.command_id != self.execution.command_id
        ):
            return 'ALLOW', 1.0, None
        return (
            self.control.state,
            self.control.speed_scale,
            self.control.minimum_clearance,
        )

    def runtime_minimum(self):
        """Return where the supervisor found its minimum in the horizon."""
        if (
            self.control is None
            or time.monotonic() - self.control_time > 1.0
            or self.execution is None
            or self.control.command_id != self.execution.command_id
            or self.control.minimum_sample_count < 2
        ):
            return None
        sample_count = int(self.control.minimum_sample_count)
        sample_number = min(
            int(self.control.minimum_sample_index) + 1,
            sample_count,
        )
        return (
            float(self.control.minimum_time_from_now),
            sample_number,
            sample_count,
            float(self.control.protective_margin),
            float(self.control.time_to_protective_volume),
            float(self.control.time_to_collision),
        )

    def select_source(self, now_monotonic, now_ros):
        execution_active = self.execution_is_active(now_ros)
        supervised_active = self.supervised_intent_is_fresh(now_monotonic)
        intent_active = self.intent_is_fresh(now_monotonic)
        if self.source == 'execution':
            return 'execution' if execution_active else 'zero'
        if self.source == 'intent':
            if supervised_active:
                return 'supervised_intent'
            return 'intent' if intent_active else 'zero'
        if execution_active:
            return 'execution'
        if self.jog_is_fresh(now_monotonic):
            return 'jog'
        if supervised_active:
            return 'supervised_intent'
        if intent_active:
            return 'intent'
        return 'zero'

    def make_poses(self, source, now_ros):
        if source in ('intent', 'supervised_intent', 'jog', 'zero'):
            if source == 'zero':
                command = None
                target = self.state
                duration = self.horizon
                prediction_source = 'zero'
            else:
                command = {
                    'intent': self.intent,
                    'supervised_intent': self.supervised_intent,
                    'jog': self.jog,
                }[source]
                target = tuple(command.positions)
                duration = message_time_seconds(command.duration)
                prediction_source = (
                    'jog' if source == 'jog' else 'intent'
                )
            prediction = predict_joint_samples(
                self.state,
                target,
                duration,
                self.horizon,
                self.count,
            )
            titles = {
                'intent': 'INTENCIÓN NOMINAL',
                'supervised_intent': 'INTENCIÓN SUPERVISADA',
                'jog': 'CONTROL MANUAL SUPERVISADO',
                'zero': 'VELOCIDAD CERO',
            }
            details = {
                'intent': 'previsualización nominal de la interfaz',
                'supervised_intent': 'intención limitada por el supervisor',
                'jog': 'velocidad limitada y supervisada en tiempo real',
                'zero': 'sin intención vigente; postura medida constante',
            }
            return list(prediction.positions), {
                'title': titles[source],
                'detail': details[source],
                'command_id': '' if command is None else command.command_id,
                'elapsed': None,
                'tracking_error': None,
                'prediction_source': prediction_source,
            }, prediction

        target = tuple(self.execution.target_positions)
        start_time = message_time_seconds(self.execution.start_time)
        elapsed = max(0.0, now_ros - start_time)
        remaining = max(
            float(self.execution.duration_sec) - elapsed,
            1.0e-6,
        )
        runtime_state, speed_scale, clearance = self.runtime_state()
        runtime_minimum = self.runtime_minimum()
        if runtime_state == 'STOP':
            prediction_target = self.state
            prediction_duration = self.horizon
            prediction_source = 'zero'
        else:
            prediction_target = target
            prediction_duration = remaining / max(speed_scale, 1.0e-6)
            prediction_source = 'execution'
        prediction = predict_joint_samples(
            self.state,
            prediction_target,
            prediction_duration,
            self.horizon,
            self.count,
        )
        clearance_text = (
            'n/a' if clearance is None else f'{clearance:.3f}m'
        )
        minimum_text = ''
        if runtime_minimum is not None:
            (
                minimum_time,
                sample_number,
                sample_count,
                protective_margin,
                protective_ttc,
                collision_ttc,
            ) = runtime_minimum
            protective_text = (
                'n/a' if protective_ttc < 0.0
                else f'{protective_ttc:.2f}s'
            )
            collision_text = (
                'n/a' if collision_ttc < 0.0
                else f'{collision_ttc:.2f}s'
            )
            minimum_text = (
                f'\nt_min=+{minimum_time:.2f}s '
                f'muestra={sample_number}/{sample_count}; '
                f'margen={protective_margin:.3f}m; '
                f'TTCp={protective_text}; TTCc={collision_text}'
            )
        tracking_error = None
        if (self.controller_feedback is not None and
                time.monotonic() - self.controller_time <= 0.5):
            tracking_error = max(
                abs(a - b)
                for a, b in zip(target, self.controller_feedback)
            )
        return list(prediction.positions), {
            'title': 'REFERENCIA ACEPTADA',
            'detail': (
                f'control={runtime_state} escala={speed_scale:.2f}; '
                f'd_futuro={clearance_text}{minimum_text}'
            ),
            'command_id': self.execution.command_id,
            'elapsed': elapsed,
            'tracking_error': tracking_error,
            'runtime_state': runtime_state,
            'prediction_source': prediction_source,
        }, prediction

    def publish_joint_prediction(self, prediction, metadata, stamp):
        output = JointTrajectoryPrediction()
        output.stamp = stamp
        output.origin_stamp = self.state_stamp
        output.command_id = metadata['command_id']
        output.source = metadata['prediction_source']
        output.joint_names = list(JOINT_NAMES)
        output.sample_count = len(prediction.positions)
        output.sample_period = prediction.sample_period
        output.horizon = prediction.horizon
        output.effective_duration = prediction.effective_duration
        output.sample_times = list(prediction.sample_times)
        output.positions = [
            value
            for sample in prediction.positions
            for value in sample
        ]
        self.prediction_publisher.publish(output)

    def empty_output(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear])

    def visualization_state(self, source, now_monotonic):
        if source == 'execution':
            return self.runtime_state()[0]
        if (
            self.proximity_status is not None
            and now_monotonic - self.proximity_time <= 0.5
            and self.proximity_status.state.upper() in STATE_COLORS
        ):
            return self.proximity_status.state.upper()
        return 'ALLOW'

    def append_volume(self, output, volume, state, stamp):
        red, green, blue, predicted_alpha = STATE_COLORS[state]
        for index, capsule in enumerate(volume.current_capsules):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'current_capsules'
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * capsule.radius
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = min(0.48, predicted_alpha + 0.20)
            marker.lifetime.sec = 1
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in capsule.points]
            output.markers.append(marker)

        for index, (radius, points) in enumerate(zip(
            CAPSULE_RADII,
            volume.predicted_points,
        )):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'predicted_volume'
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * (radius + self.margin)
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = predicted_alpha
            marker.lifetime.sec = 1
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            output.markers.append(marker)

    def append_time_traces(self, output, volume, stamp):
        maximum_travel = maximum_endpoint_travel(volume)
        for link, endpoints in enumerate(volume.endpoint_traces):
            trace = Marker()
            trace.header.frame_id = self.frame_id
            trace.header.stamp = stamp
            trace.ns = 'horizon_endpoint_traces'
            trace.id = link
            trace.type = Marker.LINE_STRIP
            trace.pose.orientation.w = 1.0
            trace.scale.x = 0.006
            trace.color.r, trace.color.g, trace.color.b = 1.0, 0.8, 0.1
            trace.color.a = 0.9
            trace.lifetime.sec = 1
            trace.points = [Point(x=x, y=y, z=z) for x, y, z in endpoints]
            output.markers.append(trace)

        for slot, fraction in enumerate((0.25, 0.5, 1.0)):
            index = round(fraction * (len(volume.samples) - 1))
            skeleton = Marker()
            skeleton.header.frame_id = self.frame_id
            skeleton.header.stamp = stamp
            skeleton.ns = 'horizon_time_poses'
            skeleton.id = slot
            skeleton.type = Marker.LINE_LIST
            skeleton.pose.orientation.w = 1.0
            skeleton.scale.x = 0.008
            skeleton.color.r = fraction
            skeleton.color.g = 1.0 - 0.5 * fraction
            skeleton.color.b = 1.0
            skeleton.color.a = 0.9
            skeleton.lifetime.sec = 1
            for capsule in volume.samples[index]:
                skeleton.points.extend([
                    Point(x=capsule.start[0], y=capsule.start[1], z=capsule.start[2]),
                    Point(x=capsule.end[0], y=capsule.end[1], z=capsule.end[2]),
                ])
            output.markers.append(skeleton)

            label = Marker()
            label.header = skeleton.header
            label.ns = 'horizon_labels'
            label.id = slot
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.orientation.w = 1.0
            endpoint = volume.samples[index][-1].end
            label.pose.position = Point(
                x=endpoint[0],
                y=endpoint[1],
                z=endpoint[2] + 0.05 + slot * 0.035,
            )
            label.scale.z = 0.025
            label.color = skeleton.color
            label.lifetime.sec = 1
            label.text = f'+{fraction * self.horizon:.2f} s'
            output.markers.append(label)
        return maximum_travel

    def append_label(
        self,
        output,
        source,
        description,
        metadata,
        hz,
        elapsed,
        compute_ms,
        state,
    ):
        label = Marker()
        stamp = self.get_clock().now().to_msg()
        label.header.frame_id = self.frame_id
        label.header.stamp = stamp
        label.ns = 'horizon_labels'
        label.id = 100
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.orientation.w = 1.0
        label.pose.position.z = 1.5
        label.scale.z = 0.045
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        label.lifetime.sec = 1
        common = (
            f'state={state} cycle={hz:.1f}Hz '
            f'compute={compute_ms:.1f}ms'
        )
        if source == 'execution':
            error = metadata['tracking_error']
            error_text = 'n/a' if error is None else f'{error:.3f}rad'
            label.text = (
                f'EJECUCIÓN H={self.horizon:.1f}s N={self.count} '
                f'cmd={metadata["command_id"]}\n'
                f't={elapsed:.2f}s {common} error={error_text}\n'
                f'{description}'
            )
        else:
            label.text = (
                f'PREVIEW H={self.horizon:.1f}s N={self.count} '
                f'margin={self.margin:.3f}m\n'
                f'{common}\n{description}'
            )
        output.markers.append(label)

    def tick(self):
        start = time.monotonic()
        hz = 0.0 if self.previous_tick is None else 1 / max(
            start - self.previous_tick,
            1e-9,
        )
        self.previous_tick = start
        output = self.empty_output()
        now_ros = self.get_clock().now().nanoseconds / 1e9
        if not self.state_is_fresh(start):
            self.publisher.publish(output)
            return

        source = self.select_source(start, now_ros)
        if source is None:
            self.publisher.publish(output)
            return

        try:
            poses, metadata, prediction = self.make_poses(
                source, now_ros
            )
        except (TypeError, ValueError, ZeroDivisionError):
            self.publisher.publish(output)
            return

        stamp = self.get_clock().now().to_msg()
        self.publish_joint_prediction(prediction, metadata, stamp)
        volume = build_swept_volume(
            poses, self.margin, self.spacing
        )
        state = self.visualization_state(source, start)
        self.append_volume(output, volume, state, stamp)
        travel = self.append_time_traces(output, volume, stamp)
        metadata['detail'] += (
            f'\nRecorrido máximo de extremos: {travel * 100:.1f} cm'
            '\nAmarillo: trazas; líneas de color: posturas futuras'
        )
        compute_ms = (time.monotonic() - start) * 1000
        self.append_label(
            output,
            source,
            metadata['detail'],
            metadata,
            hz,
            metadata['elapsed'] if source == 'execution' else 0.0,
            compute_ms,
            state,
        )
        self.publisher.publish(output)


def main(args=None):
    """Run the short-horizon occupancy visualizer."""
    rclpy.init(args=args)
    spin_node(HorizonPreview())


if __name__ == '__main__':
    main()
