"""Render the nominal or accepted short-horizon arm occupancy volume."""

import math
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from thesis_core.execution_reference import sample_reference
from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
)


JOINTS = tuple(f'j2n6s300_joint_{index}' for index in range(1, 7))
FRAME_ID = 'j2n6s300_link_base'
VALID_SOURCES = ('auto', 'intent', 'execution')


def predict_samples(current, target, duration, horizon, count):
    """Interpolate a bounded-speed nominal intent over a fixed horizon."""
    if len(current) != 6 or len(target) != 6:
        raise ValueError('Expected six joints')
    if not all(math.isfinite(x) for x in (*current, *target)):
        raise ValueError('Non-finite positions')
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('Invalid duration')
    if not math.isfinite(horizon) or horizon <= 0 or count < 2:
        raise ValueError('Invalid horizon')

    delta = [b - a for a, b in zip(current, target)]
    for index in (0, 3, 4, 5):
        delta[index] = math.atan2(
            math.sin(delta[index]),
            math.cos(delta[index]),
        )
    limits = [math.radians(value) for value in (18, 18, 18, 24, 24, 24)]
    effective_duration = max(
        duration,
        *(abs(delta_value) / limit
          for delta_value, limit in zip(delta, limits)),
    )
    return [tuple(
        current_value + delta_value * min(
            index * horizon / (count - 1) / effective_duration,
            1.0,
        )
        for current_value, delta_value in zip(current, delta)
    ) for index in range(count)]


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
            ('source', 'auto'),
            ('model_frame', FRAME_ID),
        ):
            self.declare_parameter(name, default)

        self.horizon = float(self.get_parameter('horizon_sec').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.count = int(self.get_parameter('samples').value)
        self.margin = float(self.get_parameter('margin_m').value)
        self.source = str(self.get_parameter('source').value).lower()
        self.frame_id = str(self.get_parameter('model_frame').value)
        if (not all(math.isfinite(value) for value in (
                self.horizon, self.rate, self.margin)) or
                not 0 < self.horizon <= 5 or
                not 0 < self.rate <= 30 or
                not 2 <= self.count <= 101 or
                not 0 <= self.margin <= 0.2 or
                self.source not in VALID_SOURCES or not self.frame_id):
            raise ValueError('Invalid preview parameters')

        self.state = None
        self.intent = None
        self.jog = None
        self.execution = None
        self.control = None
        self.controller_feedback = None
        self.state_time = -math.inf
        self.intent_time = -math.inf
        self.jog_time = -math.inf
        self.execution_time = -math.inf
        self.control_time = -math.inf
        self.controller_time = -math.inf
        self.publisher = self.create_publisher(
            MarkerArray,
            '/thesis/horizon_volume',
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
        self.jog_sub = self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self.receive_jog,
            10,
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
               for joint in JOINTS):
            self.state = tuple(values[joint] for joint in JOINTS)
            self.state_time = time.monotonic()

    def receive_intent(self, msg):
        self.intent = msg if tuple(msg.joint_names) == JOINTS else None
        self.intent_time = time.monotonic()

    def receive_jog(self, msg):
        self.jog = msg if tuple(msg.joint_names) == JOINTS else None
        self.jog_time = time.monotonic()

    def receive_execution(self, msg):
        if msg.status == 'ACCEPTED':
            self.execution = msg
            self.execution_time = time.monotonic()

    def receive_control(self, msg):
        self.control = msg
        self.control_time = time.monotonic()

    def receive_controller_state(self, msg):
        positions = dict(zip(
            msg.joint_names,
            msg.feedback.positions,
        ))
        if all(joint in positions and math.isfinite(positions[joint])
               for joint in JOINTS):
            self.controller_feedback = tuple(
                positions[joint] for joint in JOINTS
            )
            self.controller_time = time.monotonic()

    def state_is_fresh(self, now):
        return self.state is not None and now - self.state_time <= 0.5

    def intent_is_fresh(self, now):
        return self.intent is not None and now - self.intent_time <= 0.5

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
        )

    def select_source(self, now_monotonic, now_ros):
        execution_active = self.execution_is_active(now_ros)
        intent_active = self.intent_is_fresh(now_monotonic)
        if self.source == 'execution':
            return 'execution' if execution_active else None
        if self.source == 'intent':
            return 'intent' if intent_active else None
        if execution_active:
            return 'execution'
        if self.jog_is_fresh(now_monotonic):
            return 'jog'
        if intent_active:
            return 'intent'
        return None

    def make_poses(self, source, now_ros):
        if source in ('intent', 'jog'):
            command = self.jog if source == 'jog' else self.intent
            duration = message_time_seconds(command.duration)
            poses = predict_samples(
                self.state,
                tuple(command.positions),
                duration,
                self.horizon,
                self.count,
            )
            return poses, {
                'title': (
                    'CONTROL MANUAL SUPERVISADO'
                    if source == 'jog'
                    else 'INTENCIÓN NOMINAL'
                ),
                'detail': (
                    'velocidad limitada y supervisada en tiempo real'
                    if source == 'jog'
                    else 'sin referencia aceptada; solo visualización'
                ),
                'command_id': command.command_id,
                'elapsed': None,
                'tracking_error': None,
            }

        start = tuple(self.execution.start_positions)
        target = tuple(self.execution.target_positions)
        start_time = message_time_seconds(self.execution.start_time)
        elapsed = max(0.0, now_ros - start_time)
        runtime_state, speed_scale, clearance = self.runtime_state()
        runtime_minimum = self.runtime_minimum()
        if runtime_state == 'STOP':
            poses = [tuple(self.state) for _ in range(self.count)]
        else:
            poses = sample_reference(
                start,
                target,
                float(self.execution.duration_sec),
                elapsed,
                self.horizon,
                self.count,
            )
        clearance_text = (
            'n/a' if clearance is None else f'{clearance:.3f}m'
        )
        minimum_text = ''
        if runtime_minimum is not None:
            minimum_time, sample_number, sample_count = runtime_minimum
            minimum_text = (
                f'\nt_min=+{minimum_time:.2f}s '
                f'muestra={sample_number}/{sample_count}'
            )
        # Anchor the displayed volume at the latest measured configuration.
        poses[0] = self.state
        tracking_error = None
        if (self.controller_feedback is not None and
                time.monotonic() - self.controller_time <= 0.5):
            tracking_error = max(
                abs(a - b)
                for a, b in zip(target, self.controller_feedback)
            )
        return poses, {
            'title': 'REFERENCIA ACEPTADA',
            'detail': (
                f'control={runtime_state} escala={speed_scale:.2f}; '
                f'd_futuro={clearance_text}{minimum_text}'
            ),
            'command_id': self.execution.command_id,
            'elapsed': elapsed,
            'tracking_error': tracking_error,
            'runtime_state': runtime_state,
        }

    def empty_output(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear])

    def append_volume(self, output, poses, source, stamp):
        spacing = 0.04
        if source == 'execution':
            namespace = 'execution_horizon'
            state, _, _ = self.runtime_state()
            colors = {
                'ALLOW': (0.10, 0.75, 0.35, 0.14),
                'WARNING': (1.00, 0.75, 0.05, 0.18),
                'REDUCTION': (1.00, 0.40, 0.02, 0.20),
                'STOP': (0.90, 0.05, 0.05, 0.30),
            }
            red, green, blue, alpha = colors.get(
                state,
                colors['ALLOW'],
            )
        else:
            namespace = 'nominal_horizon'
            red, green, blue, alpha = 0.1, 0.65, 1.0, 0.10

        for index, radius in enumerate(CAPSULE_RADII):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = index
            marker.type = Marker.SPHERE_LIST
            marker.pose.orientation.w = 1.0
            diameter = 2 * math.sqrt(
                (radius + self.margin) ** 2 + (spacing / 2) ** 2
            )
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = alpha
            marker.lifetime.sec = 1
            seen = set()
            for pose in poses:
                start, end = capsule_segments(pose)[index]
                length = math.sqrt(sum(
                    (end[axis] - start[axis]) ** 2
                    for axis in range(3)
                ))
                steps = max(1, math.ceil(length / spacing))
                for step in range(steps + 1):
                    point = [
                        start[axis] + (end[axis] - start[axis])
                        * step / steps
                        for axis in range(3)
                    ]
                    key = tuple(round(value, 5) for value in point)
                    if key in seen:
                        continue
                    seen.add(key)
                    marker.points.append(Point(
                        x=point[0],
                        y=point[1],
                        z=point[2],
                    ))
            output.markers.append(marker)

    def append_time_traces(self, output, poses, stamp):
        """Draw geometric guides at true scale; these are not collision volumes."""
        segments = [capsule_segments(pose) for pose in poses]
        maximum_travel = 0.0
        for link in range(len(CAPSULE_RADII)):
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
            endpoints = [sample[link][1] for sample in segments]
            trace.points = [Point(x=p[0], y=p[1], z=p[2]) for p in endpoints]
            travel = sum(math.dist(a, b) for a, b in zip(endpoints, endpoints[1:]))
            maximum_travel = max(maximum_travel, travel)
            output.markers.append(trace)

        # Skeletons distinguish intermediate/final poses without enlarging occupancy.
        for slot, fraction in enumerate((0.25, 0.5, 1.0)):
            index = round(fraction * (len(poses) - 1))
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
            for a, b in segments[index]:
                skeleton.points.extend([Point(x=p[0], y=p[1], z=p[2]) for p in (a, b)])
            output.markers.append(skeleton)
            if maximum_travel > 0.015:
                label = Marker()
                label.header = skeleton.header
                label.ns = 'horizon_time_labels'
                label.id = slot
                label.type = Marker.TEXT_VIEW_FACING
                label.pose.orientation.w = 1.0
                p = segments[index][-1][1]
                label.pose.position = Point(x=p[0], y=p[1], z=p[2] + 0.05 + slot * 0.035)
                label.scale.z = 0.025
                label.color = skeleton.color
                label.lifetime.sec = 1
                label.text = f'+{index * self.horizon / (len(poses) - 1):.2f} s'
                output.markers.append(label)
        return maximum_travel

    def append_label(self, output, source, description, metadata, hz, elapsed):
        label = Marker()
        stamp = self.get_clock().now().to_msg()
        label.header.frame_id = self.frame_id
        label.header.stamp = stamp
        label.ns = 'horizon_label'
        label.id = 6
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.orientation.w = 1.0
        label.pose.position.z = 1.5
        label.scale.z = 0.045
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        label.lifetime.sec = 1
        if source == 'execution':
            error = metadata['tracking_error']
            error_text = 'n/a' if error is None else f'{error:.3f}rad'
            label.text = (
                f'EJECUCIÓN H={self.horizon:.1f}s N={self.count} '
                f'cmd={metadata["command_id"]}\n'
                f't={elapsed:.2f}s cycle={hz:.1f}Hz '
                f'error={error_text}\n{description}'
            )
        else:
            label.text = (
                f'PREVIEW H={self.horizon:.1f}s N={self.count} '
                f'margin={self.margin:.3f}m\n'
                f'cycle={hz:.1f}Hz compute={metadata:.1f}ms\n'
                f'{description}'
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
            poses, metadata = self.make_poses(source, now_ros)
        except (TypeError, ValueError, ZeroDivisionError):
            self.publisher.publish(output)
            return

        stamp = self.get_clock().now().to_msg()
        self.append_volume(output, poses, source, stamp)
        travel = self.append_time_traces(output, poses, stamp)
        metadata['detail'] += (
            f'\nRecorrido máximo de extremos: {travel * 100:.1f} cm'
            '\nAmarillo: trazas; líneas de color: posturas futuras'
        )
        compute_ms = (time.monotonic() - start) * 1000
        self.append_label(
            output,
            source,
            metadata['detail'],
            metadata if source == 'execution' else compute_ms,
            hz,
            metadata['elapsed'] if source == 'execution' else 0.0,
        )
        self.publisher.publish(output)


def main(args=None):
    """Run the short-horizon occupancy visualizer."""
    rclpy.init(args=args)
    node = HorizonPreview()
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
