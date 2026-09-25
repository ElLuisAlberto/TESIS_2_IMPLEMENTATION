import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage

from thesis_interfaces.msg import ProximityStatus

from thesis_core.clearance_geometry import (
    minimum_configuration_clearance,
)
from thesis_core.jaco_kinematics import CAPSULE_RADII, SEGMENT_NAMES


class ProximityMonitorNode(Node):
    """Evaluate minimum clearance between JACO2 capsules and an obstacle."""

    def __init__(self):
        super().__init__('proximity_monitor')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('evaluation_rate_hz', 20.0)
        self.declare_parameter('status_topic', '/thesis/proximity_status')
        self.declare_parameter('segment_names', ['segment'])
        self.declare_parameter('segment_start_frames', ['world'])
        self.declare_parameter('segment_end_frames', ['world'])
        self.declare_parameter('segment_radii', [0.05])
        self.declare_parameter('obstacle_x', 0.60)
        self.declare_parameter('obstacle_y', 0.0)
        self.declare_parameter('obstacle_z', 0.65)
        self.declare_parameter('obstacle_radius', 0.12)
        self.declare_parameter(
            'obstacle_pose_topic',
            '/world/jaco_world/pose/info',
        )
        self.declare_parameter('obstacle_frame', 'safety_obstacle')
        self.declare_parameter('obstacle_pose_timeout_sec', 0.5)
        self.declare_parameter('warning_distance', 0.30)
        self.declare_parameter('reduction_distance', 0.15)
        self.declare_parameter('stop_distance', 0.05)

        self.reference_frame = str(
            self.get_parameter('reference_frame').value
        )
        self.segment_names = list(
            self.get_parameter('segment_names').value
        )
        self.start_frames = list(
            self.get_parameter('segment_start_frames').value
        )
        self.end_frames = list(
            self.get_parameter('segment_end_frames').value
        )
        self.radii = [
            float(value)
            for value in self.get_parameter('segment_radii').value
        ]
        self._validate_configuration()

        status_topic = str(self.get_parameter('status_topic').value)
        self.publisher = self.create_publisher(
            ProximityStatus,
            status_topic,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obstacle_frame = str(
            self.get_parameter('obstacle_frame').value
        )
        self.obstacle_pose_timeout_sec = float(
            self.get_parameter('obstacle_pose_timeout_sec').value
        )
        if self.obstacle_pose_timeout_sec <= 0.0:
            raise ValueError(
                'obstacle_pose_timeout_sec must be greater than zero'
            )
        obstacle_pose_topic = str(
            self.get_parameter('obstacle_pose_topic').value
        )
        self.dynamic_obstacle_center = None
        self.dynamic_obstacle_velocity = (0.0, 0.0, 0.0)
        self.previous_obstacle_center = None
        self.previous_obstacle_time = None
        self.last_obstacle_pose_time = None
        self.obstacle_pose_subscription = self.create_subscription(
            TFMessage,
            obstacle_pose_topic,
            self.obstacle_pose_callback,
            10,
        )
        self.missing_frames = set()
        self.previous_state = None

        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be greater than zero')
        self.timer = self.create_timer(1.0 / rate, self.evaluate)

        self.get_logger().info(
            f'Proximity monitor ready: {len(self.segment_names)} capsules'
        )

        self.get_logger().info(
            f'Dynamic obstacle pose topic: {obstacle_pose_topic}; '
            f'frame={self.obstacle_frame}'
        )

    def obstacle_pose_callback(self, msg):
        """Track the obstacle pose bridged from Gazebo Pose_V."""
        selected = None
        for transform in msg.transforms:
            child_frame = str(transform.child_frame_id)
            root_frame = child_frame.replace('::', '/').split('/', 1)[0]
            if child_frame == self.obstacle_frame or root_frame == (
                    self.obstacle_frame):
                selected = transform.transform.translation
                break

        if selected is None:
            return

        center = Point(
            x=float(selected.x),
            y=float(selected.y),
            z=float(selected.z),
        )
        if not all(math.isfinite(value) for value in (
            center.x, center.y, center.z
        )):
            return
        now = time.monotonic()
        if self.previous_obstacle_center is not None:
            elapsed = now - self.previous_obstacle_time
            if elapsed > 1.0e-3:
                self.dynamic_obstacle_velocity = (
                    (center.x - self.previous_obstacle_center.x) / elapsed,
                    (center.y - self.previous_obstacle_center.y) / elapsed,
                    (center.z - self.previous_obstacle_center.z) / elapsed,
                )

        self.dynamic_obstacle_center = center
        self.previous_obstacle_center = center
        self.previous_obstacle_time = now
        self.last_obstacle_pose_time = now

    def _validate_configuration(self):
        segment_count = len(self.segment_names)
        sizes = (
            segment_count,
            len(self.start_frames),
            len(self.end_frames),
            len(self.radii),
        )
        if segment_count == 0 or len(set(sizes)) != 1:
            raise ValueError(
                'Capsule parameter arrays must have equal non-zero length'
            )
        if any(
            not math.isfinite(radius) or radius <= 0.0
            for radius in self.radii
        ):
            raise ValueError('Capsule radii must be finite and positive')
        if tuple(self.segment_names) != SEGMENT_NAMES:
            raise ValueError('Segment names differ from canonical geometry')
        if len(self.radii) != len(CAPSULE_RADII) or any(
            not math.isclose(actual, expected, abs_tol=1.0e-12)
            for actual, expected in zip(self.radii, CAPSULE_RADII)
        ):
            raise ValueError('Capsule radii differ from canonical geometry')

        warning = float(self.get_parameter('warning_distance').value)
        reduction = float(
            self.get_parameter('reduction_distance').value
        )
        stop = float(self.get_parameter('stop_distance').value)
        if not 0.0 <= stop < reduction < warning:
            raise ValueError(
                'Thresholds must satisfy 0 <= stop < reduction < warning'
            )

    def _frame_point(self, frame_name):
        transform = self.tf_buffer.lookup_transform(
            self.reference_frame,
            frame_name,
            Time(),
        )
        translation = transform.transform.translation
        return Point(
            x=translation.x,
            y=translation.y,
            z=translation.z,
        )

    def _obstacle(self):
        x_value = float(self.get_parameter('obstacle_x').value)
        y_value = float(self.get_parameter('obstacle_y').value)
        z_value = float(self.get_parameter('obstacle_z').value)
        radius = float(self.get_parameter('obstacle_radius').value)
        if radius <= 0.0:
            raise ValueError('obstacle_radius must be greater than zero')
        if (
            self.dynamic_obstacle_center is not None
            and self.last_obstacle_pose_time is not None
            and time.monotonic() - self.last_obstacle_pose_time
            <= self.obstacle_pose_timeout_sec
        ):
            return (
                self.dynamic_obstacle_center,
                radius,
                self.dynamic_obstacle_velocity,
            )

        return Point(x=x_value, y=y_value, z=z_value), radius, (
            0.0,
            0.0,
            0.0,
        )

    def _state_for_clearance(self, clearance):
        if clearance <= float(self.get_parameter('stop_distance').value):
            return 'STOP'
        if clearance <= float(
                self.get_parameter('reduction_distance').value):
            return 'REDUCTION'
        if clearance <= float(
                self.get_parameter('warning_distance').value):
            return 'WARNING'
        return 'ALLOW'

    def evaluate(self):
        obstacle, obstacle_radius, obstacle_velocity = self._obstacle()
        unavailable = set()
        segments = []

        for start_frame, end_frame in zip(
                self.start_frames, self.end_frames):
            try:
                start = self._frame_point(start_frame)
                end = self._frame_point(end_frame)
            except TransformException:
                unavailable.update((start_frame, end_frame))
                continue
            segments.append((
                (start.x, start.y, start.z),
                (end.x, end.y, end.z),
            ))

        if unavailable != self.missing_frames:
            if unavailable:
                self.get_logger().warning(
                    'Waiting for TF frames: '
                    + ', '.join(sorted(unavailable))
                )
            elif self.missing_frames:
                self.get_logger().info(
                    'All proximity TF frames are available'
                )
            self.missing_frames = unavailable

        if unavailable or len(segments) != len(self.segment_names):
            return

        try:
            result = minimum_configuration_clearance(
                segments,
                self.segment_names,
                self.radii,
                (obstacle.x, obstacle.y, obstacle.z),
                obstacle_radius,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid proximity geometry: {exc}')
            return

        state = self._state_for_clearance(result.clearance)
        closest = Point(
            x=result.closest_robot_point[0],
            y=result.closest_robot_point[1],
            z=result.closest_robot_point[2],
        )

        message = ProximityStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.reference_frame = self.reference_frame
        message.state = state
        message.minimum_clearance = result.clearance
        message.limiting_segment = result.segment_name
        message.closest_robot_point = closest
        message.obstacle_center = obstacle
        message.obstacle_velocity.x = obstacle_velocity[0]
        message.obstacle_velocity.y = obstacle_velocity[1]
        message.obstacle_velocity.z = obstacle_velocity[2]
        message.obstacle_radius = obstacle_radius
        self.publisher.publish(message)

        if state != self.previous_state:
            self.get_logger().info(
                f'{state}: clearance={result.clearance:.3f} m, '
                f'segment={result.segment_name}'
            )
            self.previous_state = state


def main(args=None):
    rclpy.init(args=args)
    node = ProximityMonitorNode()

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
