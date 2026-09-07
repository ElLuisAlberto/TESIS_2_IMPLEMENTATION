import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformException, TransformListener

from thesis_interfaces.msg import ProximityStatus


def closest_point_on_segment(point, start, end):
    """Return the point on a line segment closest to the supplied point."""
    ab_x = end.x - start.x
    ab_y = end.y - start.y
    ab_z = end.z - start.z
    ap_x = point.x - start.x
    ap_y = point.y - start.y
    ap_z = point.z - start.z

    denominator = ab_x * ab_x + ab_y * ab_y + ab_z * ab_z
    if denominator <= 1.0e-12:
        return Point(x=start.x, y=start.y, z=start.z)

    factor = (
        ap_x * ab_x + ap_y * ab_y + ap_z * ab_z
    ) / denominator
    factor = max(0.0, min(1.0, factor))

    return Point(
        x=start.x + factor * ab_x,
        y=start.y + factor * ab_y,
        z=start.z + factor * ab_z,
    )


def point_distance(first, second):
    """Return Euclidean distance between two geometry points."""
    return math.sqrt(
        (first.x - second.x) ** 2
        + (first.y - second.y) ** 2
        + (first.z - second.z) ** 2
    )


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
        self.missing_frames = set()
        self.previous_state = None

        rate = float(self.get_parameter('evaluation_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('evaluation_rate_hz must be greater than zero')
        self.timer = self.create_timer(1.0 / rate, self.evaluate)

        self.get_logger().info(
            f'Proximity monitor ready: {len(self.segment_names)} capsules'
        )

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
        if any(radius <= 0.0 for radius in self.radii):
            raise ValueError('Capsule radii must be greater than zero')

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
        return Point(x=x_value, y=y_value, z=z_value), radius

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
        obstacle, obstacle_radius = self._obstacle()
        unavailable = set()
        best_result = None

        for name, start_frame, end_frame, radius in zip(
                self.segment_names,
                self.start_frames,
                self.end_frames,
                self.radii):
            try:
                start = self._frame_point(start_frame)
                end = self._frame_point(end_frame)
            except TransformException:
                unavailable.update((start_frame, end_frame))
                continue

            closest = closest_point_on_segment(obstacle, start, end)
            clearance = (
                point_distance(obstacle, closest)
                - radius
                - obstacle_radius
            )
            if best_result is None or clearance < best_result[0]:
                best_result = (clearance, name, closest)

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

        if best_result is None:
            return

        clearance, limiting_segment, closest = best_result
        state = self._state_for_clearance(clearance)

        message = ProximityStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.reference_frame = self.reference_frame
        message.state = state
        message.minimum_clearance = clearance
        message.limiting_segment = limiting_segment
        message.closest_robot_point = closest
        message.obstacle_center = obstacle
        message.obstacle_radius = obstacle_radius
        self.publisher.publish(message)

        if state != self.previous_state:
            self.get_logger().info(
                f'{state}: clearance={clearance:.3f} m, '
                f'segment={limiting_segment}'
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
