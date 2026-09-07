import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Quaternion
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_z_axis(dx, dy, dz):
    """Return a quaternion that rotates the positive Z axis onto a vector."""
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1.0e-9:
        return Quaternion(w=1.0)

    ux = dx / length
    uy = dy / length
    uz = dz / length

    if uz < -0.999999:
        return Quaternion(x=1.0, w=0.0)

    scale = math.sqrt(2.0 * (1.0 + uz))
    return Quaternion(
        x=-uy / scale,
        y=ux / scale,
        z=0.0,
        w=0.5 * scale,
    )


class CapsuleVisualizer(Node):
    """Publish configurable JACO2 link capsules as RViz markers."""

    def __init__(self):
        super().__init__('capsule_visualizer')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('marker_topic', '/thesis/robot_capsules')
        self.declare_parameter('segment_names', ['segment'])
        self.declare_parameter('segment_start_frames', ['world'])
        self.declare_parameter('segment_end_frames', ['world'])
        self.declare_parameter('segment_radii', [0.05])
        self.declare_parameter('color_rgba', [0.05, 0.80, 0.35, 0.38])

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
        self.color = [
            float(value)
            for value in self.get_parameter('color_rgba').value
        ]

        self._validate_configuration()

        marker_topic = str(
            self.get_parameter('marker_topic').value
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.missing_frames = set()

        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_capsules,
        )

        self.get_logger().info(
            f'Capsule visualizer ready: {len(self.segment_names)} '
            f'segments on {marker_topic}'
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
                'segment_names, segment_start_frames, '
                'segment_end_frames and segment_radii must have '
                'the same non-zero length'
            )
        if any(radius <= 0.0 for radius in self.radii):
            raise ValueError('Every capsule radius must be greater than zero')
        if len(self.color) != 4:
            raise ValueError('color_rgba must contain four values')

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

    def _base_marker(self, marker_id, marker_type, segment_name):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_capsules'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]
        marker.text = segment_name
        return marker

    def _sphere_marker(self, marker_id, point, radius, segment_name):
        marker = self._base_marker(
            marker_id,
            Marker.SPHERE,
            segment_name,
        )
        marker.pose.position = point
        diameter = 2.0 * radius
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        return marker

    def _capsule_markers(self, index, start, end, radius):
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1.0e-6:
            return []

        marker_id = index * 3
        segment_name = self.segment_names[index]
        cylinder = self._base_marker(
            marker_id,
            Marker.CYLINDER,
            segment_name,
        )
        cylinder.pose.position = Point(
            x=(start.x + end.x) * 0.5,
            y=(start.y + end.y) * 0.5,
            z=(start.z + end.z) * 0.5,
        )
        cylinder.pose.orientation = quaternion_from_z_axis(dx, dy, dz)
        cylinder.scale.x = 2.0 * radius
        cylinder.scale.y = 2.0 * radius
        cylinder.scale.z = length

        return [
            cylinder,
            self._sphere_marker(
                marker_id + 1,
                start,
                radius,
                segment_name,
            ),
            self._sphere_marker(
                marker_id + 2,
                end,
                radius,
                segment_name,
            ),
        ]

    def publish_capsules(self):
        markers = []
        unavailable = set()

        for index, (start_frame, end_frame, radius) in enumerate(zip(
                self.start_frames,
                self.end_frames,
                self.radii)):
            try:
                start = self._frame_point(start_frame)
                end = self._frame_point(end_frame)
            except TransformException:
                unavailable.update((start_frame, end_frame))
                continue

            markers.extend(
                self._capsule_markers(index, start, end, radius)
            )

        if unavailable != self.missing_frames:
            if unavailable:
                frames = ', '.join(sorted(unavailable))
                self.get_logger().warning(
                    f'Waiting for TF frames: {frames}'
                )
            elif self.missing_frames:
                self.get_logger().info(
                    'All capsule TF frames are available'
                )
            self.missing_frames = unavailable

        if markers:
            message = MarkerArray()
            message.markers = markers
            self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CapsuleVisualizer()

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
