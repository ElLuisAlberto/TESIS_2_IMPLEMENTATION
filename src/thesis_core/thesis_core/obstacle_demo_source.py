"""Publish a known test obstacle using the perception input contract."""

import rclpy
from rclpy.node import Node
from thesis_interfaces.msg import Obstacle

from thesis_core.ros_runtime import spin_node


class ObstacleDemoSource(Node):
    """Repeat a fixed world-frame sphere estimate for end-to-end testing."""

    def __init__(self):
        super().__init__('obstacle_demo_source')
        self.declare_parameter('topic', '/thesis/obstacle_input')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('x', 0.60)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.65)
        self.declare_parameter('radius', 0.12)
        self.declare_parameter('uncertainty', 0.02)
        self.declare_parameter('rate_hz', 10.0)
        topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.center = tuple(float(self.get_parameter(axis).value)
                            for axis in ('x', 'y', 'z'))
        self.radius = float(self.get_parameter('radius').value)
        self.uncertainty = float(
            self.get_parameter('uncertainty').value
        )
        rate = float(self.get_parameter('rate_hz').value)
        if (not self.frame_id or self.radius <= 0.0 or
                self.uncertainty < 0.0 or rate <= 0.0):
            raise ValueError('invalid demo obstacle parameters')
        self.publisher = self.create_publisher(Obstacle, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_obstacle)
        self.get_logger().info(
            f'Publishing test obstacle in {self.frame_id} on {topic}'
        )

    def publish_obstacle(self):
        message = Obstacle()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        (message.center.x, message.center.y,
         message.center.z) = self.center
        message.velocity.x = 0.0
        message.velocity.y = 0.0
        message.velocity.z = 0.0
        message.radius = self.radius
        message.uncertainty = self.uncertainty
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin_node(ObstacleDemoSource())


if __name__ == '__main__':
    main()
