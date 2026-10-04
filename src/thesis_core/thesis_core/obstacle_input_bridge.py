"""Validate and transform generic obstacle estimates to the robot frame."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from thesis_interfaces.msg import Obstacle
from tf2_ros import Buffer, TransformException, TransformListener

from thesis_core.obstacle_contract import (
    transform_obstacle,
    validate_obstacle,
)
from thesis_core.ros_runtime import spin_node


def time_seconds(stamp):
    """Convert a ROS builtin time message to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ObstacleInputBridge(Node):
    """Accept one conservative sphere estimate and normalize its frame."""

    def __init__(self):
        super().__init__('obstacle_input_bridge')
        self.declare_parameter('input_topic', '/thesis/obstacle_input')
        self.declare_parameter(
            'output_topic', '/thesis/obstacle_in_model_frame'
        )
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('max_age_sec', 0.5)
        self.declare_parameter('future_tolerance_sec', 0.05)
        self.declare_parameter('transform_timeout_sec', 0.10)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        self.future_tolerance_sec = float(
            self.get_parameter('future_tolerance_sec').value
        )
        self.transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value
        )
        if (not self.target_frame or self.max_age_sec <= 0.0 or
                self.future_tolerance_sec < 0.0 or
                self.transform_timeout_sec <= 0.0):
            raise ValueError('invalid obstacle bridge parameters')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Obstacle, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Obstacle,
            self.input_topic,
            self.receive_obstacle,
            10,
        )
        self.get_logger().info(
            f'Obstacle bridge: {self.input_topic} -> {self.output_topic}, '
            f'target_frame={self.target_frame}'
        )

    def receive_obstacle(self, message):
        """Reject bad/stale data; transform and forward a valid estimate."""
        source_frame = str(message.header.frame_id).strip()
        if not source_frame:
            self.get_logger().warning('Rejected obstacle without frame_id')
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        try:
            center, velocity, radius, uncertainty = validate_obstacle(
                (message.center.x, message.center.y, message.center.z),
                (message.velocity.x, message.velocity.y,
                 message.velocity.z),
                message.radius,
                message.uncertainty,
                time_seconds(message.header.stamp),
                now_sec,
                self.max_age_sec,
                self.future_tolerance_sec,
            )
            if source_frame == self.target_frame:
                transformed_center, transformed_velocity = center, velocity
            else:
                transform = self.buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self.transform_timeout_sec),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                transformed_center, transformed_velocity = transform_obstacle(
                    center,
                    velocity,
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
        except (ValueError, TransformException) as exc:
            self.get_logger().warning(f'Rejected obstacle estimate: {exc}')
            return

        output = Obstacle()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.target_frame
        output.center.x, output.center.y, output.center.z = transformed_center
        (output.velocity.x, output.velocity.y,
         output.velocity.z) = transformed_velocity
        output.radius = radius
        output.uncertainty = uncertainty
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    spin_node(ObstacleInputBridge())


if __name__ == '__main__':
    main()
