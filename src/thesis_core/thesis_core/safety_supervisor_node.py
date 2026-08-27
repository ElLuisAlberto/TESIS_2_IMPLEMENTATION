import math

import rclpy
from rclpy.node import Node

from thesis_interfaces.msg import JointCommand


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.candidate_subscription = self.create_subscription(
            JointCommand,
            '/thesis/candidate_command',
            self.candidate_command_callback,
            10
        )

        self.supervised_publisher = self.create_publisher(
            JointCommand,
            '/thesis/supervised_command',
            10
        )

        self.get_logger().info(
            'Safety supervisor started in PASS-THROUGH mode'
        )

    def candidate_command_callback(self, msg):

        receive_time = self.get_clock().now()

        # ------------------------------------------------------
        # Structural validation
        # ------------------------------------------------------

        if len(msg.joint_names) == 0:

            self.get_logger().warning(
                f'REJECTED {msg.command_id}: no joints provided'
            )
            return

        if len(msg.joint_names) != len(msg.positions):

            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'joint_names and positions have different sizes'
            )
            return

        if not all(math.isfinite(value) for value in msg.positions):

            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'non-finite joint position detected'
            )
            return

        duration_sec = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )

        if duration_sec <= 0.0:

            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'duration must be greater than zero'
            )
            return

        # ------------------------------------------------------
        # Input latency
        # ------------------------------------------------------

        candidate_stamp_ns = (
            int(msg.stamp.sec) * 1_000_000_000
            + int(msg.stamp.nanosec)
        )

        receive_ns = receive_time.nanoseconds

        input_latency_ms = (
            receive_ns - candidate_stamp_ns
        ) / 1e6

        # ------------------------------------------------------
        # PASS-THROUGH SUPERVISION
        #
        # Current stage:
        # Every structurally valid command is allowed.
        #
        # Future stages will insert:
        # - joint limits
        # - environment distance
        # - short-horizon prediction
        # - TTC
        # - risk classification
        # ------------------------------------------------------

        self.supervised_publisher.publish(msg)

        self.get_logger().info(
            f'ALLOW id={msg.command_id} | '
            f'joints={len(msg.joint_names)} | '
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
