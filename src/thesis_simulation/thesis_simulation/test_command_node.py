import math
import time

import rclpy
from rclpy.node import Node

from thesis_interfaces.msg import JointCommand


class TestCommandNode(Node):

    def __init__(self):
        super().__init__('test_command_node')

        self.declare_parameter(
            'joint_1_target',
            0.20
        )

        self.declare_parameter(
            'duration_sec',
            3.0
        )

        self.publisher = self.create_publisher(
            JointCommand,
            '/thesis/candidate_command',
            10
        )

    def publish_test_command(self):

        target_j1 = (
            self.get_parameter(
                'joint_1_target'
            )
            .get_parameter_value()
            .double_value
        )

        duration = (
            self.get_parameter(
                'duration_sec'
            )
            .get_parameter_value()
            .double_value
        )

        msg = JointCommand()

        msg.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.command_id = (
            f'test_{self.get_clock().now().nanoseconds}'
        )

        msg.joint_names = [
            'j2n6s300_joint_1',
            'j2n6s300_joint_2',
            'j2n6s300_joint_3',
            'j2n6s300_joint_4',
            'j2n6s300_joint_5',
            'j2n6s300_joint_6',
        ]

        msg.positions = [
            target_j1,
            math.pi,
            math.pi,
            0.0,
            0.0,
            0.0,
        ]

        duration_seconds = int(duration)

        duration_nanoseconds = int(
            (duration - duration_seconds)
            * 1e9
        )

        msg.duration.sec = duration_seconds
        msg.duration.nanosec = duration_nanoseconds

        self.publisher.publish(msg)

        self.get_logger().info(
            'Candidate command published: '
            f'id={msg.command_id}, '
            f'J1={target_j1:.3f} rad, '
            f'duration={duration:.2f} s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TestCommandNode()

    # Allow DDS discovery before publishing once.
    time.sleep(0.5)

    node.publish_test_command()

    rclpy.spin_once(
        node,
        timeout_sec=0.2
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
