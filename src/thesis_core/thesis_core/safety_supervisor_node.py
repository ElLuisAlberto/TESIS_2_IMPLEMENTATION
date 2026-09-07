import math

import rclpy
from rclpy.node import Node

from thesis_interfaces.msg import JointCommand


EXPECTED_ARM_JOINTS = [
    'j2n6s300_joint_1',
    'j2n6s300_joint_2',
    'j2n6s300_joint_3',
    'j2n6s300_joint_4',
    'j2n6s300_joint_5',
    'j2n6s300_joint_6',
]

JOINT_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[1]: (
        47.0 * math.pi / 180.0,
        313.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[2]: (
        19.0 * math.pi / 180.0,
        341.0 * math.pi / 180.0,
    ),
    EXPECTED_ARM_JOINTS[3]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[4]: (-2.0 * math.pi, 2.0 * math.pi),
    EXPECTED_ARM_JOINTS[5]: (-2.0 * math.pi, 2.0 * math.pi),
}

MIN_DURATION_SEC = 0.1
MAX_DURATION_SEC = 30.0


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.candidate_subscription = self.create_subscription(
            JointCommand,
            '/thesis/candidate_command',
            self.candidate_command_callback,
            10,
        )

        self.supervised_publisher = self.create_publisher(
            JointCommand,
            '/thesis/supervised_command',
            10,
        )

        self.get_logger().info(
            'Safety supervisor started in validation/pass-through mode'
        )

    def candidate_command_callback(self, msg):
        receive_time = self.get_clock().now()

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

        if list(msg.joint_names) != EXPECTED_ARM_JOINTS:
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'expected six arm joints in canonical order'
            )
            return

        for joint_name, position in zip(
            msg.joint_names,
            msg.positions,
        ):
            lower, upper = JOINT_LIMITS[joint_name]

            if not lower <= position <= upper:
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
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                f'duration must be between '
                f'{MIN_DURATION_SEC:.1f} and '
                f'{MAX_DURATION_SEC:.1f} seconds'
            )
            return

        candidate_stamp_ns = (
            int(msg.stamp.sec) * 1_000_000_000
            + int(msg.stamp.nanosec)
        )

        receive_ns = receive_time.nanoseconds
        input_latency_ms = (
            receive_ns - candidate_stamp_ns
        ) / 1e6

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
