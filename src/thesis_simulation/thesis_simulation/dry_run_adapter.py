import rclpy
from rclpy.node import Node

from thesis_interfaces.msg import JointCommand


class DryRunAdapter(Node):

    def __init__(self):
        super().__init__('dry_run_command_adapter')

        self.declare_parameter(
            'physical_output_enabled',
            False
        )

        self.subscription = self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.command_callback,
            10
        )

        self.get_logger().info(
            'Adaptador DRY-RUN activo: no se enviaran comandos fisicos'
        )

    def command_callback(self, msg):

        duration_sec = (
            float(msg.duration.sec)
            + float(msg.duration.nanosec) * 1e-9
        )

        physical_output_enabled = (
            self.get_parameter(
                'physical_output_enabled'
            ).get_parameter_value().bool_value
        )

        if physical_output_enabled:
            self.get_logger().error(
                f'RECHAZADO id={msg.command_id}: '
                'el adaptador fisico aun no esta implementado'
            )
            return

        positions = ', '.join(
            f'{value:.3f}'
            for value in msg.positions
        )

        self.get_logger().info(
            f'DRY-RUN id={msg.command_id} | '
            f'joints={len(msg.joint_names)} | '
            f'positions=[{positions}] | '
            f'duration={duration_sec:.3f} s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DryRunAdapter()

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
