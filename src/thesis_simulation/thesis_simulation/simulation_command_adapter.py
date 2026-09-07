import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from thesis_interfaces.msg import JointCommand


class SimulationCommandAdapter(Node):

    def __init__(self):
        super().__init__('simulation_command_adapter')

        self.declare_parameter(
            'simulation_output_enabled',
            False,
        )

        self.declare_parameter(
            'action_name',
            '/arm_controller/follow_joint_trajectory',
        )

        action_name = self.get_parameter(
            'action_name'
        ).value

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name,
        )

        self.subscription = self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self.command_callback,
            10,
        )

        self.goal_active = False

        self.get_logger().info(
            'Adaptador Gazebo preparado; '
            'salida simulada desactivada por defecto'
        )

    def command_callback(self, msg):
        output_enabled = bool(
            self.get_parameter(
                'simulation_output_enabled'
            ).value
        )

        if not output_enabled:
            self.get_logger().info(
                f'SIMULATION-DRY-RUN id={msg.command_id}: '
                'salida a Gazebo desactivada'
            )
            return

        if self.goal_active:
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'ya existe una trayectoria activa en Gazebo'
            )
            return

        if len(msg.joint_names) != len(msg.positions):
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'joint_names y positions no coinciden'
            )
            return

        if not self.action_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.get_logger().error(
                'No se encontró el action server de Gazebo: '
                '/arm_controller/follow_joint_trajectory'
            )
            return

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = list(
            msg.joint_names
        )

        point = JointTrajectoryPoint()
        point.positions = list(msg.positions)
        point.time_from_start.sec = int(
            msg.duration.sec
        )
        point.time_from_start.nanosec = int(
            msg.duration.nanosec
        )

        goal.trajectory.points = [point]

        self.goal_active = True

        future = self.action_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

        self.get_logger().info(
            f'GAZEBO goal enviado: id={msg.command_id}, '
            f'joints={len(msg.joint_names)}'
        )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.goal_active = False
            self.get_logger().error(
                f'Error enviando goal a Gazebo: {exc}'
            )
            return

        if not goal_handle.accepted:
            self.goal_active = False
            self.get_logger().warning(
                'Gazebo rechazó la trayectoria'
            )
            return

        self.get_logger().info(
            'Gazebo aceptó la trayectoria'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):
        self.goal_active = False

        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(
                f'Error obteniendo resultado de Gazebo: {exc}'
            )
            return

        self.get_logger().info(
            f'Gazebo terminó la trayectoria: '
            f'error_code={result.error_code}, '
            f'{result.error_string}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SimulationCommandAdapter()

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
