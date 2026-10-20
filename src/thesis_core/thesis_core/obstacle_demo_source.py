"""Publish a known test obstacle using the perception input contract."""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from thesis_interfaces.msg import Obstacle

from thesis_core.ros_runtime import spin_node
from thesis_core.virtual_obstacle_scenarios import (
    ScenarioPoses,
    select_scenario_center,
)


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
        self.declare_parameter('scenario', 'FIXED')
        self.declare_parameter('insert_stop_active', False)
        self.declare_parameter('park_pose', [2.0, 0.0, 0.65])
        self.declare_parameter('block_start_pose', [2.0, 0.0, 0.65])
        self.declare_parameter('insert_stop_pose', [2.0, 0.0, 0.65])
        topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.radius = float(self.get_parameter('radius').value)
        self.uncertainty = float(
            self.get_parameter('uncertainty').value
        )
        rate = float(self.get_parameter('rate_hz').value)
        if (not self.frame_id or self.radius <= 0.0 or
                self.uncertainty < 0.0 or rate <= 0.0):
            raise ValueError('invalid demo obstacle parameters')
        self._validate_scenario_parameters({})
        self.add_on_set_parameters_callback(self._parameter_update)
        self.last_scenario_state = None
        self.publisher = self.create_publisher(Obstacle, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_obstacle)
        self.get_logger().info(
            f'Publishing test obstacle in {self.frame_id} on {topic}'
        )

    def _parameter_values(self, updates):
        names = (
            'scenario', 'insert_stop_active', 'park_pose',
            'block_start_pose', 'insert_stop_pose', 'x', 'y', 'z',
        )
        values = {name: self.get_parameter(name).value for name in names}
        values.update(updates)
        return values

    def _validate_scenario_parameters(self, updates):
        values = self._parameter_values(updates)
        scenario = str(values['scenario']).strip().upper()
        poses = ScenarioPoses(
            tuple(values['park_pose']), tuple(values['block_start_pose']),
            tuple(values['insert_stop_pose']),
        )
        fixed = (values['x'], values['y'], values['z'])
        select_scenario_center(
            scenario, bool(values['insert_stop_active']), fixed, poses
        )
        return scenario

    def _parameter_update(self, parameters):
        updates = {parameter.name: parameter.value
                   for parameter in parameters}
        try:
            self._validate_scenario_parameters(updates)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def scenario_center(self):
        values = self._parameter_values({})
        poses = ScenarioPoses(
            tuple(values['park_pose']), tuple(values['block_start_pose']),
            tuple(values['insert_stop_pose']),
        )
        scenario = str(values['scenario']).strip().upper()
        inserted = bool(values['insert_stop_active'])
        center = select_scenario_center(
            scenario, inserted,
            (values['x'], values['y'], values['z']), poses,
        )
        state = (scenario, inserted, center)
        if state != self.last_scenario_state:
            self.get_logger().info(
                f'Obstacle scenario={scenario}, inserted={inserted}, '
                f'center={center}'
            )
            self.last_scenario_state = state
        return center

    def publish_obstacle(self):
        message = Obstacle()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.center = self.scenario_center()
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
