"""Report when the preventive simulation stack is ready to accept commands."""

import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from thesis_interfaces.msg import Obstacle, ProximityStatus
from tf2_msgs.msg import TFMessage

from thesis_simulation.readiness_contract import sample_is_fresh


class SystemReadinessNode(Node):
    """Check graph connections, fresh data and active Gazebo control."""

    def __init__(self):
        super().__init__('system_readiness')
        self.declare_parameter('obstacle_source_mode', 'topic')
        self.declare_parameter('use_demo_obstacle', True)
        self.declare_parameter('require_gui', True)
        self.declare_parameter('readiness_timeout_sec', 0.75)
        self.declare_parameter('readiness_rate_hz', 1.0)
        self.declare_parameter(
            'controller_manager_service',
            '/controller_manager/list_controllers',
        )
        self.declare_parameter(
            'controller_action',
            '/arm_controller/follow_joint_trajectory',
        )

        self.obstacle_source_mode = str(
            self.get_parameter('obstacle_source_mode').value
        ).lower()
        self.use_demo_obstacle = bool(
            self.get_parameter('use_demo_obstacle').value
        )
        self.require_gui = bool(self.get_parameter('require_gui').value)
        self.timeout_sec = float(
            self.get_parameter('readiness_timeout_sec').value
        )
        rate_hz = float(
            self.get_parameter('readiness_rate_hz').value
        )
        if self.obstacle_source_mode not in ('topic', 'gazebo'):
            raise ValueError(
                'obstacle_source_mode must be topic or gazebo'
            )
        if (not math.isfinite(self.timeout_sec)
                or not math.isfinite(rate_hz)
                or self.timeout_sec <= 0.0 or rate_hz <= 0.0):
            raise ValueError('readiness timeouts and rate must be positive')

        self.last_received = {
            '/clock': None,
            '/joint_states': None,
            '/thesis/proximity_status': None,
        }
        self.last_received['/thesis/obstacle_source'] = None
        self.last_status = None
        self.controller_active = False
        self.controller_error = 'esperando controller_manager'
        self.controller_future = None

        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Clock, '/clock', self.clock_callback, clock_qos
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_callback,
            10,
        )
        if self.obstacle_source_mode == 'topic':
            self.create_subscription(
                Obstacle,
                '/thesis/obstacle_in_model_frame',
                self.obstacle_callback,
                10,
            )
        else:
            self.create_subscription(
                TFMessage,
                '/world/jaco_world/pose/info',
                self.obstacle_pose_callback,
                10,
            )

        self.ready_publisher = self.create_publisher(
            Bool, '/thesis/system_ready', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/thesis/system_readiness', 10
        )
        service_name = str(
            self.get_parameter('controller_manager_service').value
        )
        action_name = str(
            self.get_parameter('controller_action').value
        )
        self.controller_client = self.create_client(
            ListControllers, service_name
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory, action_name
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.evaluate)
        self.get_logger().info(
            'System readiness monitor started; waiting for the full stack'
        )

    def _mark_received(self, key):
        self.last_received[key] = time.monotonic()

    def clock_callback(self, _msg):
        """Record receipt of a live simulation clock sample."""
        self._mark_received('/clock')

    def joint_state_callback(self, _msg):
        """Record receipt of a live robot state sample."""
        self._mark_received('/joint_states')

    def proximity_callback(self, _msg):
        """Record receipt of a live proximity evaluation."""
        self._mark_received('/thesis/proximity_status')

    def obstacle_callback(self, _msg):
        """Record receipt of a transformed perception input."""
        self._mark_received('/thesis/obstacle_source')

    def obstacle_pose_callback(self, _msg):
        """Record receipt of the Gazebo obstacle pose bridge."""
        self._mark_received('/thesis/obstacle_source')

    def _update_controller_state(self):
        if self.controller_future is not None:
            if not self.controller_future.done():
                return
            try:
                response = self.controller_future.result()
                self.controller_active = any(
                    item.name == 'arm_controller'
                    and item.state.lower() == 'active'
                    for item in response.controller
                )
                self.controller_error = (
                    '' if self.controller_active
                    else 'arm_controller no está active'
                )
            except Exception as exc:  # ROS service errors are runtime data.
                self.controller_active = False
                self.controller_error = f'falló list_controllers: {exc}'
            self.controller_future = None

        if (self.controller_future is None
                and self.controller_client.service_is_ready()):
            self.controller_future = self.controller_client.call_async(
                ListControllers.Request()
            )

    def _node_graph_status(self):
        expected = {
            'gazebo_clock_bridge',
            'robot_state_publisher',
            'joint_state_broadcaster',
            'proximity_monitor',
            'safety_supervisor_node',
            'simulation_command_adapter',
            'horizon_preview',
            'system_readiness',
        }
        if self.obstacle_source_mode == 'topic':
            expected.add('obstacle_input_bridge')
            if self.use_demo_obstacle:
                expected.add('obstacle_demo_source')
        if self.require_gui:
            expected.add('joint_control_gui_node')

        counts = {}
        for name, _namespace in self.get_node_names_and_namespaces():
            counts[name] = counts.get(name, 0) + 1

        missing = [
            f'nodo ausente: /{name}'
            for name in sorted(expected)
            if counts.get(name, 0) == 0
        ]
        duplicates = [
            f'nodo duplicado: /{name} ({counts[name]} instancias)'
            for name in sorted(expected)
            if counts.get(name, 0) > 1
        ]
        return missing + duplicates

    def _freshness_status(self, now):
        required = [
            ('/clock', 'reloj de simulación'),
            ('/joint_states', 'estado articular'),
            ('/thesis/proximity_status', 'cálculo de proximidad'),
            ('/thesis/obstacle_source', 'fuente de obstáculo'),
        ]
        missing = []
        for key, label in required:
            if not sample_is_fresh(
                    self.last_received[key], now, self.timeout_sec):
                missing.append(f'sin dato reciente: {label}')
        return missing

    def _connection_status(self):
        expected_subscribers = {
            '/thesis/candidate_command': 'supervisor',
            '/thesis/jog_intent': 'supervisor',
            '/thesis/proximity_status': 'supervisor',
            '/thesis/supervised_command': 'adaptador',
            '/thesis/supervised_jog_command': 'adaptador',
        }
        missing = []
        for topic, consumer in expected_subscribers.items():
            if self.count_subscribers(topic) == 0:
                missing.append(
                    f'{consumer} no está conectado a {topic}'
                )
        return missing

    def evaluate(self):
        """Publish READY or a concise list of unavailable components."""
        self._update_controller_state()
        missing = self._node_graph_status()
        missing.extend(self._freshness_status(time.monotonic()))
        missing.extend(self._connection_status())
        if not self.controller_active:
            missing.append(self.controller_error)
        if not self.action_client.server_is_ready():
            missing.append(
                'acción /arm_controller/follow_joint_trajectory no disponible'
            )

        ready = not missing
        status = 'READY' if ready else 'WAITING: ' + '; '.join(missing)
        ready_message = Bool()
        ready_message.data = ready
        status_message = String()
        status_message.data = status
        self.ready_publisher.publish(ready_message)
        self.status_publisher.publish(status_message)

        if status != self.last_status:
            if ready:
                self.get_logger().info(
                    'SYSTEM READY: all required links are live'
                )
            else:
                self.get_logger().warning(status)
            self.last_status = status


def main(args=None):
    """Run the readiness monitor using a wall-clock timer."""
    rclpy.init(args=args)
    node = SystemReadinessNode()
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
