"""Convert continuous Cartesian velocity intentions into safe joint JOG."""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from thesis_core.cartesian_kinematics import (
    align_relative_target,
    solve_inverse_kinematics_transform,
    transform_from_translation_quaternion,
)
from thesis_core.jaco_kinematics import end_effector_transform
from thesis_core.joint_model import (
    JOINT_NAMES,
    saturate_target_by_velocity,
    shortest_joint_delta,
)
from thesis_core.ros_runtime import spin_node
from thesis_interfaces.msg import JointCommand
import tf2_ros

from thesis_simulation.cartesian_jog_model import (
    bounded_integration_step,
    integrate_world_twist,
    validate_world_twist,
)


class CartesianJogAdapter(Node):
    """Keep robot-specific IK outside the generic operator interface."""

    def __init__(self):
        super().__init__('cartesian_jog_adapter')
        self.declare_parameter('maximum_state_age_sec', 0.50)
        self.declare_parameter('horizon_sec', 1.0)
        self.declare_parameter('integration_step_sec', 0.10)
        self.declare_parameter('maximum_linear_ik_step_m', 0.005)
        self.declare_parameter(
            'maximum_angular_ik_step_rad', math.radians(1.0)
        )
        self.declare_parameter('maximum_linear_speed_mps', 0.15)
        self.declare_parameter(
            'maximum_angular_speed_radps', math.radians(30.0)
        )
        self.declare_parameter('position_tolerance_m', 0.0002)
        self.declare_parameter(
            'orientation_tolerance_rad', math.radians(0.1)
        )
        self.maximum_state_age_sec = float(
            self.get_parameter('maximum_state_age_sec').value
        )
        self.horizon_sec = float(self.get_parameter('horizon_sec').value)
        self.integration_step_sec = float(
            self.get_parameter('integration_step_sec').value
        )
        self.maximum_linear_ik_step_m = float(
            self.get_parameter('maximum_linear_ik_step_m').value
        )
        self.maximum_angular_ik_step_rad = float(
            self.get_parameter('maximum_angular_ik_step_rad').value
        )
        self.maximum_linear_speed_mps = float(
            self.get_parameter('maximum_linear_speed_mps').value
        )
        self.maximum_angular_speed_radps = float(
            self.get_parameter('maximum_angular_speed_radps').value
        )
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.orientation_tolerance_rad = float(
            self.get_parameter('orientation_tolerance_rad').value
        )
        parameters = (
            self.maximum_state_age_sec,
            self.horizon_sec,
            self.integration_step_sec,
            self.maximum_linear_ik_step_m,
            self.maximum_angular_ik_step_rad,
            self.maximum_linear_speed_mps,
            self.maximum_angular_speed_radps,
            self.position_tolerance_m,
            self.orientation_tolerance_rad,
        )
        if not all(
            math.isfinite(value) and value > 0.0 for value in parameters
        ):
            raise ValueError('Cartesian JOG parameters must be positive')
        if not 0.1 <= self.horizon_sec <= 2.0:
            raise ValueError('Cartesian JOG horizon must belong to [0.1, 2.0]')
        if self.integration_step_sec > self.horizon_sec:
            raise ValueError('Cartesian JOG integration step exceeds horizon')

        self.positions = {}
        self.state_received_at = None
        self.last_warning_at = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        rolling_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', rolling_qos
        )
        self.create_subscription(
            JointState, '/joint_states', self.receive_state, 20
        )
        self.create_subscription(
            TwistStamped,
            '/thesis/cartesian_jog_intent',
            self.receive_twist,
            rolling_qos,
        )
        self.get_logger().info(
            'Cartesian JOG adapter ready: simulation only, '
            f'horizon={self.horizon_sec:.2f} s, '
            f'linear limit={self.maximum_linear_speed_mps:.3f} m/s, '
            'angular limit='
            f'{math.degrees(self.maximum_angular_speed_radps):.1f} deg/s'
        )

    def receive_state(self, message):
        positions = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if name in JOINT_NAMES and math.isfinite(value)
        }
        if all(name in positions for name in JOINT_NAMES):
            self.positions = positions
            self.state_received_at = time.monotonic()

    def current_joint_vector(self):
        if self.state_received_at is None:
            return None
        if time.monotonic() - self.state_received_at > self.maximum_state_age_sec:
            return None
        return tuple(self.positions[name] for name in JOINT_NAMES)

    def current_world_transform(self):
        message = self.tf_buffer.lookup_transform(
            'world', 'j2n6s300_end_effector', Time()
        )
        transform = message.transform
        return transform_from_translation_quaternion(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def warn_limited(self, detail):
        now = time.monotonic()
        if self.last_warning_at is None or now - self.last_warning_at >= 1.0:
            self.get_logger().warning(f'CARTESIAN JOG HOLD: {detail}')
            self.last_warning_at = now

    def publish_target(self, source_stamp, positions):
        message = JointCommand()
        message.stamp = source_stamp
        message.command_id = 'cartesian_jog_continuous'
        message.joint_names = list(JOINT_NAMES)
        message.positions = list(positions)
        whole_seconds = int(self.horizon_sec)
        message.duration.sec = whole_seconds
        message.duration.nanosec = int(round(
            (self.horizon_sec - whole_seconds) * 1.0e9
        ))
        self.command_publisher.publish(message)

    def receive_twist(self, message):
        if message.header.frame_id != 'world':
            self.warn_limited('frame_id debe ser world.')
            return
        current = self.current_joint_vector()
        if current is None:
            self.warn_limited('estado articular ausente u obsoleto.')
            return
        linear_values = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
        )
        angular_values = (
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        )
        try:
            linear, angular = validate_world_twist(
                linear_values,
                angular_values,
                self.maximum_linear_speed_mps,
                self.maximum_angular_speed_radps,
            )
        except ValueError as exception:
            self.warn_limited(str(exception))
            self.publish_target(message.header.stamp, current)
            return

        if max(abs(value) for value in linear + angular) <= 1.0e-12:
            self.publish_target(message.header.stamp, current)
            return
        try:
            current_world = self.current_world_transform()
            integration_step = bounded_integration_step(
                linear,
                angular,
                self.integration_step_sec,
                self.maximum_linear_ik_step_m,
                self.maximum_angular_ik_step_rad,
            )
            target_world = integrate_world_twist(
                current_world,
                linear,
                angular,
                integration_step,
            )
            current_model = end_effector_transform(current)
            model_target = align_relative_target(
                current_world, target_world, current_model
            )
            result = solve_inverse_kinematics_transform(
                current,
                model_target,
                position_tolerance_m=self.position_tolerance_m,
                orientation_tolerance_rad=self.orientation_tolerance_rad,
            )
        except (ValueError, tf2_ros.TransformException) as exception:
            self.warn_limited(str(exception))
            self.publish_target(message.header.stamp, current)
            return
        if not result.success:
            self.warn_limited(
                'IK local sin solución; '
                f'error={result.position_error_m:.4f} m, '
                f'{math.degrees(result.orientation_error_rad):.2f} deg.'
            )
            self.publish_target(message.header.stamp, current)
            return
        local_velocities = tuple(
            shortest_joint_delta(name, target, start)
            / integration_step
            for name, start, target in zip(
                JOINT_NAMES, current, result.positions
            )
        )
        horizon_target = tuple(
            start + velocity * self.horizon_sec
            for start, velocity in zip(current, local_velocities)
        )
        try:
            saturation = saturate_target_by_velocity(
                current, horizon_target, self.horizon_sec
            )
        except ValueError as exception:
            self.warn_limited(str(exception))
            self.publish_target(message.header.stamp, current)
            return
        self.publish_target(message.header.stamp, saturation.positions)


def main(args=None):
    """Run the simulation-only continuous Cartesian adapter."""
    rclpy.init(args=args)
    spin_node(CartesianJogAdapter())


if __name__ == '__main__':
    main()
