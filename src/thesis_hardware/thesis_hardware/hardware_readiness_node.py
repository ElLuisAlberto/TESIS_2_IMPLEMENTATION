"""Publish readiness for the physical JACO preventive stack."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from thesis_interfaces.msg import ProximityStatus

from thesis_hardware.readiness_contract import (
    joint_state_is_valid,
    proximity_is_valid,
    readiness_reasons,
    sample_is_fresh,
)
from thesis_hardware.readiness_logging import log_readiness_transition
from thesis_core.ros_runtime import spin_node


class HardwareReadinessNode(Node):
    """Require connected hardware, fresh data and runtime arming."""

    def __init__(self):
        super().__init__('hardware_readiness')
        self.declare_parameter('timeout_sec', 0.50)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('require_armed', True)
        self.declare_parameter('require_proximity_status', True)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.require_armed = bool(
            self.get_parameter('require_armed').value
        )
        self.require_proximity = bool(
            self.get_parameter('require_proximity_status').value
        )
        if self.timeout_sec <= 0.0 or rate_hz <= 0.0:
            raise ValueError('readiness rate and timeout must be positive')

        self.connected = False
        self.armed = False
        self.last_joint_state = None
        self.last_proximity = None
        self.proximity_safe = False
        self.last_text = None

        self.create_subscription(
            Bool,
            '/thesis/hardware/connected',
            self.connected_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/thesis/hardware/armed',
            self.armed_callback,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_callback,
            10,
        )
        self.ready_publisher = self.create_publisher(
            Bool, '/thesis/system_ready', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/thesis/system_readiness', 10
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.evaluate)

    def connected_callback(self, message):
        """Store the adapter connection latch."""
        self.connected = bool(message.data)

    def armed_callback(self, message):
        """Store the runtime output latch."""
        self.armed = bool(message.data)

    def joint_state_callback(self, message):
        """Record only complete, finite and fresh physical feedback."""
        now_ros = self.get_clock().now().nanoseconds / 1e9
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        valid = joint_state_is_valid(
            message.name,
            message.position,
            stamp,
            now_ros,
            self.timeout_sec,
        )
        self.last_joint_state = time.monotonic() if valid else None

    def proximity_callback(self, message):
        """Record only fresh world-frame geometric evaluation."""
        now_ros = self.get_clock().now().nanoseconds / 1e9
        stamp = float(message.stamp.sec) + float(message.stamp.nanosec) * 1e-9
        valid = proximity_is_valid(
            message.state,
            message.reference_frame,
            float(message.minimum_clearance),
            float(message.obstacle_radius),
            stamp,
            now_ros,
            self.timeout_sec,
        )
        self.last_proximity = time.monotonic() if valid else None
        self.proximity_safe = valid and message.state != 'STOP'

    def evaluate(self):
        """Publish one fail-closed readiness decision."""
        now = time.monotonic()
        reasons = readiness_reasons(
            self.connected,
            self.armed,
            self.require_armed,
            sample_is_fresh(
                self.last_joint_state, now, self.timeout_sec
            ),
            sample_is_fresh(
                self.last_proximity, now, self.timeout_sec
            ),
            self.require_proximity,
            self.proximity_safe,
        )
        ready = not reasons
        text = 'READY' if ready else 'NOT_READY: ' + '; '.join(reasons)
        ready_message = Bool()
        ready_message.data = ready
        self.ready_publisher.publish(ready_message)
        status_message = String()
        status_message.data = text
        self.status_publisher.publish(status_message)
        if text != self.last_text:
            log_readiness_transition(self.get_logger(), ready, text)
            self.last_text = text


def main(args=None):
    rclpy.init(args=args)
    spin_node(HardwareReadinessNode())


if __name__ == '__main__':
    main()
