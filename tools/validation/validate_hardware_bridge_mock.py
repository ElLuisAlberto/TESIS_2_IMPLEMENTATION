#!/usr/bin/env python3
"""Exercise the physical adapter contract against its mock backend."""

import math
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from thesis_interfaces.msg import ExecutionTrajectory, JointCommand


JOINT_NAMES = tuple(
    f'j2n6s300_joint_{index}' for index in range(1, 7)
)


class MockBridgeValidator(Node):
    """Arm, command and verify the mock JACO through public interfaces."""

    def __init__(self):
        super().__init__('mock_hardware_bridge_validator')
        self.positions = None
        self.ready = False
        self.statuses = {}
        self.create_subscription(
            JointState, '/joint_states', self.state_callback, 20
        )
        self.create_subscription(
            Bool, '/thesis/system_ready', self.ready_callback, 10
        )
        self.create_subscription(
            ExecutionTrajectory,
            '/thesis/execution_trajectory',
            self.execution_callback,
            20,
        )
        self.candidate_publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10
        )
        self.arm_client = self.create_client(
            SetBool, '/thesis/hardware/set_armed'
        )

    def state_callback(self, message):
        """Store the canonical six-joint state by name."""
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            selected = tuple(float(values[name]) for name in JOINT_NAMES)
            if all(math.isfinite(value) for value in selected):
                self.positions = selected

    def ready_callback(self, message):
        """Store stack readiness."""
        self.ready = bool(message.data)

    def execution_callback(self, message):
        """Store the execution lifecycle for each command."""
        self.statuses.setdefault(message.command_id, []).append(message.status)

    def wait_for(self, predicate, timeout_sec, description):
        """Spin until a predicate is true or raise."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'timeout waiting for {description}')

    def set_armed(self, value):
        """Call the runtime output latch."""
        self.wait_for(
            self.arm_client.service_is_ready,
            8.0,
            'arming service',
        )
        request = SetBool.Request()
        request.data = value
        future = self.arm_client.call_async(request)
        self.wait_for(future.done, 5.0, 'arming response')
        response = future.result()
        if response is None or not response.success:
            detail = 'no response' if response is None else response.message
            raise RuntimeError(f'arming request failed: {detail}')
        return response.message

    def run(self):
        """Execute one supervised point movement and disarm."""
        self.wait_for(
            lambda: self.positions is not None,
            8.0,
            'mock joint state',
        )
        initial = tuple(self.positions)
        arm_detail = self.set_armed(True)
        self.wait_for(lambda: self.ready, 8.0, 'physical-stack readiness')

        command_id = f'MOCK_BRIDGE_{uuid.uuid4().hex[:8]}'
        target = list(initial)
        target[5] += math.radians(2.0)
        command = JointCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command_id = command_id
        command.joint_names = list(JOINT_NAMES)
        command.positions = target
        command.duration.sec = 4
        command.duration.nanosec = 0
        self.candidate_publisher.publish(command)

        self.wait_for(
            lambda: 'ACCEPTED' in self.statuses.get(command_id, []),
            5.0,
            'physical ACCEPTED status',
        )
        self.wait_for(
            lambda: 'SUCCEEDED' in self.statuses.get(command_id, []),
            10.0,
            'physical SUCCEEDED status',
        )
        final = tuple(self.positions)
        moved = abs(final[5] - initial[5])
        error = abs(final[5] - target[5])
        if moved < math.radians(1.4):
            raise RuntimeError(
                f'mock joint moved only {math.degrees(moved):.3f} deg'
            )
        if error > 0.01:
            raise RuntimeError(
                f'final target error is {error:.6f} rad'
            )

        disarm_detail = self.set_armed(False)
        print('MOCK_BRIDGE_VALIDATION=PASS')
        print(f'ARM={arm_detail}')
        print(f'STATUSES={"|".join(self.statuses[command_id])}')
        print(f'J6_MOVEMENT_DEG={math.degrees(moved):.6f}')
        print(f'J6_FINAL_ERROR_RAD={error:.6f}')
        print(f'DISARM={disarm_detail}')


def main():
    rclpy.init()
    node = MockBridgeValidator()
    result = 0
    try:
        node.run()
    except Exception as exception:
        print(f'MOCK_BRIDGE_VALIDATION=FAIL: {exception}', file=sys.stderr)
        result = 1
        try:
            node.set_armed(False)
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
