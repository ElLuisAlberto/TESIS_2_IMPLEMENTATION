#!/usr/bin/env python3
"""Validate a bounded J6 round trip through the complete physical stack."""

import math
import sys
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from thesis_interfaces.msg import (
    ExecutionControl,
    ExecutionTrajectory,
    JointCommand,
)


JOINT_NAMES = tuple(
    f'j2n6s300_joint_{index}' for index in range(1, 7)
)
TERMINAL_STATUSES = {
    'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'DRY_RUN',
}


def angular_error(target, actual, continuous):
    """Return one absolute joint error, wrapping continuous axes."""
    delta = float(target) - float(actual)
    if continuous:
        delta = math.atan2(math.sin(delta), math.cos(delta))
    return abs(delta)


class PhysicalMicroMotionValidator(Node):
    """Arm, move J6 by two degrees, return and disarm."""

    def __init__(self):
        super().__init__('kinova_physical_micro_motion_validator')
        self.positions = None
        self.position_samples = []
        self.armed = False
        self.ready = False
        self.statuses = {}
        self.controls = {}
        self.diagnostic_values = {}
        self.diagnostic_message = ''

        self.create_subscription(
            JointState, '/joint_states', self.state_callback, 50
        )
        self.create_subscription(
            Bool, '/thesis/hardware/armed', self.armed_callback, 10
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
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self.control_callback,
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            '/thesis/hardware/diagnostics',
            self.diagnostic_callback,
            10,
        )
        self.candidate_publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10
        )
        self.arm_client = self.create_client(
            SetBool, '/thesis/hardware/set_armed'
        )

    def state_callback(self, message):
        """Record finite arm positions in canonical order."""
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        positions = tuple(float(values[name]) for name in JOINT_NAMES)
        if not all(math.isfinite(value) for value in positions):
            return
        self.positions = positions
        self.position_samples.append((time.monotonic(), positions))
        if len(self.position_samples) > 4000:
            del self.position_samples[:1000]

    def armed_callback(self, message):
        """Store the runtime output latch."""
        self.armed = bool(message.data)

    def ready_callback(self, message):
        """Store complete-stack readiness."""
        self.ready = bool(message.data)

    def execution_callback(self, message):
        """Record the lifecycle of each physical reference."""
        self.statuses.setdefault(message.command_id, []).append(
            message.status
        )

    def control_callback(self, message):
        """Record correlated runtime supervisor decisions."""
        self.controls.setdefault(message.command_id, []).append(
            message.state
        )

    def diagnostic_callback(self, message):
        """Store the JACO adapter diagnostic snapshot."""
        for status in message.status:
            if status.name != 'thesis_hardware_bridge/JACO2':
                continue
            self.diagnostic_message = status.message
            self.diagnostic_values = {
                item.key: item.value for item in status.values
            }

    def wait_for(self, predicate, timeout_sec, description):
        """Spin until a predicate succeeds or raise a named timeout."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'timeout waiting for {description}')

    def set_armed(self, value):
        """Set the physical output latch and verify the published state."""
        self.wait_for(
            self.arm_client.service_is_ready, 8.0, 'arming service'
        )
        request = SetBool.Request()
        request.data = bool(value)
        future = self.arm_client.call_async(request)
        self.wait_for(future.done, 5.0, 'arming response')
        response = future.result()
        if response is None or not response.success:
            detail = 'no response' if response is None else response.message
            raise RuntimeError(f'arming request failed: {detail}')
        self.wait_for(
            lambda: self.armed == bool(value),
            3.0,
            f'armed={str(value).lower()} latch',
        )
        return response.message

    def publish_candidate(self, target, duration_sec, label):
        """Publish one six-joint candidate after discovery."""
        self.wait_for(
            lambda: self.candidate_publisher.get_subscription_count() > 0,
            8.0,
            'safety-supervisor subscription',
        )
        command_id = f'PHYSICAL_{label}_{uuid.uuid4().hex[:8]}'
        message = JointCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.command_id = command_id
        message.joint_names = list(JOINT_NAMES)
        message.positions = list(target)
        whole = int(duration_sec)
        message.duration.sec = whole
        message.duration.nanosec = int(
            round((duration_sec - whole) * 1_000_000_000)
        )
        self.candidate_publisher.publish(message)
        return command_id

    def wait_for_success(self, command_id, timeout_sec):
        """Require ACCEPTED, runtime control and terminal SUCCEEDED."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            statuses = self.statuses.get(command_id, [])
            terminal = next(
                (item for item in reversed(statuses)
                 if item in TERMINAL_STATUSES),
                None,
            )
            if terminal is None:
                continue
            if terminal != 'SUCCEEDED':
                raise RuntimeError(
                    f'{command_id} terminated as {terminal}: {statuses}'
                )
            if 'ACCEPTED' not in statuses:
                raise RuntimeError(
                    f'{command_id} succeeded without ACCEPTED trace'
                )
            decisions = self.controls.get(command_id, [])
            if not decisions:
                raise RuntimeError(
                    f'{command_id} has no correlated runtime control'
                )
            if 'STOP' in decisions:
                raise RuntimeError(
                    f'{command_id} received STOP during clear-space test'
                )
            return statuses, decisions
        raise RuntimeError(
            f'timeout waiting for {command_id}; '
            f'statuses={self.statuses.get(command_id, [])}'
        )

    def run(self):
        """Execute the two-degree J6 round trip and enforce bounds."""
        self.wait_for(
            lambda: self.positions is not None,
            10.0,
            'physical joint state',
        )
        self.wait_for(
            lambda: self.diagnostic_values.get('connected') == 'true',
            5.0,
            'connected diagnostic',
        )
        initial = tuple(self.positions)
        direction = 1.0
        delta = math.radians(2.0)
        if initial[5] + delta > 2.0 * math.pi:
            direction = -1.0
        outward_target = list(initial)
        outward_target[5] += direction * delta

        arm_detail = self.set_armed(True)
        self.wait_for(lambda: self.ready, 8.0, 'physical-stack READY')
        self.wait_for(
            lambda: self.diagnostic_values.get('armed') == 'true',
            3.0,
            'armed diagnostic',
        )
        movement_started = time.monotonic()

        outward_id = self.publish_candidate(
            outward_target, 4.0, 'J6_OUT'
        )
        outward_statuses, outward_controls = self.wait_for_success(
            outward_id, 12.0
        )
        outward_final = tuple(self.positions)
        outward_motion = angular_error(
            outward_final[5], initial[5], True
        )
        outward_error = angular_error(
            outward_target[5], outward_final[5], True
        )
        if outward_motion < math.radians(1.4):
            raise RuntimeError(
                'J6 moved only '
                f'{math.degrees(outward_motion):.3f} deg'
            )
        if outward_motion > math.radians(2.5):
            raise RuntimeError(
                'J6 exceeded the bounded movement: '
                f'{math.degrees(outward_motion):.3f} deg'
            )
        if outward_error > math.radians(0.6):
            raise RuntimeError(
                'J6 outward target error is '
                f'{math.degrees(outward_error):.3f} deg'
            )

        return_id = self.publish_candidate(initial, 4.0, 'J6_RETURN')
        return_statuses, return_controls = self.wait_for_success(
            return_id, 12.0
        )
        final = tuple(self.positions)
        final_errors = tuple(
            angular_error(expected, actual, index in (0, 3, 4, 5))
            for index, (expected, actual) in enumerate(zip(initial, final))
        )
        if max(final_errors) > math.radians(0.6):
            raise RuntimeError(
                'round-trip maximum error is '
                f'{math.degrees(max(final_errors)):.3f} deg'
            )
        movement_samples = [
            positions for stamp, positions in self.position_samples
            if stamp >= movement_started
        ]
        other_axis_motion = max(
            angular_error(
                initial[index],
                positions[index],
                index in (0, 3, 4),
            )
            for positions in movement_samples
            for index in range(5)
        )
        if other_axis_motion > math.radians(0.3):
            raise RuntimeError(
                'an axis other than J6 moved by '
                f'{math.degrees(other_axis_motion):.3f} deg'
            )
        if self.diagnostic_values.get('fault') != 'false':
            raise RuntimeError(
                'adapter diagnostic reports a fault: '
                f'{self.diagnostic_message}'
            )

        disarm_detail = self.set_armed(False)
        print('PHYSICAL_MICRO_MOTION=PASS')
        print(f'ARM={arm_detail}')
        print(f'OUTWARD_ID={outward_id}')
        print(f'OUTWARD_STATUSES={"|".join(outward_statuses)}')
        print(f'OUTWARD_CONTROL={"|".join(outward_controls)}')
        print(f'J6_MOVEMENT_DEG={math.degrees(outward_motion):.6f}')
        print(f'J6_OUTWARD_ERROR_DEG={math.degrees(outward_error):.6f}')
        print(f'RETURN_ID={return_id}')
        print(f'RETURN_STATUSES={"|".join(return_statuses)}')
        print(f'RETURN_CONTROL={"|".join(return_controls)}')
        print(
            'ROUND_TRIP_MAX_ERROR_DEG='
            f'{math.degrees(max(final_errors)):.6f}'
        )
        print(
            'OTHER_AXIS_MAX_DELTA_DEG='
            f'{math.degrees(other_axis_motion):.6f}'
        )
        print(f'DISARM={disarm_detail}')


def main():
    """Run validation and make a best-effort disarm on every failure."""
    rclpy.init()
    node = PhysicalMicroMotionValidator()
    result = 0
    try:
        node.run()
    except Exception as exception:
        print(
            f'PHYSICAL_MICRO_MOTION=FAIL: {exception}',
            file=sys.stderr,
        )
        result = 1
        try:
            node.set_armed(False)
        except Exception as disarm_exception:
            print(
                f'EMERGENCY_DISARM=FAIL: {disarm_exception}',
                file=sys.stderr,
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
