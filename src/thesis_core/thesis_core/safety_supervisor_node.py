import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from thesis_interfaces.msg import JointCommand, ProximityStatus


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

JOINT_VELOCITY_LIMITS = {
    EXPECTED_ARM_JOINTS[0]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[1]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[2]: 36.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[3]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[4]: 48.0 * math.pi / 180.0,
    EXPECTED_ARM_JOINTS[5]: 48.0 * math.pi / 180.0,
}

CONTINUOUS_JOINTS = {
    EXPECTED_ARM_JOINTS[0],
    EXPECTED_ARM_JOINTS[3],
    EXPECTED_ARM_JOINTS[4],
    EXPECTED_ARM_JOINTS[5],
}

MIN_DURATION_SEC = 0.1
MAX_DURATION_SEC = 30.0


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.declare_parameter(
            'state_topic',
            '/joint_states',
        )

        self.declare_parameter(
            'velocity_scale',
            0.5,
        )

        self.declare_parameter(
            'max_state_age_sec',
            0.5,
        )

        self.declare_parameter(
            'require_current_state',
            False,
        )

        self.declare_parameter(
            'require_proximity_status',
            True,
        )

        self.declare_parameter(
            'max_proximity_age_sec',
            0.5,
        )

        self.declare_parameter(
            'reduction_duration_scale',
            2.0,
        )

        state_topic = self.get_parameter(
            'state_topic'
        ).value

        self.current_positions = {}
        self.last_state_receive_ns = None
        self.proximity_status = None
        self.last_proximity_receive_ns = None
        self.last_proximity_decision = 'UNAVAILABLE'

        self.state_subscription = self.create_subscription(
            JointState,
            state_topic,
            self.joint_state_callback,
            10,
        )

        self.candidate_subscription = self.create_subscription(
            JointCommand,
            '/thesis/candidate_command',
            self.candidate_command_callback,
            10,
        )

        self.proximity_subscription = self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self.proximity_status_callback,
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

        self.get_logger().info(
            f'State topic configured: {state_topic}'
        )

        self.get_logger().info(
            'Proximity policy enabled on /thesis/proximity_status'
        )

    def joint_state_callback(self, msg):
        positions = {}

        for joint_name, position in zip(
            msg.name,
            msg.position,
        ):
            if math.isfinite(position):
                positions[joint_name] = float(position)

        self.current_positions = positions
        self.last_state_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def proximity_status_callback(self, msg):
        self.proximity_status = msg
        self.last_proximity_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def copy_command_with_duration(self, msg, duration_sec):
        output = JointCommand()
        output.stamp = msg.stamp
        output.command_id = msg.command_id
        output.joint_names = list(msg.joint_names)
        output.positions = list(msg.positions)

        whole_seconds = int(duration_sec)
        nanoseconds = int(round(
            (duration_sec - whole_seconds) * 1_000_000_000
        ))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        output.duration.sec = whole_seconds
        output.duration.nanosec = nanoseconds
        return output

    def apply_proximity_policy(self, msg, duration_sec):
        require_status = bool(
            self.get_parameter('require_proximity_status').value
        )

        if (
            self.proximity_status is None
            or self.last_proximity_receive_ns is None
        ):
            if require_status:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    'proximity status unavailable'
                )
                return None

            self.get_logger().warning(
                f'PROXIMITY CHECK SKIPPED {msg.command_id}: '
                'status unavailable'
            )
            self.last_proximity_decision = 'SKIPPED'
            return self.copy_command_with_duration(msg, duration_sec)

        max_age_sec = float(
            self.get_parameter('max_proximity_age_sec').value
        )
        status_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_proximity_receive_ns
        ) / 1e9

        if status_age_sec > max_age_sec:
            if require_status:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: proximity status stale '
                    f'({status_age_sec:.3f} s)'
                )
                return None

            self.get_logger().warning(
                f'PROXIMITY CHECK SKIPPED {msg.command_id}: '
                f'status stale ({status_age_sec:.3f} s)'
            )
            self.last_proximity_decision = 'SKIPPED'
            return self.copy_command_with_duration(msg, duration_sec)

        state = self.proximity_status.state.upper()
        self.last_proximity_decision = state
        clearance = self.proximity_status.minimum_clearance
        segment = self.proximity_status.limiting_segment

        if not math.isfinite(clearance):
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: '
                'non-finite proximity clearance'
            )
            return None

        if state == 'STOP':
            self.get_logger().warning(
                f'REJECTED {msg.command_id}: STOP proximity, '
                f'clearance={clearance:.3f} m, segment={segment}'
            )
            return None

        if state == 'REDUCTION':
            scale = float(
                self.get_parameter('reduction_duration_scale').value
            )
            if scale <= 1.0:
                self.get_logger().error(
                    'reduction_duration_scale must be greater than 1.0'
                )
                return None

            reduced_speed_duration = min(
                duration_sec * scale,
                MAX_DURATION_SEC,
            )
            if reduced_speed_duration <= duration_sec + 1.0e-9:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: REDUCTION requested '
                    'but duration cannot be increased safely'
                )
                return None

            self.get_logger().warning(
                f'REDUCTION {msg.command_id}: '
                f'clearance={clearance:.3f} m, segment={segment}, '
                f'duration={duration_sec:.2f}s'
                f'->{reduced_speed_duration:.2f}s'
            )
            return self.copy_command_with_duration(
                msg,
                reduced_speed_duration,
            )

        if state == 'WARNING':
            self.get_logger().warning(
                f'WARNING {msg.command_id}: '
                f'clearance={clearance:.3f} m, segment={segment}'
            )
            return self.copy_command_with_duration(msg, duration_sec)

        if state == 'ALLOW':
            return self.copy_command_with_duration(msg, duration_sec)

        self.get_logger().warning(
            f'REJECTED {msg.command_id}: unknown proximity state={state}'
        )
        return None

    def shortest_joint_delta(
        self,
        joint_name,
        target_position,
        current_position,
    ):
        delta = target_position - current_position

        if joint_name in CONTINUOUS_JOINTS:
            return math.atan2(
                math.sin(delta),
                math.cos(delta),
            )

        return delta

    def validate_velocity(
        self,
        msg,
        duration_sec,
    ):
        require_state = bool(
            self.get_parameter(
                'require_current_state'
            ).value
        )

        velocity_scale = float(
            self.get_parameter(
                'velocity_scale'
            ).value
        )

        max_state_age_sec = float(
            self.get_parameter(
                'max_state_age_sec'
            ).value
        )

        if (
            not self.current_positions
            or self.last_state_receive_ns is None
        ):
            if require_state:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    'current joint state unavailable'
                )
                return False

            self.get_logger().warning(
                f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                'current joint state unavailable'
            )
            return True

        state_age_sec = (
            self.get_clock().now().nanoseconds
            - self.last_state_receive_ns
        ) / 1e9

        if state_age_sec > max_state_age_sec:
            if require_state:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    f'joint state is stale '
                    f'({state_age_sec:.3f} s)'
                )
                return False

            self.get_logger().warning(
                f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                f'joint state is stale '
                f'({state_age_sec:.3f} s)'
            )
            return True

        for joint_name, target_position in zip(
            msg.joint_names,
            msg.positions,
        ):
            if joint_name not in self.current_positions:
                if require_state:
                    self.get_logger().warning(
                        f'REJECTED {msg.command_id}: '
                        f'no current state for {joint_name}'
                    )
                    return False

                self.get_logger().warning(
                    f'VELOCITY CHECK SKIPPED {msg.command_id}: '
                    f'no current state for {joint_name}'
                )
                return True

            current_position = self.current_positions[joint_name]

            delta = self.shortest_joint_delta(
                joint_name,
                target_position,
                current_position,
            )

            requested_velocity = abs(delta) / duration_sec

            allowed_velocity = (
                JOINT_VELOCITY_LIMITS[joint_name]
                * velocity_scale
            )

            if requested_velocity > allowed_velocity:
                self.get_logger().warning(
                    f'REJECTED {msg.command_id}: '
                    f'{joint_name} requested velocity '
                    f'{requested_velocity:.4f} rad/s exceeds '
                    f'{allowed_velocity:.4f} rad/s'
                )
                return False

        return True

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

        supervised_msg = self.apply_proximity_policy(
            msg,
            duration_sec,
        )
        if supervised_msg is None:
            return

        supervised_duration_sec = (
            float(supervised_msg.duration.sec)
            + float(supervised_msg.duration.nanosec) * 1e-9
        )

        if not self.validate_velocity(
            supervised_msg,
            supervised_duration_sec,
        ):
            return

        candidate_stamp_ns = (
            int(msg.stamp.sec) * 1_000_000_000
            + int(msg.stamp.nanosec)
        )

        receive_ns = receive_time.nanoseconds

        input_latency_ms = (
            receive_ns - candidate_stamp_ns
        ) / 1e6

        self.supervised_publisher.publish(supervised_msg)

        self.get_logger().info(
            f'{self.last_proximity_decision} FORWARDED '
            f'id={msg.command_id} | '
            f'joints={len(msg.joint_names)} | '
            f'proximity={self.last_proximity_decision} | '
            f'duration={supervised_duration_sec:.2f} s | '
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
