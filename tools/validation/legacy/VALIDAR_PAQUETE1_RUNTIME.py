#!/usr/bin/env python3
"""Runtime validation of canonical JACO2 operational joint limits."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from thesis_interfaces.msg import (
    CommandDecision,
    ExecutionControl,
    JointCommand,
    ProximityStatus,
)
from trajectory_msgs.msg import JointTrajectory

from thesis_core.joint_model import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    shortest_joint_delta,
)


JointVector = Tuple[float, float, float, float, float, float]
POSITION_TOLERANCE_RAD = 0.015
VELOCITY_TOLERANCE_RAD_S = math.radians(0.75)


class RuntimeValidator(Node):
    """Inject traceable commands and observe every supervised output."""

    def __init__(self) -> None:
        super().__init__('package1_runtime_validator')
        self.current: Optional[JointVector] = None
        self.proximity: Optional[ProximityStatus] = None
        self.supervised: Dict[str, JointCommand] = {}
        self.supervised_jog: Dict[str, JointCommand] = {}
        self.decisions: Dict[str, CommandDecision] = {}
        self.controls: Dict[str, ExecutionControl] = {}
        self.logs: List[str] = []
        self.trajectories: List[Tuple[JointTrajectory, Optional[JointVector]]] = []

        self.candidate_publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10
        )
        self.jog_publisher = self.create_publisher(
            JointCommand, '/thesis/jog_intent', 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._state_callback, 10
        )
        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self._proximity_callback,
            10,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/supervised_command',
            self._supervised_callback,
            10,
        )
        self.create_subscription(
            JointCommand,
            '/thesis/supervised_jog_command',
            self._supervised_jog_callback,
            10,
        )
        self.create_subscription(
            CommandDecision,
            '/thesis/command_decision',
            self._decision_callback,
            10,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self._control_callback,
            10,
        )
        controller_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self._trajectory_callback,
            controller_qos,
        )
        self.create_subscription(Log, '/rosout', self._log_callback, 100)

    def _state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            values = tuple(float(positions[name]) for name in JOINT_NAMES)
            if all(math.isfinite(value) for value in values):
                self.current = values  # type: ignore[assignment]

    def _proximity_callback(self, msg: ProximityStatus) -> None:
        self.proximity = msg

    def _supervised_callback(self, msg: JointCommand) -> None:
        self.supervised[msg.command_id] = msg

    def _supervised_jog_callback(self, msg: JointCommand) -> None:
        self.supervised_jog[msg.command_id] = msg

    def _decision_callback(self, msg: CommandDecision) -> None:
        self.decisions[msg.command_id] = msg

    def _control_callback(self, msg: ExecutionControl) -> None:
        self.controls[msg.command_id] = msg

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        self.trajectories.append((msg, self.current))

    def _log_callback(self, msg: Log) -> None:
        if 'safety_supervisor' in msg.name:
            self.logs.append(msg.msg)

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def wait_ready(self) -> None:
        if not self.spin_until(
            lambda: self.current is not None and self.proximity is not None,
            12.0,
        ):
            raise RuntimeError('joint state or proximity status unavailable')
        if not self.spin_until(
            lambda: self.candidate_publisher.get_subscription_count() > 0
            and self.jog_publisher.get_subscription_count() > 0,
            8.0,
        ):
            raise RuntimeError('supervisor subscriptions unavailable')

    def make_command(
        self,
        command_id: str,
        positions: Sequence[float],
        duration_sec: float = 1.0,
        joint_names: Sequence[str] = JOINT_NAMES,
    ) -> JointCommand:
        msg = JointCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.joint_names = list(joint_names)
        msg.positions = [float(value) for value in positions]
        whole = int(duration_sec)
        msg.duration.sec = whole
        msg.duration.nanosec = int(round((duration_sec - whole) * 1.0e9))
        return msg

    def diagnostic_for(self, command_id: str) -> str:
        matches = [line for line in self.logs if command_id in line]
        return ' || '.join(matches)


def duration_seconds(msg: JointCommand) -> float:
    return float(msg.duration.sec) + float(msg.duration.nanosec) * 1.0e-9


def test_candidate_limit(
    node: RuntimeValidator,
    label: str,
    joint_index: int,
    requested_deg: float,
    expected_deg: float,
) -> bool:
    assert node.current is not None
    start = node.current
    target = list(start)
    target[joint_index] += math.radians(requested_deg)
    command_id = f'p1_{label}_{time.monotonic_ns()}'
    node.logs.clear()
    node.candidate_publisher.publish(
        node.make_command(command_id, target, 1.0)
    )
    received = node.spin_until(
        lambda: command_id in node.supervised
        or command_id in node.decisions,
        5.0,
    )
    if not received or command_id not in node.supervised:
        decision = node.decisions.get(command_id)
        detail = decision.reason if decision is not None else 'no response'
        print(f'TEST_{label}=FAIL detail={detail}')
        return False

    output = node.supervised[command_id]
    if tuple(output.joint_names) != JOINT_NAMES or len(output.positions) != 6:
        print(f'TEST_{label}=FAIL detail=invalid supervised shape')
        return False
    delta = shortest_joint_delta(
        JOINT_NAMES[joint_index], output.positions[joint_index], start[joint_index]
    )
    expected = math.radians(expected_deg)
    other_ok = all(
        abs(shortest_joint_delta(name, out, initial))
        <= POSITION_TOLERANCE_RAD
        for index, (name, out, initial) in enumerate(
            zip(JOINT_NAMES, output.positions, start)
        )
        if index != joint_index
    )
    delta_ok = math.isclose(
        delta, expected, rel_tol=0.0, abs_tol=POSITION_TOLERANCE_RAD
    )
    node.spin_until(lambda: bool(node.diagnostic_for(command_id)), 1.5)
    diagnostic = node.diagnostic_for(command_id)
    diagnostic_ok = all(
        token in diagnostic for token in ('joint', 'requested', 'limited')
    ) or all(token in diagnostic for token in ('joint_lim', 'v_req', 'v_lim'))
    passed = delta_ok and other_ok and diagnostic_ok
    print(
        f'TEST_{label}={"PASS" if passed else "FAIL"} '
        f'requested_deg={requested_deg:+.3f} '
        f'supervised_delta_deg={math.degrees(delta):+.6f} '
        f'duration={duration_seconds(output):.6f}s '
        f'diagnostic={diagnostic!r}'
    )
    return passed


def test_rejection(
    node: RuntimeValidator,
    label: str,
    positions: Sequence[float],
    names: Sequence[str],
) -> bool:
    command_id = f'p1_{label}_{time.monotonic_ns()}'
    node.candidate_publisher.publish(
        node.make_command(command_id, positions, 1.0, names)
    )
    received = node.spin_until(
        lambda: command_id in node.decisions,
        5.0,
    )
    decision = node.decisions.get(command_id)
    passed = (
        received
        and decision is not None
        and not decision.accepted
        and command_id not in node.supervised
    )
    detail = decision.reason if decision is not None else 'no decision'
    print(f'TEST_{label}={"PASS" if passed else "FAIL"} detail={detail!r}')
    return passed


def run_candidate_tests(node: RuntimeValidator) -> bool:
    node.wait_ready()
    assert node.current is not None
    print(f'PROXIMITY_STATE={node.proximity.state}')
    print(f'INITIAL_POSITIONS={node.current}')
    results = [
        test_candidate_limit(node, 'J1_POS_36', 0, 36.0, 18.0),
        test_candidate_limit(node, 'J1_NEG_36', 0, -36.0, -18.0),
        test_candidate_limit(node, 'J4_POS_48', 3, 48.0, 24.0),
    ]
    current = node.current
    wrong_names = list(JOINT_NAMES)
    wrong_names[0], wrong_names[1] = wrong_names[1], wrong_names[0]
    results.append(
        test_rejection(node, 'WRONG_ORDER', current, wrong_names)
    )
    nan_positions = list(current)
    nan_positions[2] = math.nan
    results.append(
        test_rejection(node, 'NAN_REJECTION', nan_positions, JOINT_NAMES)
    )
    passed = all(results)
    print(f'CANDIDATE_TESTS={"PASS" if passed else "FAIL"}')
    return passed


def trajectory_duration(msg: JointTrajectory) -> float:
    if not msg.points:
        return 0.0
    stamp = msg.points[0].time_from_start
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def run_jog_test(node: RuntimeValidator) -> bool:
    node.wait_ready()
    assert node.current is not None
    start = node.current
    target = list(start)
    target[0] += math.radians(36.0)
    command_id = f'p1_JOG_J1_POS_36_{time.monotonic_ns()}'
    node.trajectories.clear()
    node.logs.clear()
    node.jog_publisher.publish(node.make_command(command_id, target, 1.0))
    got_supervised = node.spin_until(
        lambda: command_id in node.supervised_jog,
        5.0,
    )
    got_trajectory = node.spin_until(lambda: bool(node.trajectories), 3.0)
    if not got_supervised or not got_trajectory:
        print(
            'TEST_JOG_CHAIN=FAIL '
            f'supervised={got_supervised} trajectory={got_trajectory}'
        )
        return False

    supervised = node.supervised_jog[command_id]
    supervised_delta = shortest_joint_delta(
        JOINT_NAMES[0], supervised.positions[0], start[0]
    )
    supervised_velocity = supervised_delta / duration_seconds(supervised)
    supervised_ok = (
        supervised_velocity >= -VELOCITY_TOLERANCE_RAD_S
        and supervised_velocity
        <= JOINT_VELOCITY_LIMITS[JOINT_NAMES[0]]
        + VELOCITY_TOLERANCE_RAD_S
    )

    node.spin_until(lambda: command_id in node.controls, 2.0)
    node.spin_until(lambda: len(node.trajectories) >= 2, 0.6)
    evaluated = []
    for trajectory, state_at_output in node.trajectories:
        duration = trajectory_duration(trajectory)
        if (
            tuple(trajectory.joint_names) != JOINT_NAMES
            or not trajectory.points
            or state_at_output is None
            or duration <= 0.0
            or len(trajectory.points[0].positions) != len(JOINT_NAMES)
        ):
            continue
        velocities = tuple(
            shortest_joint_delta(name, target_value, current_value)
            / duration
            for name, target_value, current_value in zip(
                JOINT_NAMES,
                trajectory.points[0].positions,
                state_at_output,
            )
        )
        evaluated.append((trajectory, duration, velocities))

    selected = max(
        evaluated,
        key=lambda item: abs(item[2][0]),
        default=None,
    )
    controller_velocities: Sequence[float] = ()
    duration = 0.0
    controller_ok = selected is not None
    if selected is not None:
        _, duration, controller_velocities = selected
        controller_ok = (
            controller_velocities[0] > VELOCITY_TOLERANCE_RAD_S
            and all(
                abs(value)
                <= JOINT_VELOCITY_LIMITS[name]
                + VELOCITY_TOLERANCE_RAD_S
                for name, value in zip(JOINT_NAMES, controller_velocities)
            )
        )

    control = node.controls.get(command_id)
    diagnostic = control.reason if control is not None else ''
    diagnostic_ok = (
        control is not None
        and all(
            token in diagnostic
            for token in ('joint_lim', 'v_req', 'v_lim')
        )
    )
    passed = supervised_ok and controller_ok and diagnostic_ok
    print(
        f'TEST_JOG_SUPERVISOR={"PASS" if supervised_ok else "FAIL"} '
        f'velocity_deg_s={math.degrees(supervised_velocity):+.6f}'
    )
    print(
        f'TEST_CONTROLLER_OUTPUT={"PASS" if controller_ok else "FAIL"} '
        f'duration={duration:.6f}s '
        f'velocities_deg_s='
        f'{tuple(math.degrees(value) for value in controller_velocities)}'
    )
    print(
        f'TEST_JOG_DIAGNOSTIC={"PASS" if diagnostic_ok else "FAIL"} '
        f'diagnostic={diagnostic!r}'
    )

    if node.current is not None:
        hold_id = f'p1_HOLD_{time.monotonic_ns()}'
        node.jog_publisher.publish(
            node.make_command(hold_id, node.current, 1.0)
        )
        node.spin_until(lambda: hold_id in node.supervised_jog, 1.0)
    print(f'JOG_TESTS={"PASS" if passed else "FAIL"}')
    return passed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('candidate', 'jog'), required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RuntimeValidator()
    try:
        passed = (
            run_candidate_tests(node)
            if args.mode == 'candidate'
            else run_jog_test(node)
        )
        return 0 if passed else 1
    except Exception as exc:
        print(f'RUNTIME_VALIDATION_ERROR={type(exc).__name__}: {exc}')
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
