"""Shared ROS 2 runner for the five multi-joint demonstration scripts."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import CommandDecision, JointCommand
from thesis_interfaces.msg import TrajectoryPrediction


HOME_DEG = (0.0, 180.0, 180.0, 0.0, 0.0, 0.0)


class JointMotionDemo(Node):
    """Send targets through the preventive pipeline and report its result."""

    def __init__(self, demo_name):
        super().__init__(f'{demo_name}_demo')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10,
        )
        self.predictions = {}
        self.decisions = {}
        self.create_subscription(
            TrajectoryPrediction, '/thesis/trajectory_prediction',
            lambda msg: self.predictions.__setitem__(msg.command_id, msg), 20,
        )
        self.create_subscription(
            CommandDecision, '/thesis/command_decision',
            lambda msg: self.decisions.__setitem__(msg.command_id, msg), 20,
        )

    def wait_for_pipeline(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def send(self, label, target_deg, duration, settle, index):
        command_id = f'demo-{index:02d}-{time.monotonic_ns()}'
        msg = JointCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.joint_names = list(JOINT_NAMES)
        msg.positions = [math.radians(value) for value in target_deg]
        seconds = int(duration)
        msg.duration.sec = seconds
        msg.duration.nanosec = int((duration - seconds) * 1e9)
        self.publisher.publish(msg)

        values = ', '.join(f'{value:.0f}' for value in target_deg)
        print(f'\n[{index}] {label}: objetivo J1..J6=[{values}] grados')
        print(f'  duración nominal solicitada={duration:.1f} s')

        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            prediction = self.predictions.get(command_id)
            decision = self.decisions.get(command_id)
            if prediction is None or decision is None:
                continue
            print(
                f'  supervisor={decision.state}; '
                f"{'aceptado' if decision.accepted else 'bloqueado'}"
            )
            print(
                f'  mínima={prediction.minimum_clearance:.3f} m; '
                f' segmento={prediction.limiting_segment}; '
                f'muestra={prediction.sample_index}/'
                f'{prediction.sample_count - 1}; '
                f'avance={prediction.trajectory_fraction:.0%}'
            )
            break
        else:
            print('  No se recibió la decisión y predicción asociadas al comando.')

        deadline = time.monotonic() + settle
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))


def run_demo(demo_name, waypoints, duration=4.0, settle=4.5):
    """Run one waypoint list; every entry is a six-angle degree vector."""
    rclpy.init()
    node = JointMotionDemo(demo_name)
    try:
        if not node.wait_for_pipeline():
            raise RuntimeError(
                'No se detectó el supervisor; verifica safety_pipeline.launch.py.'
            )
        print(f'===== DEMOSTRACIÓN {demo_name} =====')
        print('La secuencia parte de la pose HOME [0, 180, 180, 0, 0, 0].')
        for index, (label, target) in enumerate(waypoints, start=1):
            node.send(label, target, duration, settle, index)
        print('\nFin de la secuencia. Observa /thesis/horizon_volume en RViz2.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
