#!/usr/bin/env python3
"""Publish short joint-space motion sequences for a live horizon demo.

This is a presentation stimulus, not a Cartesian planner or a gripper driver.
All targets pass through the thesis candidate-command safety pipeline.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from thesis_core.joint_model import JOINT_NAMES
from thesis_interfaces.msg import CommandDecision, JointCommand
from thesis_interfaces.msg import TrajectoryPrediction


HOME = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)

# These are illustrative joint configurations around the GUI's default pose.
# They do not encode Cartesian pick/place points or gripper open/close states.
SCENARIOS = {
    'pick_place': [
        ('Aproximación coordinada', (0.20, 2.88, 2.96, 0.20, -0.15, 0.12)),
        ('Descenso simulado', (0.28, 2.75, 3.08, 0.36, -0.28, 0.30)),
        ('Elevación coordinada', (0.16, 2.52, 2.86, 0.08, -0.08, 0.18)),
        ('Traslado con reorientación', (-0.32, 2.62, 3.02, -0.30, 0.24, -0.24)),
        ('Colocación simulada', (-0.42, 2.78, 3.16, -0.18, 0.38, -0.38)),
        ('Retorno a pose inicial', HOME),
    ],
    'barrido': [
        ('Barrido coordinado hacia un lado',
         (0.62, 3.32, 2.88, 0.42, -0.36, 0.30)),
        ('Barrido coordinado al lado opuesto',
         (-0.62, 2.96, 3.38, -0.42, 0.36, -0.30)),
        ('Retorno a pose inicial', HOME),
    ],
    'elevacion': [
        ('Flexión coordinada de hombro y muñeca',
         (0.18, 2.72, 3.02, 0.25, -0.20, 0.18)),
        ('Flexión coordinada de codo y orientación',
         (-0.14, 2.48, 2.72, -0.30, 0.28, -0.22)),
        ('Retorno a pose inicial', HOME),
    ],
}


class HorizonMotionDemo(Node):
    def __init__(self):
        super().__init__('horizon_motion_demo')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.publisher = self.create_publisher(
            JointCommand, '/thesis/candidate_command', 10,
        )
        self.predictions = {}
        self.decisions = {}
        self.create_subscription(
            TrajectoryPrediction,
            '/thesis/trajectory_prediction',
            self._prediction_callback,
            20,
        )
        self.create_subscription(
            CommandDecision,
            '/thesis/command_decision',
            self._decision_callback,
            20,
        )

    def _prediction_callback(self, msg):
        self.predictions[msg.command_id] = msg

    def _decision_callback(self, msg):
        self.decisions[msg.command_id] = msg

    def wait_for_pipeline(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def publish_target(self, label, target, duration, settle_sec, index):
        command_id = f'demo-{index:02d}-{time.monotonic_ns()}'
        msg = JointCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command_id = command_id
        msg.joint_names = list(JOINT_NAMES)
        msg.positions = list(target)
        whole = int(duration)
        msg.duration.sec = whole
        msg.duration.nanosec = int((duration - whole) * 1e9)
        self.publisher.publish(msg)
        target_degrees = ', '.join(f'{math.degrees(value):.0f}' for value in target)
        print(f'\n[{index}] {label}: candidato enviado ({duration:.1f} s)')
        print(f'  objetivo J1..J6 [grados]=[{target_degrees}]')

        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            prediction = self.predictions.get(command_id)
            decision = self.decisions.get(command_id)
            if prediction is not None and decision is not None:
                t_protect = prediction.time_to_protective_volume
                t_collision = prediction.time_to_collision
                print(
                    f"  decisión={decision.state} "
                    f"({'aceptado' if decision.accepted else 'bloqueado'})"
                )
                print(
                    f'  distancia mínima prevista='
                    f'{prediction.minimum_clearance:.3f} m; '
                    f'segmento={prediction.limiting_segment}; '
                    f'muestra={prediction.sample_index}/'
                    f'{prediction.sample_count - 1}; '
                    f'avance={prediction.trajectory_fraction:.0%}'
                )
                print(
                    f'  entrada al volumen protector='
                    f'{self._format_time(t_protect)}; '
                    f'tiempo a colisión={self._format_time(t_collision)}'
                )
                break
        else:
            print('  No llegó una pareja de decisión y predicción en 5 s.')
            print('  Revisa /thesis/command_decision y /thesis/trajectory_prediction.')

        # Spin while waiting so the UI receives the prediction and the
        # controller has time to process an authorized target.
        deadline = time.monotonic() + settle_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))

    @staticmethod
    def _format_time(value):
        return 'sin evento en horizonte' if value < 0.0 else f'{value:.2f} s'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--scenario', choices=(*SCENARIOS.keys(), 'all'), default='all',
        help='secuencia que se mostrará (por defecto: all)',
    )
    parser.add_argument('--duration', type=float, default=2.0,
                        help='duración nominal de cada objetivo, en segundos')
    parser.add_argument('--settle', type=float, default=2.5,
                        help='pausa entre objetivos, en segundos')
    parser.add_argument('--discovery-timeout', type=float, default=10.0)
    args, ros_args = parser.parse_known_args()
    if args.duration <= 0.0 or args.settle < 0.0:
        parser.error('--duration debe ser positivo y --settle no negativo')

    rclpy.init(args=ros_args)
    node = HorizonMotionDemo()
    try:
        if not node.wait_for_pipeline(args.discovery_timeout):
            raise RuntimeError(
                'No se detectó el supervisor en /thesis/candidate_command. '
                'Inicia Gazebo y safety_pipeline.launch.py primero.'
            )
        selected = list(SCENARIOS) if args.scenario == 'all' else [args.scenario]
        number = 0
        for scenario in selected:
            print(f'\n===== ESCENARIO: {scenario} =====')
            for label, target in SCENARIOS[scenario]:
                number += 1
                node.publish_target(
                    label, target, args.duration, args.settle, number,
                )
        print('\nDemostración terminada. Revisa también el volumen en RViz2.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
