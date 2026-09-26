#!/usr/bin/env python3
"""Place the Gazebo obstacle near one selected canonical robot capsule."""

import argparse
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from thesis_core.clearance_geometry import minimum_configuration_clearance
from thesis_core.jaco_kinematics import CAPSULE_RADII, SEGMENT_NAMES
from thesis_interfaces.msg import ProximityStatus


FRAME_PAIRS = (
    ('j2n6s300_link_base', 'j2n6s300_link_1'),
    ('j2n6s300_link_1', 'j2n6s300_link_2'),
    ('j2n6s300_link_2', 'j2n6s300_link_3'),
    ('j2n6s300_link_3', 'j2n6s300_link_4'),
    ('j2n6s300_link_4', 'j2n6s300_link_5'),
    ('j2n6s300_link_5', 'j2n6s300_end_effector'),
)
SEGMENTS = tuple(
    (name, frames[0], frames[1], radius)
    for name, frames, radius in zip(
        SEGMENT_NAMES, FRAME_PAIRS, CAPSULE_RADII
    )
)
NAMES = SEGMENT_NAMES
RADII = CAPSULE_RADII
OBSTACLE_RADIUS = 0.12


def vector_subtract(left, right):
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def vector_norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def normalize(vector):
    norm = vector_norm(vector)
    if norm < 1e-9:
        return None
    return tuple(value / norm for value in vector)


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


class CapsulePose(Node):
    """Resolve TF geometry, select a discriminating obstacle pose and apply it."""

    def __init__(self):
        super().__init__('e12_capsule_pose')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.latest_status = None
        self.create_subscription(
            ProximityStatus, '/thesis/proximity_status', self._status_cb, 20,
        )

    def _status_cb(self, message):
        self.latest_status = message

    def segments(self):
        resolved = []
        for _, start_frame, end_frame, _ in SEGMENTS:
            start = self.buffer.lookup_transform('world', start_frame, Time())
            end = self.buffer.lookup_transform('world', end_frame, Time())
            start_t = start.transform.translation
            end_t = end.transform.translation
            resolved.append(((start_t.x, start_t.y, start_t.z),
                             (end_t.x, end_t.y, end_t.z)))
        return tuple(resolved)

    def wait_for_geometry(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                return self.segments()
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('No se resolvieron todos los TF de las cápsulas.')

    def choose_pose(self, target_name, clearance):
        segments = self.segments()
        index = NAMES.index(target_name)
        start, end = segments[index]
        midpoint = tuple((a + b) / 2.0 for a, b in zip(start, end))
        axis = normalize(vector_subtract(end, start))
        candidates = []
        bases = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        directions = []
        for base in bases:
            normal = normalize(cross(axis, base))
            if normal is not None:
                directions.extend((normal, tuple(-value for value in normal)))
        # A few diagonal directions reduce ambiguity near adjacent joints.
        directions.extend(normalize(v) for v in (
            (1.0, 1.0, 0.0), (1.0, -1.0, 0.0),
            (1.0, 0.0, 1.0), (1.0, 0.0, -1.0),
            (0.0, 1.0, 1.0), (0.0, 1.0, -1.0),
        ))
        distance = RADII[index] + OBSTACLE_RADIUS + clearance
        for direction in directions:
            if direction is None:
                continue
            center = tuple(
                point + distance * component
                for point, component in zip(midpoint, direction)
            )
            result = minimum_configuration_clearance(
                segments, NAMES, RADII, center, OBSTACLE_RADIUS,
            )
            if result.segment_name == target_name:
                score = abs(result.clearance - clearance)
                candidates.append((score, center, result.clearance))
        if not candidates:
            raise RuntimeError(
                f'No se encontró una pose discriminante para {target_name}.'
            )
        _, center, actual = min(candidates, key=lambda item: item[0])
        return center, actual

    def set_pose(self, center):
        request = (
            'name: "safety_obstacle" '
            f'position {{ x: {center[0]:.9f} y: {center[1]:.9f} z: {center[2]:.9f} }} '
            'orientation { w: 1.0 }'
        )
        completed = subprocess.run([
            'ign', 'service', '--service', '/world/jaco_world/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
            '--req', request,
        ], text=True, capture_output=True, check=False)
        if completed.returncode != 0 or 'true' not in completed.stdout.lower():
            raise RuntimeError(
                'Gazebo rechazó set_pose: ' + completed.stdout + completed.stderr
            )

    def wait_for_status(self, segment, center, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.latest_status
            if status is None or status.limiting_segment != segment:
                continue
            observed = status.obstacle_center
            error = vector_norm((
                observed.x - center[0], observed.y - center[1],
                observed.z - center[2],
            ))
            if error <= 0.02:
                return status
        raise RuntimeError(
            f'No se confirmó {segment} como cápsula limitante en Gazebo.'
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--segment', choices=NAMES, required=True)
    parser.add_argument('--clearance', type=float, default=0.10)
    parser.add_argument('--tf-timeout-sec', type=float, default=5.0)
    parser.add_argument('--status-timeout-sec', type=float, default=5.0)
    args, ros_args = parser.parse_known_args()
    if not 0.06 <= args.clearance <= 0.14:
        raise SystemExit('clearance debe estar entre 0.06 y 0.14 m.')
    rclpy.init(args=ros_args)
    node = CapsulePose()
    try:
        node.wait_for_geometry(args.tf_timeout_sec)
        center, geometric_clearance = node.choose_pose(
            args.segment, args.clearance,
        )
        node.set_pose(center)
        status = node.wait_for_status(
            args.segment, center, args.status_timeout_sec,
        )
        print('SEGMENTO=' + args.segment)
        print('POSE={:.6f},{:.6f},{:.6f}'.format(*center))
        print('CLEARANCE_GEOMETRICA={:.6f}'.format(geometric_clearance))
        print('CLEARANCE_OBSERVADA={:.6f}'.format(status.minimum_clearance))
        print('ESTADO_OBSERVADO=' + status.state)
        print('VERIFICACION=OK')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
