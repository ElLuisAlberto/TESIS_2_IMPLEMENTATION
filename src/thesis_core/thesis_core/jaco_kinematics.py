import math

from thesis_core.clearance_geometry import (
    closest_point_on_segment as geometry_closest_point,
    minimum_configuration_clearance,
    point_distance,
)


SEGMENT_NAMES = (
    'base_to_shoulder',
    'shoulder_to_upper_arm',
    'upper_arm_to_forearm',
    'forearm_to_wrist_1',
    'wrist_1_to_wrist_2',
    'wrist_2_to_tool',
)

CAPSULE_RADII = (0.105, 0.105, 0.085, 0.075, 0.085, 0.090)

JOINT_ORIGINS = (
    ((0.0, 0.0, 0.15675), (0.0, math.pi, 0.0)),
    ((0.0, 0.0016, -0.11875), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.410, 0.0), (0.0, math.pi, 0.0)),
    ((0.0, 0.2073, -0.0114), (-math.pi / 2.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
    ((0.0, -0.03703, -0.06414), (math.pi / 3.0, 0.0, math.pi)),
)

END_EFFECTOR_ORIGIN = (
    (0.0, 0.0, -0.1600),
    (math.pi, 0.0, math.pi / 2.0),
)


def identity_matrix():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply(first, second):
    return [
        [
            sum(first[row][index] * second[index][column]
                for index in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def transform_matrix(xyz, rpy):
    x_value, y_value, z_value = xyz
    roll, pitch, yaw = rpy

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr, x_value],
        [sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr, y_value],
        [-sp, cp * sr, cp * cr, z_value],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rotation_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def translation(matrix):
    return (matrix[0][3], matrix[1][3], matrix[2][3])


def capsule_segments(joint_positions):
    if len(joint_positions) != 6:
        raise ValueError('JACO2 forward kinematics requires six joints')

    current = identity_matrix()
    frames = [translation(current)]

    for joint_position, (xyz, rpy) in zip(
            joint_positions,
            JOINT_ORIGINS):
        current = multiply(current, transform_matrix(xyz, rpy))
        current = multiply(current, rotation_z(joint_position))
        frames.append(translation(current))

    end_xyz, end_rpy = END_EFFECTOR_ORIGIN
    end_effector = multiply(
        current,
        transform_matrix(end_xyz, end_rpy),
    )

    capsule_points = frames[:6] + [translation(end_effector)]
    return tuple(zip(capsule_points[:-1], capsule_points[1:]))


def closest_point_on_segment(point, start, end):
    """Compatibility wrapper for the canonical geometric primitive."""
    return geometry_closest_point(point, start, end)


def distance(first, second):
    """Compatibility wrapper for finite Euclidean distance."""
    return point_distance(first, second)


def minimum_sphere_clearance(
    joint_positions,
    obstacle_center,
    obstacle_radius,
):
    """Return the closest of all six canonical robot capsules."""
    result = minimum_configuration_clearance(
        capsule_segments(joint_positions),
        SEGMENT_NAMES,
        CAPSULE_RADII,
        obstacle_center,
        obstacle_radius,
    )
    return result.legacy_tuple()
