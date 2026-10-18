"""Robot-independent operations on homogeneous rigid transforms."""

import math


def translation(transform):
    """Return XYZ translation from a homogeneous transform."""
    return tuple(float(transform[index][3]) for index in range(3))


def rotation(transform):
    """Return the three-by-three rotation block."""
    return tuple(
        tuple(float(transform[row][column]) for column in range(3))
        for row in range(3)
    )


def transpose(matrix):
    """Return a transposed three-by-three matrix."""
    return tuple(zip(*matrix))


def multiply_rotation(first, second):
    """Multiply two three-by-three matrices."""
    return tuple(
        tuple(
            sum(
                first[row][index] * second[index][column]
                for index in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )


def rotation_vector_between(target_rotation, current_rotation):
    """Return the world-frame rotation vector from current to target."""
    relative = multiply_rotation(
        target_rotation,
        transpose(current_rotation),
    )
    cosine = (
        relative[0][0] + relative[1][1] + relative[2][2] - 1.0
    ) * 0.5
    cosine = max(-1.0, min(1.0, cosine))
    angle = math.acos(cosine)
    skew = (
        relative[2][1] - relative[1][2],
        relative[0][2] - relative[2][0],
        relative[1][0] - relative[0][1],
    )
    if angle < 1.0e-9:
        return tuple(0.5 * value for value in skew)
    sine = math.sin(angle)
    if abs(sine) < 1.0e-7:
        return tuple(0.5 * value for value in skew)
    scale = angle / (2.0 * sine)
    return tuple(scale * value for value in skew)


def pose_error(target_transform, current_transform):
    """Return Cartesian translation and rotation-vector error."""
    target_position = translation(target_transform)
    current_position = translation(current_transform)
    position_error = tuple(
        target - current
        for target, current in zip(target_position, current_position)
    )
    orientation_error = rotation_vector_between(
        rotation(target_transform),
        rotation(current_transform),
    )
    return position_error, orientation_error


def transform_from_translation_quaternion(
    translation_values,
    quaternion_values,
):
    """Build a homogeneous transform from XYZ and quaternion XYZW."""
    xyz = tuple(float(value) for value in translation_values)
    quaternion = tuple(float(value) for value in quaternion_values)
    if len(xyz) != 3 or len(quaternion) != 4:
        raise ValueError('transform requires XYZ and quaternion XYZW')
    if not all(math.isfinite(value) for value in xyz + quaternion):
        raise ValueError('transform values must be finite')
    x_value, y_value, z_value, w_value = quaternion
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError('invalid zero quaternion')
    x_value /= norm
    y_value /= norm
    z_value /= norm
    w_value /= norm
    return [
        [
            1.0 - 2.0 * (y_value * y_value + z_value * z_value),
            2.0 * (x_value * y_value - z_value * w_value),
            2.0 * (x_value * z_value + y_value * w_value),
            xyz[0],
        ],
        [
            2.0 * (x_value * y_value + z_value * w_value),
            1.0 - 2.0 * (x_value * x_value + z_value * z_value),
            2.0 * (y_value * z_value - x_value * w_value),
            xyz[1],
        ],
        [
            2.0 * (x_value * z_value - y_value * w_value),
            2.0 * (y_value * z_value + x_value * w_value),
            1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            xyz[2],
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
