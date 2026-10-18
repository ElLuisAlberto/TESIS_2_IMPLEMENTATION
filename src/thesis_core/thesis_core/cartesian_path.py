"""Robot-independent interpolation for bounded Cartesian trajectories."""

import math

from thesis_core.rigid_transform import (
    pose_error,
    transform_from_translation_quaternion,
    translation,
)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def rotation_quaternion(transform):
    """Return normalized XYZW quaternion from a homogeneous transform."""
    matrix = transform
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (matrix[2][1] - matrix[1][2]) / scale
        y_value = (matrix[0][2] - matrix[2][0]) / scale
        z_value = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]
        ) * 2.0
        w_value = (matrix[2][1] - matrix[1][2]) / scale
        x_value = 0.25 * scale
        y_value = (matrix[0][1] + matrix[1][0]) / scale
        z_value = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]
        ) * 2.0
        w_value = (matrix[0][2] - matrix[2][0]) / scale
        x_value = (matrix[0][1] + matrix[1][0]) / scale
        y_value = 0.25 * scale
        z_value = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]
        ) * 2.0
        w_value = (matrix[1][0] - matrix[0][1]) / scale
        x_value = (matrix[0][2] + matrix[2][0]) / scale
        y_value = (matrix[1][2] + matrix[2][1]) / scale
        z_value = 0.25 * scale
    quaternion = (x_value, y_value, z_value, w_value)
    norm = _norm(quaternion)
    if norm <= 1.0e-12:
        raise ValueError('rotation produced a zero quaternion')
    return tuple(value / norm for value in quaternion)


def quaternion_slerp(first, second, fraction):
    """Interpolate normalized XYZW quaternions along the shortest arc."""
    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError('interpolation fraction must belong to [0, 1]')
    first = tuple(float(value) for value in first)
    second = tuple(float(value) for value in second)
    if len(first) != 4 or len(second) != 4:
        raise ValueError('quaternions must contain four values')
    first_norm = _norm(first)
    second_norm = _norm(second)
    if first_norm <= 1.0e-12 or second_norm <= 1.0e-12:
        raise ValueError('cannot interpolate a zero quaternion')
    first = tuple(value / first_norm for value in first)
    second = tuple(value / second_norm for value in second)
    dot = sum(left * right for left, right in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        result = tuple(
            left + fraction * (right - left)
            for left, right in zip(first, second)
        )
        result_norm = _norm(result)
        return tuple(value / result_norm for value in result)
    angle = math.acos(dot)
    sine = math.sin(angle)
    first_weight = math.sin((1.0 - fraction) * angle) / sine
    second_weight = math.sin(fraction * angle) / sine
    return tuple(
        first_weight * left + second_weight * right
        for left, right in zip(first, second)
    )


def interpolate_transform(start, target, fraction):
    """Interpolate translation linearly and rotation with quaternion SLERP."""
    start_position = translation(start)
    target_position = translation(target)
    position = tuple(
        first + float(fraction) * (second - first)
        for first, second in zip(start_position, target_position)
    )
    quaternion = quaternion_slerp(
        rotation_quaternion(start),
        rotation_quaternion(target),
        fraction,
    )
    return transform_from_translation_quaternion(position, quaternion)


def cartesian_distance(start, target):
    """Return translation metres and shortest orientation radians."""
    position_error, orientation_error = pose_error(target, start)
    return _norm(position_error), _norm(orientation_error)


def synchronized_duration(
    linear_distance_m,
    angular_distance_rad,
    linear_speed_mps,
    angular_speed_radps,
):
    """Return common duration that respects both nominal speed requests."""
    values = (
        linear_distance_m,
        angular_distance_rad,
        linear_speed_mps,
        angular_speed_radps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('Cartesian duration values must be finite')
    if linear_distance_m < 0.0 or angular_distance_rad < 0.0:
        raise ValueError('Cartesian distances cannot be negative')
    if linear_speed_mps <= 0.0 or angular_speed_radps <= 0.0:
        raise ValueError('Cartesian speeds must be positive')
    linear_duration = linear_distance_m / linear_speed_mps
    angular_duration = angular_distance_rad / angular_speed_radps
    return max(linear_duration, angular_duration, 0.10)


def path_segment_count(
    linear_distance_m,
    angular_distance_rad,
    maximum_linear_step_m=0.01,
    maximum_angular_step_rad=math.radians(2.0),
):
    """Return deterministic spatial discretization for sequential IK."""
    if maximum_linear_step_m <= 0.0 or maximum_angular_step_rad <= 0.0:
        raise ValueError('Cartesian path steps must be positive')
    return max(
        1,
        int(math.ceil(linear_distance_m / maximum_linear_step_m)),
        int(math.ceil(angular_distance_rad / maximum_angular_step_rad)),
    )


def interpolate_joint_path(path, fraction):
    """Interpolate a uniformly sampled joint path at normalized progress."""
    if len(path) < 2:
        raise ValueError('joint path requires at least two configurations')
    fraction = min(max(float(fraction), 0.0), 1.0)
    position = fraction * (len(path) - 1)
    lower_index = min(int(math.floor(position)), len(path) - 2)
    local_fraction = position - lower_index
    lower = path[lower_index]
    upper = path[lower_index + 1]
    if len(lower) != len(upper):
        raise ValueError('joint path configurations have different sizes')
    return tuple(
        start + local_fraction * (end - start)
        for start, end in zip(lower, upper)
    )
