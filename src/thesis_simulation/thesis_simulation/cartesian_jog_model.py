"""Pure helpers for bounded Cartesian velocity jogging in a world frame."""

import math

from thesis_core.rigid_transform import (
    multiply_rotation,
    rotation,
    translation,
)


def vector_norm(values):
    """Return the Euclidean norm of a finite three-dimensional vector."""
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError('Cartesian vectors must contain three finite values')
    return math.sqrt(sum(value * value for value in vector))


def validate_world_twist(
    linear_velocity,
    angular_velocity,
    maximum_linear_speed_mps,
    maximum_angular_speed_radps,
):
    """Validate and return one bounded world-frame Cartesian velocity."""
    linear = tuple(float(value) for value in linear_velocity)
    angular = tuple(float(value) for value in angular_velocity)
    linear_limit = float(maximum_linear_speed_mps)
    angular_limit = float(maximum_angular_speed_radps)
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (linear_limit, angular_limit)
    ):
        raise ValueError('Cartesian velocity limits must be finite and positive')
    linear_norm = vector_norm(linear)
    angular_norm = vector_norm(angular)
    if linear_norm > linear_limit + 1.0e-12:
        raise ValueError(
            f'linear speed {linear_norm:.4f} m/s exceeds '
            f'{linear_limit:.4f} m/s'
        )
    if angular_norm > angular_limit + 1.0e-12:
        raise ValueError(
            f'angular speed {angular_norm:.4f} rad/s exceeds '
            f'{angular_limit:.4f} rad/s'
        )
    return linear, angular


def apply_twist_deadzone(linear, angular, linear_deadzone, angular_deadzone):
    """Suppress sub-threshold Cartesian noise without changing larger input."""
    linear_threshold = float(linear_deadzone)
    angular_threshold = float(angular_deadzone)
    if not all(
        math.isfinite(value) and value >= 0.0
        for value in (linear_threshold, angular_threshold)
    ):
        raise ValueError('Cartesian deadzones must be finite and non-negative')
    filtered_linear = tuple(
        0.0 if abs(float(value)) <= linear_threshold else float(value)
        for value in linear
    )
    filtered_angular = tuple(
        0.0 if abs(float(value)) <= angular_threshold else float(value)
        for value in angular
    )
    vector_norm(filtered_linear)
    vector_norm(filtered_angular)
    return filtered_linear, filtered_angular


def limit_twist_acceleration(
    previous_linear,
    previous_angular,
    requested_linear,
    requested_angular,
    elapsed_sec,
    maximum_linear_acceleration_mps2,
    maximum_angular_acceleration_radps2,
):
    """Apply a vector-norm slew limit to a non-zero Cartesian request."""
    elapsed = float(elapsed_sec)
    linear_acceleration = float(maximum_linear_acceleration_mps2)
    angular_acceleration = float(maximum_angular_acceleration_radps2)
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (elapsed, linear_acceleration, angular_acceleration)
    ):
        raise ValueError('Cartesian acceleration limits must be positive')

    def limited(previous, requested, maximum_delta):
        previous = tuple(float(value) for value in previous)
        requested = tuple(float(value) for value in requested)
        delta = tuple(
            target - current
            for current, target in zip(previous, requested)
        )
        norm = vector_norm(delta)
        if norm <= maximum_delta:
            return requested
        scale = maximum_delta / norm
        return tuple(
            current + scale * change
            for current, change in zip(previous, delta)
        )

    return (
        limited(
            previous_linear,
            requested_linear,
            linear_acceleration * elapsed,
        ),
        limited(
            previous_angular,
            requested_angular,
            angular_acceleration * elapsed,
        ),
    )


def bounded_integration_step(
    linear_velocity,
    angular_velocity,
    nominal_step_sec,
    maximum_linear_step_m=0.005,
    maximum_angular_step_rad=math.radians(1.0),
):
    """Shorten the local IK step so high requested rates remain solvable."""
    nominal = float(nominal_step_sec)
    linear_step = float(maximum_linear_step_m)
    angular_step = float(maximum_angular_step_rad)
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (nominal, linear_step, angular_step)
    ):
        raise ValueError('Cartesian integration limits must be positive')
    linear_norm = vector_norm(linear_velocity)
    angular_norm = vector_norm(angular_velocity)
    candidates = [nominal]
    if linear_norm > 1.0e-12:
        candidates.append(linear_step / linear_norm)
    if angular_norm > 1.0e-12:
        candidates.append(angular_step / angular_norm)
    return min(candidates)


def rotation_from_vector(rotation_vector):
    """Return a rotation matrix for one world-frame axis-angle vector."""
    values = tuple(float(value) for value in rotation_vector)
    angle = vector_norm(values)
    if angle <= 1.0e-12:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    x_value, y_value, z_value = (
        value / angle for value in values
    )
    cosine = math.cos(angle)
    sine = math.sin(angle)
    complement = 1.0 - cosine
    return (
        (
            cosine + x_value * x_value * complement,
            x_value * y_value * complement - z_value * sine,
            x_value * z_value * complement + y_value * sine,
        ),
        (
            y_value * x_value * complement + z_value * sine,
            cosine + y_value * y_value * complement,
            y_value * z_value * complement - x_value * sine,
        ),
        (
            z_value * x_value * complement - y_value * sine,
            z_value * y_value * complement + x_value * sine,
            cosine + z_value * z_value * complement,
        ),
    )


def integrate_world_twist(current_transform, linear, angular, horizon_sec):
    """Integrate a constant world-frame twist over a short horizon."""
    horizon = float(horizon_sec)
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError('Cartesian JOG horizon must be finite and positive')
    current = tuple(
        tuple(float(value) for value in row)
        for row in current_transform
    )
    if (
        len(current) != 4
        or any(len(row) != 4 for row in current)
        or not all(math.isfinite(value) for row in current for value in row)
    ):
        raise ValueError('current transform must be a finite 4 by 4 matrix')
    linear = tuple(float(value) for value in linear)
    angular = tuple(float(value) for value in angular)
    vector_norm(linear)
    vector_norm(angular)
    current_position = translation(current)
    target_position = tuple(
        value + velocity * horizon
        for value, velocity in zip(current_position, linear)
    )
    delta_rotation = rotation_from_vector(
        tuple(value * horizon for value in angular)
    )
    target_rotation = multiply_rotation(
        delta_rotation,
        rotation(current),
    )
    return [
        [*target_rotation[0], target_position[0]],
        [*target_rotation[1], target_position[1]],
        [*target_rotation[2], target_position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
