"""Local numerical inverse kinematics for small JACO2 Cartesian steps."""

from dataclasses import dataclass
import math

from thesis_core.jaco_kinematics import (
    end_effector_transform,
    multiply,
    transform_matrix,
    translation,
)
from thesis_core.joint_model import (
    CONTINUOUS_JOINT_INDEXES,
    JOINT_NAMES,
    JOINT_POSITION_LIMITS,
    validate_position_limits,
)


@dataclass(frozen=True)
class IkResult:
    """Result of one bounded inverse-kinematics request."""

    success: bool
    positions: tuple
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    detail: str


def _rotation(transform):
    return tuple(
        tuple(transform[row][column] for column in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(zip(*matrix))


def _multiply_3(first, second):
    return tuple(
        tuple(
            sum(first[row][index] * second[index][column]
                for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def rotation_vector_between(target_rotation, current_rotation):
    """Return the world-frame rotation vector from current to target."""
    relative = _multiply_3(
        target_rotation,
        _transpose(current_rotation),
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
        _rotation(target_transform),
        _rotation(current_transform),
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


def inverse_rigid_transform(transform):
    """Return the inverse of a finite homogeneous rigid transform."""
    values = tuple(tuple(float(value) for value in row) for row in transform)
    if len(values) != 4 or any(len(row) != 4 for row in values):
        raise ValueError('transform must be a 4 by 4 matrix')
    if not all(math.isfinite(value) for row in values for value in row):
        raise ValueError('transform values must be finite')
    rotation = _rotation(values)
    rotation_transpose = _transpose(rotation)
    position = translation(values)
    inverse_position = tuple(
        -sum(rotation_transpose[row][column] * position[column]
             for column in range(3))
        for row in range(3)
    )
    return [
        [*rotation_transpose[0], inverse_position[0]],
        [*rotation_transpose[1], inverse_position[1]],
        [*rotation_transpose[2], inverse_position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def align_relative_target(
    current_world_transform,
    target_world_transform,
    current_model_transform,
):
    """Map a live-TF target into the locally calibrated IK reference."""
    model_from_world = multiply(
        current_model_transform,
        inverse_rigid_transform(current_world_transform),
    )
    return multiply(model_from_world, target_world_transform)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def _solve_linear(matrix, vector):
    size = len(vector)
    augmented = [
        [float(value) for value in row] + [float(vector[index])]
        for index, row in enumerate(matrix)
    ]
    for column in range(size):
        pivot = max(
            range(column, size),
            key=lambda row: abs(augmented[row][column]),
        )
        if abs(augmented[pivot][column]) < 1.0e-12:
            raise ValueError('singular inverse-kinematics system')
        augmented[column], augmented[pivot] = (
            augmented[pivot], augmented[column]
        )
        divisor = augmented[column][column]
        augmented[column] = [
            value / divisor for value in augmented[column]
        ]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * pivot_value
                for current, pivot_value in zip(
                    augmented[row], augmented[column]
                )
            ]
    return tuple(augmented[row][-1] for row in range(size))


def _bounded_positions(positions, reference):
    bounded = []
    for index, (name, value, reference_value) in enumerate(zip(
            JOINT_NAMES, positions, reference)):
        lower, upper = JOINT_POSITION_LIMITS[name]
        if index in CONTINUOUS_JOINT_INDEXES:
            value = reference_value + math.atan2(
                math.sin(value - reference_value),
                math.cos(value - reference_value),
            )
        bounded.append(min(max(value, lower), upper))
    return tuple(bounded)


def _numerical_jacobian(positions, current_transform, epsilon):
    current_rotation = _rotation(current_transform)
    current_position = translation(current_transform)
    columns = []
    for index in range(len(JOINT_NAMES)):
        perturbed = list(positions)
        perturbed[index] += epsilon
        perturbed_transform = end_effector_transform(perturbed)
        position_column = tuple(
            (value - current_position[axis]) / epsilon
            for axis, value in enumerate(translation(perturbed_transform))
        )
        rotation_column = tuple(
            value / epsilon for value in rotation_vector_between(
                _rotation(perturbed_transform), current_rotation
            )
        )
        columns.append(position_column + rotation_column)
    return tuple(
        tuple(columns[column][row] for column in range(len(JOINT_NAMES)))
        for row in range(6)
    )


def _weighted_cost(position_error, orientation_error, position_weight):
    return math.sqrt(
        position_weight * position_weight * _norm(position_error) ** 2
        + _norm(orientation_error) ** 2
    )


def solve_inverse_kinematics(
    seed_positions,
    target_pose,
    maximum_iterations=120,
    position_tolerance_m=0.002,
    orientation_tolerance_rad=math.radians(1.0),
    damping=0.04,
    maximum_joint_step_rad=0.15,
    position_weight=4.0,
):
    """Solve a nearby six-dimensional pose with damped least squares."""
    target = tuple(float(value) for value in target_pose)
    if len(target) != 6 or not all(math.isfinite(value) for value in target):
        raise ValueError('target pose must contain six finite values')
    return solve_inverse_kinematics_transform(
        seed_positions,
        transform_matrix(target[:3], target[3:]),
        maximum_iterations=maximum_iterations,
        position_tolerance_m=position_tolerance_m,
        orientation_tolerance_rad=orientation_tolerance_rad,
        damping=damping,
        maximum_joint_step_rad=maximum_joint_step_rad,
        position_weight=position_weight,
    )


def solve_inverse_kinematics_transform(
    seed_positions,
    target_transform,
    maximum_iterations=120,
    position_tolerance_m=0.002,
    orientation_tolerance_rad=math.radians(1.0),
    damping=0.04,
    maximum_joint_step_rad=0.15,
    position_weight=4.0,
):
    """Solve a nearby homogeneous target with damped least squares."""
    seed = validate_position_limits(seed_positions)
    target_transform = tuple(
        tuple(float(value) for value in row)
        for row in target_transform
    )
    if (
        len(target_transform) != 4
        or any(len(row) != 4 for row in target_transform)
        or not all(
            math.isfinite(value)
            for row in target_transform
            for value in row
        )
    ):
        raise ValueError('target transform must be a finite 4 by 4 matrix')
    if maximum_iterations < 1:
        raise ValueError('maximum_iterations must be positive')
    positive = (
        position_tolerance_m,
        orientation_tolerance_rad,
        damping,
        maximum_joint_step_rad,
        position_weight,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('inverse-kinematics parameters must be positive')

    positions = seed
    position_error = (math.inf,) * 3
    orientation_error = (math.inf,) * 3

    for iteration in range(maximum_iterations + 1):
        current_transform = end_effector_transform(positions)
        position_error, orientation_error = pose_error(
            target_transform, current_transform
        )
        position_norm = _norm(position_error)
        orientation_norm = _norm(orientation_error)
        if (
            position_norm <= position_tolerance_m
            and orientation_norm <= orientation_tolerance_rad
        ):
            return IkResult(
                True,
                tuple(positions),
                iteration,
                position_norm,
                orientation_norm,
                'converged',
            )
        if iteration == maximum_iterations:
            break

        jacobian = _numerical_jacobian(
            positions, current_transform, 1.0e-5
        )
        weighted_jacobian = tuple(
            tuple(
                value * (position_weight if row < 3 else 1.0)
                for value in jacobian[row]
            )
            for row in range(6)
        )
        error = tuple(
            value * position_weight for value in position_error
        ) + orientation_error
        normal = [
            [
                sum(
                    weighted_jacobian[row][first]
                    * weighted_jacobian[row][second]
                    for row in range(6)
                ) + (damping * damping if first == second else 0.0)
                for second in range(6)
            ]
            for first in range(6)
        ]
        right = [
            sum(
                weighted_jacobian[row][column] * error[row]
                for row in range(6)
            )
            for column in range(6)
        ]
        try:
            step = _solve_linear(normal, right)
        except ValueError:
            break
        maximum_step = max(abs(value) for value in step)
        if maximum_step > maximum_joint_step_rad:
            scale = maximum_joint_step_rad / maximum_step
            step = tuple(value * scale for value in step)

        old_cost = _weighted_cost(
            position_error, orientation_error, position_weight
        )
        accepted = False
        scale = 1.0
        for _ in range(8):
            candidate = _bounded_positions(
                tuple(
                    value + scale * increment
                    for value, increment in zip(positions, step)
                ),
                seed,
            )
            candidate_error = pose_error(
                target_transform,
                end_effector_transform(candidate),
            )
            if _weighted_cost(*candidate_error, position_weight) < old_cost:
                positions = candidate
                accepted = True
                break
            scale *= 0.5
        if not accepted:
            break

    return IkResult(
        False,
        tuple(positions),
        maximum_iterations,
        _norm(position_error),
        _norm(orientation_error),
        'target is unreachable from the current local solution',
    )
