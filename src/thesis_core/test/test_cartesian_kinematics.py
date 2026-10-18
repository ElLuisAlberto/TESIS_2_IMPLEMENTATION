import math

from thesis_core.cartesian_kinematics import (
    align_relative_target,
    pose_error,
    solve_inverse_kinematics,
    solve_inverse_kinematics_transform,
)
from thesis_core.jaco_kinematics import (
    end_effector_pose,
    end_effector_transform,
    rotation_matrix_to_rpy,
    transform_matrix,
    translation,
)


READY_JOINTS = tuple(
    math.radians(value)
    for value in (0.0, 150.0, 210.0, 0.0, 20.0, 0.0)
)

# A deterministic live-state perturbation around Gazebo's declared
# (0, pi, pi, 0, 0, 0) initial posture.  The exact nominal posture is a
# singular symmetry for first-order Y translation.
GAZEBO_INITIAL_JOINTS = (
    -3.5233447033367524e-05,
    3.141522823424578,
    3.141622840484401,
    -8.551274266649145e-05,
    7.176400861337844e-06,
    -2.686221661748289e-05,
)

# Joint state captured during the rejected Gazebo Z repetition.  This is
# effectively the fully extended initial posture and has no upward workspace
# for the requested 20 mm Cartesian step.
GAZEBO_Z_CAPTURED_JOINTS = (
    2.760754810700683e-09,
    3.141592655710413,
    3.141592682920455,
    -5.595139011173943e-07,
    -6.068647476774854e-07,
    -5.149753083753082e-07,
)


def pose_distance(first, second):
    return math.sqrt(sum(
        (left - right) ** 2
        for left, right in zip(first[:3], second[:3])
    ))


def test_exact_pose_returns_seed_without_iterations():
    pose = end_effector_pose(READY_JOINTS)
    result = solve_inverse_kinematics(READY_JOINTS, pose)
    assert result.success
    assert result.iterations == 0
    assert result.positions == READY_JOINTS


def test_nearby_six_dimensional_pose_converges():
    pose = end_effector_pose(READY_JOINTS)
    target = tuple(
        value + increment
        for value, increment in zip(
            pose,
            (
                0.01,
                -0.01,
                -0.01,
                math.radians(3.0),
                math.radians(2.0),
                math.radians(4.0),
            ),
        )
    )
    result = solve_inverse_kinematics(READY_JOINTS, target)
    achieved = end_effector_pose(result.positions)
    assert result.success
    assert pose_distance(target, achieved) <= 0.002
    assert result.orientation_error_rad <= math.radians(1.0)


def test_far_unreachable_pose_is_not_reported_as_success():
    pose = end_effector_pose(READY_JOINTS)
    target = (pose[0] + 2.0, *pose[1:])
    result = solve_inverse_kinematics(
        READY_JOINTS,
        target,
        maximum_iterations=30,
    )
    assert not result.success
    assert result.position_error_m > 0.10


def gazebo_initial_y_target():
    """Return a 20 mm Y target from the perturbed Gazebo initial state."""
    target = [
        row[:] for row in end_effector_transform(GAZEBO_INITIAL_JOINTS)
    ]
    target[1][3] += 0.020
    return target


def test_initial_y_target_fails_with_two_tenths_millimetre_tolerance():
    result = solve_inverse_kinematics_transform(
        GAZEBO_INITIAL_JOINTS,
        gazebo_initial_y_target(),
        position_tolerance_m=0.0002,
    )

    assert not result.success
    assert 0.0002 < result.position_error_m <= 0.0005


def test_initial_y_target_passes_with_half_millimetre_tolerance():
    result = solve_inverse_kinematics_transform(
        GAZEBO_INITIAL_JOINTS,
        gazebo_initial_y_target(),
        position_tolerance_m=0.0005,
    )

    assert result.success
    assert 0.0002 < result.position_error_m <= 0.0005


def test_initial_z_target_preserves_unreachable_rejection():
    target = [
        row[:] for row in end_effector_transform(GAZEBO_Z_CAPTURED_JOINTS)
    ]
    target[2][3] += 0.020

    result = solve_inverse_kinematics_transform(
        GAZEBO_Z_CAPTURED_JOINTS,
        target,
        position_tolerance_m=0.0005,
    )

    assert not result.success
    assert result.position_error_m > 0.010


def test_ready_pose_supports_twenty_millimetre_z_target():
    target = [row[:] for row in end_effector_transform(READY_JOINTS)]
    target[2][3] += 0.020

    result = solve_inverse_kinematics_transform(
        READY_JOINTS,
        target,
        position_tolerance_m=0.0005,
    )

    assert result.success
    assert result.position_error_m <= 0.0005
    assert result.orientation_error_rad <= math.radians(1.0)


def test_live_tf_offset_does_not_become_a_false_cartesian_step():
    current_model = end_effector_transform(READY_JOINTS)
    reference_offset = transform_matrix((0.0, 0.0, 0.311), (0.0, 0.0, 0.0))
    current_world = [row[:] for row in current_model]
    current_world[0][3] += reference_offset[0][3]
    current_world[1][3] += reference_offset[1][3]
    current_world[2][3] += reference_offset[2][3]
    current_world_rpy = rotation_matrix_to_rpy(current_world)
    target_world = transform_matrix(
        (
            translation(current_world)[0] + 0.01,
            translation(current_world)[1],
            translation(current_world)[2],
        ),
        (
            current_world_rpy[0],
            current_world_rpy[1],
            current_world_rpy[2] + math.radians(5.0),
        ),
    )

    target_model = align_relative_target(
        current_world,
        target_world,
        current_model,
    )
    position_error, orientation_error = pose_error(
        target_model,
        current_model,
    )
    assert math.isclose(position_error[0], 0.01, abs_tol=1.0e-9)
    assert math.isclose(position_error[1], 0.0, abs_tol=1.0e-9)
    assert math.isclose(position_error[2], 0.0, abs_tol=1.0e-9)
    assert math.isclose(
        math.sqrt(sum(value * value for value in orientation_error)),
        math.radians(5.0),
        abs_tol=1.0e-9,
    )

    result = solve_inverse_kinematics_transform(
        READY_JOINTS,
        target_model,
    )
    assert result.success
    assert result.position_error_m <= 0.002


def test_three_millimetre_tolerance_accepts_bounded_local_residual():
    target = [row[:] for row in end_effector_transform(READY_JOINTS)]
    target[0][3] += 0.0355

    strict = solve_inverse_kinematics_transform(
        READY_JOINTS,
        target,
        position_tolerance_m=0.002,
    )
    practical = solve_inverse_kinematics_transform(
        READY_JOINTS,
        target,
        position_tolerance_m=0.003,
    )

    assert not strict.success
    assert practical.success
    assert practical.position_error_m <= 0.003
