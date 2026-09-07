import math

from thesis_core.jaco_kinematics import (
    capsule_segments,
    minimum_sphere_clearance,
)


INITIAL_JOINTS = (0.0, math.pi, math.pi, 0.0, 0.0, 0.0)


def test_capsule_segment_count():
    assert len(capsule_segments(INITIAL_JOINTS)) == 6


def test_initial_pose_clearance_matches_tf_measurement():
    result = minimum_sphere_clearance(
        INITIAL_JOINTS,
        (0.60, 0.0, 0.65),
        0.12,
    )

    clearance, name, _, _, _ = result
    assert name == 'upper_arm_to_forearm'
    assert math.isclose(clearance, 0.395002, abs_tol=1.0e-5)
