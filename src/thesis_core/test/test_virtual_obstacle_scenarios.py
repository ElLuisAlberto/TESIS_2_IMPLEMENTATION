import math

from thesis_core.horizon_clearance import evaluate_joint_horizon
from thesis_core.joint_prediction import predict_joint_samples
from thesis_core.safety_policy import SafetyPolicyInput
from thesis_core.safety_policy import decide_preventive_state
from thesis_core.virtual_obstacle_scenarios import ScenarioPoses
from thesis_core.virtual_obstacle_scenarios import derive_physical_scenario_poses
from thesis_core.virtual_obstacle_scenarios import select_scenario_center


MEASURED_JOINTS = (
    -1.551814113493139,
    2.7496557685950496,
    4.1168651479621845,
    5.844626591509698,
    5.973422388148109,
    2.0808752988032992,
)
EFFECTIVE_RADIUS = 0.12


def _horizon(center, delta_deg=3.0):
    target = list(MEASURED_JOINTS)
    target[0] += math.radians(delta_deg)
    prediction = predict_joint_samples(
        MEASURED_JOINTS, target, 1.0, 1.0, 25
    )
    return evaluate_joint_horizon(
        prediction.positions,
        prediction.sample_times,
        center,
        (0.0, 0.0, 0.0),
        EFFECTIVE_RADIUS,
        0.02,
    )


def _decision(geometry):
    return decide_preventive_state(SafetyPolicyInput(
        current_clearance=geometry.current.clearance,
        nominal_clearance=geometry.minimum.clearance,
        supervised_clearance=geometry.minimum.clearance,
        protective_margin=0.02,
        stop_distance=0.05,
        warning_distance=0.30,
        minimum_scale=0.05,
        candidate_scale=(
            1.0 if geometry.minimum.clearance >= 0.02 else 0.0
        ),
        withdrawal_safe=False,
    ))


def test_calibrated_single_sphere_scenarios_have_unambiguous_states():
    poses = derive_physical_scenario_poses(MEASURED_JOINTS)

    park = _horizon(poses.park)
    reverse_park = _horizon(poses.park, -3.0)
    assert 0.39 <= park.current.clearance <= 0.41
    assert park.minimum.clearance > 0.375
    assert reverse_park.minimum.clearance > 0.375
    assert _decision(park).state == 'ALLOW'
    assert _decision(reverse_park).state == 'ALLOW'

    block = _horizon(poses.block_start)
    assert 0.0 < block.current.clearance < 0.05
    assert block.minimum.sample_index == 0
    assert _decision(block).state == 'STOP'

    inserted = _horizon(poses.insert_stop)
    assert 0.0 < inserted.current.clearance < 0.05
    assert inserted.minimum.clearance < 0.05
    assert inserted.minimum.sample_index == 24
    assert _decision(inserted).state == 'STOP'


def test_insert_stop_stays_parked_until_explicit_insertion():
    poses = derive_physical_scenario_poses(MEASURED_JOINTS)
    fixed = (0.6, 0.0, 0.65)
    assert select_scenario_center('PARK', False, fixed, poses) == poses.park
    assert select_scenario_center(
        'BLOCK_START', False, fixed, poses
    ) == poses.block_start
    assert select_scenario_center(
        'INSERT_STOP', False, fixed, poses
    ) == poses.park
    assert select_scenario_center(
        'INSERT_STOP', True, fixed, poses
    ) == poses.insert_stop


def test_calibration_matches_committed_reproducible_poses():
    poses = derive_physical_scenario_poses(MEASURED_JOINTS)
    committed = ScenarioPoses(
        park=(0.59, 0.0, 0.65),
        block_start=(
            -0.2541344553253112,
            0.47314710635248247,
            0.7877059751122166,
        ),
        insert_stop=(
            0.19163771787018302,
            0.5676933085630497,
            0.8102059751122166,
        ),
    )
    for calculated, expected in zip(
            (poses.park, poses.block_start, poses.insert_stop),
            (committed.park, committed.block_start, committed.insert_stop)):
        assert all(math.isclose(actual, reference, abs_tol=1.0e-15)
                   for actual, reference in zip(calculated, expected))


def test_calibration_rejects_invalid_safety_thresholds():
    try:
        derive_physical_scenario_poses(
            MEASURED_JOINTS, stop_distance=0.30, warning_distance=0.30
        )
    except ValueError as exc:
        assert 'thresholds' in str(exc)
    else:
        raise AssertionError('invalid thresholds must be rejected')
