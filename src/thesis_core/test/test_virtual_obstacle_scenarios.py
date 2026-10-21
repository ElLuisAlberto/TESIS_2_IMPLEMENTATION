import math

from thesis_core.horizon_clearance import evaluate_joint_horizon
from thesis_core.joint_prediction import predict_joint_samples
from thesis_core.safety_policy import SafetyPolicyInput
from thesis_core.safety_policy import decide_preventive_state
from thesis_core.virtual_obstacle_scenarios import ScenarioPoses
from thesis_core.virtual_obstacle_scenarios import derive_physical_scenario_poses
from thesis_core.virtual_obstacle_scenarios import select_scenario_center


MEASURED_JOINTS = (
    2.2334760271614935,
    2.483514895586853,
    1.0155264405650484,
    5.740923098662012,
    5.969486236056973,
    2.079310957978987,
)
EFFECTIVE_RADIUS = 0.12
REDUCTION_POSE = (
    -0.021236321380452394,
    0.3137637299440237,
    0.5083259709067948,
)


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
    assert 0.80 <= park.current.clearance <= 0.82
    assert park.minimum.clearance > 0.80
    assert reverse_park.minimum.clearance > 0.80
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


def test_calibrated_reduction_probe_is_unambiguous():
    geometry = _horizon(REDUCTION_POSE)

    assert 0.09 < geometry.current.clearance < 0.11
    assert 0.09 < geometry.minimum.clearance < 0.11
    assert _decision(geometry).state == 'WARNING'

    predictive = decide_preventive_state(SafetyPolicyInput(
        current_clearance=geometry.current.clearance,
        nominal_clearance=0.015,
        supervised_clearance=0.020,
        protective_margin=0.02,
        stop_distance=0.05,
        warning_distance=0.30,
        minimum_scale=0.05,
        candidate_scale=0.50,
    ))
    assert predictive.state == 'REDUCTION'
    assert predictive.reason_code == 'PREDICTIVE_INTRUSION'


def test_calibration_matches_committed_reproducible_poses():
    poses = derive_physical_scenario_poses(MEASURED_JOINTS)
    committed = ScenarioPoses(
        park=(1.22, 0.0, 0.60),
        block_start=(
            0.017715572026672233,
            0.24629707151187663,
            0.5423259709067948,
        ),
        insert_stop=(
            0.23442858697668922,
            -0.12962618026812636,
            0.6773259709067948,
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
