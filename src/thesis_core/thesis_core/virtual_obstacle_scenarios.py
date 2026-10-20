"""Deterministic single-sphere scenarios for physical JACO2 validation."""

from dataclasses import dataclass
import math

from thesis_core.horizon_clearance import evaluate_joint_horizon
from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments
from thesis_core.joint_prediction import predict_joint_samples


SCENARIO_NAMES = ('FIXED', 'PARK', 'BLOCK_START', 'INSERT_STOP')
DISTAL_SEGMENT_INDEX = 5


@dataclass(frozen=True)
class ScenarioPoses:
    """World-frame centers for one sphere in every named scenario."""

    park: tuple
    block_start: tuple
    insert_stop: tuple


def _finite_vector(values, size, name):
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must contain {size} finite values')
    return result


def capsule_offset_pose(
    joint_positions,
    segment_index,
    obstacle_radius,
    desired_clearance,
    azimuth_deg,
    vertical_component,
):
    """Place a sphere at a documented offset from one capsule midpoint."""
    joints = _finite_vector(joint_positions, 6, 'joint_positions')
    index = int(segment_index)
    if not 0 <= index < len(CAPSULE_RADII):
        raise ValueError('segment_index is outside the capsule model')
    obstacle = float(obstacle_radius)
    clearance = float(desired_clearance)
    vertical = float(vertical_component)
    if (not math.isfinite(obstacle) or obstacle <= 0.0 or
            not math.isfinite(clearance) or clearance < 0.0 or
            not math.isfinite(vertical) or abs(vertical) >= 1.0):
        raise ValueError('invalid sphere offset geometry')
    angle = math.radians(float(azimuth_deg))
    horizontal = math.sqrt(1.0 - vertical * vertical)
    normal = (
        math.cos(angle) * horizontal,
        math.sin(angle) * horizontal,
        vertical,
    )
    start, end = capsule_segments(joints)[index]
    midpoint = tuple((first + second) * 0.5
                     for first, second in zip(start, end))
    distance = CAPSULE_RADII[index] + obstacle + clearance
    return tuple(midpoint[axis] + normal[axis] * distance
                 for axis in range(3))


def _derive_park_pose(
    predictions, obstacle_radius, warning_distance, target_clearance
):
    """Find a +X grid point near the requested surface clearance."""
    reference = predictions[0]
    # Align the search plane with the upper-arm endpoint, quantized to the
    # same 5 cm model grid used for reproducible scenario calibration.
    upper_arm_end = capsule_segments(reference.positions[0])[2][1]
    park_z = round(upper_arm_end[2] / 0.05) * 0.05
    required_projected = max(warning_distance + 0.05, target_clearance - 0.025)
    for step in range(1, 1001):
        center = (step * 0.01, 0.0, park_z)
        geometries = tuple(evaluate_joint_horizon(
            prediction.positions, prediction.sample_times, center,
            (0.0, 0.0, 0.0), obstacle_radius,
        ) for prediction in predictions)
        if (geometries[0].current.clearance >= target_clearance and
                all(geometry.minimum.clearance >= required_projected
                    for geometry in geometries)):
            return center
    raise ValueError('no PARK pose satisfies the required clearance')


def derive_physical_scenario_poses(
    measured_joints,
    sphere_radius=0.10,
    uncertainty=0.02,
    j1_delta_deg=3.0,
    stop_distance=0.05,
    warning_distance=0.30,
    park_clearance=0.40,
):
    """Derive PARK/start/final poses from measured joints and capsules."""
    current = _finite_vector(measured_joints, 6, 'measured_joints')
    raw_radius = float(sphere_radius)
    margin = float(uncertainty)
    stop = float(stop_distance)
    warning = float(warning_distance)
    park_target = float(park_clearance)
    if (not math.isfinite(raw_radius) or raw_radius <= 0.0 or
            not math.isfinite(margin) or margin < 0.0 or
            not math.isfinite(stop) or not math.isfinite(warning) or
            not math.isfinite(park_target) or
            not 0.0 < stop < warning < park_target):
        raise ValueError('sphere geometry and thresholds must be valid')
    positive_target = list(current)
    positive_target[0] += math.radians(float(j1_delta_deg))
    negative_target = list(current)
    negative_target[0] -= math.radians(float(j1_delta_deg))
    prediction = predict_joint_samples(
        current, positive_target, 1.0, 1.0, 25
    )
    negative_prediction = predict_joint_samples(
        current, negative_target, 1.0, 1.0, 25
    )
    effective_radius = raw_radius + margin
    park = _derive_park_pose(
        (prediction, negative_prediction), effective_radius,
        warning, park_target,
    )
    stop_clearance = 0.30 * stop

    # These two normals are deterministic winners of a 5-degree/vertical-grid
    # search against all six capsules. BLOCK minimizes at sample 0; INSERT at
    # sample 24/24. Coordinates are therefore derived, not picked in RViz.
    block_start = capsule_offset_pose(
        prediction.positions[0], DISTAL_SEGMENT_INDEX,
        effective_radius, stop_clearance, 195.0, 0.25,
    )
    insert_stop = capsule_offset_pose(
        prediction.positions[-1], DISTAL_SEGMENT_INDEX,
        effective_radius, stop_clearance, 10.0, 0.35,
    )
    return ScenarioPoses(
        park=park,
        block_start=block_start,
        insert_stop=insert_stop,
    )


def select_scenario_center(
    scenario, insert_stop_active, fixed, poses
):
    """Return the only sphere center for the requested scenario state."""
    name = str(scenario).strip().upper()
    if name not in SCENARIO_NAMES:
        raise ValueError(f'unknown obstacle scenario: {scenario!r}')
    if name == 'FIXED':
        return _finite_vector(fixed, 3, 'fixed pose')
    if name == 'PARK':
        return _finite_vector(poses.park, 3, 'PARK pose')
    if name == 'BLOCK_START':
        return _finite_vector(poses.block_start, 3, 'BLOCK_START pose')
    return _finite_vector(
        poses.insert_stop if insert_stop_active else poses.park,
        3,
        'INSERT_STOP pose',
    )
