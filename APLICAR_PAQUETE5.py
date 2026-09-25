#!/usr/bin/env python3
"""Apply Package 5 uncertainty and TTC changes to the local thesis workspace."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Callable, Dict


EXPECTED = {
    'src/thesis_core/thesis_core/horizon_clearance.py':
        'eec6ee005a3c6bd858ed3915e3de2107597e3a09372c71a410c1d4e8a94bec41',
    'src/thesis_core/thesis_core/safety_supervisor_node.py':
        'a2c81c0c47b6395d4aba257fb8fda93d31b4865c7f19140659863ab74d77b91f',
    'src/thesis_core/thesis_core/horizon_preview.py':
        '74bc6670d21a55d42a353f2352cb862d05bf864a5ebf665752df3a4e511898f2',
    'src/thesis_simulation/thesis_simulation/capsule_visualizer.py':
        '5a375cee7ad0a8c7f701114ba7554d1dda506caec6f3f7c4c1f0bb6eeda286ac',
    'src/thesis_interfaces/msg/ExecutionControl.msg':
        '0e9513d335f860f140b624fe35e1df8b72952d28cc67abaa33ea1ff26fd52855',
    'src/thesis_interfaces/msg/TrajectoryPrediction.msg':
        '8dd3640bfe65c877321f6c21dc23fc1cbfede20dd1a6ee28bfd29273b201536a',
}


HORIZON_CLEARANCE = '''"""Deterministic distance and crossing-time evaluation over a horizon."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from thesis_core.clearance_geometry import (
    Segment3,
    SegmentClearance,
    finite_point,
    minimum_configuration_clearance,
)
from thesis_core.jaco_kinematics import (
    CAPSULE_RADII,
    SEGMENT_NAMES,
    capsule_segments,
)


KINEMATIC_TF_TOLERANCE_M = 0.005
MODEL_UNCERTAINTY_M = 0.010
SAMPLING_UNCERTAINTY_M = 0.005
LATENCY_UNCERTAINTY_M = 0.005
DEFAULT_PROTECTIVE_MARGIN_M = (
    MODEL_UNCERTAINTY_M
    + SAMPLING_UNCERTAINTY_M
    + LATENCY_UNCERTAINTY_M
)
NO_EVENT_TIME = -1.0


@dataclass(frozen=True)
class HorizonClearance:
    """Global minimum, measured state and threshold-crossing times."""

    current: SegmentClearance
    minimum: SegmentClearance
    sample_count: int
    evaluated_combinations: int
    protective_margin: float
    minimum_protective_clearance: float
    first_protective_entry_time: Optional[float]
    first_collision_time: Optional[float]


def total_protective_margin(
    model_uncertainty: float,
    sampling_uncertainty: float,
    latency_uncertainty: float,
) -> float:
    """Validate and add the three uncertainty contributions."""
    components = tuple(float(value) for value in (
        model_uncertainty,
        sampling_uncertainty,
        latency_uncertainty,
    ))
    if not all(math.isfinite(value) and value >= 0.0 for value in components):
        raise ValueError('uncertainty components must be finite and non-negative')
    total = sum(components)
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError('total protective margin must be finite and positive')
    return total


def event_time_or_invalid(value: Optional[float]) -> float:
    """Represent an absent future event with the documented negative value."""
    return NO_EVENT_TIME if value is None else float(value)


def _sample_times(values: Sequence[float], count: int) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != count or count == 0:
        raise ValueError('sample_times must match the non-empty trajectory')
    if not all(math.isfinite(value) and value >= 0.0 for value in result):
        raise ValueError('sample_times must be finite and non-negative')
    if not math.isclose(result[0], 0.0, abs_tol=1.0e-12):
        raise ValueError('the first sample time must be zero')
    if any(end <= start for start, end in zip(result, result[1:])):
        raise ValueError('sample_times must be strictly increasing')
    return result


def interpolated_threshold_crossing_time(
    sample_times: Sequence[float],
    clearances: Sequence[float],
    threshold: float,
) -> Optional[float]:
    """Return the first downward threshold crossing using linear interpolation."""
    values = tuple(float(value) for value in clearances)
    times = _sample_times(sample_times, len(values))
    limit = float(threshold)
    if not math.isfinite(limit):
        raise ValueError('threshold must be finite')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('clearances must be finite')
    if values[0] <= limit:
        return times[0]
    for index in range(len(values) - 1):
        first = values[index]
        second = values[index + 1]
        if first > limit and second <= limit:
            denominator = first - second
            if denominator <= 0.0 or not math.isfinite(denominator):
                return times[index + 1]
            fraction = (first - limit) / denominator
            fraction = min(max(fraction, 0.0), 1.0)
            return times[index] + fraction * (times[index + 1] - times[index])
    return None


def evaluate_capsule_horizon(
    capsule_samples: Sequence[Sequence[Segment3]],
    sample_times: Sequence[float],
    capsule_radii: Sequence[float],
    segment_names: Sequence[str],
    obstacle_center: Sequence[float],
    obstacle_velocity: Sequence[float],
    obstacle_radius: float,
    protective_margin: float = DEFAULT_PROTECTIVE_MARGIN_M,
) -> HorizonClearance:
    """Evaluate every capsule/sample pair and both safety thresholds."""
    samples = tuple(tuple(sample) for sample in capsule_samples)
    times = _sample_times(sample_times, len(samples))
    center = finite_point(obstacle_center, 'obstacle center')
    velocity = finite_point(obstacle_velocity, 'obstacle velocity')
    names = tuple(segment_names)
    radii = tuple(capsule_radii)
    margin = float(protective_margin)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError('protective margin must be finite and non-negative')
    results = []
    for sample_index, (sample_time, segments) in enumerate(
        zip(times, samples)
    ):
        predicted_center = tuple(
            center[axis] + velocity[axis] * sample_time
            for axis in range(3)
        )
        results.append(minimum_configuration_clearance(
            segments,
            names,
            radii,
            predicted_center,
            obstacle_radius,
            sample_index,
            sample_time,
        ))
    clearances = tuple(result.clearance for result in results)
    minimum = min(results, key=lambda result: result.clearance)
    return HorizonClearance(
        current=results[0],
        minimum=minimum,
        sample_count=len(samples),
        evaluated_combinations=len(samples) * len(radii),
        protective_margin=margin,
        minimum_protective_clearance=minimum.clearance - margin,
        first_protective_entry_time=interpolated_threshold_crossing_time(
            times,
            clearances,
            margin,
        ),
        first_collision_time=interpolated_threshold_crossing_time(
            times,
            clearances,
            0.0,
        ),
    )


def evaluate_joint_horizon(
    joint_samples: Sequence[Sequence[float]],
    sample_times: Sequence[float],
    obstacle_center: Sequence[float],
    obstacle_velocity: Sequence[float],
    obstacle_radius: float,
    protective_margin: float = DEFAULT_PROTECTIVE_MARGIN_M,
) -> HorizonClearance:
    """Convert joint samples into capsule axes and evaluate the horizon."""
    samples = tuple(tuple(float(value) for value in sample)
                    for sample in joint_samples)
    if any(len(sample) != 6 for sample in samples):
        raise ValueError('every joint sample must contain six positions')
    capsule_samples = tuple(capsule_segments(sample) for sample in samples)
    return evaluate_capsule_horizon(
        capsule_samples,
        sample_times,
        CAPSULE_RADII,
        SEGMENT_NAMES,
        obstacle_center,
        obstacle_velocity,
        obstacle_radius,
        protective_margin,
    )
'''


TIMING_TESTS = '''"""Deterministic tests for Package 5 safety timing metrics."""

import math

import pytest

from thesis_core.horizon_clearance import (
    DEFAULT_PROTECTIVE_MARGIN_M,
    NO_EVENT_TIME,
    event_time_or_invalid,
    evaluate_capsule_horizon,
    interpolated_threshold_crossing_time,
    total_protective_margin,
)


def test_uncertainty_components_sum_to_initial_margin():
    margin = total_protective_margin(0.010, 0.005, 0.005)
    assert margin == pytest.approx(0.020)
    assert margin == pytest.approx(DEFAULT_PROTECTIVE_MARGIN_M)


def test_interpolates_crossing_between_samples():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05),
        (0.04, -0.01),
        0.02,
    )
    assert crossing == pytest.approx(0.02)


def test_start_inside_threshold_returns_zero():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05),
        (0.01, -0.01),
        0.02,
    )
    assert crossing == 0.0


def test_moving_away_has_no_future_event():
    crossing = interpolated_threshold_crossing_time(
        (0.0, 0.05, 0.10),
        (0.10, 0.15, 0.20),
        0.02,
    )
    assert crossing is None
    assert event_time_or_invalid(crossing) == NO_EVENT_TIME


def test_protective_entry_precedes_geometric_collision():
    times = (0.0, 1.0, 2.0, 3.0)
    clearances = (0.30, 0.20, 0.10, 0.0)
    protective = interpolated_threshold_crossing_time(
        times, clearances, 0.15
    )
    collision = interpolated_threshold_crossing_time(
        times, clearances, 0.0
    )
    assert protective == pytest.approx(1.5)
    assert collision == pytest.approx(3.0)
    assert protective < collision


def test_reduction_delays_or_removes_protective_entry():
    times = (0.0, 0.5, 1.0)
    nominal = (0.10, 0.01, -0.05)
    supervised = (0.10, 0.08, 0.06)
    nominal_ttc = interpolated_threshold_crossing_time(
        times, nominal, 0.02
    )
    supervised_ttc = interpolated_threshold_crossing_time(
        times, supervised, 0.02
    )
    assert nominal_ttc is not None
    assert supervised_ttc is None


def test_ttc_decreases_for_a_faster_uniform_approach():
    times = (0.0, 0.5, 1.0)
    slow = interpolated_threshold_crossing_time(
        times, (0.10, 0.04, -0.02), 0.02
    )
    fast = interpolated_threshold_crossing_time(
        times, (0.10, 0.00, -0.10), 0.02
    )
    assert slow is not None
    assert fast is not None
    assert fast < slow


def test_increasing_sample_count_reduces_interpolation_error():
    exact = math.sqrt(0.8)
    errors = []
    for count in (21, 41, 81):
        times = tuple(index / float(count - 1) for index in range(count))
        clearances = tuple(1.0 - value * value for value in times)
        crossing = interpolated_threshold_crossing_time(
            times, clearances, 0.2
        )
        assert crossing is not None
        errors.append(abs(crossing - exact))
    assert errors[2] <= errors[1] <= errors[0]
    assert errors[2] < 1.0e-4


def test_horizon_reports_interpolated_protective_and_collision_times():
    segment = (((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),)
    result = evaluate_capsule_horizon(
        (segment, segment),
        (0.0, 1.0),
        (0.10,),
        ('test_segment',),
        (0.5, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        0.10,
        0.02,
    )
    assert result.first_protective_entry_time == pytest.approx(0.78)
    assert result.first_collision_time == pytest.approx(0.80)
    assert result.minimum_protective_clearance == pytest.approx(-0.22)
'''


def digest_text(text: str) -> str:
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{label}: expected one match, found {count}')
    return text.replace(old, new, 1)


def transform_supervisor(text: str) -> str:
    text = replace_once(
        text,
        '''from thesis_core.horizon_clearance import (
    KINEMATIC_TF_TOLERANCE_M,
    evaluate_joint_horizon,
)
''',
        '''from thesis_core.horizon_clearance import (
    KINEMATIC_TF_TOLERANCE_M,
    LATENCY_UNCERTAINTY_M,
    MODEL_UNCERTAINTY_M,
    NO_EVENT_TIME,
    SAMPLING_UNCERTAINTY_M,
    evaluate_joint_horizon,
    event_time_or_invalid,
    total_protective_margin,
)
''',
        'horizon timing imports',
    )
    text = replace_once(
        text,
        "        self.declare_parameter('predictive_margin_m', 0.02)\n",
        "        self.declare_parameter('predictive_margin_m', 0.02)\n"
        "        self.declare_parameter(\n"
        "            'model_uncertainty_m', MODEL_UNCERTAINTY_M\n"
        "        )\n"
        "        self.declare_parameter(\n"
        "            'sampling_uncertainty_m', SAMPLING_UNCERTAINTY_M\n"
        "        )\n"
        "        self.declare_parameter(\n"
        "            'latency_uncertainty_m', LATENCY_UNCERTAINTY_M\n"
        "        )\n",
        'uncertainty parameters',
    )
    text = replace_once(
        text,
        '''        if (not math.isfinite(tf_tolerance) or
                not 0.0 < tf_tolerance <= 0.05):
            raise ValueError(
                'kinematic_tf_tolerance_m must be in (0, 0.05]'
            )
        self.runtime_timer = None
''',
        '''        if (not math.isfinite(tf_tolerance) or
                not 0.0 < tf_tolerance <= 0.05):
            raise ValueError(
                'kinematic_tf_tolerance_m must be in (0, 0.05]'
            )
        self.protective_margin()
        self.runtime_timer = None
''',
        'startup margin validation',
    )
    text = replace_once(
        text,
        '''    @staticmethod
    def message_time_seconds(value):
        return float(value.sec) + float(value.nanosec) * 1e-9

    def runtime_obstacle(self):
''',
        '''    @staticmethod
    def message_time_seconds(value):
        return float(value.sec) + float(value.nanosec) * 1e-9

    def protective_margin(self):
        """Return the documented sum of modeling, sampling and latency."""
        total = total_protective_margin(
            self.get_parameter('model_uncertainty_m').value,
            self.get_parameter('sampling_uncertainty_m').value,
            self.get_parameter('latency_uncertainty_m').value,
        )
        configured = float(
            self.get_parameter('predictive_margin_m').value
        )
        if (not math.isfinite(configured) or
                not math.isclose(total, configured, abs_tol=1.0e-12)):
            raise ValueError(
                'predictive_margin_m must equal the uncertainty sum'
            )
        return total

    def runtime_obstacle(self):
''',
        'protective margin method',
    )
    text = replace_once(
        text,
        '''        geometry=None,
        current_clearance=math.nan,
        nominal_clearance=math.nan,
    ):
''',
        '''        geometry=None,
        nominal_geometry=None,
        current_clearance=math.nan,
        nominal_clearance=math.nan,
    ):
''',
        'execution timing arguments',
    )
    text = replace_once(
        text,
        '''        control.minimum_clearance = float(clearance)
        control.limiting_segment = segment
        control.time_to_collision = float(time_to_collision)
        control.minimum_time_from_now = float(minimum_time_from_now)
''',
        '''        control.minimum_clearance = float(clearance)
        control.limiting_segment = segment
        control.protective_margin = self.protective_margin()
        control.time_to_protective_volume = NO_EVENT_TIME
        control.time_to_collision = NO_EVENT_TIME
        control.supervised_time_to_protective_volume = NO_EVENT_TIME
        control.supervised_time_to_collision = NO_EVENT_TIME
        if nominal_geometry is not None:
            control.time_to_protective_volume = event_time_or_invalid(
                nominal_geometry.first_protective_entry_time
            )
            control.time_to_collision = event_time_or_invalid(
                nominal_geometry.first_collision_time
            )
        if geometry is not None:
            control.supervised_time_to_protective_volume = (
                event_time_or_invalid(
                    geometry.first_protective_entry_time
                )
            )
            control.supervised_time_to_collision = event_time_or_invalid(
                geometry.first_collision_time
            )
        control.minimum_time_from_now = float(minimum_time_from_now)
''',
        'execution timing fields',
    )
    text = replace_once(
        text,
        '''        result = evaluate_joint_horizon(
            samples,
            sample_times,
            obstacle_center,
            obstacle_velocity,
            obstacle_radius,
        )
''',
        '''        result = evaluate_joint_horizon(
            samples,
            sample_times,
            obstacle_center,
            obstacle_velocity,
            obstacle_radius,
            self.protective_margin(),
        )
''',
        'runtime evaluator margin',
    )
    for label, old, new in (
        (
            'candidate nominal margin',
            '''        nominal_geometry = evaluate_joint_horizon(
            joint_prediction.positions,
            joint_prediction.sample_times,
            obstacle,
            obstacle_velocity,
            obstacle_radius,
        )
''',
            '''        nominal_geometry = evaluate_joint_horizon(
            joint_prediction.positions,
            joint_prediction.sample_times,
            obstacle,
            obstacle_velocity,
            obstacle_radius,
            self.protective_margin(),
        )
''',
        ),
        (
            'candidate stop margin',
            '''            supervised_geometry = evaluate_joint_horizon(
                supervised_samples,
                joint_prediction.sample_times,
                obstacle,
                obstacle_velocity,
                obstacle_radius,
            )
''',
            '''            supervised_geometry = evaluate_joint_horizon(
                supervised_samples,
                joint_prediction.sample_times,
                obstacle,
                obstacle_velocity,
                obstacle_radius,
                self.protective_margin(),
            )
''',
        ),
        (
            'candidate supervised margin',
            '''            supervised_geometry = evaluate_joint_horizon(
                supervised_prediction.positions,
                supervised_prediction.sample_times,
                obstacle,
                obstacle_velocity,
                obstacle_radius,
            )
''',
            '''            supervised_geometry = evaluate_joint_horizon(
                supervised_prediction.positions,
                supervised_prediction.sample_times,
                obstacle,
                obstacle_velocity,
                obstacle_radius,
                self.protective_margin(),
            )
''',
        ),
    ):
        text = replace_once(text, old, new, label)
    text = replace_once(
        text,
        '''        prediction.supervised_clearance = (
            supervised_geometry.minimum.clearance
        )
        prediction.closest_robot_point.x = witness.closest_robot_point[0]
''',
        '''        prediction.supervised_clearance = (
            supervised_geometry.minimum.clearance
        )
        prediction.protective_margin = nominal_geometry.protective_margin
        prediction.time_to_protective_volume = event_time_or_invalid(
            nominal_geometry.first_protective_entry_time
        )
        prediction.time_to_collision = event_time_or_invalid(
            nominal_geometry.first_collision_time
        )
        prediction.supervised_time_to_protective_volume = (
            event_time_or_invalid(
                supervised_geometry.first_protective_entry_time
            )
        )
        prediction.supervised_time_to_collision = event_time_or_invalid(
            supervised_geometry.first_collision_time
        )
        prediction.closest_robot_point.x = witness.closest_robot_point[0]
''',
        'candidate timing fields',
    )
    text = text.replace(
        '''            geometry=supervised_geometry,
            current_clearance=current_clearance,
            nominal_clearance=nominal_geometry.minimum.clearance,
''',
        '''            geometry=supervised_geometry,
            nominal_geometry=nominal_geometry,
            current_clearance=current_clearance,
            nominal_clearance=nominal_geometry.minimum.clearance,
''',
    )
    if text.count('nominal_geometry=nominal_geometry,') != 2:
        raise RuntimeError('expected two nominal geometry publications')
    text = replace_once(
        text,
        "            f'TTC={collision_time:.3f} s'\n",
        "            f'TTC_nom={event_time_or_invalid(\n"
        "                nominal_geometry.first_collision_time\n"
        "            ):.3f} s'\n",
        'runtime nominal TTC log',
    )
    return text


def transform_execution_control(text: str) -> str:
    text = replace_once(
        text,
        '''float64 minimum_clearance
string limiting_segment
float64 time_to_collision
''',
        '''float64 minimum_clearance
string limiting_segment

# All absent events use -1.0. Nominal TTC is never overwritten by scaling.
float64 protective_margin
float64 time_to_protective_volume
float64 time_to_collision
float64 supervised_time_to_protective_volume
float64 supervised_time_to_collision
''',
        'execution timing interface',
    )
    return text


def transform_trajectory_prediction(text: str) -> str:
    return text + '''

# Package 5 uncertainty and TTC evidence. Absent events use -1.0.
float64 protective_margin
float64 time_to_protective_volume
float64 time_to_collision
float64 supervised_time_to_protective_volume
float64 supervised_time_to_collision
'''


def transform_horizon_preview(text: str) -> str:
    text = replace_once(
        text,
        '''        return (
            float(self.control.minimum_time_from_now),
            sample_number,
            sample_count,
        )
''',
        '''        return (
            float(self.control.minimum_time_from_now),
            sample_number,
            sample_count,
            float(self.control.protective_margin),
            float(self.control.time_to_protective_volume),
            float(self.control.time_to_collision),
        )
''',
        'runtime timing metadata',
    )
    text = replace_once(
        text,
        '''            minimum_time, sample_number, sample_count = runtime_minimum
            minimum_text = (
                f'\\nt_min=+{minimum_time:.2f}s '
                f'muestra={sample_number}/{sample_count}'
            )
''',
        '''            (
                minimum_time,
                sample_number,
                sample_count,
                protective_margin,
                protective_ttc,
                collision_ttc,
            ) = runtime_minimum
            protective_text = (
                'n/a' if protective_ttc < 0.0
                else f'{protective_ttc:.2f}s'
            )
            collision_text = (
                'n/a' if collision_ttc < 0.0
                else f'{collision_ttc:.2f}s'
            )
            minimum_text = (
                f'\\nt_min=+{minimum_time:.2f}s '
                f'muestra={sample_number}/{sample_count}; '
                f'margen={protective_margin:.3f}m; '
                f'TTCp={protective_text}; TTCc={collision_text}'
            )
''',
        'horizon timing label',
    )
    return text


def transform_capsule_visualizer(text: str) -> str:
    return replace_once(
        text,
        '''        label.scale.z = 0.055
        label.text = (
            f'CANDIDATO {prediction.state} | '
            f'd={prediction.minimum_clearance:.3f} m | '
            f't={prediction.minimum_time_from_now:.2f} s | '
            f'{prediction.trajectory_fraction * 100.0:.0f}% trayectoria'
        )
''',
        '''        label.scale.z = 0.055
        protective_ttc = (
            'n/a'
            if prediction.time_to_protective_volume < 0.0
            else f'{prediction.time_to_protective_volume:.2f}s'
        )
        collision_ttc = (
            'n/a'
            if prediction.time_to_collision < 0.0
            else f'{prediction.time_to_collision:.2f}s'
        )
        label.text = (
            f'CANDIDATO {prediction.state} | '
            f'd={prediction.minimum_clearance:.3f} m | '
            f't_min={prediction.minimum_time_from_now:.2f}s | '
            f'm={prediction.protective_margin:.3f}m | '
            f'TTCp={protective_ttc} | TTCc={collision_ttc}'
        )
''',
        'candidate timing label',
    )


TRANSFORMS: Dict[str, Callable[[str], str]] = {
    'src/thesis_core/thesis_core/horizon_clearance.py':
        lambda _: HORIZON_CLEARANCE,
    'src/thesis_core/thesis_core/safety_supervisor_node.py':
        transform_supervisor,
    'src/thesis_core/thesis_core/horizon_preview.py':
        transform_horizon_preview,
    'src/thesis_simulation/thesis_simulation/capsule_visualizer.py':
        transform_capsule_visualizer,
    'src/thesis_interfaces/msg/ExecutionControl.msg':
        transform_execution_control,
    'src/thesis_interfaces/msg/TrajectoryPrediction.msg':
        transform_trajectory_prediction,
}


def atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.',
        dir=str(path.parent),
        text=True,
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main() -> int:
    root = Path.cwd()
    if not (root / 'src').is_dir():
        print('ERROR: run this script from the workspace root', file=sys.stderr)
        return 2
    test_path = root / 'src/thesis_core/test/test_safety_timing.py'
    if test_path.exists():
        print(f'ERROR: new test already exists: {test_path}', file=sys.stderr)
        return 2

    originals: Dict[str, str] = {}
    outputs: Dict[str, str] = {}
    for relative, expected in EXPECTED.items():
        path = root / relative
        if not path.is_file():
            print(f'ERROR: missing {relative}', file=sys.stderr)
            return 2
        text = path.read_text(encoding='utf-8')
        actual = digest_text(text)
        if actual != expected:
            print(
                f'ERROR: {relative} changed; expected {expected}, got {actual}',
                file=sys.stderr,
            )
            return 2
        originals[relative] = text
        outputs[relative] = TRANSFORMS[relative](text)

    stamp = __import__('datetime').datetime.now().strftime('%Y%m%d_%H%M%S')
    backup_root = Path.home() / 'RESPALDO_TESIS_PAQUETE5' / stamp
    written = []
    try:
        for relative, original in originals.items():
            backup = backup_root / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            backup.write_text(original, encoding='utf-8')
        for relative, output in outputs.items():
            atomic_write(root / relative, output)
            written.append(relative)
        atomic_write(test_path, TIMING_TESTS)
        written.append(str(test_path.relative_to(root)))
    except BaseException as exc:
        for relative in written:
            target = root / relative
            if relative in originals:
                atomic_write(target, originals[relative])
            elif target.exists():
                target.unlink()
        print(f'ERROR: application rolled back: {exc}', file=sys.stderr)
        return 1

    print(f'BACKUP={backup_root}')
    for relative in written:
        path = root / relative
        print(f'UPDATED {digest_text(path.read_text(encoding="utf-8"))} {relative}')
    print('RESULTADO_APLICACION=PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
