#!/usr/bin/env python3
"""Validate Package 4 geometric clearance evidence in a running ROS 2 graph."""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

from thesis_interfaces.msg import ExecutionControl
from thesis_interfaces.msg import ProximityStatus
from thesis_interfaces.msg import TrajectoryPrediction


TOLERANCE_M = 0.005
EXPECTED_COMBINATIONS = 6 * 21
EXPECTED_SAMPLE_COUNT = 21
EXPECTED_HORIZON_SEC = 1.0
EXPECTED_FRAME = 'j2n6s300_link_base'


@dataclass(frozen=True)
class ControlObservation:
    """One execution-control message paired with the latest proximity sample."""

    received_at: float
    control: ExecutionControl
    proximity: Optional[ProximityStatus]


class RuntimeValidator(Node):
    """Collect Package 4 messages without altering the safety pipeline."""

    def __init__(self) -> None:
        super().__init__('package4_runtime_validator')
        self.latest_proximity: Optional[ProximityStatus] = None
        self.proximities: List[ProximityStatus] = []
        self.controls: List[ControlObservation] = []
        self.predictions: List[TrajectoryPrediction] = []

        self.create_subscription(
            ProximityStatus,
            '/thesis/proximity_status',
            self._proximity_callback,
            20,
        )
        self.create_subscription(
            ExecutionControl,
            '/thesis/execution_control',
            self._control_callback,
            20,
        )
        self.create_subscription(
            TrajectoryPrediction,
            '/thesis/trajectory_prediction',
            self._prediction_callback,
            20,
        )

    def reset_capture(self) -> None:
        """Clear phase-local samples while retaining the last proximity value."""
        self.proximities.clear()
        self.controls.clear()
        self.predictions.clear()

    def _proximity_callback(self, message: ProximityStatus) -> None:
        self.latest_proximity = message
        self.proximities.append(message)

    def _control_callback(self, message: ExecutionControl) -> None:
        self.controls.append(
            ControlObservation(
                received_at=time.monotonic(),
                control=message,
                proximity=self.latest_proximity,
            )
        )

    def _prediction_callback(self, message: TrajectoryPrediction) -> None:
        self.predictions.append(message)


def point_values(point: object) -> Tuple[float, float, float]:
    """Return coordinates from a geometry_msgs/Point-like object."""
    return (float(point.x), float(point.y), float(point.z))


def all_finite(values: Sequence[float]) -> bool:
    """Return true only when every scalar is finite."""
    return all(math.isfinite(float(value)) for value in values)


def median_tail(values: Sequence[float], count: int = 10) -> float:
    """Return a robust value from the newest observations."""
    selected = list(values[-count:])
    if not selected:
        return math.nan
    return float(statistics.median(selected))


def third_medians(values: Sequence[float]) -> Tuple[float, float]:
    """Return medians for the first and last thirds of a capture."""
    if len(values) < 6:
        return (math.nan, math.nan)
    width = max(2, len(values) // 3)
    return (
        float(statistics.median(values[:width])),
        float(statistics.median(values[-width:])),
    )


def capture(
    node: RuntimeValidator,
    label: str,
    duration_sec: float,
) -> Tuple[List[ProximityStatus], List[ControlObservation]]:
    """Capture one named runtime phase."""
    node.reset_capture()
    print(f'CAPTURA_INICIADA={label} DURACION={duration_sec:.1f}s', flush=True)
    deadline = time.monotonic() + duration_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(
        f'CAPTURA_FINALIZADA={label} '
        f'PROXIMITY={len(node.proximities)} '
        f'CONTROL={len(node.controls)} '
        f'PREDICTION={len(node.predictions)}',
        flush=True,
    )
    return (list(node.proximities), list(node.controls))


def countdown(instruction: str) -> None:
    """Give the operator a deterministic preparation interval."""
    input(f'\n{instruction}\nPresiona Enter cuando estés listo: ')
    for remaining in range(3, 0, -1):
        print(f'INICIO_EN={remaining}', flush=True)
        time.sleep(1.0)


def print_check(name: str, passed: bool, detail: str = '') -> bool:
    """Print one machine-readable validation result."""
    suffix = f' {detail}' if detail else ''
    print(f'{"PASS" if passed else "FAIL"} {name}{suffix}')
    return passed


def validate_static(
    proximities: Sequence[ProximityStatus],
    observations: Sequence[ControlObservation],
) -> Tuple[bool, float]:
    """Validate the stationary k=0 and horizon evidence."""
    results: List[bool] = []
    results.append(print_check('PROXIMITY_MESSAGES', len(proximities) >= 10))
    results.append(print_check('CONTROL_MESSAGES', len(observations) >= 10))
    if not proximities or not observations:
        return (False, math.nan)

    controls = [item.control for item in observations]
    recent_controls = controls[-20:]
    recent_proximities = proximities[-20:]

    frames = sorted({message.reference_frame for message in recent_proximities})
    results.append(
        print_check(
            'COMMON_REFERENCE_FRAME',
            frames == [EXPECTED_FRAME],
            f'frames={frames}',
        )
    )

    combinations = sorted(
        {int(message.evaluated_combinations) for message in recent_controls}
    )
    results.append(
        print_check(
            'EVALUATED_COMBINATIONS_126',
            combinations == [EXPECTED_COMBINATIONS],
            f'values={combinations}',
        )
    )

    sample_counts = sorted(
        {int(message.minimum_sample_count) for message in recent_controls}
    )
    results.append(
        print_check(
            'SAMPLE_COUNT_21',
            sample_counts == [EXPECTED_SAMPLE_COUNT],
            f'values={sample_counts}',
        )
    )

    coherent = True
    maximum_time_error = 0.0
    for message in recent_controls:
        count = int(message.minimum_sample_count)
        index = int(message.minimum_sample_index)
        if count < 2 or index >= count:
            coherent = False
            continue
        expected_time = (
            EXPECTED_HORIZON_SEC * index / float(count - 1)
        )
        error = abs(float(message.minimum_time_from_now) - expected_time)
        maximum_time_error = max(maximum_time_error, error)
        if error > 1.0e-6:
            coherent = False
    results.append(
        print_check(
            'MINIMUM_INDEX_TIME_COHERENT',
            coherent,
            f'max_error={maximum_time_error:.9f}s',
        )
    )

    finite_evidence = all(
        all_finite(
            (
                message.minimum_clearance,
                message.current_clearance,
                message.nominal_clearance,
                message.supervised_clearance,
                message.minimum_time_from_now,
                message.capsule_radius,
                *point_values(message.closest_robot_point),
                *point_values(message.obstacle_center_at_minimum),
                *point_values(message.capsule_start),
                *point_values(message.capsule_end),
            )
        )
        for message in recent_controls
    )
    results.append(print_check('FINITE_GEOMETRIC_EVIDENCE', finite_evidence))

    equality_error = max(
        abs(
            float(message.minimum_clearance)
            - float(message.supervised_clearance)
        )
        for message in recent_controls
    )
    results.append(
        print_check(
            'MINIMUM_EQUALS_SUPERVISED',
            equality_error <= 1.0e-9,
            f'max_error={equality_error:.9f}m',
        )
    )

    paired_errors = [
        abs(
            float(item.control.current_clearance)
            - float(item.proximity.minimum_clearance)
        )
        for item in observations[-20:]
        if item.proximity is not None
    ]
    maximum_pair_error = max(paired_errors, default=math.inf)
    results.append(
        print_check(
            'K0_VS_PROXIMITY_WITHIN_0_005M',
            maximum_pair_error <= TOLERANCE_M,
            f'max_error={maximum_pair_error:.9f}m',
        )
    )

    velocity_norms = [
        math.sqrt(
            float(message.obstacle_velocity.x) ** 2
            + float(message.obstacle_velocity.y) ** 2
            + float(message.obstacle_velocity.z) ** 2
        )
        for message in recent_proximities
    ]
    maximum_velocity = max(velocity_norms, default=math.inf)
    results.append(
        print_check(
            'STATIC_OBSTACLE',
            maximum_velocity <= 1.0e-6,
            f'max_speed={maximum_velocity:.9f}m/s',
        )
    )

    segments = sorted(
        {message.limiting_segment for message in recent_controls}
    )
    results.append(
        print_check(
            'LIMITING_SEGMENT_PRESENT',
            bool(segments) and all(segments),
            f'segments={segments}',
        )
    )

    baseline = median_tail(
        [float(message.nominal_clearance) for message in recent_controls]
    )
    print(f'BASELINE_NOMINAL_CLEARANCE={baseline:.9f}m')
    return (all(results), baseline)


def validate_motion(
    label: str,
    observations: Sequence[ControlObservation],
    expect_decrease: bool,
) -> Tuple[bool, float]:
    """Validate clearance direction during one commanded movement."""
    values = [
        float(item.control.nominal_clearance)
        for item in observations
        if math.isfinite(float(item.control.nominal_clearance))
    ]
    if len(values) < 6:
        print_check(f'{label}_ENOUGH_MESSAGES', False, f'count={len(values)}')
        return (False, math.nan)

    first, last = third_medians(values)
    delta = last - first
    if expect_decrease:
        passed = delta < -1.0e-4
        check_name = f'{label}_REDUCES_D_MIN'
    else:
        passed = delta > 1.0e-4
        check_name = f'{label}_DOES_NOT_CREATE_FALSE_REDUCTION'
    print(f'{label}_FIRST_MEDIAN={first:.9f}m')
    print(f'{label}_LAST_MEDIAN={last:.9f}m')
    print(f'{label}_DELTA={delta:+.9f}m')
    return (print_check(check_name, passed), last)


def main() -> int:
    """Run stationary, approach and retreat validation phases."""
    rclpy.init()
    node = RuntimeValidator()
    checks: List[bool] = []
    try:
        countdown(
            'REPOSO: no pulses ningún JOG y mantén inmóvil el brazo.'
        )
        static_proximity, static_controls = capture(node, 'REPOSO', 5.0)
        static_ok, _ = validate_static(static_proximity, static_controls)
        checks.append(static_ok)

        countdown(
            'APROXIMACIÓN: mantén J2 NEGATIVO desde INICIO_EN=1 '
            'hasta CAPTURA_FINALIZADA.'
        )
        _, approach_controls = capture(node, 'APROXIMACION', 4.0)
        approach_ok, _ = validate_motion(
            'APPROACH',
            approach_controls,
            expect_decrease=True,
        )
        checks.append(approach_ok)

        countdown(
            'ALEJAMIENTO: mantén J2 POSITIVO desde INICIO_EN=1 '
            'hasta CAPTURA_FINALIZADA.'
        )
        _, retreat_controls = capture(node, 'ALEJAMIENTO', 4.0)
        retreat_ok, _ = validate_motion(
            'RETREAT',
            retreat_controls,
            expect_decrease=False,
        )
        checks.append(retreat_ok)

        print('\n=== RESULTADO AUTOMÁTICO ===')
        if all(checks):
            print('RESULTADO_VALIDACION_RUNTIME=PASS')
            return 0
        print('RESULTADO_VALIDACION_RUNTIME=FAIL')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
