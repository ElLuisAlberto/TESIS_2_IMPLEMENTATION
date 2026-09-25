"""Deterministic construction of inflated JACO2 swept capsules."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple, cast

from thesis_core.jaco_kinematics import CAPSULE_RADII, capsule_segments


Point3 = Tuple[float, float, float]
JointConfiguration = Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class CapsuleAxis:
    """One inflated capsule axis and its spatial samples."""

    start: Point3
    end: Point3
    radius: float
    points: Tuple[Point3, ...]


@dataclass(frozen=True)
class SweptVolume:
    """Discrete future capsules grouped by time and segment."""

    samples: Tuple[Tuple[CapsuleAxis, ...], ...]
    current_capsules: Tuple[CapsuleAxis, ...]
    predicted_points: Tuple[Tuple[Point3, ...], ...]
    endpoint_traces: Tuple[Tuple[Point3, ...], ...]
    margin: float
    maximum_spacing: float


def _finite_point(values: Sequence[float], name: str) -> Point3:
    if len(values) != 3:
        raise ValueError(f'{name} must contain three values')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} contains a non-finite value')
    return cast(Point3, result)


def _configuration(values: Sequence[float]) -> JointConfiguration:
    if len(values) != 6:
        raise ValueError('each configuration must contain six joints')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError('configuration contains a non-finite value')
    return cast(JointConfiguration, result)


def sample_capsule_axis(
    start: Sequence[float],
    end: Sequence[float],
    maximum_spacing: float,
) -> Tuple[Point3, ...]:
    """Sample a segment including both endpoints with bounded spacing."""
    start_point = _finite_point(start, 'start')
    end_point = _finite_point(end, 'end')
    spacing = float(maximum_spacing)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('maximum_spacing must be positive and finite')

    length = math.dist(start_point, end_point)
    steps = max(1, math.ceil(length / spacing))
    intermediate = tuple(cast(Point3, tuple(
        start_point[axis]
        + (end_point[axis] - start_point[axis]) * index / steps
        for axis in range(3)
    )) for index in range(1, steps))
    return (start_point, *intermediate, end_point)


def build_swept_volume(
    configurations: Sequence[Sequence[float]],
    margin: float,
    maximum_spacing: float,
) -> SweptVolume:
    """Return the union samples of six inflated capsules over time."""
    if not configurations:
        raise ValueError('at least one configuration is required')
    inflated_margin = float(margin)
    spacing = float(maximum_spacing)
    if not math.isfinite(inflated_margin) or inflated_margin < 0.0:
        raise ValueError('margin must be finite and non-negative')
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('maximum_spacing must be positive and finite')

    temporal_samples = []
    for configuration in configurations:
        segments = capsule_segments(_configuration(configuration))
        if len(segments) != len(CAPSULE_RADII):
            raise ValueError('kinematics and capsule radii size mismatch')
        axes = tuple(
            CapsuleAxis(
                start=_finite_point(start, 'capsule start'),
                end=_finite_point(end, 'capsule end'),
                radius=float(radius) + inflated_margin,
                points=sample_capsule_axis(start, end, spacing),
            )
            for (start, end), radius in zip(segments, CAPSULE_RADII)
        )
        temporal_samples.append(axes)

    samples = tuple(temporal_samples)
    predicted_points = []
    endpoint_traces = []
    for segment_index in range(len(CAPSULE_RADII)):
        unique = {}
        trace = []
        for sample in samples:
            axis = sample[segment_index]
            trace.append(axis.end)
            for point in axis.points:
                key = tuple(round(value, 9) for value in point)
                unique.setdefault(key, point)
        predicted_points.append(tuple(unique.values()))
        endpoint_traces.append(tuple(trace))

    return SweptVolume(
        samples=samples,
        current_capsules=samples[0],
        predicted_points=tuple(predicted_points),
        endpoint_traces=tuple(endpoint_traces),
        margin=inflated_margin,
        maximum_spacing=spacing,
    )


def maximum_endpoint_travel(volume: SweptVolume) -> float:
    """Return the greatest accumulated endpoint travel of all links."""
    return max((
        sum(math.dist(first, second) for first, second in zip(
            trace, trace[1:]
        ))
        for trace in volume.endpoint_traces
    ), default=0.0)
