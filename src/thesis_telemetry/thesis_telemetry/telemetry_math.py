"""Deterministic helpers for telemetry statistics and STOP detection."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


def percentile95(values: list[float]) -> float:
    """Return the linearly interpolated 95th percentile."""
    if not values:
        return -1.0
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = 0.95 * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


@dataclass
class RunningStats:
    """Accumulate finite observations and expose descriptive statistics."""

    values: list[float] = field(default_factory=list)

    def add(self, value: float) -> None:
        """Add one finite non-negative latency observation."""
        numeric = float(value)
        if math.isfinite(numeric) and numeric >= 0.0:
            self.values.append(numeric)

    def summary(self) -> tuple[int, float, float, float, float, float]:
        """Return count, mean, population stddev, min, max and p95."""
        if not self.values:
            return 0, -1.0, -1.0, -1.0, -1.0, -1.0
        count = len(self.values)
        mean = sum(self.values) / count
        variance = sum((value - mean) ** 2 for value in self.values) / count
        return (
            count,
            mean,
            math.sqrt(variance),
            min(self.values),
            max(self.values),
            percentile95(self.values),
        )


class ZeroVelocityDetector:
    """Confirm velocity zero only after consecutive qualifying samples."""

    def __init__(self, epsilon: float, required_samples: int) -> None:
        if not math.isfinite(epsilon) or epsilon <= 0.0:
            raise ValueError('epsilon must be finite and positive')
        if isinstance(required_samples, bool) or required_samples <= 0:
            raise ValueError('required_samples must be positive')
        self.epsilon = float(epsilon)
        self.required_samples = int(required_samples)
        self.count = 0

    def reset(self) -> None:
        """Discard the current confirmation sequence."""
        self.count = 0

    def update(self, velocities: tuple[float, ...]) -> bool:
        """Return true at and after the configured zero confirmation."""
        if not velocities or not all(math.isfinite(v) for v in velocities):
            self.count = 0
            return False
        if max(abs(value) for value in velocities) < self.epsilon:
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.required_samples


def stamp_to_ns(stamp) -> int:
    """Convert one builtin_interfaces/Time-like value to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def vector_text(values: tuple[float, ...]) -> str:
    """Serialize one numeric vector into a non-empty CSV-safe field."""
    return '[' + ';'.join(f'{value:.9f}' for value in values) + ']'
