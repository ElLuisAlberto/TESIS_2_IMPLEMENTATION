"""Deterministic temporal stabilization for preventive control decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


STATE_SEVERITY = {
    'ALLOW': 0,
    'WARNING': 1,
    'REDUCTION': 2,
    'STOP': 3,
}


@dataclass(frozen=True)
class StabilityResult:
    """Requested and filtered decision plus recovery diagnostics."""

    requested_state: str
    requested_speed_scale: float
    state: str
    speed_scale: float
    recovery_count: int
    recovery_required_count: int
    transition: bool
    transition_reason: str


class ControlStabilityFilter:
    """Apply restrictions immediately and recover only after confirmation."""

    def __init__(
        self,
        recovery_required_samples: int = 5,
        maximum_scale_increment: float = 0.10,
        recovery_sample_period_sec: float = 0.10,
    ) -> None:
        if (
            isinstance(recovery_required_samples, bool)
            or not isinstance(recovery_required_samples, int)
            or recovery_required_samples <= 0
        ):
            raise ValueError('recovery_required_samples must be positive')
        increment = float(maximum_scale_increment)
        period = float(recovery_sample_period_sec)
        if not math.isfinite(increment) or not 0.0 < increment <= 1.0:
            raise ValueError('maximum_scale_increment must be in (0, 1]')
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError('recovery_sample_period_sec must be positive')

        self.recovery_required_samples = recovery_required_samples
        self.maximum_scale_increment = increment
        self.recovery_sample_period_sec = period
        self.command_id: Optional[str] = None
        self.state: Optional[str] = None
        self.speed_scale: Optional[float] = None
        self.recovery_count = 0
        self.recovery_confirmed = False
        self.last_recovery_sample_sec: Optional[float] = None
        self.last_reset_reason = 'INITIAL'

    def reset(self, reason: str = 'EXTERNAL_RESET') -> None:
        """Forget temporal history without manufacturing a new decision."""
        self.command_id = None
        self.state = None
        self.speed_scale = None
        self.recovery_count = 0
        self.recovery_confirmed = False
        self.last_recovery_sample_sec = None
        self.last_reset_reason = str(reason)

    @staticmethod
    def _validated_request(
        state: str,
        speed_scale: float,
    ) -> tuple[str, float]:
        requested_state = str(state)
        if requested_state not in STATE_SEVERITY:
            raise ValueError(f'unknown safety state: {requested_state}')
        requested_scale = float(speed_scale)
        if not math.isfinite(requested_scale):
            raise ValueError('speed_scale must be finite')
        if requested_state == 'STOP':
            return requested_state, 0.0
        if not 0.0 < requested_scale <= 1.0:
            raise ValueError('non-STOP speed_scale must be in (0, 1]')
        if requested_state in {'ALLOW', 'WARNING'} and not math.isclose(
            requested_scale,
            1.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError('ALLOW and WARNING require speed_scale=1')
        return requested_state, requested_scale

    @staticmethod
    def _validated_time(now_sec: float) -> float:
        value = float(now_sec)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError('now_sec must be finite and non-negative')
        return value

    def _result(
        self,
        requested_state: str,
        requested_scale: float,
        transition: bool,
        reason: str,
    ) -> StabilityResult:
        if self.state is None or self.speed_scale is None:
            raise RuntimeError('stability filter has no current decision')
        return StabilityResult(
            requested_state=requested_state,
            requested_speed_scale=requested_scale,
            state=self.state,
            speed_scale=self.speed_scale,
            recovery_count=self.recovery_count,
            recovery_required_count=self.recovery_required_samples,
            transition=transition,
            transition_reason=reason,
        )

    def _recovery_tick(self, now_sec: float) -> bool:
        if self.last_recovery_sample_sec is None:
            self.last_recovery_sample_sec = now_sec
            return True
        elapsed = now_sec - self.last_recovery_sample_sec
        if elapsed < -1.0e-9:
            self.recovery_count = 0
            self.recovery_confirmed = False
            self.last_recovery_sample_sec = now_sec
            return True
        if elapsed + 1.0e-9 < self.recovery_sample_period_sec:
            return False
        self.last_recovery_sample_sec = now_sec
        return True

    def update(
        self,
        command_id: str,
        requested_state: str,
        requested_speed_scale: float,
        now_sec: float,
    ) -> StabilityResult:
        """Filter one decision sample while preserving fail-safe priority."""
        state, scale = self._validated_request(
            requested_state,
            requested_speed_scale,
        )
        now = self._validated_time(now_sec)
        identifier = str(command_id)
        if not identifier:
            raise ValueError('command_id must not be empty')

        if self.command_id != identifier or self.state is None:
            self.command_id = identifier
            self.state = state
            self.speed_scale = scale
            self.recovery_count = 0
            self.recovery_confirmed = False
            self.last_recovery_sample_sec = None
            return self._result(
                state,
                scale,
                True,
                'COMMAND_CHANGED_RESET',
            )

        if state == 'STOP':
            transition = self.state != 'STOP' or self.speed_scale != 0.0
            self.state = 'STOP'
            self.speed_scale = 0.0
            self.recovery_count = 0
            self.recovery_confirmed = False
            self.last_recovery_sample_sec = None
            return self._result(
                state,
                scale,
                transition,
                'RESTRICTION_IMMEDIATE' if transition else 'STOP_LATCHED',
            )

        assert self.speed_scale is not None
        current_severity = STATE_SEVERITY[self.state]
        requested_severity = STATE_SEVERITY[state]
        more_restrictive = (
            requested_severity > current_severity
            or scale < self.speed_scale - 1.0e-12
        )
        if more_restrictive:
            self.state = state
            self.speed_scale = scale
            self.recovery_count = 0
            self.recovery_confirmed = False
            self.last_recovery_sample_sec = None
            return self._result(
                state,
                scale,
                True,
                'RESTRICTION_IMMEDIATE',
            )

        recovering = (
            requested_severity < current_severity
            or scale > self.speed_scale + 1.0e-12
        )
        if not recovering:
            self.recovery_count = 0
            self.recovery_confirmed = False
            self.last_recovery_sample_sec = None
            return self._result(state, scale, False, 'STABLE')

        if not self._recovery_tick(now):
            return self._result(
                state,
                scale,
                False,
                'RECOVERY_WAITING_PERIOD',
            )

        if not self.recovery_confirmed:
            self.recovery_count += 1
            if self.recovery_count < self.recovery_required_samples:
                return self._result(
                    state,
                    scale,
                    False,
                    'RECOVERY_COUNTING',
                )
            self.recovery_confirmed = True

        previous_state = self.state
        previous_scale = self.speed_scale
        new_scale = min(
            scale,
            self.speed_scale + self.maximum_scale_increment,
        )
        if (
            math.isclose(new_scale, scale, abs_tol=1.0e-12)
            and requested_severity <= current_severity
        ):
            new_state = state
        else:
            new_state = 'REDUCTION'
        self.state = new_state
        self.speed_scale = new_scale
        transition = (
            previous_state != new_state
            or not math.isclose(previous_scale, new_scale, abs_tol=1.0e-12)
        )
        complete = (
            self.state == state
            and math.isclose(self.speed_scale, scale, abs_tol=1.0e-12)
        )
        return self._result(
            state,
            scale,
            transition,
            'RECOVERY_COMPLETE' if complete else 'RECOVERY_RATE_LIMITED',
        )
