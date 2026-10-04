"""Regression tests for readiness transition logging."""

import inspect

from thesis_hardware.readiness_logging import log_readiness_transition


class StrictCallsiteLogger:
    """Reproduce Humble's one-severity-per-call-site restriction."""

    def __init__(self):
        self.severities = {}
        self.entries = []

    def _record(self, severity, message, frame):
        callsite = (frame.f_code.co_filename, frame.f_lineno)
        previous = self.severities.setdefault(callsite, severity)
        if previous != severity:
            raise ValueError(
                'Logger severity cannot be changed between calls.'
            )
        self.entries.append((severity, message))

    def info(self, message):
        """Record an INFO call and its caller."""
        frame = inspect.currentframe()
        self._record('INFO', message, frame.f_back)

    def warning(self, message):
        """Record a WARN call and its caller."""
        frame = inspect.currentframe()
        self._record('WARN', message, frame.f_back)


def test_ready_transition_uses_separate_severity_callsites():
    logger = StrictCallsiteLogger()

    log_readiness_transition(logger, False, 'NOT_READY')
    log_readiness_transition(logger, True, 'READY')
    log_readiness_transition(logger, False, 'NOT_READY again')

    assert logger.entries == [
        ('WARN', 'NOT_READY'),
        ('INFO', 'READY'),
        ('WARN', 'NOT_READY again'),
    ]
