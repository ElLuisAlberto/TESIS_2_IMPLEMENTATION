"""Regression checks for monotonic E11 HOLD attribution."""

from types import SimpleNamespace

from thesis_validation.scenario_recorder import ScenarioRecorder


def _stamp(seconds, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def _trace(
    stage,
    source='adapter',
    detail='JOG_WATCHDOG_EXPIRED',
    command_id='e11-command',
    monotonic_ns=0,
):
    return SimpleNamespace(
        stage=stage,
        source=source,
        detail=detail,
        command_id=command_id,
        monotonic_ns=monotonic_ns,
        stamp=_stamp(10),
        intent_stamp=_stamp(10),
    )


def _fake(scenario_id='E11', command_id='e11-command'):
    fake = SimpleNamespace(
        scenario_id=scenario_id,
        command_id=command_id,
        latest={'state': 'ALLOW'},
        jog_hold_age_ms=-1.0,
        _supervisor_receive_monotonic_ns={},
        _controller_publish_monotonic_ns={},
        _hold_publish_monotonic_ns={},
    )
    fake._update_e11_monotonic_timing = (
        lambda event_id: ScenarioRecorder._update_e11_monotonic_timing(
            fake, event_id
        )
    )
    return fake


def test_stop_detection_alone_is_not_evidence_of_hold():
    """Detection may precede publication and must not close E11."""
    fake = _fake()
    ScenarioRecorder._timing_cb(
        fake, _trace('STOP_DETECTED', monotonic_ns=1_100_000_000)
    )
    assert fake.latest['reason_code'] == 'JOG_WATCHDOG_EXPIRED'
    assert fake.latest['state'] == 'ALLOW'
    assert fake.jog_hold_age_ms == -1.0


def test_monotonic_events_measure_controller_and_hold():
    """Inter-publisher delivery order cannot corrupt the E11 timing basis."""
    fake = _fake()
    ScenarioRecorder._timing_cb(
        fake,
        _trace('CONTROLLER_PUBLISH', monotonic_ns=1_004_000_000),
    )
    ScenarioRecorder._timing_cb(
        fake,
        _trace('HOLD_PUBLISH', monotonic_ns=1_252_000_000),
    )
    assert fake.jog_hold_age_ms == -1.0

    ScenarioRecorder._timing_cb(
        fake,
        _trace(
            'SUPERVISOR_RECEIVE',
            source='supervisor',
            detail='',
            monotonic_ns=1_000_000_000,
        ),
    )

    assert abs(fake.latest['latency_end_to_end_ms'] - 4.0) < 1e-6
    assert abs(fake.jog_hold_age_ms - 252.0) < 1e-6
    assert fake.latest['state'] == 'HOLD'
    assert fake.latest['reason_code'] == 'JOG_WATCHDOG_EXPIRED'


def test_unrelated_command_does_not_close_e11():
    """A HOLD trace from another command cannot be attributed to E11."""
    fake = _fake()
    ScenarioRecorder._timing_cb(
        fake,
        _trace(
            'HOLD_PUBLISH',
            command_id='different-command',
            monotonic_ns=1_252_000_000,
        ),
    )
    assert fake.latest == {
        'state': 'ALLOW',
        'last_timing_stage': 'HOLD_PUBLISH',
    }
    assert fake.jog_hold_age_ms == -1.0


def test_watchdog_trace_does_not_contaminate_other_scenarios():
    """A late watchdog event must not overwrite an E02 observation."""
    fake = _fake(scenario_id='E02')
    fake.latest = {'state': 'WARNING', 'reason_code': 'SAFE_CLEARANCE'}
    ScenarioRecorder._timing_cb(
        fake,
        _trace('STOP_DETECTED', monotonic_ns=1_250_000_000),
    )
    assert fake.latest['state'] == 'WARNING'
    assert fake.latest['reason_code'] == 'SAFE_CLEARANCE'
