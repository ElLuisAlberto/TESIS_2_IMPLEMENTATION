"""Regression check for HOLD attribution in live scenario evidence."""

from types import SimpleNamespace

from thesis_validation.scenario_recorder import ScenarioRecorder


def test_zero_velocity_reference_does_not_override_warning():
    """E02 stays WARNING even when the adapter sends a neutral reference."""
    fake = SimpleNamespace(
        scenario_id='E02', latest={'state': 'WARNING'},
        last_jog_time=1.0, last_joint_positions=[0.0] * 6,
        jog_hold_age_ms=-1.0,
    )
    point = SimpleNamespace(positions=[0.0] * 6, velocities=[0.0] * 6)
    message = SimpleNamespace(points=[point])
    ScenarioRecorder._generic_cb(
        fake, '/arm_controller/joint_trajectory', message,
    )
    assert fake.latest['state'] == 'WARNING'


def test_watchdog_records_first_confirmed_hold_only(monkeypatch):
    """Later hold messages must not inflate the measured response time."""
    fake = SimpleNamespace(
        scenario_id='E11',
        latest={'state': 'ALLOW', 'reason_code': 'JOG_WATCHDOG_EXPIRED'},
        last_jog_time=10.0, last_joint_positions=[0.0] * 6,
        jog_hold_age_ms=-1.0, _pending_hold_age_ms=-1.0,
    )
    point = SimpleNamespace(positions=[0.0] * 6, velocities=[0.0] * 6)
    message = SimpleNamespace(points=[point])
    monkeypatch.setattr(
        'thesis_validation.scenario_recorder.time.monotonic',
        lambda: 10.27,
    )
    ScenarioRecorder._generic_cb(
        fake, '/arm_controller/joint_trajectory', message,
    )
    assert abs(fake.jog_hold_age_ms - 270.0) < 1e-6
    monkeypatch.setattr(
        'thesis_validation.scenario_recorder.time.monotonic',
        lambda: 10.75,
    )
    ScenarioRecorder._generic_cb(
        fake, '/arm_controller/joint_trajectory', message,
    )
    assert abs(fake.jog_hold_age_ms - 270.0) < 1e-6


def test_hold_seen_before_watchdog_trace_is_correlated(monkeypatch):
    """ROS topics may deliver the hold before the watchdog trace."""
    fake = SimpleNamespace(
        scenario_id='E11', latest={'state': 'ALLOW'},
        last_jog_time=10.0, last_joint_positions=[0.0] * 6,
        jog_hold_age_ms=-1.0, _pending_hold_age_ms=-1.0,
    )
    point = SimpleNamespace(positions=[0.0] * 6, velocities=[0.0] * 6)
    monkeypatch.setattr(
        'thesis_validation.scenario_recorder.time.monotonic',
        lambda: 10.35,
    )
    ScenarioRecorder._generic_cb(
        fake, '/arm_controller/joint_trajectory',
        SimpleNamespace(points=[point]),
    )
    assert fake.jog_hold_age_ms == -1.0
    assert fake._pending_hold_age_ms > 0.0
    trace = SimpleNamespace(
        stage='STOP_DETECTED', detail='JOG_WATCHDOG_EXPIRED',
        stamp=None, intent_stamp=None,
    )
    ScenarioRecorder._timing_cb(fake, trace)
    assert abs(fake.jog_hold_age_ms - 350.0) < 1e-6
    assert fake.latest['state'] == 'HOLD'
