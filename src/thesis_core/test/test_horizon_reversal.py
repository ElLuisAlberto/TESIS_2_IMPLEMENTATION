"""Verify that a changed JOG direction prompts an immediate horizon."""

from types import SimpleNamespace

from thesis_core.horizon_preview import (
    HorizonPreview, direction_reversed, movement_directions,
)
from thesis_core.joint_model import JOINT_NAMES


def test_direction_reversal_ignores_small_motion_noise():
    """Only a true opposite command triggers an extra prediction."""
    measured = (0.0,) * 6
    forward = movement_directions(measured, (0.1, 0, 0, 0, 0, 0))
    noise = movement_directions(measured, (0.0001, 0, 0, 0, 0, 0))
    reverse = movement_directions(measured, (-0.1, 0, 0, 0, 0, 0))
    assert not direction_reversed(None, forward)
    assert not direction_reversed(forward, noise)
    assert direction_reversed(forward, reverse)


def test_reversal_publishes_prediction_without_waiting_for_tick():
    """A fresh supervised reversal publishes from the measured state."""
    outputs = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000,
        to_msg=lambda: 'stamp',
    ))
    node = SimpleNamespace(
        jog=None,
        jog_time=0.0,
        state=(0.0,) * 6,
        _previous_jog_direction=None,
        state_is_fresh=lambda now: True,
        get_clock=lambda: clock,
        select_source=lambda now, ros: 'jog',
        make_poses=lambda source, now: (
            (), {'command_id': 'jog'}, 'prediction',
        ),
        publish_joint_prediction=lambda prediction, metadata, stamp:
            outputs.append((prediction, metadata, stamp)),
    )

    def jog(delta):
        return SimpleNamespace(
            joint_names=JOINT_NAMES,
            positions=(delta,) + (0.0,) * 5,
        )

    HorizonPreview.receive_jog(node, jog(0.1))
    assert not outputs
    HorizonPreview.receive_jog(node, jog(-0.1))
    assert outputs == [('prediction', {'command_id': 'jog'}, 'stamp')]
    assert node.jog.positions[0] == -0.1
