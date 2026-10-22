"""Static contract tests for the isolated physical state launch."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "jaco_read_only_state.launch.py"


def _module():
    spec = importlib.util.spec_from_file_location("jaco_read_only", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_read_only_launch_has_only_state_and_diagnostic_nodes():
    text = LAUNCH.read_text(encoding="utf-8")
    assert text.count("Node(") == 3
    for required in (
        'executable="jaco_hardware_node"',
        'executable="robot_state_publisher"',
        'executable="hardware_readiness"',
        '"hardware_output_enabled": False',
    ):
        assert required in text
    for forbidden in (
        "safety_supervisor", "obstacle_input_bridge", "proximity_monitor",
        "cartesian_command_adapter", "cartesian_jog_adapter",
        "cartesian_trajectory_adapter", "joint_gui", "horizon_preview",
        "capsule_visualizer", "telemetry_logger", "rviz2",
    ):
        assert forbidden not in text


def test_read_only_launch_rejects_output_true():
    module = _module()
    with pytest.raises(RuntimeError):
        module._reject_physical_output_value("true")
    module._reject_physical_output_value("false")
