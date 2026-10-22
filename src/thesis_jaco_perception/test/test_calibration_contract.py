from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_final_extrinsic_is_invalid_by_default():
    data = yaml.safe_load((ROOT / "config" / "d435i_extrinsic_final.yaml").read_text())
    params = data["d435i_extrinsic_final"]["ros__parameters"]
    assert params["valid"] is False
    assert params["session_only"] is True
    assert params["parent_frame"] == ""
    assert params["child_frame"] == ""


def test_candidate_is_temporary_and_requires_detector():
    data = yaml.safe_load((ROOT / "config" / "d435i_calibration_candidate.yaml").read_text())
    params = data["d435i_calibration_candidate"]["ros__parameters"]
    assert params["session_only"] is True
    assert params["detector_available"] is False
    assert params["min_samples"] >= 20


def test_shadow_launch_is_disabled_and_has_no_tf_or_safety_connection():
    launch = (ROOT / "launch" / "d435i_calibration_shadow.launch.py").read_text()
    assert 'DeclareLaunchArgument("enabled", default_value="false")' in launch
    assert "candidate_extrinsic_file" in launch
    assert "static_transform_publisher" not in launch
    assert "jaco_self_filter_node" not in launch
    assert "obstacle_input" not in launch
    assert "supervisor" not in launch
