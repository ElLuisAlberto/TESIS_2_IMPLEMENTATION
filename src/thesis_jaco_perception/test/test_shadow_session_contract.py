import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "jaco_d435i_shadow_session.launch.py"
SHADOW_LAUNCH = ROOT / "launch" / "jaco_rgbd_shadow.launch.py"
CAMERA_PIPELINE = ROOT / "launch" / "d435i_shadow_camera_pipeline.launch.py"
BASE_CAPSULES = ROOT / "config" / "jaco_self_filter_capsules.yaml"
COVERAGE_PROFILE = ROOT / "config" / "jaco_self_filter_shadow_coverage_v1.yaml"
COVERAGE_PROFILE_V2 = ROOT / "config" / "jaco_self_filter_shadow_coverage_v2.yaml"
MESH_SHADOW_RVIZ = ROOT / "rviz" / "jaco_d435i_mesh_shadow_comparison.rviz"
MESH_ARM_REMOVED_RVIZ = ROOT / "rviz" / "jaco_d435i_mesh_arm_removed.rviz"
ALIGNMENT_RVIZ = ROOT / "rviz" / "jaco_d435i_alignment.rviz"
SESSION_CANDIDATE = (
    ROOT.parents[1] / "evidence" / "codex_reports" /
    "2026-10-21_d435i_extrinsic_session_candidate.yaml"
)


def _module():
    spec = importlib.util.spec_from_file_location("shadow_session", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _camera_module():
    spec = importlib.util.spec_from_file_location("shadow_camera_pipeline", CAMERA_PIPELINE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _shadow_module():
    spec = importlib.util.spec_from_file_location("jaco_rgbd_shadow", SHADOW_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _candidate(**updates):
    value = {
        "valid": True,
        "accepted_for_session": True,
        "session_only": True,
        "parent_frame": "world",
        "child_frame": "d435i_link",
        "camera_optical_frame": "d435i_depth_optical_frame",
        "translation": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "metrics": {"sample_count": 20},
    }
    value.update(updates)
    return {"extrinsic_candidate": value}


def _coverage_profile(**updates):
    metadata = {
        "session_only": True,
        "not_for_safety": True,
        "profile_kind": "shadow_coverage",
    }
    parameters = {
        "capsule_start_frames": [
            "j2n6s300_link_base", "j2n6s300_link_1", "j2n6s300_link_2",
            "j2n6s300_link_3", "j2n6s300_link_4", "j2n6s300_link_5"],
        "capsule_end_frames": [
            "j2n6s300_link_1", "j2n6s300_link_2", "j2n6s300_link_3",
            "j2n6s300_link_4", "j2n6s300_link_5", "j2n6s300_end_effector"],
        "capsule_inner_radii_m": [0.105, 0.105, 0.100, 0.090, 0.100, 0.105],
    }
    value = {
        "shadow_coverage_metadata": metadata,
        "jaco_self_filter_node": {"ros__parameters": parameters},
    }
    value.update(updates)
    return value


def test_defaults_are_explicit_and_safe():
    text = LAUNCH.read_text()
    for argument in ("start_jaco_read_only", "start_camera", "start_rviz",
                     "enable_self_filter", "publish_session_extrinsic"):
        assert f'DeclareLaunchArgument("{argument}", default_value="false")' in text
    assert 'DeclareLaunchArgument("candidate_extrinsic_file", default_value="")' in text
    assert 'DeclareLaunchArgument("hardware_output", default_value="false")' in text
    assert 'DeclareLaunchArgument("session_epsilon_total_m", default_value="")' in text
    assert 'DeclareLaunchArgument("shadow_coverage_profile_file", default_value="")' in text
    for name, value in (
            ("enable_crop", "false"), ("min_x", "-1.5"), ("max_x", "1.5"),
            ("min_y", "-1.5"), ("max_y", "1.5"),
            ("min_z", "-1.0"), ("max_z", "2.5")):
        assert f'DeclareLaunchArgument("{name}", default_value="{value}")' in text


def test_rejects_absent_invalid_temporary_and_empty_frame_candidates(tmp_path):
    module = _module()
    with pytest.raises(RuntimeError):
        module._validate_candidate("")
    for updates in ({"valid": False}, {"session_only": False}, {"accepted_for_session": False},
                    {"parent_frame": ""}, {"child_frame": ""}):
        candidate = tmp_path / "candidate.yaml"
        candidate.write_text(yaml.safe_dump(_candidate(**updates)))
        with pytest.raises(RuntimeError):
            module._validate_candidate(str(candidate))


def test_rejects_optical_frame_as_extrinsic_child(tmp_path):
    module = _module()
    candidate = tmp_path / "optical_child.yaml"
    candidate.write_text(yaml.safe_dump(_candidate(
        child_frame="d435i_depth_optical_frame")))
    with pytest.raises(RuntimeError, match="must not be an optical frame"):
        module._validate_candidate(str(candidate))


def test_accepts_only_world_to_camera_root_and_preserves_optical_frame(tmp_path):
    module = _module()
    candidate = tmp_path / "root_child.yaml"
    candidate.write_text(yaml.safe_dump(_candidate()))
    accepted = module._validate_candidate(str(candidate))
    assert accepted["parent_frame"] == "world"
    assert accepted["child_frame"] == module.CAMERA_ROOT_FRAME == "d435i_link"
    assert accepted["camera_optical_frame"] == module.CAMERA_OPTICAL_FRAME
    assert accepted["child_frame"] != accepted["camera_optical_frame"]

    wrong_root = tmp_path / "wrong_root.yaml"
    wrong_root.write_text(yaml.safe_dump(_candidate(child_frame="d435i_depth_frame")))
    with pytest.raises(RuntimeError, match="world -> d435i_link"):
        module._validate_candidate(str(wrong_root))


def test_session_candidate_attaches_world_to_the_realsense_root():
    module = _module()
    accepted = module._validate_candidate(str(SESSION_CANDIDATE))
    assert accepted["parent_frame"] == "world"
    assert accepted["child_frame"] == "d435i_link"
    assert accepted["camera_optical_frame"] == "d435i_depth_optical_frame"


def test_rejects_hardware_output_true():
    module = _module()
    with pytest.raises(RuntimeError):
        module._validate_hardware_output("true")
    module._validate_hardware_output("false")


def test_empty_coverage_argument_preserves_the_historical_capsule_profile():
    session_text = LAUNCH.read_text()
    shadow_text = SHADOW_LAUNCH.read_text()
    assert 'DeclareLaunchArgument("shadow_coverage_profile_file", default_value="")' in session_text
    assert '"shadow_coverage_profile_file",' in shadow_text
    assert 'default_value=default_capsules' in shadow_text
    assert 'if str(coverage_profile).strip():' in shadow_text
    assert '_validate_capsule_profile(LaunchConfiguration("capsule_profile_file").perform(context))' in shadow_text
    base = yaml.safe_load(BASE_CAPSULES.read_text())
    assert base["jaco_self_filter_node"]["ros__parameters"]["capsule_inner_radii_m"] == [
        0.105, 0.105, 0.085, 0.075, 0.085, 0.090]


@pytest.mark.parametrize("module_factory", (_module, _shadow_module))
def test_accepts_only_the_explicit_valid_shadow_coverage_profile(module_factory):
    module = module_factory()
    accepted = module._validate_shadow_coverage_profile(str(COVERAGE_PROFILE))
    assert set(accepted) == {
        "capsule_start_frames", "capsule_end_frames", "capsule_inner_radii_m"}
    profile = yaml.safe_load(COVERAGE_PROFILE.read_text())
    assert profile["shadow_coverage_metadata"] == {
        "session_only": True,
        "not_for_safety": True,
        "profile_kind": "shadow_coverage",
    }
    assert profile["jaco_self_filter_node"]["ros__parameters"]["capsule_inner_radii_m"] == [
        0.105, 0.105, 0.100, 0.090, 0.100, 0.105]
    assert accepted == profile["jaco_self_filter_node"]["ros__parameters"]


@pytest.mark.parametrize("module_factory", (_module, _shadow_module))
def test_coverage_profile_rejects_missing_metadata_and_bad_geometry(module_factory, tmp_path):
    module = module_factory()
    cases = [
        _coverage_profile(shadow_coverage_metadata={}),
        _coverage_profile(shadow_coverage_metadata={
            "session_only": True, "not_for_safety": False, "profile_kind": "shadow_coverage"}),
        _coverage_profile(shadow_coverage_metadata={
            "session_only": True, "not_for_safety": True, "profile_kind": "other"}),
    ]
    nonpositive = _coverage_profile()
    nonpositive["jaco_self_filter_node"]["ros__parameters"]["capsule_inner_radii_m"][3] = 0.0
    nonfinite = _coverage_profile()
    nonfinite["jaco_self_filter_node"]["ros__parameters"]["capsule_inner_radii_m"][3] = float("nan")
    invalid_frame = _coverage_profile()
    invalid_frame["jaco_self_filter_node"]["ros__parameters"]["capsule_end_frames"][3] = ""
    inconsistent = _coverage_profile()
    inconsistent["jaco_self_filter_node"]["ros__parameters"]["capsule_end_frames"].pop()
    forbidden_parameter = _coverage_profile()
    forbidden_parameter["jaco_self_filter_node"]["ros__parameters"]["epsilon_total_m"] = 0.07
    cases.extend((nonpositive, nonfinite, invalid_frame, inconsistent, forbidden_parameter))
    for index, value in enumerate(cases):
        profile = tmp_path / f"invalid_{index}.yaml"
        profile.write_text(yaml.safe_dump(value))
        with pytest.raises(RuntimeError):
            module._validate_shadow_coverage_profile(str(profile))
    with pytest.raises(RuntimeError):
        module._validate_shadow_coverage_profile(str(tmp_path / "missing.yaml"))
    malformed = tmp_path / "malformed.yaml"
    malformed.write_text("shadow_coverage_metadata: [")
    with pytest.raises(RuntimeError):
        module._validate_shadow_coverage_profile(str(malformed))


def test_coverage_profile_is_forwarded_only_to_the_shadow_self_filter():
    session_text = LAUNCH.read_text()
    shadow_text = SHADOW_LAUNCH.read_text()
    assert '"shadow_coverage_profile_file": LaunchConfiguration(' in session_text
    assert '"shadow_coverage_profile_file"),' in session_text
    assert 'SetLaunchConfiguration(' not in shadow_text
    assert 'parameters.append(_validate_shadow_coverage_profile(coverage_profile))' in shadow_text
    assert 'parameters = [' in shadow_text
    assert 'LaunchConfiguration("capsule_profile_file")' in shadow_text
    assert 'never passing coverage YAML as a params file' in shadow_text
    camera_text = CAMERA_PIPELINE.read_text()
    for forbidden_text in (
            "shadow_coverage_profile_file",
            "safety_supervisor",
            "obstacle_input",
            "obstacle_extractor",
            "hardware_bridge",
        ):
        assert forbidden_text not in camera_text


@pytest.mark.parametrize("module_factory", (_module, _shadow_module))
def test_coverage_v2_is_exactly_the_approved_partial_profile(module_factory):
    module = module_factory()
    v1 = yaml.safe_load(COVERAGE_PROFILE.read_text())
    v2 = yaml.safe_load(COVERAGE_PROFILE_V2.read_text())
    assert v2["shadow_coverage_metadata"] == {
        "session_only": True,
        "not_for_safety": True,
        "profile_kind": "shadow_coverage",
    }
    v1_parameters = v1["jaco_self_filter_node"]["ros__parameters"]
    v2_parameters = v2["jaco_self_filter_node"]["ros__parameters"]
    assert set(v2_parameters) == {
        "capsule_start_frames", "capsule_end_frames", "capsule_inner_radii_m"}
    assert v2_parameters["capsule_start_frames"] == v1_parameters["capsule_start_frames"]
    assert v2_parameters["capsule_end_frames"] == v1_parameters["capsule_end_frames"]
    assert len(v2_parameters["capsule_start_frames"]) == 6
    assert len(v2_parameters["capsule_end_frames"]) == 6
    assert v2_parameters["capsule_inner_radii_m"] == [
        0.105, 0.105, 0.100, 0.114, 0.123, 0.126]
    assert v2_parameters["capsule_inner_radii_m"][:3] == \
        v1_parameters["capsule_inner_radii_m"][:3]
    assert v2_parameters["capsule_inner_radii_m"][3:] == [0.114, 0.123, 0.126]
    p95_required_m = [0.11342283441561084, 0.12218581675213991, 0.12591543346805348]
    for radio_v2, p95 in zip(v2_parameters["capsule_inner_radii_m"][3:], p95_required_m):
        assert radio_v2 >= p95
        assert radio_v2 - p95 < 0.001
    assert module._validate_shadow_coverage_profile(str(COVERAGE_PROFILE_V2)) == v2_parameters


def test_coverage_v2_is_extracted_not_added_as_a_second_params_file():
    shadow_text = SHADOW_LAUNCH.read_text()
    assert 'parameters.append(_validate_shadow_coverage_profile(coverage_profile))' in shadow_text
    assert 'parameters.append(coverage_profile)' not in shadow_text
    assert 'parameters.append(LaunchConfiguration("shadow_coverage_profile_file"))' not in shadow_text
    assert str(COVERAGE_PROFILE_V2) not in shadow_text


@pytest.mark.parametrize("module_factory", (_module, _camera_module))
def test_crop_defaults_are_finite_and_preserve_historical_bounds(module_factory):
    module = module_factory()
    assert module._validate_crop_values("false", {
        "min_x": "-1.5", "max_x": "1.5",
        "min_y": "-1.5", "max_y": "1.5",
        "min_z": "-1.0", "max_z": "2.5",
    }) == {
        "min_x": -1.5, "max_x": 1.5,
        "min_y": -1.5, "max_y": 1.5,
        "min_z": -1.0, "max_z": 2.5,
    }


@pytest.mark.parametrize("module_factory", (_module, _camera_module))
@pytest.mark.parametrize("values", (
    {"min_x": "1", "max_x": "1", "min_y": "-1", "max_y": "1", "min_z": "-1", "max_z": "1"},
    {"min_x": "-1", "max_x": "1", "min_y": "2", "max_y": "1", "min_z": "-1", "max_z": "1"},
    {"min_x": "-1", "max_x": "1", "min_y": "-1", "max_y": "1", "min_z": "nan", "max_z": "1"},
    {"min_x": "-1", "max_x": "inf", "min_y": "-1", "max_y": "1", "min_z": "-1", "max_z": "1"},
))
def test_crop_rejects_inverted_or_nonfinite_bounds(module_factory, values):
    with pytest.raises(RuntimeError):
        module_factory()._validate_crop_values("true", values)


def test_accepts_an_explicit_temporary_epsilon():
    module = _module()
    assert module._validate_session_epsilon("0.05") == pytest.approx(0.05)


@pytest.mark.parametrize("value", ("", "-1", "NaN", "inf", "not-a-number"))
def test_rejects_missing_nonfinite_negative_or_nonnumeric_session_epsilon(value):
    module = _module()
    with pytest.raises(RuntimeError):
        module._validate_session_epsilon(value)


def test_shadow_contract_has_no_safety_connections():
    text = LAUNCH.read_text()
    for forbidden in (
        'executable="safety_supervisor"',
        'executable="obstacle_input_bridge"',
        'executable="obstacle_extractor"',
        '"/thesis/obstacle_input"',
    ):
        assert forbidden not in text
    assert "static_transform_publisher" in text


def test_uses_the_installed_read_only_hardware_launch_contract():
    text = LAUNCH.read_text()
    assert "jaco_read_only_state.launch.py" in text
    assert "jaco_physical_system.launch.py" not in text
    assert "thesis_hardware.launch.py" not in text
    assert 'launch_arguments={"hardware_output": "false"}.items()' in text


def test_shadow_session_supplies_a_real_camera_and_capsule_contract():
    text = LAUNCH.read_text()
    assert '"input_topic": "/thesis/perception/points_filtered"' in text
    assert '"camera_optical_frame": CAMERA_OPTICAL_FRAME' in text
    assert '"epsilon_total_m": LaunchConfiguration("session_epsilon_total_m")' in text
    assert '"expected_frame": "world"' in text


def test_mesh_shadow_robot_description_is_typed_and_isolated():
    shadow_text = SHADOW_LAUNCH.read_text()
    session_text = LAUNCH.read_text()
    assert 'DeclareLaunchArgument("enable_mesh_shadow_comparison", default_value="false")' in shadow_text
    assert 'OpaqueFunction(function=_create_mesh_shadow_node)' in shadow_text
    assert '"robot_description": ParameterValue(xml, value_type=str)' in shadow_text
    assert 'j2n6s300_standalone.xacro' in shadow_text
    assert '"robot_description": LaunchConfiguration("robot_description"),' not in session_text
    xml = '<robot name="jaco:shadow">\n  <link name="quoted &quot;link&quot;"/>\n</robot>'
    assert isinstance(xml, str) and '<' in xml and ':' in xml and '\n' in xml
    mesh_scope = shadow_text
    for forbidden in ('obstacle_input', 'safety_supervisor', 'hardware_bridge', 'obstacle_extractor'):
        assert forbidden not in mesh_scope


def test_mesh_robot_description_validation_is_canonical_and_fail_closed():
    module = _shadow_module()
    valid = '''<robot name="jaco:shadow">\n<link name="j2n6s300_link_base"><collision><geometry><mesh filename="package://x/base.dae"/></geometry></collision></link>\n<link name="j2n6s300_link_6"/>\n<link name="j2n6s300_link_finger_1"/>\n</robot>'''
    assert module._validate_mesh_robot_description(valid) is None
    for invalid in ("", "not xml", "<root/>", "<robot name='x'><link name='j2n6s300_link_base'/></robot>"):
        with pytest.raises(RuntimeError, match="MESH_SHADOW_ROBOT_DESCRIPTION_INVALID"):
            module._validate_mesh_robot_description(invalid)


def test_mesh_shadow_rviz_profile_has_exactly_the_six_active_comparison_displays():
    profile = yaml.safe_load(MESH_SHADOW_RVIZ.read_text())
    manager = profile["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "world"
    displays = manager["Displays"]
    assert any(display["Class"] == "rviz_default_plugins/RobotModel" for display in displays)
    assert any(display["Class"] == "rviz_default_plugins/TF" for display in displays)
    clouds = [
        display for display in displays
        if display["Class"] == "rviz_default_plugins/PointCloud2"
    ]
    expected = {
        "Capsules Self Confirmed": (
            "/thesis/jaco_perception/points_self_confirmed", "0; 255; 0"),
        "Capsules External Candidate": (
            "/thesis/jaco_perception/points_external_candidate", "255; 0; 0"),
        "Capsules Occluded Unknown": (
            "/thesis/jaco_perception/points_occluded_unknown", "255; 255; 0"),
        "Mesh Self Confirmed": (
            "/thesis/jaco_perception/mesh_shadow/points_self_confirmed", "0; 255; 255"),
        "Mesh External Candidate": (
            "/thesis/jaco_perception/mesh_shadow/points_external_candidate", "255; 0; 255"),
        "Mesh Occluded Unknown": (
            "/thesis/jaco_perception/mesh_shadow/points_occluded_unknown", "255; 128; 0"),
    }
    assert len(clouds) == len(expected)
    assert {display["Name"] for display in clouds} == set(expected)
    topics = []
    for display in clouds:
        topic, color = expected[display["Name"]]
        assert display["Enabled"] is True
        assert display["Color"] == color
        assert display["Topic"] == {
            "Value": topic,
            "Reliability Policy": "Best Effort",
            "Durability Policy": "Volatile",
            "History Policy": "Keep Last",
            "Depth": 5,
        }
        topics.append(display["Topic"]["Value"])
    assert len(topics) == len(set(topics)) == 6
    profile_text = MESH_SHADOW_RVIZ.read_text().lower()
    for forbidden in ("/thesis/obstacle_input", "supervisor", "extractor", "comando", "command"):
        assert forbidden not in profile_text


def test_alignment_rviz_profile_is_the_original_unchanged_profile():
    assert hashlib.sha256(ALIGNMENT_RVIZ.read_bytes()).hexdigest() == (
        "ca236c66efcb20632b0c4312a7f9ace73bb66a726b2b661e754822f7fc2a3c5c")


def test_mesh_arm_removed_rviz_profile_is_minimal_and_preserves_existing_profiles():
    assert hashlib.sha256(MESH_SHADOW_RVIZ.read_bytes()).hexdigest() == (
        "383cd022a12ebea507f68ba64333093c2d356a7555951afa9b6222c9264213dc")
    profile = yaml.safe_load(MESH_ARM_REMOVED_RVIZ.read_text())
    manager = profile["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "world"
    displays = manager["Displays"]
    assert any(display["Class"] == "rviz_default_plugins/RobotModel" for display in displays)
    assert any(display["Class"] == "rviz_default_plugins/TF" for display in displays)
    clouds = [display for display in displays
              if display["Class"] == "rviz_default_plugins/PointCloud2"]
    assert len(clouds) == 1
    assert clouds[0]["Name"] == "Mesh External Candidate — Arm Removed"
    assert clouds[0]["Enabled"] is True
    assert clouds[0]["Color"] == "255; 0; 255"
    assert clouds[0]["Topic"] == {
        "Value": "/thesis/jaco_perception/mesh_shadow/points_external_candidate",
        "Reliability Policy": "Best Effort",
        "Durability Policy": "Volatile",
        "History Policy": "Keep Last",
        "Depth": 5,
    }
    profile_text = MESH_ARM_REMOVED_RVIZ.read_text().lower()
    for forbidden in (
            "points_filtered", "points_self_confirmed", "points_occluded_unknown",
            "/thesis/jaco_perception/points_external_candidate", "supervisor",
            "horizon", "extractor", "comando", "command", "obstacle_input", "control"):
        assert forbidden not in profile_text


def test_camera_pipeline_requests_an_exact_stamp_world_cloud():
    text = CAMERA_PIPELINE.read_text()
    assert '"target_frame": "world"' in text
    assert '"use_latest_transform": "false"' in text
    assert '"filtered_topic": "/thesis/perception/points_filtered"' in text
    assert '"voxel_leaf_size": "0.01"' in text
    assert '"max_valid_range_m": "1.5"' in text
    for name in ("enable_crop", "min_x", "max_x", "min_y", "max_y", "min_z", "max_z"):
        assert f'DeclareLaunchArgument("{name}"' in text
        assert f'"{name}": LaunchConfiguration("{name}")' in text


def test_camera_pipeline_declares_the_realsense_root_to_optical_chain():
    text = CAMERA_PIPELINE.read_text()
    assert '"camera_name": "d435i"' in text
    assert '"base_frame_id": "link"' in text
    assert '"tf_prefix": ""' in text
    assert '"camera_link_frame": "d435i_link"' in text


def test_shadow_contract_connects_world_once_without_safety_connections():
    text = LAUNCH.read_text()
    assert '"--frame-id", candidate["parent_frame"]' in text
    assert '"--child-frame-id", candidate["child_frame"]' in text
    assert 'CAMERA_ROOT_FRAME = "d435i_link"' in text
    assert 'CAMERA_OPTICAL_FRAME = "d435i_depth_optical_frame"' in text
    assert '"/thesis/obstacle_input"' not in text


def test_camera_pipeline_seals_realsense_from_session_arguments():
    text = CAMERA_PIPELINE.read_text()
    assert "forwarding=False" in text
    camera_scope = text.split("processing = GroupAction", 1)[0]
    for internal_argument in (
        "start_jaco_read_only",
        "hardware_output",
        "candidate_extrinsic_file",
        "enable_self_filter",
        "publish_session_extrinsic",
    ):
        assert internal_argument not in camera_scope
    for roi_argument in ("enable_crop", "min_x", "max_x", "min_y", "max_y", "min_z", "max_z"):
        assert f'"{roi_argument}":' not in camera_scope


def test_session_forwards_roi_only_to_the_shadow_camera_pipeline():
    text = LAUNCH.read_text()
    for name in ("enable_crop", "min_x", "max_x", "min_y", "max_y", "min_z", "max_z"):
        assert f'"{name}": LaunchConfiguration("{name}")' in text


def test_camera_pipeline_has_no_extractor_obstacle_or_supervisor_connection():
    text = CAMERA_PIPELINE.read_text()
    assert '"start_extractor": "false"' in text
    for forbidden in (
        "obstacle_input",
        "obstacle_extractor",
        "safety_supervisor",
    ):
        assert forbidden not in text
