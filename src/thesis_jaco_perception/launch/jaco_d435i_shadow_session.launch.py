"""Explicit, isolated JACO + RGB-D self-filter shadow-session contract.

Raw cloud: /camera/d435i/depth/color/points
World cloud: /thesis/perception/points_filtered (target_frame=world, exact stamp)
Robot state: /joint_states; base: world; camera root: d435i_link;
cloud/filter optical frame: d435i_depth_optical_frame.
Shadow outputs are owned by jaco_self_filter_node. This launch never includes
extractor, obstacle_input or a supervisor.
"""

import math
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERA_ROOT_FRAME = "d435i_link"
CAMERA_OPTICAL_FRAME = "d435i_depth_optical_frame"
CROP_LIMIT_NAMES = ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")
SHADOW_COVERAGE_CAPSULE_COUNT = 6
SHADOW_COVERAGE_PARAMETER_KEYS = {
    "capsule_start_frames", "capsule_end_frames", "capsule_inner_radii_m"}


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _validate_candidate(path):
    candidate_path = Path(path)
    if not path or not candidate_path.is_file():
        raise RuntimeError("candidate_extrinsic_file must name an existing candidate YAML")
    data = yaml.safe_load(candidate_path.read_text())
    candidate = data.get("extrinsic_candidate", {}) if isinstance(data, dict) else {}
    required = ("valid", "accepted_for_session", "session_only", "parent_frame",
                "child_frame", "translation", "quaternion_xyzw", "metrics")
    if any(key not in candidate for key in required):
        raise RuntimeError("candidate extrinsic YAML is incomplete")
    if not candidate["valid"] or not candidate["accepted_for_session"]:
        raise RuntimeError("candidate extrinsic is not accepted for this session")
    if not candidate["session_only"]:
        raise RuntimeError("persistent extrinsics are forbidden in a shadow session")
    if not candidate["parent_frame"] or not candidate["child_frame"]:
        raise RuntimeError("candidate extrinsic has empty frames")
    child_frame = candidate["child_frame"]
    if child_frame.endswith("_optical_frame"):
        raise RuntimeError("candidate extrinsic child_frame must not be an optical frame")
    if candidate["parent_frame"] != "world" or child_frame != CAMERA_ROOT_FRAME:
        raise RuntimeError("candidate frames must be world -> d435i_link")
    if candidate.get("camera_optical_frame") != CAMERA_OPTICAL_FRAME:
        raise RuntimeError("candidate must preserve d435i_depth_optical_frame separately")
    if len(candidate["translation"]) != 3 or len(candidate["quaternion_xyzw"]) != 4:
        raise RuntimeError("candidate extrinsic has invalid transform dimensions")
    return candidate


def _validate_hardware_output(value):
    if _as_bool(value):
        raise RuntimeError("hardware_output=true is forbidden by the shadow-session contract")


def _validate_shadow_coverage_profile(path):
    """Validate an explicitly requested, session-only coverage hypothesis."""
    profile_path = Path(str(path).strip())
    if not str(path).strip() or not profile_path.is_file():
        raise RuntimeError("shadow_coverage_profile_file must name an existing YAML file")
    try:
        data = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise RuntimeError("shadow coverage profile must be valid YAML") from error
    if not isinstance(data, dict):
        raise RuntimeError("shadow coverage profile must be a YAML mapping")
    metadata = data.get("shadow_coverage_metadata")
    if not isinstance(metadata, dict) or metadata.get("session_only") is not True or \
            metadata.get("not_for_safety") is not True or \
            metadata.get("profile_kind") != "shadow_coverage":
        raise RuntimeError("shadow coverage profile requires session_only/not_for_safety metadata")
    try:
        parameters = data["jaco_self_filter_node"]["ros__parameters"]
        starts = parameters["capsule_start_frames"]
        ends = parameters["capsule_end_frames"]
        radii = parameters["capsule_inner_radii_m"]
    except (KeyError, TypeError) as error:
        raise RuntimeError("shadow coverage profile has no self-filter capsule parameters") from error
    if not isinstance(parameters, dict) or set(parameters) != SHADOW_COVERAGE_PARAMETER_KEYS:
        raise RuntimeError("shadow coverage profile may override only capsule frame and radius lists")
    if not isinstance(starts, list) or len(starts) != SHADOW_COVERAGE_CAPSULE_COUNT or \
            len(ends) != len(starts) or len(radii) != len(starts):
        raise RuntimeError("shadow coverage profile must contain exactly six consistent capsules")
    pairs = set()
    for start, end, radius in zip(starts, ends, radii):
        if not isinstance(start, str) or not start or not isinstance(end, str) or not end:
            raise RuntimeError("shadow coverage capsule frames must be non-empty strings")
        if start == end or (start, end) in pairs:
            raise RuntimeError("shadow coverage capsule frames must be non-degenerate and unique")
        if not isinstance(radius, (int, float)) or isinstance(radius, bool) or \
                not math.isfinite(radius) or radius <= 0.0:
            raise RuntimeError("shadow coverage capsule radii must be finite and positive")
        pairs.add((start, end))
    return {key: parameters[key] for key in sorted(SHADOW_COVERAGE_PARAMETER_KEYS)}


def _validate_session_epsilon(value):
    """Require an explicit finite geometric uncertainty for an enabled filter."""
    text = str(value).strip()
    if not text:
        raise RuntimeError("session_epsilon_total_m is required when self-filter is enabled")
    try:
        epsilon = float(text)
    except (TypeError, ValueError) as error:
        raise RuntimeError("session_epsilon_total_m must be numeric") from error
    if not math.isfinite(epsilon):
        raise RuntimeError("session_epsilon_total_m must be finite")
    if epsilon < 0.0:
        raise RuntimeError("session_epsilon_total_m must be non-negative")
    return epsilon


def _validate_crop_values(enable_crop, values):
    """Validate finite world-frame bounds before any child launch starts."""
    bounds = {}
    for name in CROP_LIMIT_NAMES:
        try:
            value = float(values[name])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f"{name} must be numeric") from error
        if not math.isfinite(value):
            raise RuntimeError(f"{name} must be finite")
        bounds[name] = value
    if _as_bool(enable_crop) and not (
            bounds["min_x"] < bounds["max_x"] and
            bounds["min_y"] < bounds["max_y"] and
            bounds["min_z"] < bounds["max_z"]):
        raise RuntimeError("enabled crop requires min_x < max_x, min_y < max_y, min_z < max_z")
    return bounds


def _validate_crop_bounds(context):
    _validate_crop_values(
        LaunchConfiguration("enable_crop").perform(context),
        {name: LaunchConfiguration(name).perform(context) for name in CROP_LIMIT_NAMES})
    return []


def _guard_and_session_tf(context):
    _validate_hardware_output(LaunchConfiguration("hardware_output").perform(context))
    _validate_crop_bounds(context)
    coverage_profile = LaunchConfiguration("shadow_coverage_profile_file").perform(context)
    if str(coverage_profile).strip():
        _validate_shadow_coverage_profile(coverage_profile)
    if _as_bool(LaunchConfiguration("enable_self_filter").perform(context)):
        _validate_session_epsilon(
            LaunchConfiguration("session_epsilon_total_m").perform(context))
    if not _as_bool(LaunchConfiguration("publish_session_extrinsic").perform(context)):
        return []
    candidate = _validate_candidate(LaunchConfiguration("candidate_extrinsic_file").perform(context))
    translation = candidate["translation"]
    quaternion = candidate["quaternion_xyzw"]
    return [Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435i_session_candidate_extrinsic",
        arguments=[
            "--x", str(translation[0]), "--y", str(translation[1]), "--z", str(translation[2]),
            "--qx", str(quaternion[0]), "--qy", str(quaternion[1]),
            "--qz", str(quaternion[2]), "--qw", str(quaternion[3]),
            "--frame-id", candidate["parent_frame"], "--child-frame-id", candidate["child_frame"],
        ],
    )]


def generate_launch_description():
    hardware_launch = PathJoinSubstitution([
        FindPackageShare("thesis_hardware"), "launch", "jaco_read_only_state.launch.py"])
    camera_launch = PathJoinSubstitution([
        FindPackageShare("thesis_jaco_perception"),
        "launch",
        "d435i_shadow_camera_pipeline.launch.py",
    ])
    shadow_launch = PathJoinSubstitution([
        FindPackageShare("thesis_jaco_perception"), "launch", "jaco_rgbd_shadow.launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("start_jaco_read_only", default_value="false"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("enable_self_filter", default_value="false"),
        DeclareLaunchArgument("enable_mesh_shadow_comparison", default_value="false"),
        DeclareLaunchArgument("session_epsilon_total_m", default_value=""),
        DeclareLaunchArgument("publish_session_extrinsic", default_value="false"),
        DeclareLaunchArgument("candidate_extrinsic_file", default_value=""),
        DeclareLaunchArgument("hardware_output", default_value="false"),
        DeclareLaunchArgument("shadow_coverage_profile_file", default_value=""),
        DeclareLaunchArgument("enable_crop", default_value="false"),
        DeclareLaunchArgument("min_x", default_value="-1.5"),
        DeclareLaunchArgument("max_x", default_value="1.5"),
        DeclareLaunchArgument("min_y", default_value="-1.5"),
        DeclareLaunchArgument("max_y", default_value="1.5"),
        DeclareLaunchArgument("min_z", default_value="-1.0"),
        DeclareLaunchArgument("max_z", default_value="2.5"),
        OpaqueFunction(function=_guard_and_session_tf),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            condition=IfCondition(LaunchConfiguration("start_jaco_read_only")),
            launch_arguments={"hardware_output": "false"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration("start_camera")),
            launch_arguments={
                "start_rviz": LaunchConfiguration("start_rviz"),
                "enable_crop": LaunchConfiguration("enable_crop"),
                "min_x": LaunchConfiguration("min_x"),
                "max_x": LaunchConfiguration("max_x"),
                "min_y": LaunchConfiguration("min_y"),
                "max_y": LaunchConfiguration("max_y"),
                "min_z": LaunchConfiguration("min_z"),
                "max_z": LaunchConfiguration("max_z"),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shadow_launch),
            condition=IfCondition(LaunchConfiguration("enable_self_filter")),
            launch_arguments={
                "enabled": "true", "input_topic": "/thesis/perception/points_filtered",
                "expected_frame": "world",
                "camera_optical_frame": CAMERA_OPTICAL_FRAME,
                "epsilon_total_m": LaunchConfiguration("session_epsilon_total_m"),
                "shadow_coverage_profile_file": LaunchConfiguration(
                    "shadow_coverage_profile_file"),
                "enable_mesh_shadow_comparison": LaunchConfiguration("enable_mesh_shadow_comparison"),
            }.items()),
    ])
