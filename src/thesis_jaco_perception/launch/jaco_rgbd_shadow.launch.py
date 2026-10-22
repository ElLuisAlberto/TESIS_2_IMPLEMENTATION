"""Start only the disabled-by-default JACO RGB-D self-filter shadow node."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SHADOW_COVERAGE_CAPSULE_COUNT = 6
SHADOW_COVERAGE_PARAMETER_KEYS = {
    "capsule_start_frames", "capsule_end_frames", "capsule_inner_radii_m"}


def _validate_capsule_profile(path):
    profile_path = Path(path)
    if not path or not profile_path.is_file():
        raise RuntimeError("capsule_profile_file must name an existing capsule YAML")
    data = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    try:
        parameters = data["jaco_self_filter_node"]["ros__parameters"]
        starts = parameters["capsule_start_frames"]
        ends = parameters["capsule_end_frames"]
        radii = parameters["capsule_inner_radii_m"]
    except (KeyError, TypeError) as error:
        raise RuntimeError("capsule profile has no self-filter capsule parameters") from error
    if not isinstance(starts, list) or not starts or len(ends) != len(starts) or len(radii) != len(starts):
        raise RuntimeError("capsule lists must be non-empty and equal-sized")
    pairs = set()
    for start, end, radius in zip(starts, ends, radii):
        if not isinstance(start, str) or not start or not isinstance(end, str) or not end:
            raise RuntimeError("capsule frames must be non-empty strings")
        if start == end or (start, end) in pairs:
            raise RuntimeError("capsule frame pairs must be non-degenerate and unique")
        if not isinstance(radius, (int, float)) or isinstance(radius, bool) or \
                not math.isfinite(radius) or radius <= 0.0:
            raise RuntimeError("capsule radii must be positive")
        pairs.add((start, end))
    return parameters


def _validate_shadow_coverage_profile(path):
    """Accept only a deliberately marked, six-capsule session hypothesis."""
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


def _create_self_filter_node(context):
    """Build the node after validation, never passing coverage YAML as a params file."""
    coverage_profile = LaunchConfiguration("shadow_coverage_profile_file").perform(context)
    parameters = [
        LaunchConfiguration("config_file"),
        LaunchConfiguration("capsule_profile_file"),
    ]
    if str(coverage_profile).strip():
        parameters.append(_validate_shadow_coverage_profile(coverage_profile))
    else:
        _validate_capsule_profile(LaunchConfiguration("capsule_profile_file").perform(context))
    return [Node(
        package="thesis_jaco_perception",
        executable="jaco_self_filter_node",
        name="jaco_self_filter_node",
        output="screen",
        parameters=parameters + [{
            "enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
            "input_topic": LaunchConfiguration("input_topic"),
            "expected_frame": LaunchConfiguration("expected_frame"),
            "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
            "epsilon_total_m": ParameterValue(
                LaunchConfiguration("epsilon_total_m"), value_type=float),
        }],
    )]


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _create_mesh_shadow_node(context):
    """Expand the canonical read-only JACO2 Xacro only for mesh comparison."""
    if not _as_bool(LaunchConfiguration("enable_mesh_shadow_comparison").perform(context)):
        return []
    xacro_file = str(Path(get_package_share_directory("thesis_description")) /
                     "urdf" / "j2n6s300_standalone.xacro")
    xml = Command(["xacro ", xacro_file]).perform(context)
    _validate_mesh_robot_description(xml)
    return [Node(
        package="thesis_jaco_perception", executable="jaco_mesh_shadow_node",
        name="jaco_mesh_shadow_node", output="screen",
        parameters=[{
            "input_topic": "/thesis/perception/points_filtered",
            "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
            "robot_description": ParameterValue(xml, value_type=str),
            "epsilon_mesh_total_m": ParameterValue(
                LaunchConfiguration("mesh_epsilon_total_m"), value_type=float),
        }])]


def _validate_mesh_robot_description(xml):
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as error:
        raise RuntimeError("MESH_SHADOW_ROBOT_DESCRIPTION_INVALID") from error
    collisions = root.findall(".//collision/geometry/mesh")
    names = {link.get("name") for link in root.findall("link")}
    required = {"j2n6s300_link_base", "j2n6s300_link_6", "j2n6s300_link_finger_1"}
    if root.tag != "robot" or not xml.strip() or not collisions or not required.issubset(names):
        raise RuntimeError("MESH_SHADOW_ROBOT_DESCRIPTION_INVALID")


def generate_launch_description():
    """Create the isolated shadow composition without sensors or robot stack."""
    default_config = PathJoinSubstitution([
        FindPackageShare("thesis_jaco_perception"),
        "config",
        "jaco_d435i_shadow.yaml",
    ])
    default_capsules = PathJoinSubstitution([
        FindPackageShare("thesis_jaco_perception"),
        "config",
        "jaco_self_filter_capsules.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description=(
                "Validated future shadow configuration; the packaged default "
                "remains disabled."
            ),
        ),
        DeclareLaunchArgument(
            "capsule_profile_file",
            default_value=default_capsules,
            description="Validated JACO2 capsule-frame profile required by the filter.",
        ),
        DeclareLaunchArgument(
            "shadow_coverage_profile_file",
            default_value="",
            description=(
                "Explicit session-only, not-for-safety coverage hypothesis; empty preserves "
                "the historical capsule profile."
            ),
        ),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("input_topic", default_value="/thesis/perception/points_world"),
        DeclareLaunchArgument("expected_frame", default_value="world"),
        DeclareLaunchArgument("camera_optical_frame", default_value=""),
        DeclareLaunchArgument("epsilon_total_m", default_value="-1.0"),
        DeclareLaunchArgument("enable_mesh_shadow_comparison", default_value="false"),
        DeclareLaunchArgument("mesh_epsilon_total_m", default_value="0.07"),
        OpaqueFunction(function=_create_mesh_shadow_node),
        OpaqueFunction(function=_create_self_filter_node),
    ])
