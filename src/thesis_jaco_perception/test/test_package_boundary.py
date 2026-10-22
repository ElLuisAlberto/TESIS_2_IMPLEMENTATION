"""Protect pure-library and Phase 3 shadow-launch safety boundaries."""

from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_pure_geometry_library_has_no_runtime_sensor_or_hardware_dependencies():
    """Phase 1 must remain a deterministic library with no ROS runtime node."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    implementation = (
        PACKAGE_ROOT / "src" / "self_filter_geometry.cpp"
    ).read_text(encoding="utf-8")
    header = (
        PACKAGE_ROOT / "include" / "thesis_jaco_perception"
        / "self_filter_geometry.hpp"
    ).read_text(encoding="utf-8")
    combined = "\n".join((implementation, header)).lower()

    for forbidden in (
        "rclcpp",
        "pcl",
        "tf2",
        "gazebo",
        "realsense",
        "hardware_output",
        "create_subscription",
        "create_publisher",
    ):
        assert forbidden not in combined
    assert "add_library(self_filter_geometry" in cmake
    assert "add_library(shadow_filter_logic" in cmake


def test_shadow_node_output_boundary_excludes_the_supervisor_input():
    """The shadow node may publish diagnostics and clouds, never safety input."""
    node = (PACKAGE_ROOT / "src" / "jaco_self_filter_node.cpp").read_text(
        encoding="utf-8"
    )
    assert "/thesis/obstacle_input" not in node
    assert "/thesis/jaco_perception/points_self_confirmed" in node
    assert "/thesis/jaco_perception/points_external_candidate" in node
    assert "/thesis/jaco_perception/points_occluded_unknown" in node
    assert "sensor_msgs::msg::PointCloud2 output = input" in node
    assert "output.header.frame_id = kWorldFrame" in node
    assert "JointState" not in node


def test_observation_probe_is_passive_and_has_no_safety_or_tf_boundary():
    """The future camera probe may observe and diagnose only."""
    probe = (PACKAGE_ROOT / "src" / "d435i_observation_probe.cpp").read_text(
        encoding="utf-8"
    ).lower()
    for forbidden in (
        "/thesis/obstacle_input",
        "thesis_hardware",
        "hardware_output",
        "gazebo",
        "jointstate",
        "static_transform_publisher",
        "tf2",
        "create_publisher<sensor_msgs::msg::pointcloud2>",
        "create_publisher<thesis_interfaces::msg::obstacle>",
    ):
        assert forbidden not in probe
    assert "sensordataqos().keep_last(1)" in probe
    assert "d435i_observation_diagnostics" in probe


def test_shadow_configuration_is_intentionally_invalid_until_calibrated():
    """The package default must never impersonate a physical calibration."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "jaco_d435i_shadow.yaml").read_text(
            encoding="utf-8"
        )
    )["jaco_self_filter_node"]["ros__parameters"]
    assert config["enabled"] is False
    assert config["input_topic"] == "/thesis/perception/points_world"
    assert config["expected_frame"] == "world"
    assert config["camera_optical_frame"] == ""
    assert config["capsule_start_frames"] == []
    assert config["capsule_end_frames"] == []
    assert config["capsule_inner_radii_m"] == []
    assert config["epsilon_total_m"] == -1.0
    assert config["max_cloud_age_sec"] == 0.5
    assert config["tf_timeout_sec"] == 0.05
    assert not any("extrinsic" in key.lower() for key in config)


def test_shadow_launch_starts_only_isolated_filters_with_mesh_opt_in():
    """Only the self-filter is default; mesh comparison requires explicit opt-in."""
    launch = (
        PACKAGE_ROOT / "launch" / "jaco_rgbd_shadow.launch.py"
    ).read_text(encoding="utf-8")
    assert launch.count("Node(") == 2
    assert 'package="thesis_jaco_perception"' in launch
    assert 'executable="jaco_self_filter_node"' in launch
    assert 'executable="jaco_mesh_shadow_node"' in launch
    assert '"enable_mesh_shadow_comparison", default_value="false"' in launch
    mesh_section = launch.split("def _create_mesh_shadow_node(context):", 1)[1].split(
        "def _validate_mesh_robot_description", 1)[0]
    assert "if not _as_bool(LaunchConfiguration(\"enable_mesh_shadow_comparison\")" in mesh_section
    assert "return []" in mesh_section.split("return [Node(", 1)[0]
    assert 'DeclareLaunchArgument(\n            "config_file"' in launch
    assert "jaco_d435i_shadow.yaml" in launch
    assert "jaco_self_filter_capsules.yaml" in launch
    assert '"capsule_profile_file"' in launch
    for forbidden in (
        "realsense",
        "rviz",
        "gazebo",
        "thesis_hardware",
        "robot_state_publisher",
        "static_transform_publisher",
        "/thesis/obstacle_input",
        "hardware_output",
        "armed",
    ):
        assert forbidden not in launch.lower()
