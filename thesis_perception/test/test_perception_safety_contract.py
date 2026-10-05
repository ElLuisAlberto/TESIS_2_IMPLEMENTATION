# Copyright 2026 Luis Alberto Munoz Marin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Protect the isolation boundary of standalone RGB-D perception."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_candidate_output_is_not_the_supervisor_input():
    """Keep unvalidated detections away from preventive decisions."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    topic = config["obstacle_extractor"]["ros__parameters"][
        "obstacle_topic"
    ]
    assert topic == "/thesis/perception/obstacle_candidate"
    assert topic != "/thesis/obstacle_input"


def test_standalone_defaults_do_not_invent_an_extrinsic():
    """Require an explicit opt-in before a world-to-camera TF is published."""
    launch_text = (
        PACKAGE_ROOT / "launch" / "perception_processing.launch.py"
    ).read_text(encoding="utf-8")
    assert '"publish_static_extrinsic"' in launch_text
    assert 'default_value="false"' in launch_text


def test_fixed_camera_does_not_require_imu_by_default():
    """Do not fail RGB-D geometry because optional inertial data is absent."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    health = config["perception_health"]["ros__parameters"]
    assert health["imu_required"] is False

    standalone_launch = (
        PACKAGE_ROOT / "launch" / "d435i_standalone.launch.py"
    ).read_text(encoding="utf-8")
    assert '"enable_gyro": LaunchConfiguration("enable_imu")' in (
        standalone_launch
    )
    assert '"enable_accel": LaunchConfiguration("enable_imu")' in (
        standalone_launch
    )
    enable_imu_declaration = standalone_launch.split(
        '"enable_imu",', maxsplit=1
    )[1]
    assert 'default_value="false"' in enable_imu_declaration


def test_standalone_preserves_source_frame_and_clock():
    """Avoid fabricated geometry or simulated timestamps in standalone mode."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "perception.yaml").read_text(
            encoding="utf-8"
        )
    )
    preprocessor = config["pointcloud_preprocessor"]["ros__parameters"]
    assert preprocessor["target_frame"] == ""
    assert preprocessor["restamp_output"] is False
    assert preprocessor["min_valid_range_m"] >= 0.0
    assert preprocessor["max_valid_range_m"] > (
        preprocessor["min_valid_range_m"]
    )


def test_standalone_exposes_workspace_filter_arguments():
    """Allow workspace bounds to be tuned without editing source files."""
    launch_text = (
        PACKAGE_ROOT / "launch" / "d435i_standalone.launch.py"
    ).read_text(encoding="utf-8")
    arguments = (
        "enable_crop",
        "enable_voxel",
        "voxel_leaf_size",
        "min_x",
        "max_x",
        "min_y",
        "max_y",
        "min_z",
        "max_z",
    )
    for argument in arguments:
        assert launch_text.count(f'"{argument}"') >= 2


def test_standalone_uses_sensor_qos_and_depth_stabilization():
    """Keep large clouds low-latency and stabilize fixed-camera depth."""
    launch_text = (
        PACKAGE_ROOT / "launch" / "d435i_standalone.launch.py"
    ).read_text(encoding="utf-8")
    assert 'name="pointcloud.pointcloud_qos"' in launch_text
    assert 'value="SENSOR_DATA"' in launch_text
    assert (
        '"spatial_filter.enable": LaunchConfiguration(' in launch_text
    )
    assert (
        '"temporal_filter.enable": LaunchConfiguration(' in launch_text
    )
    for argument in (
        "enable_decimation_filter",
        "decimation_magnitude",
        "enable_spatial_filter",
        "enable_temporal_filter",
    ):
        assert launch_text.count(f'"{argument}"') >= 2


def test_large_cloud_consumers_keep_only_the_newest_sample():
    """Avoid stale obstacle decisions when processing briefly falls behind."""
    preprocessor = (
        PACKAGE_ROOT / "src" / "pointcloud_preprocessor_node.cpp"
    ).read_text(encoding="utf-8")
    extractor = (
        PACKAGE_ROOT / "src" / "obstacle_extractor_node.cpp"
    ).read_text(encoding="utf-8")
    rviz = (
        PACKAGE_ROOT / "rviz" / "d435i_perception.rviz"
    ).read_text(encoding="utf-8")
    assert "SensorDataQoS().keep_last(1)" in preprocessor
    assert "SensorDataQoS().keep_last(1)" in extractor
    assert "Depth: 1" in rviz
