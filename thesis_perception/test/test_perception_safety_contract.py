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
