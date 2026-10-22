import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools/perception/mesh_arm_auto_mask_offline.py"
spec = importlib.util.spec_from_file_location("mesh_arm_auto_mask_offline_8s", SCRIPT)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
SNAPSHOT = ROOT / "evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot"


def test_world_link_camera_composition_on_known_point():
    world_link = np.eye(4)
    world_link[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    world_link[:3, 3] = [0.3, -0.2, 0.1]
    link_camera = np.eye(4)
    link_camera[:3, :3] = Rotation.from_euler("x", 90, degrees=True).as_matrix()
    link_camera[:3, 3] = [0.1, 0, 0]
    camera_point = np.array([0.2, -0.1, 1.0, 1.0])
    world_point = world_link @ link_camera @ camera_point
    recovered = np.linalg.inv(world_link @ link_camera) @ world_point
    np.testing.assert_allclose(recovered, camera_point, atol=1e-12)
    uv = np.array([100*recovered[0]/recovered[2]+320,
                   100*recovered[1]/recovered[2]+240])
    np.testing.assert_allclose(uv, [340, 230], atol=1e-12)


def test_frozen_optical_transforms_are_inverse_but_link_chain_is_absent():
    metadata = json.loads((SNAPSHOT / "metadata.json").read_text())
    links = json.loads((SNAPSHOT / "link_transforms_at_stamp.json").read_text())
    world_color = geometry.matrix_from_transform(
        metadata["world_to_color_optical_at_cloud_stamp"])
    color_world = geometry.matrix_from_transform(
        metadata["color_optical_from_world_at_cloud_stamp"])
    np.testing.assert_allclose(world_color @ color_world, np.eye(4), atol=1e-9)
    assert all(tf["source_frame"] != "d435i_link" for tf in links["transforms"])
    assert "world_to_d435i_link_at_cloud_stamp" not in metadata
    assert "d435i_link_to_color_optical_at_cloud_stamp" not in metadata
