import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[3] / "tools/perception/mesh_arm_auto_mask_offline.py"
spec = importlib.util.spec_from_file_location("mesh_arm_auto_mask_offline", SCRIPT)
mask = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mask)


def test_frozen_geometry_and_projection_inputs():
    metadata, transforms, manifest, hashes = mask.verify_frozen_snapshot()
    assert len(manifest) == len(transforms["transforms"]) == 13
    assert len(hashes["collision_meshes_sha256"]) == 8
    assert metadata["reference_stamp_ns"] == transforms["reference_stamp_ns"]
    identity = mask.matrix_from_transform({
        "translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]})
    np.testing.assert_allclose(identity, np.eye(4))


def test_projects_a_frozen_triangle_with_camera_intrinsics():
    dtype = np.dtype([("link", "<u2"), ("xyz", "<f4", (3, 3))])
    triangle = np.zeros(1, dtype=dtype)
    triangle["xyz"][0] = [[0, 0, 1], [0.1, 0, 1], [0, 0.1, 1]]
    metadata = {
        "color_optical_from_world_at_cloud_stamp": {
            "translation_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
        "camera_info_k": [100, 0, 320, 0, 100, 240, 0, 0, 1],
        "camera_info_d": [0, 0, 0, 0, 0],
    }
    pixels, valid = mask.projected_triangles(triangle, metadata)
    assert valid.tolist() == [True]
    np.testing.assert_allclose(pixels[0], [[320, 240], [330, 240], [320, 250]])


def test_binary_connectivity_and_external_component_rejection():
    seed = np.zeros((20, 20), dtype=np.uint8)
    seed[4:7, 4:7] = 255
    candidate = np.zeros_like(seed)
    candidate[3:8, 3:8] = 255
    candidate[14:17, 14:17] = 255
    kept, removed = mask.connected_to_seed(candidate, seed)
    assert removed == 1
    assert set(np.unique(kept)) == {0, 255}
    assert int((kept > 0).sum()) == 25
    assert kept[15, 15] == 0


def test_rejects_small_or_outside_band_mask():
    silhouette = np.zeros((40, 40), dtype=np.uint8)
    silhouette[10:30, 10:30] = 255
    band = np.zeros_like(silhouette)
    band[8:32, 8:32] = 255
    candidate = np.zeros_like(silhouette)
    candidate[0:10, 0:10] = 255
    entries = [{"link": "j2n6s300_link_base"}]
    metrics = mask.evaluate_mask(candidate, silhouette, band, silhouette[None], entries)
    assert not metrics["tests"]["nonempty"]
    assert not metrics["tests"]["outside_band_lt_2_percent"]
