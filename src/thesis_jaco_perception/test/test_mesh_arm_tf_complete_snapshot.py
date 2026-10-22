import importlib.util
from pathlib import Path
import subprocess

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


SCRIPT = Path(__file__).resolve().parents[3] / "tools/perception/mesh_arm_tf_complete_snapshot_once.py"
spec = importlib.util.spec_from_file_location("mesh_arm_tf_complete_snapshot_once", SCRIPT)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
FIT_SCRIPT = Path(__file__).resolve().parents[3] / "tools/perception/fase8t_depth_silhouette_fit.py"
fit_spec = importlib.util.spec_from_file_location("fase8t_depth_silhouette_fit", FIT_SCRIPT)
fit = importlib.util.module_from_spec(fit_spec)
fit_spec.loader.exec_module(fit)


def transform(target, source, matrix, stamp):
    return {"target_frame": target, "source_frame": source,
            "returned_stamp_ns": stamp, "translation_m": matrix[:3, 3].tolist(),
            "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()}


def sample_chain():
    stamp = 123456789
    link = np.eye(4)
    link[:3, :3] = Rotation.from_euler("xyz", [0.1, 0.2, -0.3]).as_matrix()
    link[:3, 3] = [0.1, -0.6, 0.3]
    optical = np.eye(4)
    optical[:3, :3] = Rotation.from_euler("xyz", [-1.5, 0.01, 0.03]).as_matrix()
    optical[:3, 3] = [0.02, 0, 0]
    color = link @ optical
    frame = "d435i_color_optical_frame"
    transforms = {
        "world_depth": transform("world", "d435i_depth_optical_frame", color, stamp),
        "color_world": transform(frame, "world", np.linalg.inv(color), stamp),
        "world_color": transform("world", frame, color, stamp),
        "world_camera_link": transform("world", "d435i_link", link, stamp),
        "camera_link_color": transform("d435i_link", frame, optical, 0),
        "color_camera_link": transform(frame, "d435i_link", np.linalg.inv(optical), 0),
        "camera_link_world": transform("d435i_link", "world", np.linalg.inv(link), stamp),
    }
    return transforms, stamp, frame, link, optical


def test_composition_inverse_and_direct_equivalence():
    transforms, stamp, frame, link, optical = sample_chain()
    result = capture.validate_tf_chain(transforms, stamp, frame)
    assert result["translation_error_m"] < 1e-12
    assert result["rotation_error_rad"] < 1e-12
    point = np.array([0.2, -0.1, 1.0, 1.0])
    np.testing.assert_allclose(link @ optical @ point,
                               capture.rigid_matrix(transforms["world_color"]) @ point,
                               atol=1e-12)


def test_recover_known_world_perturbation():
    transforms, stamp, frame, link, optical = sample_chain()
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler("xyz", [0.03, -0.02, 0.04]).as_matrix()
    delta[:3, 3] = [0.02, -0.01, 0.03]
    candidate_color = delta @ link @ optical
    recovered = candidate_color @ np.linalg.inv(optical) @ np.linalg.inv(link)
    np.testing.assert_allclose(recovered, delta, atol=1e-12)


def test_reject_nonrigid_or_inconsistent_tf():
    transforms, stamp, frame, _, _ = sample_chain()
    transforms["camera_link_color"]["quaternion_xyzw"][3] = 2.0
    with pytest.raises(ValueError, match="Nonunit"):
        capture.validate_tf_chain(transforms, stamp, frame)
    transforms, stamp, frame, _, _ = sample_chain()
    transforms["world_color"]["translation_m"][0] += 0.001
    with pytest.raises(ValueError, match="composition"):
        capture.validate_tf_chain(transforms, stamp, frame)


def test_zbuffer_occludes_far_triangle_and_cost_prefers_matching_edge(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[3] / "tools/perception/mesh_zbuffer_offline.cpp"
    executable = tmp_path / "mesh_zbuffer_offline"
    subprocess.run(["g++", "-O2", "-std=c++17", str(source), "-o", str(executable)], check=True)
    monkeypatch.setattr(fit, "RENDERER", executable)
    dtype = np.dtype([("link", "<u2"), ("xyz", "<f4", (3, 3))])
    triangles = np.zeros(2, dtype=dtype)
    triangles[0]["link"] = 1
    triangles[0]["xyz"] = [[-0.5, -0.5, 2], [0.5, -0.5, 2], [0, 0.5, 2]]
    triangles[1]["link"] = 2
    triangles[1]["xyz"] = [[-0.5, -0.5, 1], [0.5, -0.5, 1], [0, 0.5, 1]]
    renderer = fit.Renderer(triangles, 64, 64, [40, 40, 32, 32])
    try:
        depth, labels = renderer.render(np.eye(4))
    finally:
        renderer.close()
    assert labels[32, 32] == 2
    assert depth[32, 32] == pytest.approx(1.0)
    occupied = (labels != 255).astype(np.uint8)
    contour = fit.cv2.morphologyEx(occupied, fit.cv2.MORPH_GRADIENT,
                                   fit.cv2.getStructuringElement(fit.cv2.MORPH_ELLIPSE, (3, 3))) > 0
    support = np.ones_like(contour, dtype=bool)
    matching_distance = fit.distance_transform_edt(~contour).astype(np.float32)
    mismatch_distance = np.full_like(matching_distance, 20.0)
    matching, coverage, _ = fit.contour_and_cost(labels, contour, support, matching_distance)
    mismatching, _, _ = fit.contour_and_cost(labels, contour, support, mismatch_distance)
    assert matching < mismatching
    assert coverage == pytest.approx(1.0)
