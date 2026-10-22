#!/usr/bin/env python3
"""Offline, conservative 8T mesh/depth-edge fit; never publishes a transform."""

import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
SNAPSHOT = ROOT / "evidence/codex_reports/2026-10-22_fase8t_tf_complete_snapshot"
RENDERER = Path("/tmp/mesh_zbuffer_offline_8t")
spec = importlib.util.spec_from_file_location("mesh_arm_geometry_8t_fit",
                                          ROOT / "tools/perception/mesh_arm_auto_mask_offline.py")
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def ensure_offline_binaries():
    if not geometry.EXPORTER.is_file():
        subprocess.run(["g++", "-O2", "-std=c++17",
                        str(ROOT / "tools/perception/export_frozen_collision_triangles.cpp"),
                        "-lassimp", "-o", str(geometry.EXPORTER)], check=True)
    if not RENDERER.is_file():
        subprocess.run(["g++", "-O3", "-std=c++17",
                        str(ROOT / "tools/perception/mesh_zbuffer_offline.cpp"),
                        "-o", str(RENDERER)], check=True)


def matrix(tf):
    return geometry.matrix_from_transform(tf)


def verify():
    capture_hashes = json.loads((SNAPSHOT / "capture_hashes.json").read_text())
    for name, expected in capture_hashes.items():
        actual = hashlib.sha256((SNAPSHOT / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"Frozen 8T file hash changed: {name}")
    metadata = json.loads((SNAPSHOT / "metadata.json").read_text())
    transforms = json.loads((SNAPSHOT / "link_transforms_at_stamp.json").read_text())
    manifest = json.loads((SNAPSHOT / "collision_mesh_manifest.json").read_text())
    hashes = json.loads((SNAPSHOT / "hashes.json").read_text())
    if hashes != metadata["geometry_hashes"]:
        raise ValueError("Geometry hash manifest changed")
    if geometry.digest(SNAPSHOT / "robot_description.urdf") != hashes["robot_description_sha256"]:
        raise ValueError("URDF hash changed")
    for name, value in hashes["collision_meshes_sha256"].items():
        if geometry.digest(SNAPSHOT / "meshes" / Path(name).name) != value:
            raise ValueError(f"Mesh hash changed: {name}")
    stamp = metadata["reference_stamp_ns"]
    if (transforms["reference_stamp_ns"] != stamp or len(transforms["transforms"]) != 13 or
            len(manifest) != 13 or metadata["rgb_stamp_ns"] != stamp or
            metadata["camera_info_stamp_ns"] != stamp):
        raise ValueError("Capture stamps or link count changed")
    for tf in transforms["transforms"]:
        if tf["returned_stamp_ns"] != stamp:
            raise ValueError("Link TF stamp changed")
    checks = metadata["composition_check"]
    if checks["translation_error_m"] > 1e-6 or checks["rotation_error_rad"] > 1e-6:
        raise ValueError("Frozen TF composition failed")
    if any(abs(value) > 1e-12 for value in metadata["camera_info_d"]):
        raise ValueError("Renderer requires zero frozen lens distortion")
    return metadata, transforms, manifest, hashes


def project_cloud(xyz, color_world, k, shape):
    height, width = shape
    points = xyz.astype(np.float64) @ color_world[:3, :3].T + color_world[:3, 3]
    uv = np.rint(np.column_stack((k[0]*points[:, 0]/points[:, 2] + k[2],
                                  k[1]*points[:, 1]/points[:, 2] + k[3]))).astype(np.int32)
    keep = ((points[:, 2] > 0.02) & (uv[:, 0] >= 0) & (uv[:, 0] < width) &
            (uv[:, 1] >= 0) & (uv[:, 1] < height))
    depth = np.full(shape, np.inf, np.float32)
    np.minimum.at(depth, (uv[keep, 1], uv[keep, 0]), points[keep, 2])
    present = np.isfinite(depth)
    distances, nearest = distance_transform_edt(~present, return_indices=True)
    support = distances <= 6
    interpolated = depth[nearest[0], nearest[1]]
    boundary = cv2.morphologyEx(support.astype(np.uint8), cv2.MORPH_GRADIENT,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))) > 0
    gradient = np.zeros(shape, bool)
    gradient[:, 1:] |= support[:, 1:] & support[:, :-1] & (
        np.abs(interpolated[:, 1:] - interpolated[:, :-1]) > 0.08)
    gradient[1:, :] |= support[1:, :] & support[:-1, :] & (
        np.abs(interpolated[1:, :] - interpolated[:-1, :]) > 0.08)
    edges = boundary | gradient
    return edges, support, int(keep.sum())


def transform_candidate(params, world_link, link_color):
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_rotvec(params[3:]).as_matrix()
    delta[:3, 3] = params[:3]
    candidate_link = delta @ world_link
    return candidate_link, candidate_link @ link_color


class Renderer:
    def __init__(self, triangles, width, height, intrinsic):
        aligned = np.zeros(len(triangles), dtype=np.dtype([
            ("link", "<u2"), ("padding", "<u2"), ("xyz", "<f4", (3, 3))]))
        aligned["link"] = triangles["link"]
        aligned["xyz"] = triangles["xyz"]
        self.width, self.height = width, height
        self.process = subprocess.Popen([str(RENDERER)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE)
        self.process.stdin.write(struct.pack("<III4d", width, height, len(triangles), *intrinsic))
        self.process.stdin.write(aligned.tobytes())
        self.process.stdin.flush()

    def render(self, world_color):
        color_world = np.linalg.inv(world_color)
        self.process.stdin.write(np.asarray(color_world, dtype="<f8").tobytes())
        self.process.stdin.flush()
        pixels = self.width*self.height
        raw = self.process.stdout.read(5*pixels)
        if len(raw) != 5*pixels:
            raise RuntimeError("Z-buffer renderer stopped")
        depth = np.frombuffer(raw[:4*pixels], dtype="<f4").reshape(self.height, self.width)
        labels = np.frombuffer(raw[4*pixels:], dtype=np.uint8).reshape(self.height, self.width)
        return depth, labels

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=5)
        self.process.stdout.close()
        if self.process.returncode:
            raise RuntimeError(f"Z-buffer renderer failed: {self.process.returncode}")


def contour_and_cost(labels, observed, support, distance):
    occupied = (labels != 255).astype(np.uint8)
    contour = cv2.morphologyEx(occupied, cv2.MORPH_GRADIENT,
                               cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))) > 0
    valid = contour & support
    count = int(valid.sum())
    if count < 40:
        return 1e3, 0.0, contour
    distances = distance[valid]
    delta = 4.0
    huber = np.where(distances <= delta, 0.5*distances**2,
                     delta*(distances-0.5*delta))
    coverage = float(np.mean(distances <= 5.0))
    # A small missing-support penalty prevents an empty contour from winning.
    support_fraction = count / max(int(contour.sum()), 1)
    cost = float(np.mean(huber) + 8.0*(1.0-support_fraction))
    return cost, coverage, contour


def overlay(rgb, contour, labels):
    result = rgb.copy()
    result[contour] = [0, 255, 0]
    return result


def main():
    metadata, transforms, manifest, hashes = verify()
    if (SNAPSHOT / "optimization_runs.json").exists() and "--recompute" not in sys.argv:
        raise RuntimeError("8T optimization evidence already exists; refusing to overwrite")
    ensure_offline_binaries()
    triangles = geometry.load_world_triangles(SNAPSHOT, manifest, transforms)
    world_link = matrix(metadata["tf_at_cloud_stamp"]["world_camera_link"])
    link_color = matrix(metadata["tf_at_cloud_stamp"]["camera_link_color"])
    world_color = matrix(metadata["tf_at_cloud_stamp"]["world_color"])
    if np.max(np.abs(world_link @ link_color - world_color)) > 1e-6:
        raise ValueError("Frozen direct and compound camera poses disagree")
    xyz = np.load(SNAPSHOT / "cloud_world_xyz.npz")["xyz"]
    rgb = np.asarray(Image.open(SNAPSHOT / "rgb.png").convert("RGB"))
    height, width = rgb.shape[:2]
    k = np.asarray(metadata["camera_info_k"]).reshape(3, 3)
    intrinsic = [float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])]
    observed, support, projected = project_cloud(
        xyz, np.linalg.inv(world_color), intrinsic, (height, width))
    Image.fromarray(observed.astype(np.uint8)*255).save(SNAPSHOT / "depth_edges.png")
    bound = np.array([0.30]*3 + [np.deg2rad(25)]*3)
    starts = np.array([[0,0,0,0,0,0], [0.08,0,0,0,0.08,0],
                       [-0.08,0,0,0,-0.08,0], [0,0.08,0,0.08,0,0]])
    runs = []
    for scale, maxfev, selected in ((0.25, 400, list(range(len(starts)))),
                                    (0.5, 250, None)):
        small_size = (int(width*scale), int(height*scale))
        small_obs = cv2.resize(observed.astype(np.uint8), small_size,
                               interpolation=cv2.INTER_NEAREST) > 0
        small_support = cv2.resize(support.astype(np.uint8), small_size,
                                   interpolation=cv2.INTER_NEAREST) > 0
        dist = distance_transform_edt(~small_obs).astype(np.float32)
        scaled_k = [intrinsic[0]*scale, intrinsic[1]*scale,
                    intrinsic[2]*scale, intrinsic[3]*scale]
        renderer = Renderer(triangles, *small_size, scaled_k)

        def evaluate(params):
            _, candidate = transform_candidate(params, world_link, link_color)
            _, labels = renderer.render(candidate)
            cost, _, _ = contour_and_cost(labels, small_obs, small_support, dist)
            return cost

        try:
            if selected is None:
                selected = [index for index, _ in sorted(enumerate(runs),
                            key=lambda row: row[1]["final_cost"])[:2]]
            for index in selected:
                start = starts[index] if scale == 0.25 else np.asarray(runs[index]["final_parameters"])
                initial_cost = evaluate(start)
                result = minimize(evaluate, start, method="Powell",
                    bounds=[(-x, x) for x in bound],
                    options={"maxfev": maxfev, "xtol": 0.005, "ftol": 0.005})
                final = np.clip(result.x, -bound, bound)
                final_cost = evaluate(final)
                _, candidate = transform_candidate(final, world_link, link_color)
                initial_link, _ = transform_candidate(start, world_link, link_color)
                final_link, _ = transform_candidate(final, world_link, link_color)
                _, labels = renderer.render(candidate)
                _, coverage, _ = contour_and_cost(labels, small_obs, small_support, dist)
                at_limit = bool(np.any(np.abs(final) >= bound - 1e-3))
                entry = {"resolution": list(small_size), "start_index": int(index),
                         "initial_parameters": start.tolist(), "final_parameters": final.tolist(),
                         "initial_world_link_row_major": initial_link.tolist(),
                         "final_world_link_row_major": final_link.tolist(),
                         "initial_cost": float(initial_cost), "final_cost": float(final_cost),
                         "success": bool(result.success), "message": str(result.message),
                         "evaluations": int(result.nfev), "at_limit": at_limit,
                         "edge_coverage": coverage}
                runs.append(entry)
        finally:
            renderer.close()

    full_distance = distance_transform_edt(~observed).astype(np.float32)
    renderer = Renderer(triangles, width, height, intrinsic)
    try:
        _, initial_labels = renderer.render(world_color)
        initial_cost, initial_coverage, initial_contour = contour_and_cost(
            initial_labels, observed, support, full_distance)
        # Reduced-resolution objectives are not comparable with each other. Choose
        # the best hypothesis only after evaluating every run at 640x480.
        for run in runs:
            params = np.asarray(run["final_parameters"])
            _, camera_pose = transform_candidate(params, world_link, link_color)
            _, labels = renderer.render(camera_pose)
            cost, coverage, _ = contour_and_cost(labels, observed, support, full_distance)
            run["validation_640_cost"] = cost
            run["validation_640_coverage"] = coverage
        best_run = min(runs, key=lambda run: run["validation_640_cost"])
        best_params = np.asarray(best_run["final_parameters"])
        candidate_link, candidate_color = transform_candidate(best_params, world_link, link_color)
        _, final_labels = renderer.render(candidate_color)
        final_cost, final_coverage, final_contour = contour_and_cost(
            final_labels, observed, support, full_distance)
    finally:
        renderer.close()
    Image.fromarray(overlay(rgb, initial_contour, initial_labels)).save(SNAPSHOT / "initial_mesh_overlay.png")
    Image.fromarray(overlay(rgb, final_contour, final_labels)).save(SNAPSHOT / "optimized_mesh_overlay.png")
    regions = {}
    for name, ids in {"brazo_superior": [2], "antebrazo": [3],
                      "muneca_efector": list(range(4, 13))}.items():
        values = []
        for labels, contour in ((initial_labels, initial_contour), (final_labels, final_contour)):
            selected = contour & support & np.isin(labels, ids)
            values.append({"edge_pixels": int(selected.sum()),
                           "mean_distance_px": float(full_distance[selected].mean())
                           if np.any(selected) else None,
                           "coverage_within_5px": float(np.mean(full_distance[selected] <= 5))
                           if np.any(selected) else None})
        regions[name] = {"initial": values[0], "final": values[1],
                         "improved": bool(values[0]["mean_distance_px"] is not None and
                                          values[1]["mean_distance_px"] is not None and
                                          values[1]["mean_distance_px"] < values[0]["mean_distance_px"] and
                                          values[1]["edge_pixels"] >= 0.8*values[0]["edge_pixels"] and
                                          values[1]["coverage_within_5px"] >=
                                          values[0]["coverage_within_5px"])}
    improvement = (initial_cost-final_cost)/initial_cost if initial_cost > 0 else 0
    final_at_limit = bool(np.any(np.abs(best_params) >= bound-1e-3))
    # Three distinct starts must reach equivalent final six-DOF solutions.
    converged = [run for run in runs if run["resolution"] == [int(width*0.25), int(height*0.25)]
                 and run["success"]]
    stable = False
    if len(converged) >= 3:
        for a in converged:
            cluster = [b for b in converged if
                       np.linalg.norm(np.asarray(a["final_parameters"][:3])-
                                      np.asarray(b["final_parameters"][:3])) <= 0.02 and
                       np.linalg.norm(np.asarray(a["final_parameters"][3:])-
                                      np.asarray(b["final_parameters"][3:])) <= np.deg2rad(2)]
            if len(cluster) >= 3:
                stable = True
    checks = {"tf_composition": True, "cost_improvement_ge_40pct": bool(improvement >= 0.40),
              "not_at_limit": not final_at_limit, "three_starts_equivalent": stable,
              "depth_edge_coverage_ge_65pct": bool(final_coverage >= 0.65),
              "all_three_regions_improved": all(v["improved"] for v in regions.values()),
              "visual_overlay_verified": False}
    metrics = {"status": "EXTRINSIC_CANDIDATE_NOT_TRUSTWORTHY", "stamp_ns": metadata["reference_stamp_ns"],
               "initial_cost": initial_cost, "final_cost": final_cost,
               "relative_improvement": improvement, "initial_coverage": initial_coverage,
               "final_coverage": final_coverage, "projected_cloud_points": projected,
               "observed_depth_edge_pixels": int(observed.sum()),
               "best_hypothesis_delta_parameters": best_params.tolist(),
               "best_run_resolution": best_run["resolution"],
               "best_run_start_index": best_run["start_index"],
               "best_hypothesis_world_link_row_major": candidate_link.tolist(),
               "bounds_translation_m_per_axis": 0.30, "bounds_rotation_deg_per_axis": 25,
               "checks": checks, "regions": regions,
               "note": "Hypothesis only; no transform is applicable unless every check passes."}
    (SNAPSHOT / "optimization_runs.json").write_text(json.dumps(runs, indent=2) + "\n")
    (SNAPSHOT / "candidate_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps({"status": metrics["status"], "initial_cost": initial_cost,
                      "final_cost": final_cost, "coverage": final_coverage,
                      "checks": checks}))


if __name__ == "__main__":
    main()
