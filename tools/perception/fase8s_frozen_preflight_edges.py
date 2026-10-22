#!/usr/bin/env python3
"""Validate frozen 8R inputs and prepare edge evidence without ROS or fitting."""

import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt


ROOT = Path(__file__).resolve().parents[2]
SNAPSHOT = ROOT / "evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot"
OUT = ROOT / "evidence/codex_reports/2026-10-22_fase8s_silhouette_alignment"
MODULE = ROOT / "tools/perception/mesh_arm_auto_mask_offline.py"
spec = importlib.util.spec_from_file_location("mesh_arm_auto_mask_offline_8s", MODULE)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def project_cloud_to_color(xyz, metadata):
    matrix = geometry.matrix_from_transform(metadata["color_optical_from_world_at_cloud_stamp"])
    camera = xyz.astype(np.float64) @ matrix[:3, :3].T + matrix[:3, 3]
    intrinsic = np.asarray(metadata["camera_info_k"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(metadata["camera_info_d"], dtype=np.float64)
    pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), intrinsic, distortion)
    return pixels.reshape(-1, 2), camera[:, 2]


def depth_edges_from_sparse_cloud(xyz, metadata, height, width):
    pixels, z = project_cloud_to_color(xyz, metadata)
    uv = np.rint(pixels).astype(np.int32)
    inside = ((z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < width) &
              (uv[:, 1] >= 0) & (uv[:, 1] < height))
    sparse = np.full((height, width), np.inf, dtype=np.float32)
    np.minimum.at(sparse, (uv[inside, 1], uv[inside, 0]), z[inside])
    present = np.isfinite(sparse)
    distances, nearest = distance_transform_edt(~present, return_indices=True)
    valid = distances <= 6.0
    depth = sparse[nearest[0], nearest[1]]
    boundary = cv2.morphologyEx(valid.astype(np.uint8), cv2.MORPH_GRADIENT,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))) > 0
    gradient = np.zeros_like(valid)
    horizontal = valid[:, 1:] & valid[:, :-1] & (np.abs(depth[:, 1:] - depth[:, :-1]) > 0.08)
    vertical = valid[1:, :] & valid[:-1, :] & (np.abs(depth[1:, :] - depth[:-1, :]) > 0.08)
    gradient[:, 1:] |= horizontal
    gradient[1:, :] |= vertical
    return ((boundary | gradient).astype(np.uint8) * 255), valid, int(inside.sum())


def main():
    if OUT.exists():
        raise RuntimeError("8S evidence directory already exists; refusing to overwrite")
    metadata, transforms, manifest, hashes = geometry.verify_frozen_snapshot(SNAPSHOT)
    stamp = metadata["reference_stamp_ns"]
    for key in ("world_to_color_optical_at_cloud_stamp",
                "color_optical_from_world_at_cloud_stamp",
                "world_to_depth_optical_at_cloud_stamp"):
        if metadata[key]["returned_stamp_ns"] != stamp:
            raise ValueError(f"Camera TF stamp mismatch: {key}")
    if metadata["rgb_stamp_ns"] != stamp or metadata["camera_info_stamp_ns"] != stamp:
        raise ValueError("RGB and CameraInfo are not exactly synchronous")
    world_color = geometry.matrix_from_transform(
        metadata["world_to_color_optical_at_cloud_stamp"])
    color_world = geometry.matrix_from_transform(
        metadata["color_optical_from_world_at_cloud_stamp"])
    inverse_error = float(np.max(np.abs(world_color @ color_world - np.eye(4))))
    if inverse_error > 1e-8:
        raise ValueError("Frozen color transforms are not inverses")
    has_link_chain = any(tf["source_frame"] == "d435i_link"
                         for tf in transforms["transforms"]) or (
        "world_to_d435i_link_at_cloud_stamp" in metadata or
        "d435i_link_to_color_optical_at_cloud_stamp" in metadata)
    if has_link_chain:
        raise ValueError("Unexpected link chain: this preflight is for the missing-chain case")

    rgb = np.asarray(Image.open(SNAPSHOT / "rgb.png").convert("RGB"))
    height, width = rgb.shape[:2]
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    rgb_edges = cv2.Canny(gray, 80, 160)
    xyz = np.load(SNAPSHOT / "cloud_world_xyz.npz")["xyz"]
    depth_edges, depth_valid, projected = depth_edges_from_sparse_cloud(
        xyz, metadata, height, width)
    silhouette = np.asarray(Image.open(SNAPSHOT / "mesh_silhouette.png").convert("L"))
    possible = cv2.dilate(silhouette, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (91, 91))) > 0
    near_cloud = cv2.dilate(depth_valid.astype(np.uint8),
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))) > 0
    zone = possible | near_cloud
    rgb_edges = (rgb_edges * (depth_valid & zone)).astype(np.uint8)
    depth_edges = (depth_edges * zone).astype(np.uint8)
    combined = np.zeros((height, width, 3), dtype=np.uint8)
    combined[:, :, 0] = rgb_edges
    combined[:, :, 2] = depth_edges
    initial = rgb.copy()
    contours, _ = cv2.findContours(silhouette, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(initial, contours, -1, (0, 255, 0), 1)

    metrics = {
        "status": "BLOCKED_IMPLEMENTATION",
        "reason": "T_world_d435i_link and fixed d435i_link-to-optical transform are absent from frozen 8R artifacts",
        "reference_stamp_ns": stamp,
        "rgb_minus_cloud_ns": metadata["rgb_minus_cloud_ns"],
        "camera_info_minus_cloud_ns": metadata["camera_info_minus_cloud_ns"],
        "collision_link_tfs": len(transforms["transforms"]),
        "collision_mesh_instances": len(manifest),
        "frozen_mesh_hashes_verified": len(hashes["collision_meshes_sha256"]),
        "frozen_robot_description_sha256": hashes["robot_description_sha256"],
        "world_from_color_optical_row_major": world_color.tolist(),
        "color_optical_from_world_row_major": color_world.tolist(),
        "inverse_composition_max_abs_error": inverse_error,
        "initial_world_from_d435i_link": None,
        "projected_cloud_points": projected,
        "rgb_edge_pixels_depth_supported": int((rgb_edges > 0).sum()),
        "depth_edge_pixels": int((depth_edges > 0).sum()),
        "edge_provenance": {"red": "RGB Canny 80/160 within depth support",
                            "blue": "cloud-depth boundary or >0.08 m gradient"},
        "optimization_performed": False,
        "candidate_applicable": False,
    }
    OUT.mkdir(parents=True, exist_ok=False)
    Image.fromarray(rgb_edges).save(OUT / "rgb_edges.png")
    Image.fromarray(depth_edges).save(OUT / "depth_edges.png")
    Image.fromarray(combined).save(OUT / "combined_edges.png")
    Image.fromarray(initial).save(OUT / "initial_mesh_overlay.png")
    (OUT / "candidate_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (OUT / "optimization_runs.json").write_text(json.dumps({
        "status": "NOT_RUN_MISSING_T_WORLD_D435I_LINK", "runs": [],
        "bounds": {"translation_m_per_axis": 0.30, "rotation_deg_per_axis": 25},
    }, indent=2) + "\n")
    print(json.dumps({"status": metrics["status"], "stamp_ns": stamp,
                      "rgb_edge_pixels": metrics["rgb_edge_pixels_depth_supported"],
                      "depth_edge_pixels": metrics["depth_edge_pixels"]}))


if __name__ == "__main__":
    main()
