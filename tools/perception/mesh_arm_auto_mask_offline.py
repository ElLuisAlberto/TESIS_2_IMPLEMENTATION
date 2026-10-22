#!/usr/bin/env python3
"""Conservative diagnostic mask from one frozen geometric snapshot only."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess

import cv2
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
SNAPSHOT = ROOT / "evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot"
EXPORTER = Path("/tmp/export_frozen_collision_triangles")
DILATION_PX = 45
EROSION_PX = 8
SUPPORT_PX = 6
DEPTH_EDGE_M = 0.12


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix_from_transform(transform):
    x, y, z, w = transform["quaternion_xyzw"]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    result[:3, 3] = transform["translation_m"]
    return result


def collision_origin(entry):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_euler("xyz", entry["origin_rpy_rad"]).as_matrix()
    result[:3, 3] = entry["origin_xyz_m"]
    return result


def verify_frozen_snapshot(snapshot=SNAPSHOT):
    metadata = json.loads((snapshot / "metadata.json").read_text())
    transforms = json.loads((snapshot / "link_transforms_at_stamp.json").read_text())
    manifest = json.loads((snapshot / "collision_mesh_manifest.json").read_text())
    hashes = json.loads((snapshot / "hashes.json").read_text())
    if digest(snapshot / "robot_description.urdf") != hashes["robot_description_sha256"]:
        raise ValueError("Frozen robot_description hash mismatch")
    for relative, expected in hashes["collision_meshes_sha256"].items():
        path = snapshot / "meshes" / Path(relative).name
        if digest(path) != expected:
            raise ValueError(f"Frozen mesh hash mismatch: {relative}")
    stamp = metadata["reference_stamp_ns"]
    if transforms["reference_stamp_ns"] != stamp or len(transforms["transforms"]) != len(manifest):
        raise ValueError("Frozen link TF count or stamp mismatch")
    by_link = {tf["source_frame"]: tf for tf in transforms["transforms"]}
    if len(by_link) != len(manifest):
        raise ValueError("Duplicate or missing frozen link TF")
    for entry in manifest:
        tf = by_link.get(entry["link"])
        if tf is None or tf["target_frame"] != "world" or tf["returned_stamp_ns"] != stamp:
            raise ValueError(f"Invalid frozen TF for {entry['link']}")
        relative = entry["uri"].removeprefix("package://thesis_description/")
        if entry["sha256"] != hashes["collision_meshes_sha256"][relative]:
            raise ValueError(f"Manifest hash mismatch for {entry['link']}")
    camera = metadata["color_optical_from_world_at_cloud_stamp"]
    if (camera["returned_stamp_ns"] != stamp or camera["source_frame"] != "world" or
            camera["target_frame"] != metadata["rgb_frame"]):
        raise ValueError("Frozen camera TF mismatch")
    return metadata, transforms, manifest, hashes


def load_world_triangles(snapshot, manifest, transforms):
    by_link = {tf["source_frame"]: tf for tf in transforms["transforms"]}
    packet = bytearray(struct.pack("<I", len(manifest)))
    for entry in manifest:
        path = str((snapshot / entry["frozen_path"]).resolve()).encode()
        world = matrix_from_transform(by_link[entry["link"]])
        origin = collision_origin(entry)
        packet += struct.pack("<H", len(path)) + path
        packet += world.astype("<f8").tobytes()
        packet += origin.astype("<f8").tobytes()
        packet += np.asarray(entry["scale_xyz"], dtype="<f8").tobytes()
    result = subprocess.run([str(EXPORTER)], input=bytes(packet), capture_output=True, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace"))
    raw = result.stdout
    if not raw.startswith(b"TMSH8R1") or len(raw) < 11:
        raise RuntimeError("Invalid frozen mesh triangle packet")
    count = struct.unpack_from("<I", raw, 7)[0]
    dtype = np.dtype([("link", "<u2"), ("xyz", "<f4", (3, 3))], align=False)
    triangles = np.frombuffer(raw, dtype=dtype, offset=11)
    if len(triangles) != count:
        raise RuntimeError("Truncated frozen mesh triangle packet")
    return triangles.copy()


def ellipse(radius):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))


def projected_triangles(triangles, metadata):
    matrix = matrix_from_transform(metadata["color_optical_from_world_at_cloud_stamp"])
    xyz_world = triangles["xyz"].astype(np.float64).reshape(-1, 3)
    camera = xyz_world @ matrix[:3, :3].T + matrix[:3, 3]
    valid_triangle = (camera[:, 2].reshape(-1, 3) > 0.01).all(axis=1)
    intrinsic = np.asarray(metadata["camera_info_k"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(metadata["camera_info_d"], dtype=np.float64)
    pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), intrinsic, distortion)
    return pixels.reshape(-1, 3, 2), valid_triangle


def render_link_silhouettes(triangles, pixels, valid, height, width, links):
    masks = np.zeros((len(links), height, width), dtype=np.uint8)
    for index in np.flatnonzero(valid):
        polygon = np.rint(pixels[index]).astype(np.int32)
        cv2.fillConvexPoly(masks[int(triangles["link"][index])], polygon, 255)
    union = np.max(masks, axis=0)
    return masks, union


def depth_supported_pixels(xyz, metadata, height, width):
    matrix = matrix_from_transform(metadata["color_optical_from_world_at_cloud_stamp"])
    camera = xyz.astype(np.float64) @ matrix[:3, :3].T + matrix[:3, 3]
    intrinsic = np.asarray(metadata["camera_info_k"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(metadata["camera_info_d"], dtype=np.float64)
    pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), intrinsic, distortion)
    pixels = np.rint(pixels.reshape(-1, 2)).astype(np.int32)
    valid = ((camera[:, 2] > 0) & (pixels[:, 0] >= 0) & (pixels[:, 0] < width) &
             (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
    depth = np.full((height, width), np.inf, dtype=np.float32)
    np.minimum.at(depth, (pixels[valid, 1], pixels[valid, 0]), camera[valid, 2])
    local_min = cv2.erode(depth, ellipse(9))
    good = valid.copy()
    indices = np.flatnonzero(valid)
    good[indices] = camera[indices, 2] <= (
        local_min[pixels[indices, 1], pixels[indices, 0]] + DEPTH_EDGE_M)
    centers = np.zeros((height, width), dtype=np.uint8)
    centers[pixels[good, 1], pixels[good, 0]] = 255
    support = cv2.dilate(centers, ellipse(SUPPORT_PX))
    return support, int(valid.sum()), int((valid & ~good).sum())


def connected_to_seed(candidate, seed):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype(np.uint8), 8)
    result = np.zeros_like(candidate, dtype=np.uint8)
    removed = 0
    for label in range(1, count):
        component = labels == label
        if np.any(component & (seed > 0)):
            result[component] = 255
        else:
            removed += 1
    return result, removed


def region_name(link):
    if link.endswith("link_base") or link.endswith("link_1"):
        return "base_hombro"
    if link.endswith("link_2"):
        return "brazo_superior"
    if link.endswith("link_3"):
        return "antebrazo"
    if link.endswith("link_4") or link.endswith("link_5"):
        return "muneca"
    return "efector"


def evaluate_mask(mask, silhouette, band, per_link, manifest):
    if mask.dtype != np.uint8 or mask.shape != silhouette.shape or not np.isin(np.unique(mask), [0, 255]).all():
        raise ValueError("Candidate is not a binary image matching RGB")
    white = mask > 0
    total = int(white.sum())
    outside_band = int((white & (band == 0)).sum())
    outside_silhouette = int((white & (silhouette == 0)).sum())
    region_masks = {}
    for index, entry in enumerate(manifest):
        name = region_name(entry["link"])
        region_masks.setdefault(name, np.zeros_like(mask))
        region_masks[name] = cv2.bitwise_or(region_masks[name], per_link[index])
    regions = {name: {"projected_area_px": int((shape > 0).sum()),
                      "white_overlap_px": int((white & (shape > 0)).sum())}
               for name, shape in region_masks.items()}
    covered = [name for name, values in regions.items()
               if values["projected_area_px"] >= 100 and values["white_overlap_px"] >= 50]
    count, labels = cv2.connectedComponents(white.astype(np.uint8), 8)
    components = [int((labels == label).sum()) for label in range(1, count)]
    tests = {
        "nonempty": total >= 500,
        "outside_band_lt_2_percent": total > 0 and outside_band / total < 0.02,
        "outside_silhouette_le_5_percent": total > 0 and outside_silhouette / total <= 0.05,
        "at_least_three_visible_regions": len(covered) >= 3,
    }
    return {"white_area_px": total, "components": components,
            "outside_band_px": outside_band, "outside_silhouette_px": outside_silhouette,
            "regions": regions, "covered_regions": covered, "tests": tests}


def main():
    metadata, transforms, manifest, hashes = verify_frozen_snapshot()
    height, width = metadata["rgb_height"], metadata["rgb_width"]
    if (width, height) != (640, 480):
        raise ValueError("Expected frozen RGB size 640x480")
    triangles = load_world_triangles(SNAPSHOT, manifest, transforms)
    pixels, valid_triangles = projected_triangles(triangles, metadata)
    link_masks, silhouette = render_link_silhouettes(
        triangles, pixels, valid_triangles, height, width, manifest)
    band = cv2.dilate(silhouette, ellipse(DILATION_PX))
    seed = cv2.erode(silhouette, ellipse(EROSION_PX))
    xyz = np.load(SNAPSHOT / "cloud_world_xyz.npz")["xyz"]
    support, projected_points, depth_rejected = depth_supported_pixels(xyz, metadata, height, width)
    seed = cv2.bitwise_and(seed, support)
    grabcut = np.full((height, width), cv2.GC_BGD, dtype=np.uint8)
    grabcut[(band > 0) & (support > 0)] = cv2.GC_PR_BGD
    grabcut[(silhouette > 0) & (support > 0)] = cv2.GC_PR_FGD
    grabcut[seed > 0] = cv2.GC_FGD
    candidate = np.zeros((height, width), dtype=np.uint8)
    if np.any(seed) and np.any(grabcut == cv2.GC_PR_BGD):
        cv2.grabCut(np.asarray(Image.open(SNAPSHOT / "rgb.png").convert("RGB")),
                    grabcut, None, np.zeros((1, 65), np.float64),
                    np.zeros((1, 65), np.float64), 5, cv2.GC_INIT_WITH_MASK)
        candidate[np.isin(grabcut, [cv2.GC_FGD, cv2.GC_PR_FGD])] = 255
    candidate = cv2.bitwise_and(candidate, band)
    candidate = cv2.bitwise_and(candidate, support)
    candidate, removed = connected_to_seed(candidate, seed)
    metrics = evaluate_mask(candidate, silhouette, band, link_masks, manifest)
    metrics.update({
        "status": "AUTO_MASK_CANDIDATE_READY" if all(metrics["tests"].values()) else
            "AUTO_MASK_NOT_TRUSTWORTHY",
        "algorithm": "frozen collision triangle silhouette; eroded seed; local GrabCut; depth-supported component filter",
        "dilation_px": DILATION_PX, "erosion_px": EROSION_PX,
        "cloud_support_radius_px": SUPPORT_PX, "depth_edge_threshold_m": DEPTH_EDGE_M,
        "mesh_triangles": len(triangles), "triangles_in_front": int(valid_triangles.sum()),
        "silhouette_area_px": int((silhouette > 0).sum()),
        "band_area_px": int((band > 0).sum()),
        "seed_area_px": int((seed > 0).sum()),
        "projected_cloud_points": projected_points,
        "depth_edge_rejected_points": depth_rejected,
        "disconnected_components_removed": removed,
        "frozen_robot_description_sha256": hashes["robot_description_sha256"],
        "frozen_mesh_hashes_verified": len(hashes["collision_meshes_sha256"]),
    })
    Image.fromarray(silhouette).save(SNAPSHOT / "mesh_silhouette.png")
    rgb = np.asarray(Image.open(SNAPSHOT / "rgb.png").convert("RGB"))
    overlay = rgb.copy()
    overlay[candidate > 0] = (0.5*overlay[candidate > 0] +
                              0.5*np.array([255, 0, 0])).astype(np.uint8)
    contours, _ = cv2.findContours(silhouette, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
    Image.fromarray(overlay).save(SNAPSHOT / "rgb_auto_mask_overlay.png")
    (SNAPSHOT / "auto_mask_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    if metrics["status"] == "AUTO_MASK_CANDIDATE_READY":
        Image.fromarray(candidate).save(SNAPSHOT / "mask_brazo_auto.png")
    print(json.dumps({"status": metrics["status"], "white_area_px": metrics["white_area_px"],
                      "covered_regions": metrics["covered_regions"], "tests": metrics["tests"]}))


if __name__ == "__main__":
    main()
