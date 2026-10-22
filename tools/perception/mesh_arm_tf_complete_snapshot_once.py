#!/usr/bin/env python3
"""One read-only 8T capture, extending the existing 8R capture client."""

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys

import numpy as np
from PIL import Image as PILImage
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot"
OUT = ROOT / "evidence/codex_reports/2026-10-22_fase8t_tf_complete_snapshot"
BINARY = Path("/tmp/mesh_arm_snapshot_once_8t")


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


geometry = load_module("mesh_arm_geometry_8t", ROOT / "tools/perception/mesh_arm_auto_mask_offline.py")
previous = load_module("mesh_arm_packet_8t", ROOT / "src/thesis_jaco_perception/scripts/mesh_arm_snapshot_once.py")


def rigid_matrix(tf):
    values = np.asarray(tf["translation_m"] + tf["quaternion_xyzw"], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite TF")
    q = values[3:]
    if abs(np.linalg.norm(q) - 1.0) > 1e-6:
        raise ValueError("Nonunit TF quaternion")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = values[:3]
    if (not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-9) or
            abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-9):
        raise ValueError("Nonrigid TF")
    return matrix


def rotation_error(a, b):
    return float(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude())


def validate_tf_chain(tf_map, stamp, rgb_frame):
    expected = {
        "world_depth": ("world", "d435i_depth_optical_frame"),
        "color_world": (rgb_frame, "world"),
        "world_color": ("world", rgb_frame),
        "world_camera_link": ("world", "d435i_link"),
        "camera_link_color": ("d435i_link", rgb_frame),
        "color_camera_link": (rgb_frame, "d435i_link"),
        "camera_link_world": ("d435i_link", "world"),
    }
    matrices = {}
    for name, (target, source) in expected.items():
        tf = tf_map[name]
        if (tf["target_frame"], tf["source_frame"]) != (target, source):
            raise ValueError(f"Wrong TF frames: {name}")
        # Static link/optical TF has header stamp zero in tf2, despite exact-time lookup.
        allowed_stamps = (0, stamp) if name in ("camera_link_color", "color_camera_link") else (stamp,)
        if tf["returned_stamp_ns"] not in allowed_stamps:
            raise ValueError(f"Wrong TF stamp: {name}")
        matrices[name] = rigid_matrix(tf)
    direct = matrices["world_color"]
    compound = matrices["world_camera_link"] @ matrices["camera_link_color"]
    translation = float(np.linalg.norm(direct[:3, 3] - compound[:3, 3]))
    rotation = rotation_error(direct, compound)
    if translation > 1e-6 or rotation > 1e-6:
        raise ValueError(f"TF composition failure: {translation} m, {rotation} rad")
    for forward, inverse in (("world_color", "color_world"),
                             ("world_camera_link", "camera_link_world"),
                             ("camera_link_color", "color_camera_link")):
        error = matrices[forward] @ matrices[inverse]
        if np.linalg.norm(error[:3, 3]) > 1e-6 or rotation_error(error, np.eye(4)) > 1e-6:
            raise ValueError(f"TF inverse failure: {forward}")
    return {"translation_error_m": translation, "rotation_error_rad": rotation}


def parse_packet(raw, links):
    if not raw.startswith(b"MESH8T1"):
        raise ValueError("Invalid 8T packet")
    types = [Image, CameraInfo, PointCloud2] + [TransformStamped] * (7 + len(links))
    offset, messages = 7, []
    for message_type in types:
        if offset + 4 > len(raw):
            raise ValueError("Truncated 8T packet")
        length = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        if offset + length > len(raw):
            raise ValueError("Truncated 8T message")
        messages.append(deserialize_message(raw[offset:offset + length], message_type))
        offset += length
    if offset != len(raw):
        raise ValueError("Trailing 8T packet bytes")
    rgb, info, cloud = messages[:3]
    stamp = previous.stamp_ns(cloud)
    rgb_delta, info_delta = previous.delta_ns(cloud, rgb), previous.delta_ns(cloud, info)
    if (cloud.header.frame_id != "world" or rgb.header.frame_id != info.header.frame_id or
            max(abs(rgb_delta), abs(info_delta)) > 33_000_000):
        raise ValueError("Invalid frames or RGB/CameraInfo synchronization")
    names = ("world_depth", "color_world", "world_color", "world_camera_link",
             "camera_link_color", "color_camera_link", "camera_link_world")
    tf_map = {name: previous.transform_json(msg) for name, msg in zip(names, messages[3:10])}
    check = validate_tf_chain(tf_map, stamp, rgb.header.frame_id)
    link_tfs = [previous.transform_json(msg) for msg in messages[10:]]
    if len(link_tfs) != len(links):
        raise ValueError("Missing collision link TF")
    for link, tf in zip(links, link_tfs):
        if (tf["target_frame"], tf["source_frame"], tf["returned_stamp_ns"]) != ("world", link, stamp):
            raise ValueError(f"Invalid collision link TF: {link}")
        rigid_matrix(tf)
    return rgb, info, cloud, tf_map, link_tfs, check, rgb_delta, info_delta


def main():
    if OUT.exists():
        raise RuntimeError("8T output directory already exists; refusing to capture")
    metadata_8r, transforms_8r, manifest, hashes = geometry.verify_frozen_snapshot(SOURCE)
    links = [entry["link"] for entry in manifest]
    if len(links) != 13 or len(set(links)) != 13 or not BINARY.is_file():
        raise RuntimeError("Invalid frozen collision links or missing compiled capture client")
    # This is the sole runtime capture attempt. All processing below is offline.
    result = subprocess.run([str(BINARY), "--tf-complete", *links],
                            capture_output=True, timeout=12, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace").strip())
    rgb, info, cloud, tf_map, link_tfs, check, rgb_delta, info_delta = parse_packet(
        result.stdout, links)
    xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    rgb_array = previous.image_rgb(rgb)
    stamp = previous.stamp_ns(cloud)
    metadata = {
        "status": "TF_COMPLETE_SNAPSHOT_READY", "reference_stamp_ns": stamp,
        "rgb_stamp_ns": previous.stamp_ns(rgb), "camera_info_stamp_ns": previous.stamp_ns(info),
        "rgb_minus_cloud_ns": rgb_delta, "camera_info_minus_cloud_ns": info_delta,
        "sync_limit_ns": 33_000_000, "cloud_frame": cloud.header.frame_id,
        "rgb_frame": rgb.header.frame_id, "rgb_encoding": rgb.encoding,
        "rgb_width": rgb.width, "rgb_height": rgb.height,
        "camera_info_k": list(info.k), "camera_info_d": list(info.d),
        "camera_info_p": list(info.p), "camera_info_distortion_model": info.distortion_model,
        "world_xyz_points": len(xyz), "tf_at_cloud_stamp": tf_map,
        "composition_check": check,
        "geometry_source": "8R frozen collision meshes and robot_description",
        "geometry_hashes": hashes,
    }
    OUT.mkdir(parents=True, exist_ok=False)
    (OUT / "meshes").mkdir()
    PILImage.fromarray(rgb_array).save(OUT / "rgb.png")
    np.savez_compressed(OUT / "cloud_world_xyz.npz", xyz=xyz)
    for name in ("robot_description.urdf", "collision_mesh_manifest.json", "hashes.json"):
        shutil.copy2(SOURCE / name, OUT / name)
    for path in (SOURCE / "meshes").iterdir():
        shutil.copy2(path, OUT / "meshes" / path.name)
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (OUT / "link_transforms_at_stamp.json").write_text(json.dumps({
        "reference_stamp_ns": stamp, "transforms": link_tfs}, indent=2) + "\n")
    frozen_files = ("rgb.png", "cloud_world_xyz.npz", "metadata.json",
                    "link_transforms_at_stamp.json", "robot_description.urdf",
                    "collision_mesh_manifest.json", "hashes.json")
    capture_hashes = {name: hashlib.sha256((OUT / name).read_bytes()).hexdigest()
                      for name in frozen_files}
    capture_hashes.update({str(Path("meshes") / path.name):
        hashlib.sha256(path.read_bytes()).hexdigest() for path in (OUT / "meshes").iterdir()})
    (OUT / "capture_hashes.json").write_text(json.dumps(capture_hashes, indent=2) + "\n")
    print(json.dumps({"status": metadata["status"], "stamp_ns": stamp,
                      "rgb_delta_ns": rgb_delta, "info_delta_ns": info_delta,
                      "links": len(link_tfs), "points": len(xyz), **check}))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"BLOCKED_TF_CHAIN_OR_SYNC: {error}", file=sys.stderr)
        sys.exit(2)
