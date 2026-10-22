#!/usr/bin/env python3
"""Freeze one read-only RGB/cloud/TF/URDF collision snapshot."""

import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PILImage
import psutil
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "evidence/codex_reports/2026-10-22_fase8r_geometric_snapshot"
DESCRIPTION_SHARE = ROOT / "install/thesis_description/share/thesis_description"
CAPTURE_BINARY = Path("/tmp/mesh_arm_snapshot_once")
MODULE_PATH = ROOT / "src/thesis_jaco_perception/scripts/mesh_arm_snapshot_once.py"
spec = importlib.util.spec_from_file_location("mesh_arm_snapshot_8p", MODULE_PATH)
previous = importlib.util.module_from_spec(spec)
spec.loader.exec_module(previous)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def effective_xml_and_manifest():
    matches = [process for process in psutil.process_iter(["pid", "cmdline", "create_time"])
               if process.info["cmdline"] and
               any("/thesis_jaco_perception/jaco_mesh_shadow_node" in arg
                   for arg in process.info["cmdline"])]
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one active mesh-shadow process")
    process = matches[0]
    command = process.info["cmdline"]
    parameter_files = [command[index + 1] for index, arg in enumerate(command[:-1])
                       if arg == "--params-file"]
    if len(parameter_files) != 1:
        raise RuntimeError("Cannot identify the effective mesh parameter file")
    parameters = yaml.safe_load(Path(parameter_files[0]).read_text())
    xml = parameters["/**"]["ros__parameters"]["robot_description"]
    if not isinstance(xml, str) or not xml.strip():
        raise RuntimeError("Effective robot_description is unavailable")
    tree = ET.fromstring(xml)
    manifest = []
    frozen_meshes = {}
    for link in tree.findall("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("./geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.attrib["filename"]
            prefix = "package://thesis_description/"
            if not uri.startswith(prefix):
                raise RuntimeError(f"Unexpected collision URI: {uri}")
            relative = Path(uri[len(prefix):])
            if relative.is_absolute() or ".." in relative.parts:
                raise RuntimeError(f"Unsafe collision URI: {uri}")
            source = DESCRIPTION_SHARE / relative
            if not source.is_file() or source.stat().st_mtime > process.info["create_time"]:
                raise RuntimeError(f"Collision mesh missing or changed since node start: {uri}")
            data = source.read_bytes()
            frozen_meshes[str(relative)] = data
            origin = collision.find("origin")
            manifest.append({
                "link": link.attrib["name"],
                "collision_name": collision.attrib.get("name", ""),
                "uri": uri,
                "frozen_path": str(Path("meshes") / relative.name),
                "scale_xyz": [float(v) for v in mesh.attrib.get("scale", "1 1 1").split()],
                "origin_xyz_m": [float(v) for v in (origin.attrib.get("xyz", "0 0 0")
                    if origin is not None else "0 0 0").split()],
                "origin_rpy_rad": [float(v) for v in (origin.attrib.get("rpy", "0 0 0")
                    if origin is not None else "0 0 0").split()],
                "sha256": sha256(data),
            })
    links = [entry["link"] for entry in manifest]
    if len(links) != 13 or len(set(links)) != 13:
        raise RuntimeError(f"Expected 13 distinct collision mesh links, got {len(links)}")
    if not CAPTURE_BINARY.is_file():
        raise RuntimeError("Capture binary is unavailable")
    return xml.encode(), manifest, frozen_meshes, links, {
        "mesh_node_pid": process.pid,
        "mesh_node_start_unix_s": process.info["create_time"],
        "mesh_parameter_file": parameter_files[0],
    }


def parse_packet(raw, links):
    if not raw.startswith(b"MESH8R1"):
        raise RuntimeError("Invalid geometric capture packet")
    offset = 7
    messages = []
    types = [Image, CameraInfo, PointCloud2, TransformStamped,
             TransformStamped, TransformStamped] + [TransformStamped] * len(links)
    for message_type in types:
        if offset + 4 > len(raw):
            raise RuntimeError("Truncated capture packet")
        length = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        if offset + length > len(raw):
            raise RuntimeError("Truncated serialized message")
        messages.append(deserialize_message(raw[offset:offset + length], message_type))
        offset += length
    if offset != len(raw):
        raise RuntimeError("Trailing bytes in capture packet")
    rgb, info, cloud, depth_tf, color_tf, world_color_tf = messages[:6]
    link_tfs = messages[6:]
    cloud_stamp = previous.stamp_ns(cloud)
    rgb_delta = previous.delta_ns(cloud, rgb)
    info_delta = previous.delta_ns(cloud, info)
    if cloud.header.frame_id != "world" or rgb.header.frame_id != info.header.frame_id:
        raise RuntimeError("Capture frames do not match")
    if max(abs(rgb_delta), abs(info_delta)) > 33_000_000:
        raise RuntimeError("Capture exceeds 33 ms sync tolerance")
    expected = [("world", "d435i_depth_optical_frame", depth_tf),
                (rgb.header.frame_id, "world", color_tf),
                ("world", rgb.header.frame_id, world_color_tf)]
    expected += [("world", link, tf) for link, tf in zip(links, link_tfs)]
    for target, source, tf in expected:
        if (tf.header.frame_id != target or tf.child_frame_id != source or
                previous.stamp_ns(tf) != cloud_stamp):
            raise RuntimeError(f"TF frame or stamp mismatch: {target} <- {source}")
    return messages, rgb_delta, info_delta


def save_snapshot(xml, manifest, frozen_meshes, provenance, messages, rgb_delta, info_delta):
    rgb, info, cloud, depth_tf, color_tf, world_color_tf = messages[:6]
    link_tfs = messages[6:]
    xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    rgb_array = previous.image_rgb(rgb)
    hashes = {"robot_description_sha256": sha256(xml),
              "collision_meshes_sha256": {name: sha256(data)
                  for name, data in frozen_meshes.items()}}
    metadata = {
        "status": "GEOMETRIC_SNAPSHOT_READY",
        "reference_stamp_ns": previous.stamp_ns(cloud),
        "rgb_stamp_ns": previous.stamp_ns(rgb),
        "camera_info_stamp_ns": previous.stamp_ns(info),
        "rgb_minus_cloud_ns": rgb_delta,
        "camera_info_minus_cloud_ns": info_delta,
        "sync_limit_ns": 33_000_000,
        "cloud_frame": cloud.header.frame_id,
        "rgb_frame": rgb.header.frame_id,
        "rgb_encoding": rgb.encoding,
        "rgb_width": rgb.width,
        "rgb_height": rgb.height,
        "camera_info_k": list(info.k),
        "camera_info_d": list(info.d),
        "camera_info_p": list(info.p),
        "camera_info_distortion_model": info.distortion_model,
        "world_to_depth_optical_at_cloud_stamp": previous.transform_json(depth_tf),
        "color_optical_from_world_at_cloud_stamp": previous.transform_json(color_tf),
        "world_to_color_optical_at_cloud_stamp": previous.transform_json(world_color_tf),
        "world_xyz_points": len(xyz),
        "collision_mesh_count": len(manifest),
        "collision_link_count": len(link_tfs),
        "provenance": provenance,
    }
    transforms = {"reference_stamp_ns": previous.stamp_ns(cloud),
                  "transforms": [previous.transform_json(tf) for tf in link_tfs]}
    OUT.mkdir(parents=True, exist_ok=False)
    (OUT / "meshes").mkdir()
    PILImage.fromarray(rgb_array).save(OUT / "rgb.png")
    np.savez_compressed(OUT / "cloud_world_xyz.npz", xyz=xyz)
    (OUT / "robot_description.urdf").write_bytes(xml)
    for relative, data in frozen_meshes.items():
        (OUT / "meshes" / Path(relative).name).write_bytes(data)
    for name, data in [("metadata.json", metadata),
                       ("link_transforms_at_stamp.json", transforms),
                       ("collision_mesh_manifest.json", manifest),
                       ("hashes.json", hashes)]:
        (OUT / name).write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"status": "GEOMETRIC_SNAPSHOT_READY", "stamp_ns":
        metadata["reference_stamp_ns"], "rgb_delta_ns": rgb_delta,
        "info_delta_ns": info_delta, "links": len(link_tfs), "points": len(xyz)}))


def main():
    if OUT.exists():
        raise RuntimeError("Output directory already exists; no capture started")
    xml, manifest, frozen_meshes, links, provenance = effective_xml_and_manifest()
    result = subprocess.run([str(CAPTURE_BINARY), *links], capture_output=True,
                            timeout=12, check=False)
    if result.returncode:
        print(result.stderr.decode(errors="replace"), file=sys.stderr)
        return result.returncode
    messages, rgb_delta, info_delta = parse_packet(result.stdout, links)
    save_snapshot(xml, manifest, frozen_meshes, provenance, messages, rgb_delta, info_delta)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"BLOCKED_IMPLEMENTATION: {error}", file=sys.stderr)
        sys.exit(2)
