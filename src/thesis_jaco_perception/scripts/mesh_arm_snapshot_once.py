#!/usr/bin/env python3
"""One diagnostic RGB/cloud/TF capture. It never publishes ROS data."""

import json
import pathlib
import struct
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


MAX_DELTA_NS = 33_000_000
OUT = pathlib.Path("evidence/codex_reports/2026-10-22_fase8p_snapshot")


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def delta_ns(reference, message):
    return stamp_ns(message) - stamp_ns(reference)


def nearest(reference, messages):
    return min(messages, key=lambda message: abs(delta_ns(reference, message)))


def image_rgb(message):
    if message.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise ValueError(f"Unsupported RGB encoding: {message.encoding}")
    channels = 4 if "a8" in message.encoding else 3
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if message.encoding == "rgb8":
        return pixels.copy()
    if message.encoding == "bgr8":
        return pixels[:, :, ::-1].copy()
    if message.encoding == "rgba8":
        return pixels[:, :, :3].copy()
    return pixels[:, :, [2, 1, 0]].copy()


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ], dtype=np.float64)


def project_world_points(xyz_world, transform, info):
    translation = transform.transform.translation
    xyz_camera = xyz_world @ rotation_matrix(transform.transform.rotation).T
    xyz_camera += np.array([translation.x, translation.y, translation.z])
    in_front = xyz_camera[:, 2] > 0
    camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64)
    pixels = np.full((len(xyz_world), 2), np.nan, dtype=np.float64)
    if in_front.any():
        projected, _ = cv2.projectPoints(
            xyz_camera[in_front], np.zeros(3), np.zeros(3), camera_matrix, distortion)
        pixels[in_front] = projected.reshape(-1, 2)
    return pixels


def validate_manual_mask(mask, width, height):
    """Validate an operator supplied mask; never create one automatically."""
    if mask.dtype != np.uint8 or mask.shape != (height, width):
        raise ValueError("Mask must be 8-bit grayscale with the RGB dimensions")
    values = np.unique(mask)
    if not np.isin(values, [0, 255]).all() or not np.any(mask == 255):
        raise ValueError("Mask must contain white arm pixels and only 0/255 values")
    return True


def transform_json(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return {
        "target_frame": transform.header.frame_id,
        "source_frame": transform.child_frame_id,
        "returned_stamp_ns": stamp_ns(transform),
        "translation_m": [translation.x, translation.y, translation.z],
        "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
    }


def save_snapshot(found):
    rgb, info, cloud, depth_tf, color_tf, differences = found
    rgb_array = image_rgb(rgb)
    xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    pixels = project_world_points(xyz.astype(np.float64), color_tf, info)
    visible = (np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0) &
               (pixels[:, 0] < rgb.width) & (pixels[:, 1] >= 0) &
               (pixels[:, 1] < rgb.height))

    OUT.mkdir(parents=True, exist_ok=False)
    PILImage.fromarray(rgb_array).save(OUT / "rgb.png")
    np.savez_compressed(OUT / "cloud_world_xyz.npz", xyz=xyz)
    preview = PILImage.fromarray(rgb_array).copy()
    drawing = ImageDraw.Draw(preview)
    for x, y in pixels[visible]:
        drawing.ellipse((x-1, y-1, x+1, y+1), fill=(0, 255, 255))
    preview.save(OUT / "rgb_proyeccion_nube.png")
    operator = PILImage.fromarray(rgb_array).copy()
    drawing = ImageDraw.Draw(operator)
    drawing.rectangle((0, 0, rgb.width, 38), fill=(0, 0, 0))
    drawing.text((6, 4), "Mascara manual: blanco=brazo visible; negro=resto", fill=(255, 255, 255))
    drawing.text((6, 21), "Guardar mascara nueva con el mismo tamano de rgb.png", fill=(255, 255, 255))
    operator.save(OUT / "rgb_para_mascara.png")
    metadata = {
        "status": "SNAPSHOT_READY_WAITING_OPERATOR_MASK",
        "reference_stamp_ns": stamp_ns(cloud),
        "rgb_stamp_ns": stamp_ns(rgb),
        "camera_info_stamp_ns": stamp_ns(info),
        "rgb_minus_cloud_ns": differences[0],
        "camera_info_minus_cloud_ns": differences[1],
        "sync_limit_ns": MAX_DELTA_NS,
        "cloud_frame": cloud.header.frame_id,
        "rgb_frame": rgb.header.frame_id,
        "camera_info_frame": info.header.frame_id,
        "rgb_encoding": rgb.encoding,
        "rgb_width": rgb.width,
        "rgb_height": rgb.height,
        "camera_info_k": list(info.k),
        "camera_info_d": list(info.d),
        "camera_info_p": list(info.p),
        "camera_info_distortion_model": info.distortion_model,
        "world_to_depth_optical_at_cloud_stamp": transform_json(depth_tf),
        "world_to_color_optical_at_cloud_stamp": transform_json(color_tf),
        "world_xyz_points": len(xyz),
        "points_projecting_into_rgb": int(visible.sum()),
        "mask_status": "ABSENT_OPERATOR_REQUIRED",
        "note": "Cloud is diagnostic only. No mask or classification was generated.",
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps({"status": metadata["status"], "output": str(OUT),
                      "rgb_minus_cloud_ns": differences[0],
                      "camera_info_minus_cloud_ns": differences[1],
                      "world_xyz_points": len(xyz),
                      "points_projecting_into_rgb": int(visible.sum())}))


def main():
    if OUT.exists():
        print("BLOCKED_IMPLEMENTATION: output directory already exists", file=sys.stderr)
        return 2
    try:
        result = subprocess.run(["/tmp/mesh_arm_snapshot_once"], capture_output=True,
                                timeout=12, check=False)
    except subprocess.TimeoutExpired:
        print("BLOCKED_SYNC_UNAVAILABLE: capture process exceeded 12 s", file=sys.stderr)
        return 3
    if result.returncode:
        print(result.stderr.decode(errors="replace"), file=sys.stderr)
        return result.returncode
    raw = result.stdout
    if not raw.startswith(b"MESH8P1"):
        print("BLOCKED_IMPLEMENTATION: invalid capture packet", file=sys.stderr)
        return 2
    offset = 7
    messages = []
    for message_type in (Image, CameraInfo, PointCloud2, TransformStamped,
                         TransformStamped):
        length = struct.unpack_from("<I", raw, offset)[0]
        offset += 4
        messages.append(deserialize_message(raw[offset:offset + length], message_type))
        offset += length
    if offset != len(raw):
        print("BLOCKED_IMPLEMENTATION: trailing bytes in capture packet", file=sys.stderr)
        return 2
    rgb, info, cloud, depth_tf, color_tf = messages
    differences = (delta_ns(cloud, rgb), delta_ns(cloud, info))
    if max(map(abs, differences)) > MAX_DELTA_NS:
        print("BLOCKED_SYNC_UNAVAILABLE: packet outside 33 ms", file=sys.stderr)
        return 3
    save_snapshot((rgb, info, cloud, depth_tf, color_tf, differences))
    return 0


if __name__ == "__main__":
    sys.exit(main())
