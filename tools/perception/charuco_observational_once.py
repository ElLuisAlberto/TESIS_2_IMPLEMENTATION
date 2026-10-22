#!/usr/bin/env python3
"""Future one-shot, read-only ChArUco camera calibration; never publishes TF."""

import argparse
from collections import deque
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
board_spec = importlib.util.spec_from_file_location(
    "charuco_board_generate_for_capture", Path(__file__).with_name("charuco_board_generate.py"))
board_module = importlib.util.module_from_spec(board_spec)
board_spec.loader.exec_module(board_module)
board_factory = board_module.board_factory
EVIDENCE_ROOT = ROOT / "evidence/codex_reports"
MAX_SYNC_NS = 33_000_000
MIN_CORNERS = 12
MAX_RMS_PX = 1.5


def rigid(matrix):
    value = np.asarray(matrix, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all() or
            not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10) or
            not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8) or
            abs(np.linalg.det(value[:3, :3]) - 1) > 1e-8):
        raise ValueError("Matrix is not a finite rigid transform")
    return value


def transform_from_tf_json(tf, target, source):
    if (tf.get("target_frame"), tf.get("source_frame")) != (target, source):
        raise ValueError("Fixed camera TF frames are wrong")
    translation = np.asarray(tf.get("translation_m"), dtype=np.float64)
    quaternion = np.asarray(tf.get("quaternion_xyzw"), dtype=np.float64)
    if (translation.shape != (3,) or quaternion.shape != (4,) or
            not np.isfinite(translation).all() or not np.isfinite(quaternion).all() or
            abs(np.linalg.norm(quaternion)-1) > 1e-6):
        raise ValueError("Fixed camera TF is incomplete or not rigid")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = translation
    return rigid(result)


def load_fixed_link_color(path):
    path = Path(path)
    manifest = path.parent / "capture_hashes.json"
    if not manifest.is_file():
        raise ValueError("Frozen camera-chain hash manifest is missing")
    expected = json.loads(manifest.read_text()).get(path.name)
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError("Frozen camera-chain metadata hash mismatch")
    metadata = json.loads(path.read_text())
    tf = metadata["tf_at_cloud_stamp"]["camera_link_color"]
    matrix = transform_from_tf_json(tf, "d435i_link", "d435i_color_optical_frame")
    return matrix, hashlib.sha256(path.read_bytes()).hexdigest()


def board_pose_matrix(values):
    required_text = ("measurement_method", "measurement_date_and_operator",
                     "mount_description", "world_origin_reference", "world_axes_reference")
    if (values.get("session_only") is not True or values.get("not_for_safety") is not True or
            values.get("parent_frame") != "world" or
            values.get("child_frame") != "charuco_board" or
            values.get("board_origin") != "upper_left_outer_corner_of_printed_pattern" or
            values.get("board_axes") != "x_right_y_down_z_into_printed_sheet_from_front" or
            any(not isinstance(values.get(key), str) or not values[key].strip() or
                values[key] == "REQUIRED" for key in required_text)):
        raise ValueError("Board pose provenance or frames are incomplete")
    fields = ("verified_square_length_m", "verified_marker_length_m",
              "verified_pattern_width_m", "verified_pattern_height_m",
              "tx_m", "ty_m", "tz_m", "roll_rad", "pitch_rad", "yaw_rad",
              "position_tolerance_m", "orientation_tolerance_rad")
    numbers = {}
    for key in fields:
        raw = values.get(key)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
            raise ValueError(f"Board pose field {key} is REQUIRED and must be finite")
        numbers[key] = float(raw)
    for key, nominal, tolerance in (("verified_square_length_m", 0.030, 0.0005),
                                     ("verified_marker_length_m", 0.022, 0.0005),
                                     ("verified_pattern_width_m", 0.210, 0.001),
                                     ("verified_pattern_height_m", 0.150, 0.001)):
        if abs(numbers[key]-nominal) > tolerance:
            raise ValueError(f"Measured board dimension outside printable tolerance: {key}")
    if (not 0 < numbers["position_tolerance_m"] <= 0.005 or
            not 0 < numbers["orientation_tolerance_rad"] <= math.radians(2)):
        raise ValueError("Physical board-pose tolerance is too large or invalid")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", [numbers["roll_rad"],
        numbers["pitch_rad"], numbers["yaw_rad"]]).as_matrix()
    result[:3, 3] = [numbers["tx_m"], numbers["ty_m"], numbers["tz_m"]]
    return rigid(result)


def compose_world_camera_link(world_board, camera_board, link_color):
    world_board, camera_board, link_color = map(rigid,
                                                (world_board, camera_board, link_color))
    world_color = rigid(world_board @ np.linalg.inv(camera_board))
    world_link = rigid(world_color @ np.linalg.inv(link_color))
    if not np.allclose(world_link @ link_color, world_color, atol=1e-9):
        raise ValueError("TF composition is inconsistent")
    return world_link, world_color


def validate_intrinsics(k, d):
    intrinsic = np.asarray(k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(d, dtype=np.float64).reshape(-1)
    if (not np.isfinite(intrinsic).all() or not np.isfinite(distortion).all() or
            intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or
            not np.allclose(intrinsic[2], [0, 0, 1], atol=1e-9)):
        raise ValueError("Invalid CameraInfo intrinsics")
    return intrinsic, distortion


def estimate_from_corners(corners, ids, k, d, world_board, link_color):
    board = board_factory()
    ids = np.asarray(ids, dtype=np.int32).reshape(-1)
    image = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if (len(ids) != len(image) or len(ids) < MIN_CORNERS or
            len(set(ids.tolist())) != len(ids) or np.any(ids < 0) or
            np.any(ids >= len(board.chessboardCorners))):
        raise ValueError("Fewer than 12 valid, distinct ChArUco corners")
    if cv2.aruco.testCharucoCornersCollinear(board, ids.reshape(-1, 1)):
        raise ValueError("ChArUco corners are collinear")
    intrinsic, distortion = validate_intrinsics(k, d)
    object_points = np.asarray(board.chessboardCorners[ids], dtype=np.float64)
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points, image, intrinsic, distortion,
        iterationsCount=300, reprojectionError=2.0, confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not success or inliers is None or len(inliers) < MIN_CORNERS:
        raise ValueError("solvePnP/RANSAC has fewer than 12 inliers")
    chosen = inliers.reshape(-1)
    rvec, tvec = cv2.solvePnPRefineLM(object_points[chosen], image[chosen],
                                      intrinsic, distortion, rvec, tvec)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, intrinsic, distortion)
    residual = np.linalg.norm(projected.reshape(-1, 2)-image, axis=1)
    rms = float(np.sqrt(np.mean(residual**2)))
    if not np.isfinite(rms) or rms > MAX_RMS_PX:
        raise ValueError(f"Reprojection RMS {rms:.3f} px exceeds 1.5 px")
    if float(residual.max()) > 3.0:
        raise ValueError("Maximum corner reprojection error exceeds 3 px")
    rotation, _ = cv2.Rodrigues(rvec)
    camera_board = np.eye(4)
    camera_board[:3, :3] = rotation
    camera_board[:3, 3] = tvec.reshape(3)
    rigid(camera_board)
    camera_points = object_points @ rotation.T + tvec.reshape(1, 3)
    if np.any(camera_points[:, 2] <= 0) or np.dot(rotation[:, 2], tvec.reshape(3)) <= 0:
        raise ValueError("Board is behind camera or faces away")
    world_link, world_color = compose_world_camera_link(world_board, camera_board, link_color)
    return {"world_link": world_link, "world_color": world_color,
            "camera_board": camera_board, "rms_px": rms,
            "max_error_px": float(residual.max()),
            "charuco_corners": len(ids), "ransac_inliers": len(inliers)}


def image_gray(message):
    encoding = message.encoding.lower()
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported RGB encoding: {encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width*channels].reshape(message.height, message.width, channels)
    if channels == 1:
        return pixels[:, :, 0].copy()
    code = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY,
            "rgba8": cv2.COLOR_RGBA2GRAY, "bgra8": cv2.COLOR_BGRA2GRAY}[encoding]
    return cv2.cvtColor(pixels, code)


def detect(gray, k, d):
    board = board_factory()
    parameters = cv2.aruco.DetectorParameters_create()
    markers, marker_ids, _ = cv2.aruco.detectMarkers(gray, board.dictionary,
                                                       parameters=parameters)
    if marker_ids is None or len(marker_ids) < 4:
        raise ValueError("Too few ChArUco markers detected")
    intrinsic, distortion = validate_intrinsics(k, d)
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(
        markers, marker_ids, gray, board, cameraMatrix=intrinsic, distCoeffs=distortion)
    if ids is None or count < MIN_CORNERS:
        raise ValueError("Fewer than 12 interpolated ChArUco corners")
    return corners, ids


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec*1_000_000_000 + stamp.nanosec


def capture_pair(deadline_seconds=10.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class Once(Node):
        def __init__(self):
            super().__init__("charuco_observational_once", enable_rosout=False,
                             start_parameter_services=False)
            self.images = deque(maxlen=12)
            self.infos = deque(maxlen=12)
            self.create_subscription(Image, "/camera/d435i/color/image_raw",
                                     self.images.append, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/camera/d435i/color/camera_info",
                                     self.infos.append, qos_profile_sensor_data)

    rclpy.init(args=None)
    node = Once()
    try:
        deadline = time.monotonic() + deadline_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if not node.images or not node.infos:
                continue
            for image in reversed(node.images):
                info = min(node.infos, key=lambda item: abs(stamp_ns(item)-stamp_ns(image)))
                if abs(stamp_ns(info)-stamp_ns(image)) <= MAX_SYNC_NS:
                    return image, info
        raise RuntimeError("No synchronized RGB/CameraInfo pair within 10 s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-pose", type=Path, required=True)
    parser.add_argument("--camera-chain", type=Path, required=True,
                        help="Frozen 8T metadata.json with fixed link-to-color TF")
    parser.add_argument("--output", type=Path, required=True,
                        help="New evidence YAML path, never an operational config")
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(EVIDENCE_ROOT.resolve()) or output.suffix != ".yaml":
        raise ValueError("Candidate output must be a YAML under evidence/codex_reports")
    if output.exists():
        raise FileExistsError("Candidate evidence already exists; refusing to overwrite")
    board_data = yaml.safe_load(args.board_pose.read_text())
    world_board = board_pose_matrix(board_data)
    link_color, camera_chain_hash = load_fixed_link_color(args.camera_chain)
    image, info = capture_pair()
    if (image.header.frame_id != "d435i_color_optical_frame" or
            info.header.frame_id != image.header.frame_id or
            info.width != image.width or info.height != image.height or
            abs(stamp_ns(info)-stamp_ns(image)) > MAX_SYNC_NS):
        raise ValueError("CameraInfo/RGB frames, dimensions or timestamps differ")
    gray = image_gray(image)
    corners, ids = detect(gray, info.k, info.d)
    estimate = estimate_from_corners(corners, ids, info.k, info.d,
                                     world_board, link_color)
    world_link = estimate["world_link"]
    candidate = {
        "status": "CHARUCO_EXTRINSIC_CANDIDATE_UNAPPLIED",
        "session_only": True, "not_for_safety": True,
        "parent_frame": "world", "child_frame": "d435i_link",
        "rgb_stamp_ns": stamp_ns(image), "camera_info_stamp_ns": stamp_ns(info),
        "sync_delta_ns": stamp_ns(info)-stamp_ns(image),
        "board_pose_sha256": hashlib.sha256(args.board_pose.read_bytes()).hexdigest(),
        "camera_chain_metadata_sha256": camera_chain_hash,
        "board_position_tolerance_m": board_data["position_tolerance_m"],
        "board_orientation_tolerance_rad": board_data["orientation_tolerance_rad"],
        "charuco_corners": estimate["charuco_corners"],
        "ransac_inliers": estimate["ransac_inliers"],
        "reprojection_rms_px": estimate["rms_px"],
        "reprojection_max_px": estimate["max_error_px"],
        "translation_m": world_link[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(world_link[:3, :3]).as_quat().tolist(),
        "transform_row_major": world_link.tolist(),
        "note": "Evidence only. Never load into launch or TF publisher automatically.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(candidate, sort_keys=False))
    print(json.dumps({"status": candidate["status"], "rms_px": estimate["rms_px"],
                      "corners": estimate["charuco_corners"], "output": str(output)}))


if __name__ == "__main__":
    main()
