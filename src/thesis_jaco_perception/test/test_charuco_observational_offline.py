import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools/perception/charuco_observational_once.py"
spec = importlib.util.spec_from_file_location("charuco_observational_once", SCRIPT)
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)
BOARD = ROOT / "evidence/codex_reports/2026-10-22_fase8u_charuco/charuco_board.png"
TEMPLATE = ROOT / "evidence/codex_reports/2026-10-22_fase8u_charuco/board_pose_relative_to_world_TEMPLATE.yaml"


def pose(translation, euler):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", euler).as_matrix()
    result[:3, 3] = translation
    return result


def synthetic():
    board = calibration.board_factory()
    camera_board = pose([0.03, -0.02, 0.6], [0.2, -0.1, 0.1])
    world_board = pose([0.2, 0.1, 0.3], [0.02, 0.1, -0.2])
    link_color = pose([0.015, 0, 0], [-1.57, 0, 1.57])
    k = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=float)
    d = np.zeros(5)
    rvec, _ = cv2.Rodrigues(camera_board[:3, :3])
    corners, _ = cv2.projectPoints(board.chessboardCorners, rvec,
                                    camera_board[:3, 3], k, d)
    ids = np.arange(len(board.chessboardCorners), dtype=np.int32)
    expected = world_board @ np.linalg.inv(camera_board) @ np.linalg.inv(link_color)
    return corners.reshape(-1, 2), ids, k, d, world_board, link_color, expected, camera_board


def test_generated_board_detects_at_least_twelve_interpolated_corners():
    gray = cv2.imread(str(BOARD), cv2.IMREAD_GRAYSCALE)
    assert gray.shape == (1500, 2100)
    corners, ids = calibration.detect(gray, [2000, 0, 1050, 0, 2000, 750, 0, 0, 1], [0]*5)
    assert len(ids) >= 12
    assert len(corners) == len(ids)


def test_synthetic_pose_recovers_correct_composition_and_rejects_inverse_order():
    corners, ids, k, d, world_board, link_color, expected, camera_board = synthetic()
    result = calibration.estimate_from_corners(corners, ids, k, d, world_board, link_color)
    np.testing.assert_allclose(result["world_link"], expected, atol=1e-5)
    np.testing.assert_allclose(result["world_color"], expected @ link_color, atol=1e-5)
    assert result["rms_px"] < 1e-3
    inverted_wrong = world_board @ camera_board @ link_color
    assert np.linalg.norm(result["world_link"] - inverted_wrong) > 0.1


def test_reject_fewer_than_twelve_corners_and_high_rms():
    corners, ids, k, d, world_board, link_color, _, _ = synthetic()
    with pytest.raises(ValueError, match="Fewer than 12"):
        calibration.estimate_from_corners(corners[:11], ids[:11], k, d,
                                          world_board, link_color)
    corrupted = corners.copy()
    corrupted[12:] += [8.0, 0.0]
    with pytest.raises(ValueError, match="RMS"):
        calibration.estimate_from_corners(corrupted, ids, k, d,
                                          world_board, link_color)


def test_reject_incomplete_board_pose_and_nonrigid_matrix():
    template = yaml.safe_load(TEMPLATE.read_text())
    with pytest.raises(ValueError, match="incomplete|REQUIRED"):
        calibration.board_pose_matrix(template)
    nonrigid = np.eye(4)
    nonrigid[0, 0] = 1.1
    with pytest.raises(ValueError, match="not a finite rigid"):
        calibration.rigid(nonrigid)


def test_complete_board_pose_uses_verified_dimensions_and_tolerances():
    template = yaml.safe_load(TEMPLATE.read_text())
    template.update({"verified_square_length_m": 0.030,
                     "verified_marker_length_m": 0.022,
                     "verified_pattern_width_m": 0.210,
                     "verified_pattern_height_m": 0.150,
                     "tx_m": 0.1, "ty_m": -0.2, "tz_m": 0.3,
                     "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0,
                     "position_tolerance_m": 0.002,
                     "orientation_tolerance_rad": math.radians(1),
                     "measurement_method": "fixture survey",
                     "measurement_date_and_operator": "synthetic test",
                     "mount_description": "synthetic fixed base",
                     "world_origin_reference": "synthetic origin",
                     "world_axes_reference": "synthetic axes"})
    np.testing.assert_allclose(calibration.board_pose_matrix(template)[:3, 3],
                               [0.1, -0.2, 0.3])
    template["position_tolerance_m"] = 0.02
    with pytest.raises(ValueError, match="tolerance"):
        calibration.board_pose_matrix(template)
