#!/usr/bin/env python3
"""Generate one physically scaled diagnostic ChArUco board; no ROS access."""

import hashlib
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas
import yaml


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "evidence/codex_reports/2026-10-22_fase8u_charuco"
SQUARES_X, SQUARES_Y = 7, 5
SQUARE_M, MARKER_M = 0.030, 0.022
PIXELS_PER_MM = 10


def board_factory():
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_4X4_50"):
        raise RuntimeError("BLOCKED_ARUCO_UNAVAILABLE")
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y, SQUARE_M, MARKER_M, dictionary)
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y), SQUARE_M, MARKER_M, dictionary)
    raise RuntimeError("BLOCKED_ARUCO_UNAVAILABLE")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():
        raise RuntimeError("8U evidence directory already exists; refusing to overwrite")
    board = board_factory()
    width_mm, height_mm = 210, 150
    pixels = board.draw((width_mm*PIXELS_PER_MM, height_mm*PIXELS_PER_MM),
                        marginSize=0, borderBits=1)
    if pixels.shape != (1500, 2100) or set(np.unique(pixels)) != {0, 255}:
        raise RuntimeError("Board raster dimensions or colors are invalid")
    OUT.mkdir(parents=True, exist_ok=False)
    png = OUT / "charuco_board.png"
    Image.fromarray(pixels, mode="L").save(png, dpi=(254, 254))
    pdf = OUT / "charuco_board.pdf"
    page_width, page_height = landscape(A4)
    left = (page_width - width_mm*mm)/2
    bottom = (page_height - height_mm*mm)/2
    sheet = canvas.Canvas(str(pdf), pagesize=(page_width, page_height))
    sheet.setTitle("ChArUco 7x5, cuadrados de 30 mm, imprimir al 100 %")
    sheet.drawImage(ImageReader(Image.fromarray(pixels, mode="L")),
                    left, bottom, width=width_mm*mm, height=height_mm*mm,
                    mask="auto")
    sheet.setFont("Helvetica", 9)
    sheet.drawString(left, page_height-14*mm,
                     "ChArUco DICT_4X4_50 | 7 x 5 | cuadrados 30 mm | imprimir al 100 %")
    sheet.drawString(left, 13*mm,
                     "Verificar un cuadrado con regla: 30 mm. No ajustar a pagina.")
    sheet.setLineWidth(0.5)
    sheet.line(left, 10*mm, left+30*mm, 10*mm)
    sheet.line(left, 9*mm, left, 11*mm)
    sheet.line(left+30*mm, 9*mm, left+30*mm, 11*mm)
    sheet.save()
    specification = {
        "purpose": "session_only_observational_calibration_board",
        "not_for_safety": True,
        "dictionary": "DICT_4X4_50", "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y, "square_length_m": SQUARE_M,
        "marker_length_m": MARKER_M,
        "physical_pattern_width_m": SQUARES_X*SQUARE_M,
        "physical_pattern_height_m": SQUARES_Y*SQUARE_M,
        "raster_width_px": width_mm*PIXELS_PER_MM,
        "raster_height_px": height_mm*PIXELS_PER_MM,
        "raster_dpi": 254,
        "pdf_page": "A4 landscape",
        "pdf_pattern_width_mm": width_mm,
        "pdf_pattern_height_mm": height_mm,
        "charuco_inner_corner_count": (SQUARES_X-1)*(SQUARES_Y-1),
        "board_frame_origin": "upper-left outer corner of printed pattern",
        "board_frame_axes": "x right, y down, z into printed sheet when viewed from front",
        "opencv_version": cv2.__version__,
        "sha256": {"charuco_board.png": sha256(png),
                   "charuco_board.pdf": sha256(pdf)},
    }
    (OUT / "charuco_specification.yaml").write_text(
        yaml.safe_dump(specification, sort_keys=False, allow_unicode=True))
    print(f"generated {png} and {pdf}; {len(board.chessboardCorners)} inner corners")


if __name__ == "__main__":
    main()
